#include "tcp_control_server.hpp"

#include <QHostAddress>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QTcpServer>
#include <QTcpSocket>

TcpControlServer::TcpControlServer(QObject *parent) : QObject(parent) {
    m_server = new QTcpServer(this);
    connect(m_server, &QTcpServer::newConnection,
            this, &TcpControlServer::onNewConnection);
}

bool TcpControlServer::start(const QString &host, int port, QString *error) {
    QHostAddress address;
    if (host.isEmpty() || host == "0.0.0.0")
        address = QHostAddress::Any;
    else if (host == "127.0.0.1" || host == "localhost")
        address = QHostAddress::LocalHost;
    else if (!address.setAddress(host)) {
        if (error) *error = QString("Invalid TCP bind address: %1").arg(host);
        return false;
    }
    if (!m_server->listen(address, quint16(port))) {
        if (error) *error = m_server->errorString();
        return false;
    }
    emit statusMessage(QString("UI control TCP listening on %1:%2")
                       .arg(host).arg(port));
    return true;
}

void TcpControlServer::send(QTcpSocket *socket, const QJsonObject &object) {
    if (!socket) return;
    socket->write(QJsonDocument(object).toJson(QJsonDocument::Compact));
    socket->write("\n");
    socket->flush();
}

void TcpControlServer::broadcast(const QJsonObject &object) {
    for (QTcpSocket *client : m_clients)
        send(client, object);
}

void TcpControlServer::onNewConnection() {
    while (QTcpSocket *socket = m_server->nextPendingConnection()) {
        m_clients.append(socket);
        connect(socket, &QTcpSocket::readyRead, this, &TcpControlServer::onReadyRead);
        connect(socket, &QTcpSocket::disconnected, this, &TcpControlServer::onDisconnected);
        emit statusMessage("UI control TCP client connected");
    }
}

void TcpControlServer::onReadyRead() {
    QTcpSocket *socket = qobject_cast<QTcpSocket*>(sender());
    if (!socket) return;
    while (socket->canReadLine()) {
        QByteArray line = socket->readLine().trimmed();
        if (line.isEmpty()) continue;
        QJsonParseError err;
        QJsonDocument doc = QJsonDocument::fromJson(line, &err);
        if (err.error != QJsonParseError::NoError || !doc.isObject()) {
            QJsonObject out;
            out["type"] = "error";
            out["code"] = "bad_json";
            out["message"] = err.errorString();
            send(socket, out);
            continue;
        }
        emit commandReceived(doc.object(), socket);
    }
}

void TcpControlServer::onDisconnected() {
    QTcpSocket *socket = qobject_cast<QTcpSocket*>(sender());
    if (!socket) return;
    m_clients.removeAll(socket);
    socket->deleteLater();
    emit statusMessage("UI control TCP client disconnected");
}
