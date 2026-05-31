#pragma once

#include <QObject>
#include <QJsonObject>
#include <QList>

class QTcpServer;
class QTcpSocket;

class TcpControlServer : public QObject {
    Q_OBJECT
public:
    explicit TcpControlServer(QObject *parent = nullptr);
    bool start(const QString &host, int port, QString *error = nullptr);
    void send(QTcpSocket *socket, const QJsonObject &object);
    void broadcast(const QJsonObject &object);

signals:
    void commandReceived(QJsonObject command, QTcpSocket *socket);
    void statusMessage(QString message);

private slots:
    void onNewConnection();
    void onReadyRead();
    void onDisconnected();

private:
    QTcpServer *m_server = nullptr;
    QList<QTcpSocket*> m_clients;
};
