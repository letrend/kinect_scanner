#include "actuator_tcp_server.hpp"

#include <QDateTime>
#include <QHostAddress>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QStringList>
#include <QTcpServer>
#include <QTcpSocket>

namespace {

bool isTurntableName(const QString &name) {
    const QString n = name.trimmed().toLower();
    return n == "turntable_angle_deg" ||
           n == "turntable_angle" ||
           n == "turntable" ||
           n == "angle_deg";
}

bool isStageName(const QString &name) {
    const QString n = name.trimmed().toLower();
    return n == "linear_stage_mm" ||
           n == "linear_stage" ||
           n == "stage_mm" ||
           n == "stage";
}

bool readNumber(const QJsonObject &object, const QStringList &keys, float &out) {
    for (const QString &key : keys) {
        if (object.contains(key) && object.value(key).isDouble()) {
            out = (float)object.value(key).toDouble();
            return true;
        }
    }
    return false;
}

bool readNamedJointArray(const QJsonObject &object, float &angleDeg, float &stageMm,
                         bool &hasAngle, bool &hasStage) {
    const QJsonArray names = object.value("joint_names").toArray();
    const QJsonArray positions = object.value("positions").toArray();
    if (names.isEmpty() || positions.isEmpty() || names.size() != positions.size())
        return false;

    for (int i = 0; i < names.size(); ++i) {
        const QString name = names.at(i).toString();
        if (!positions.at(i).isDouble())
            continue;
        if (isTurntableName(name)) {
            angleDeg = (float)positions.at(i).toDouble();
            hasAngle = true;
        } else if (isStageName(name)) {
            stageMm = (float)positions.at(i).toDouble();
            hasStage = true;
        }
    }
    return hasAngle || hasStage;
}

} // namespace

ActuatorTcpServer::ActuatorTcpServer(QObject *parent) : QObject(parent) {
    m_server = new QTcpServer(this);
    connect(m_server, &QTcpServer::newConnection,
            this, &ActuatorTcpServer::onNewConnection);
}

bool ActuatorTcpServer::start(const QString &host, int port, QString *error) {
    QHostAddress address;
    if (host.isEmpty() || host == "0.0.0.0")
        address = QHostAddress::Any;
    else if (host == "127.0.0.1" || host == "localhost")
        address = QHostAddress::LocalHost;
    else if (!address.setAddress(host)) {
        if (error) *error = QString("Invalid actuator TCP bind address: %1").arg(host);
        return false;
    }
    if (!m_server->listen(address, quint16(port))) {
        if (error) *error = m_server->errorString();
        return false;
    }
    emit statusMessage(QString("Actuator TCP listening on %1:%2").arg(host).arg(port));
    return true;
}

bool ActuatorTcpServer::isListening() const {
    return m_server && m_server->isListening();
}

quint64 ActuatorTcpServer::sendTarget(float turntableAngleDeg, float linearStageMm) {
    const quint64 seq = m_nextCommandSeq++;
    QJsonObject command;
    command["type"] = "cmd";
    command["seq"] = QString::number(seq);
    command["turntable_angle_deg"] = turntableAngleDeg;
    command["linear_stage_mm"] = linearStageMm;

    QJsonArray names;
    names.append("turntable_angle_deg");
    names.append("linear_stage_mm");
    QJsonArray positions;
    positions.append(turntableAngleDeg);
    positions.append(linearStageMm);
    command["joint_names"] = names;
    command["positions"] = positions;

    broadcast(command);
    emit statusMessage(QString("Actuator target #%1: angle %2 deg, stage %3 mm")
                       .arg(seq).arg(turntableAngleDeg, 0, 'f', 2)
                       .arg(linearStageMm, 0, 'f', 2));
    return seq;
}

void ActuatorTcpServer::sendStop() {
    QJsonObject command;
    command["type"] = "stop";
    broadcast(command);
}

void ActuatorTcpServer::send(QTcpSocket *socket, const QJsonObject &object) {
    if (!socket) return;
    socket->write(QJsonDocument(object).toJson(QJsonDocument::Compact));
    socket->write("\n");
    socket->flush();
}

void ActuatorTcpServer::broadcast(const QJsonObject &object) {
    for (QTcpSocket *client : m_clients)
        send(client, object);
}

void ActuatorTcpServer::onNewConnection() {
    while (QTcpSocket *socket = m_server->nextPendingConnection()) {
        m_clients.append(socket);
        connect(socket, &QTcpSocket::readyRead,
                this, &ActuatorTcpServer::onReadyRead);
        connect(socket, &QTcpSocket::disconnected,
                this, &ActuatorTcpServer::onDisconnected);
        emit statusMessage("Actuator TCP client connected");
    }
}

void ActuatorTcpServer::onReadyRead() {
    QTcpSocket *socket = qobject_cast<QTcpSocket*>(sender());
    if (!socket) return;

    while (socket->canReadLine()) {
        QByteArray line = socket->readLine().trimmed();
        if (line.isEmpty()) continue;

        QJsonParseError parseError;
        QJsonDocument doc = QJsonDocument::fromJson(line, &parseError);
        if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
            QJsonObject out;
            out["type"] = "error";
            out["code"] = "bad_json";
            out["message"] = parseError.errorString();
            send(socket, out);
            emit protocolError(QString("Actuator bad JSON: %1").arg(parseError.errorString()));
            continue;
        }

        QJsonObject object = doc.object();
        const QString type = object.value("type").toString();
        if (type == "hello") {
            QJsonObject out;
            out["type"] = "hello";
            out["ok"] = true;
            send(socket, out);
            continue;
        }

        ActuatorState next;
        QString error;
        if (!parseState(object, next, &error)) {
            QJsonObject out;
            out["type"] = "error";
            out["code"] = "bad_state";
            out["message"] = error;
            send(socket, out);
            emit protocolError(QString("Actuator state rejected: %1").arg(error));
            continue;
        }

        m_state = next;
        emit stateChanged(m_state);
    }
}

void ActuatorTcpServer::onDisconnected() {
    QTcpSocket *socket = qobject_cast<QTcpSocket*>(sender());
    if (!socket) return;
    m_clients.removeAll(socket);
    socket->deleteLater();
    emit statusMessage("Actuator TCP client disconnected");
}

bool ActuatorTcpServer::parseState(const QJsonObject &object, ActuatorState &state,
                                   QString *error) const {
    const QString type = object.value("type").toString();
    if (!type.isEmpty() && type != "state" && type != "feedback" && type != "pose") {
        if (error) *error = QString("Expected type state, feedback, or pose; got %1").arg(type);
        return false;
    }

    state = m_state;
    bool hasAngle = false;
    bool hasStage = false;

    hasAngle = readNumber(object,
                          {"turntable_angle_deg", "turntableAngleDeg", "angle_deg", "angleDeg"},
                          state.turntableAngleDeg);
    hasStage = readNumber(object,
                          {"linear_stage_mm", "linearStageMm", "stage_mm", "stageMm"},
                          state.linearStageMm);

    QJsonObject joints = object.value("joints").toObject();
    if (!joints.isEmpty()) {
        if (readNumber(joints, {"turntable_angle_deg", "turntableAngleDeg",
                                "angle_deg", "angleDeg"}, state.turntableAngleDeg))
            hasAngle = true;
        if (readNumber(joints, {"linear_stage_mm", "linearStageMm",
                                "stage_mm", "stageMm"}, state.linearStageMm))
            hasStage = true;
    }

    QJsonObject position = object.value("position").toObject();
    if (!position.isEmpty()) {
        if (readNumber(position, {"turntable_angle_deg", "turntableAngleDeg",
                                  "angle_deg", "angleDeg"}, state.turntableAngleDeg))
            hasAngle = true;
        if (readNumber(position, {"linear_stage_mm", "linearStageMm",
                                  "stage_mm", "stageMm"}, state.linearStageMm))
            hasStage = true;
    }

    readNamedJointArray(object, state.turntableAngleDeg, state.linearStageMm,
                        hasAngle, hasStage);

    if (!hasAngle || !hasStage) {
        if (error) *error = "State requires turntable_angle_deg and linear_stage_mm";
        return false;
    }

    if (object.contains("moving"))
        state.moving = object.value("moving").toBool(false);
    else if (object.contains("in_position"))
        state.moving = !object.value("in_position").toBool(false);
    else
        state.moving = false;

    if (object.contains("seq")) {
        if (object.value("seq").isString())
            state.seq = object.value("seq").toString().toULongLong();
        else
            state.seq = (quint64)object.value("seq").toDouble(state.seq);
    } else {
        state.seq += 1;
    }
    state.valid = true;
    state.updatedMsec = QDateTime::currentMSecsSinceEpoch();
    return true;
}
