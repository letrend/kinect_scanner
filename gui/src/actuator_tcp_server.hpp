#pragma once

#include <QObject>
#include <QList>
#include <QString>

class QTcpServer;
class QTcpSocket;
class QJsonObject;

struct ActuatorState {
    bool valid = false;
    float turntableAngleDeg = 0.0f;
    float linearStageMm = 0.0f;
    bool moving = false;
    quint64 seq = 0;
    qint64 updatedMsec = 0;
};

Q_DECLARE_METATYPE(ActuatorState)

class ActuatorTcpServer : public QObject {
    Q_OBJECT
public:
    explicit ActuatorTcpServer(QObject *parent = nullptr);

    bool start(const QString &host, int port, QString *error = nullptr);
    bool isListening() const;
    bool hasClients() const { return !m_clients.isEmpty(); }

    ActuatorState state() const { return m_state; }
    quint64 sendTarget(float turntableAngleDeg, float linearStageMm);
    void sendStop();

signals:
    void statusMessage(QString msg);
    void stateChanged(ActuatorState state);
    void protocolError(QString msg);

private slots:
    void onNewConnection();
    void onReadyRead();
    void onDisconnected();

private:
    void send(QTcpSocket *socket, const QJsonObject &object);
    void broadcast(const QJsonObject &object);
    bool parseState(const QJsonObject &object, ActuatorState &state, QString *error) const;

    QTcpServer *m_server = nullptr;
    QList<QTcpSocket*> m_clients;
    ActuatorState m_state;
    quint64 m_nextCommandSeq = 1;
};
