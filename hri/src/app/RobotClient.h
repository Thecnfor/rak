#pragma once

#include <QJsonObject>
#include <QObject>
#include <QString>

class QNetworkAccessManager;
class QTimer;
class WsClient;

// 机器人控制后端的客户端: HTTP 发命令, WS 收实时状态事件。
class RobotClient : public QObject {
    Q_OBJECT
    Q_PROPERTY(bool online READ online NOTIFY onlineChanged)
public:
    explicit RobotClient(QObject *parent = nullptr);
    ~RobotClient() override;

    bool online() const;

    Q_INVOKABLE void connectTo(const QString &host, quint16 port);
    Q_INVOKABLE void disconnectFrom();
    Q_INVOKABLE void fetchStatus();

    // 命令 (HTTP POST)
    Q_INVOKABLE void start(int fromIndex = -1);
    Q_INVOKABLE void runTask(const QString &name);
    Q_INVOKABLE void stop();
    Q_INVOKABLE void skip();
    Q_INVOKABLE void reset();
    Q_INVOKABLE void setTaskSpeed(const QString &name, double speed);

signals:
    void onlineChanged(bool online);
    void eventReceived(const QJsonObject &event);
    void requestFailed(const QString &message);

private:
    void openWebSocket();
    void postCommand(const QString &path, const QJsonObject &body = QJsonObject(),
                     bool isGet = false);
    void fetchHello();

    QNetworkAccessManager *m_net;
    WsClient *m_ws;
    QTimer *m_reconnect;
    QString m_host = QStringLiteral("localhost");
    quint16 m_port = 8500;
    bool m_online = false;
    bool m_intentionalClose = false;
};
