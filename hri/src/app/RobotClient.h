#pragma once

#include <QJsonObject>
#include <QObject>
#include <QString>
#include <QStringList>
#include <QVariantMap>

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
    // 设置触摸屏选中的任务子集（选中哪几个就只跑哪几个）
    Q_INVOKABLE void setSelectedTasks(const QStringList &tasks);
    // 设置某任务的参数覆盖（lane PID / 触发参数）
    Q_INVOKABLE void setTaskConfig(const QString &name, const QVariantMap &config);
    // 拉取某任务的参数配置（lane PID / 触发参数）
    Q_INVOKABLE void fetchTaskConfig(const QString &name);

signals:
    void onlineChanged(bool online);
    void eventReceived(const QJsonObject &event);
    void taskConfigReceived(const QString &name, const QVariantMap &config);
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
