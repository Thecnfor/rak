#pragma once

#include <QJsonObject>
#include <QObject>
#include <QString>
#include <QVariantList>
#include <QVector>

class RobotClient;

// 应用级控制器: QML 与机器人控制后端之间的桥梁
// 持有真实 8 任务链, 暴露任务/状态/速度给 QML; 动作经 RobotClient 转发到后端
class AppController : public QObject {
    Q_OBJECT
    // 顶层页面的当前索引, 由底部导航切换
    Q_PROPERTY(int currentPage MEMBER m_currentPage NOTIFY currentPageChanged)
    // 当前页面标题, 随页面切换变化
    Q_PROPERTY(QString pageTitle READ pageTitle NOTIFY pageTitleChanged)
    // 任务链 (只读数组, 元素含 key/name/description/speed/status)
    Q_PROPERTY(QVariantList tasks READ tasks NOTIFY tasksChanged)
    // 聚焦舱当前选中的任务索引
    Q_PROPERTY(int selectedTask READ selectedTask NOTIFY selectedTaskChanged)
    // 当前正在运行的任务索引 (-1 = 未运行)
    Q_PROPERTY(int currentTask READ currentTask NOTIFY currentTaskChanged)
    // 是否运行中 (后端 active)
    Q_PROPERTY(bool running READ running NOTIFY runningChanged)
    // 当前速度 (选中任务的速度), 顶栏实时显示
    Q_PROPERTY(double currentSpeed READ currentSpeed NOTIFY currentSpeedChanged)
    // 已选中的任务数量 (触摸屏勾选)
    Q_PROPERTY(int selectedCount READ selectedCount NOTIFY tasksChanged)
    // 后端是否在线
    Q_PROPERTY(bool backendOnline READ backendOnline NOTIFY backendOnlineChanged)
    // 后端地址 (host:port), 供状态页展示
    Q_PROPERTY(QString backendAddress READ backendAddress NOTIFY backendOnlineChanged)
    // 当前选中任务的参数配置 (lane PID / 触发参数), 供聚焦舱调节
    Q_PROPERTY(QVariantMap taskConfig READ taskConfig NOTIFY taskConfigChanged)

public:
    explicit AppController(QObject *parent = nullptr);
    ~AppController() override;

    QString pageTitle() const;
    QVariantList tasks() const;
    int selectedTask() const;
    int currentTask() const;
    bool running() const;
    double currentSpeed() const;
    int selectedCount() const;
    bool backendOnline() const;
    QString backendAddress() const;
    QVariantMap taskConfig() const;

    void connectBackend(const QString &host, quint16 port);

public slots:
    // 底部导航: 切换到指定页面索引
    void setPage(int index);
    // 聚焦舱: 选中某个任务
    void selectTask(int index);
    // 触摸屏: 切换某任务的选中状态（选中哪几个就只跑哪几个）
    void toggleTaskSelected(int index);
    // 触摸屏: 全选 / 全不选
    void selectAllTasks(bool all);
    // 聚焦舱: 设定某个任务的独立速度
    void setTaskSpeed(int index, double speed);
    // 聚焦舱: 拉取某任务的参数配置 (lane PID / 触发参数)
    void fetchTaskConfig(int index);
    // 聚焦舱: 设置某任务的参数覆盖 (lane PID / 触发参数)
    void setTaskConfig(int index, const QVariantMap &config);
    // 从任务 index 起, 自动跑完后面所有任务
    void startFrom(int index);
    // 仅运行单个任务 (调试用)
    void runSingle(int index);
    // 停止当前运行 (急停)
    void stop();
    // 跳过当前任务
    void skip();
    // 重置已完成集合 (重来)
    void reset();

signals:
    void currentPageChanged(int index);
    void pageTitleChanged();
    void tasksChanged();
    void selectedTaskChanged();
    void currentTaskChanged();
    void runningChanged();
    void currentSpeedChanged();
    void backendOnlineChanged();
    void taskConfigChanged();
    // 面向用户的提示 (离线/忙碌等)
    void notice(const QString &message);

private:
    struct Task {
        QString key;         // 英文标识 (seeding...)
        QString name;        // 中文显示名
        QString description;
        double speed = 0.3;
        QString status = QStringLiteral("pending");
        bool selected = true;  // 触摸屏勾选：是否参与本轮执行
    };

    int indexOfKey(const QString &key) const;
    QStringList selectedKeys() const;
    void applyEvent(const QJsonObject &ev);
    void applyHello(const QJsonObject &hello);
    void setTaskStatus(int index, const QString &status);

    RobotClient *m_client;
    QVector<Task> m_tasks;
    int m_currentPage = 0;
    int m_selectedTask = 0;
    int m_currentTask = -1;
    bool m_running = false;
    QString m_host = QStringLiteral("localhost");
    quint16 m_port = 8500;
    QVariantMap m_taskConfig;   // 当前选中任务的参数配置 (lane PID / 触发参数)
};
