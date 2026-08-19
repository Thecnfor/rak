#include "AppController.h"

#include <QJsonArray>
#include <QJsonValue>
#include <QVariantMap>

#include "RobotClient.h"

AppController::AppController(QObject *parent)
    : QObject(parent), m_client(new RobotClient(this)) {
    // 真实比赛任务链 (英文 key 与 baidu_smartcar_2026 的 TASK_ORDER 对应)
    m_tasks = {
        { QStringLiteral("seeding"),           QStringLiteral("播种"),     QStringLiteral("沿里程计行至播种区完成播种"), 0.3 },
        { QStringLiteral("target_detection"),  QStringLiteral("识别虫害"),  QStringLiteral("视觉识别虫害目标并确认"),    0.3 },
        { QStringLiteral("watering"),          QStringLiteral("灌溉"),     QStringLiteral("巡线至水源区完成灌溉"),       0.3 },
        { QStringLiteral("shooting"),          QStringLiteral("射击除害"),  QStringLiteral("视觉锁定害虫目标并射击"),     0.3 },
        { QStringLiteral("harvesting"),        QStringLiteral("作物收集"),  QStringLiteral("收集黄色/蓝色作物球"),       0.3 },
        { QStringLiteral("sorting"),           QStringLiteral("作物储存"),  QStringLiteral("按颜色归类储存作物"),         0.3 },
        { QStringLiteral("ordering"),          QStringLiteral("订单获取"),  QStringLiteral("巡线行至订单点获取订单"),     0.3 },
        { QStringLiteral("delivery"),          QStringLiteral("订单配送"),  QStringLiteral("按订单将作物配送至目标点"),   0.3 },
    };

    connect(m_client, &RobotClient::onlineChanged, this,
            [this](bool online) {
                if (!online && m_running) {
                    m_running = false;
                    m_currentTask = -1;
                    emit runningChanged();
                    emit currentTaskChanged();
                }
                emit backendOnlineChanged();
            });
    connect(m_client, &RobotClient::eventReceived, this,
            [this](const QJsonObject &ev) { applyEvent(ev); });
    connect(m_client, &RobotClient::requestFailed, this,
            [this](const QString &msg) { emit notice(msg); });
}

AppController::~AppController() = default;

void AppController::connectBackend(const QString &host, quint16 port) {
    m_host = host;
    m_port = port;
    m_client->connectTo(host, port);
    emit backendOnlineChanged();
}

QString AppController::pageTitle() const {
    static const QStringList titles = {
        QStringLiteral("主控台"),
        QStringLiteral("状态"),
        QStringLiteral("设置"),
    };
    return titles.value(m_currentPage, QStringLiteral("主控台"));
}

QVariantList AppController::tasks() const {
    QVariantList list;
    for (const auto &t : m_tasks) {
        list.append(QVariantMap{
            { QStringLiteral("key"),         t.key },
            { QStringLiteral("name"),        t.name },
            { QStringLiteral("description"), t.description },
            { QStringLiteral("speed"),       t.speed },
            { QStringLiteral("status"),      t.status },
        });
    }
    return list;
}

int AppController::selectedTask() const { return m_selectedTask; }
int AppController::currentTask() const { return m_currentTask; }
bool AppController::running() const { return m_running; }
bool AppController::backendOnline() const { return m_client->online(); }

double AppController::currentSpeed() const {
    if (m_selectedTask >= 0 && m_selectedTask < m_tasks.size())
        return m_tasks[m_selectedTask].speed;
    return 0.0;
}

QString AppController::backendAddress() const {
    return QStringLiteral("%1:%2").arg(m_host).arg(m_port);
}

void AppController::setPage(int index) {
    if (m_currentPage == index)
        return;
    m_currentPage = index;
    emit currentPageChanged(m_currentPage);
    emit pageTitleChanged();
}

void AppController::selectTask(int index) {
    if (index < 0 || index >= m_tasks.size() || index == m_selectedTask)
        return;
    m_selectedTask = index;
    emit selectedTaskChanged();
    emit currentSpeedChanged();
}

void AppController::setTaskSpeed(int index, double speed) {
    if (index < 0 || index >= m_tasks.size())
        return;
    m_tasks[index].speed = speed;
    emit tasksChanged();
    if (index == m_selectedTask)
        emit currentSpeedChanged();
    if (backendOnline())
        m_client->setTaskSpeed(m_tasks[index].key, speed);
}

void AppController::startFrom(int index) {
    if (index < 0 || index >= m_tasks.size())
        return;
    if (!backendOnline()) {
        emit notice(QStringLiteral("后端离线，无法启动"));
        return;
    }
    // 乐观更新: index 前标 done, index 起跑
    for (int i = 0; i < m_tasks.size(); ++i)
        m_tasks[i].status = i < index ? QStringLiteral("done")
                         : i == index ? QStringLiteral("running")
                         : QStringLiteral("pending");
    m_currentTask = index;
    m_running = true;
    emit tasksChanged();
    emit currentTaskChanged();
    emit runningChanged();
    m_client->start(index);
}

void AppController::runSingle(int index) {
    if (index < 0 || index >= m_tasks.size())
        return;
    if (!backendOnline()) {
        emit notice(QStringLiteral("后端离线，无法运行"));
        return;
    }
    m_tasks[index].status = QStringLiteral("running");
    m_currentTask = index;
    m_running = true;
    emit tasksChanged();
    emit currentTaskChanged();
    emit runningChanged();
    m_client->runTask(m_tasks[index].key);
}

void AppController::stop() {
    if (!backendOnline()) {
        emit notice(QStringLiteral("后端离线，无法停止"));
        return;
    }
    m_client->stop();
    // 乐观停止, 由 run:finished / task:skipped 事件收敛
    m_running = false;
    m_currentTask = -1;
    for (auto &t : m_tasks)
        if (t.status == QStringLiteral("running"))
            t.status = QStringLiteral("pending");
    emit tasksChanged();
    emit currentTaskChanged();
    emit runningChanged();
}

void AppController::skip() {
    if (backendOnline())
        m_client->skip();
}

void AppController::reset() {
    if (backendOnline())
        m_client->reset();
    // 乐观清零, reset 事件会再收敛
    for (auto &t : m_tasks)
        t.status = QStringLiteral("pending");
    m_running = false;
    m_currentTask = -1;
    emit tasksChanged();
    emit currentTaskChanged();
    emit runningChanged();
}

// ====================================================================
// 事件处理
// ====================================================================
int AppController::indexOfKey(const QString &key) const {
    for (int i = 0; i < m_tasks.size(); ++i)
        if (m_tasks[i].key == key)
            return i;
    return -1;
}

void AppController::setTaskStatus(int index, const QString &status) {
    if (index < 0 || index >= m_tasks.size() || m_tasks[index].status == status)
        return;
    m_tasks[index].status = status;
    emit tasksChanged();
}

void AppController::applyEvent(const QJsonObject &ev) {
    const QString type = ev.value(QStringLiteral("type")).toString();
    if (type == QStringLiteral("hello")) {
        applyHello(ev);
        return;
    }
    if (type == QStringLiteral("run:started")) {
        m_running = true;
        emit runningChanged();
        return;
    }
    if (type == QStringLiteral("run:finished")) {
        m_running = false;
        m_currentTask = -1;
        for (auto &t : m_tasks)
            if (t.status == QStringLiteral("running"))
                t.status = QStringLiteral("pending");
        emit tasksChanged();
        emit currentTaskChanged();
        emit runningChanged();
        return;
    }
    if (type == QStringLiteral("task:started")) {
        int idx = indexOfKey(ev.value(QStringLiteral("task")).toString());
        if (idx >= 0) {
            setTaskStatus(idx, QStringLiteral("running"));
            m_currentTask = idx;
            m_running = true;
            emit currentTaskChanged();
            emit runningChanged();
        }
        return;
    }
    if (type == QStringLiteral("task:done")) {
        int idx = indexOfKey(ev.value(QStringLiteral("task")).toString());
        if (idx >= 0) {
            setTaskStatus(idx, QStringLiteral("done"));
            if (m_currentTask == idx) {
                m_currentTask = -1;
                emit currentTaskChanged();
            }
        }
        return;
    }
    if (type == QStringLiteral("task:skipped")) {
        int idx = indexOfKey(ev.value(QStringLiteral("task")).toString());
        if (idx >= 0) {
            setTaskStatus(idx, QStringLiteral("pending"));
            if (m_currentTask == idx) {
                m_currentTask = -1;
                emit currentTaskChanged();
            }
        }
        return;
    }
    if (type == QStringLiteral("task:error")) {
        int idx = indexOfKey(ev.value(QStringLiteral("task")).toString());
        if (idx >= 0) {
            setTaskStatus(idx, QStringLiteral("failed"));
            if (m_currentTask == idx) {
                m_currentTask = -1;
                emit currentTaskChanged();
            }
        }
        emit notice(ev.value(QStringLiteral("error")).toString());
        return;
    }
    if (type == QStringLiteral("reset")) {
        for (auto &t : m_tasks)
            t.status = QStringLiteral("pending");
        m_running = false;
        m_currentTask = -1;
        emit tasksChanged();
        emit currentTaskChanged();
        emit runningChanged();
        return;
    }
    if (type == QStringLiteral("error")) {
        emit notice(ev.value(QStringLiteral("message")).toString(QStringLiteral("后端错误")));
        return;
    }
    // odom / 其它: 暂不处理
}

void AppController::applyHello(const QJsonObject &hello) {
    const QJsonArray arr = hello.value(QStringLiteral("tasks")).toArray();
    if (arr.isEmpty())
        return;
    for (const auto &v : arr) {
        const QJsonObject o = v.toObject();
        int idx = indexOfKey(o.value(QStringLiteral("name")).toString());
        if (idx < 0)
            continue;
        m_tasks[idx].name = o.value(QStringLiteral("name_cn")).toString(
            m_tasks[idx].name);
        m_tasks[idx].description = o.value(QStringLiteral("description")).toString(
            m_tasks[idx].description);
        m_tasks[idx].speed = o.value(QStringLiteral("speed")).toDouble(
            m_tasks[idx].speed);
        m_tasks[idx].status = o.value(QStringLiteral("status")).toString(
            QStringLiteral("pending"));
    }
    m_running = hello.value(QStringLiteral("active")).toBool();
    const QString cur = hello.value(QStringLiteral("current")).toString();
    m_currentTask = cur.isEmpty() ? -1 : indexOfKey(cur);
    emit tasksChanged();
    emit currentTaskChanged();
    emit runningChanged();
}
