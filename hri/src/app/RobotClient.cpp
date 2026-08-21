#include "RobotClient.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QTimer>
#include <QUrl>

#include "WsClient.h"

RobotClient::RobotClient(QObject *parent)
    : QObject(parent),
      m_net(new QNetworkAccessManager(this)),
      m_ws(new WsClient(this)),
      m_reconnect(new QTimer(this)) {
    m_reconnect->setSingleShot(true);
    m_reconnect->setInterval(2000);
    connect(m_reconnect, &QTimer::timeout, this, &RobotClient::openWebSocket);

    connect(m_ws, &WsClient::connected, this, [this]() {
        m_online = true;
        emit onlineChanged(true);
        fetchHello(); // 连接后拉一份完整快照
    });
    connect(m_ws, &WsClient::textReceived, this, [this](const QString &text) {
        QJsonParseError err;
        QJsonDocument doc = QJsonDocument::fromJson(text.toUtf8(), &err);
        if (err.error == QJsonParseError::NoError && doc.isObject())
            emit eventReceived(doc.object());
    });
    connect(m_ws, &WsClient::disconnected, this, [this]() {
        if (m_online) {
            m_online = false;
            emit onlineChanged(false);
        }
        if (!m_intentionalClose && !m_reconnect->isActive())
            m_reconnect->start();
    });
}

RobotClient::~RobotClient() = default;

bool RobotClient::online() const { return m_online; }

void RobotClient::connectTo(const QString &host, quint16 port) {
    m_intentionalClose = false;
    m_host = host;
    m_port = port;
    m_ws->close();
    openWebSocket();
}

void RobotClient::disconnectFrom() {
    m_intentionalClose = true;
    m_reconnect->stop();
    m_ws->close();
    if (m_online) {
        m_online = false;
        emit onlineChanged(false);
    }
}

void RobotClient::openWebSocket() {
    QUrl url;
    url.setScheme(QStringLiteral("ws"));
    url.setHost(m_host);
    url.setPort(m_port);
    url.setPath(QStringLiteral("/ws"));
    m_ws->open(url);
}

void RobotClient::fetchStatus() { fetchHello(); }
void RobotClient::fetchHello() { postCommand(QStringLiteral("/api/hello"), {}, true); }

void RobotClient::postCommand(const QString &path, const QJsonObject &body, bool isGet) {
    QUrl url(QStringLiteral("http://%1:%2%3").arg(m_host).arg(m_port).arg(path));
    QNetworkRequest req(url);
    QNetworkReply *reply = nullptr;
    if (isGet) {
        reply = m_net->get(req);
    } else {
        req.setHeader(QNetworkRequest::ContentTypeHeader,
                      QStringLiteral("application/json"));
        reply = m_net->post(req, QJsonDocument(body).toJson(QJsonDocument::Compact));
    }
    connect(reply, &QNetworkReply::finished, reply, [this, reply]() {
        reply->deleteLater();
        if (reply->error() != QNetworkReply::NoError) {
            emit requestFailed(QStringLiteral("请求失败: %1")
                                   .arg(reply->errorString()));
            return;
        }
        QByteArray data = reply->readAll();
        QJsonParseError err;
        QJsonDocument doc = QJsonDocument::fromJson(data, &err);
        if (err.error == QJsonParseError::NoError && doc.isObject()) {
            // 若 GET hello, 把结果作为 hello 事件交给上层
            if (reply->url().path().endsWith(QStringLiteral("/api/hello")))
                emit eventReceived(doc.object());
        }
    });
}

void RobotClient::start(int fromIndex) {
    postCommand(QStringLiteral("/api/start"),
                QJsonObject{{QStringLiteral("from_index"), fromIndex}});
}

void RobotClient::runTask(const QString &name) {
    postCommand(QStringLiteral("/api/run/%1").arg(name));
}

void RobotClient::stop() { postCommand(QStringLiteral("/api/stop")); }
void RobotClient::skip() { postCommand(QStringLiteral("/api/skip")); }
void RobotClient::reset() { postCommand(QStringLiteral("/api/reset")); }

void RobotClient::setTaskSpeed(const QString &name, double speed) {
    postCommand(QStringLiteral("/api/tasks/%1/speed").arg(name),
                QJsonObject{{QStringLiteral("speed"), speed}});
}

void RobotClient::setSelectedTasks(const QStringList &tasks) {
    QJsonArray arr;
    for (const auto &t : tasks)
        arr.append(t);
    postCommand(QStringLiteral("/api/select"),
                QJsonObject{{QStringLiteral("tasks"), arr}});
}

void RobotClient::setTaskConfig(const QString &name, const QVariantMap &config) {
    postCommand(QStringLiteral("/api/tasks/%1/config").arg(name),
                QJsonObject{{QStringLiteral("config"), QJsonObject::fromVariantMap(config)}});
}

void RobotClient::fetchTaskConfig(const QString &name) {
    QUrl url(QStringLiteral("http://%1:%2/api/tasks/%3/config")
                 .arg(m_host).arg(m_port).arg(name));
    QNetworkRequest req(url);
    QNetworkReply *reply = m_net->get(req);
    connect(reply, &QNetworkReply::finished, reply, [this, reply, name]() {
        reply->deleteLater();
        if (reply->error() != QNetworkReply::NoError) {
            emit requestFailed(QStringLiteral("拉取参数失败: %1")
                                   .arg(reply->errorString()));
            return;
        }
        QJsonParseError err;
        QJsonDocument doc = QJsonDocument::fromJson(reply->readAll(), &err);
        if (err.error == QJsonParseError::NoError && doc.isObject()) {
            const QJsonObject obj = doc.object();
            emit taskConfigReceived(name, obj.value(QStringLiteral("config")).toObject().toVariantMap());
        }
    });
}
