// 集成自检: 无 GUI 环境用 AppController 连 mock 后端, 验证 C++ 网络层
// 编译: moc 三个 Q_OBJECT 头 + 本文件 + AppController/RobotClient/WsClient.cpp, 链 Qt6 Core+Network
#include <QCoreApplication>
#include <QDebug>
#include <QTimer>
#include <QVariantList>
#include <QVariantMap>

#include "app/AppController.h"

static void dump(const AppController &c, const char *tag) {
    const QVariantList tasks = c.tasks();
    QStringList st;
    for (const auto &t : tasks)
        st << t.toMap().value("key").toString() + "=" + t.toMap().value("status").toString();
    qInfo() << "[test]" << tag << "| online=" << c.backendOnline()
            << "running=" << c.running() << "| " << st.join(" ");
}

int main(int argc, char **argv) {
    QCoreApplication app(argc, argv);
    AppController c;

    QObject::connect(&c, &AppController::backendOnlineChanged, [&]() {
        qInfo() << "[test] backendOnlineChanged ->" << c.backendOnline();
    });
    QObject::connect(&c, &AppController::notice, [](const QString &m) {
        qInfo() << "[test] notice:" << m;
    });
    QObject::connect(&c, &AppController::tasksChanged,
                     [&]() { dump(c, "tasksChanged"); });
    QObject::connect(&c, &AppController::currentTaskChanged,
                     [&]() { qInfo() << "[test] currentTask ->" << c.currentTask(); });
    QObject::connect(&c, &AppController::runningChanged,
                     [&]() { qInfo() << "[test] running ->" << c.running(); });

    c.connectBackend("localhost", 8500);

    QTimer::singleShot(2500, [&]() { qInfo() << ">>> startFrom(0)"; c.startFrom(0); });
    QTimer::singleShot(4200, [&]() { qInfo() << ">>> skip()"; c.skip(); });
    QTimer::singleShot(5600, [&]() { qInfo() << ">>> stop()"; c.stop(); });
    QTimer::singleShot(7200, [&]() {
        qInfo() << ">>> setTaskSpeed(delivery, 1.2)"; c.setTaskSpeed(7, 1.2);
    });
    QTimer::singleShot(8000, [&]() {
        dump(c, "FINAL");
        app.quit();
    });

    return app.exec();
}
