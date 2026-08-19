#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QCursor>
#include <QDir>
#include <QGuiApplication>
#include <QInputMethod>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcessEnvironment>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickStyle>
#include <QQuickWindow>
#include <QRect>
#include <QScreen>
#include <QSize>
#include <QStandardPaths>
#include <QSurfaceFormat>

#include "app/AppController.h"

// ============================================================================
// 触摸屏适配：
//   1. 启动模式：--fullscreen（Orin 独占全屏，无边框无标题栏，覆盖整个桌面）
//               --windowed（x86 开发机预览用，1024x600 有边框）
//               --kiosk    （更强：无装饰 + 固定屏 + 隐藏光标 + 禁用退出手势）
//   2. 环境变量已在外部（systemd / start_hri.sh）设置 QT_QPA_PLATFORM=eglfs / linuxfb
//      时，本程序自动隐藏鼠标光标；桌面 X11/Wayland 下用 --hide-cursor 手动隐藏。
//   3. 触摸事件：Qt 6 默认合成鼠标事件，保持；若出现双触发用
//      QT_QUICK_TOUCH_COMPRESSION_MAX_EVENTS / --compress-touch 控制。
//   4. 虚拟键盘：QtQuick VirtualKeyboard 在 QML 端 InputPanel 自动弹出，
//      设置页/后续文本框可直接 qtvirtualkeyboard 模块。
// ============================================================================

namespace {

struct LaunchOptions {
    bool fullscreen = false;
    bool kiosk = false;
    bool windowed = false;
    bool hideCursor = false;
    bool frame = true;
    QSize fixedSize{1024, 600};
    int scalePercent = 100;  // >100 在 1024x600 屏上整体放大，适配触控小按钮
};

LaunchOptions parseArgs(int argc, char *argv[]) {
    LaunchOptions opts;
    // QCommandLineParser 需要 QCoreApplication，但我们要在 app 构造之前设置
    // QGuiApplication::setAttribute / setHighDpi，所以用手工解析 + 二次确认。
    for (int i = 1; i < argc; ++i) {
        const QString a = QString::fromLocal8Bit(argv[i]);
        if (a == QStringLiteral("--fullscreen"))
            opts.fullscreen = true;
        else if (a == QStringLiteral("--kiosk"))
            opts.kiosk = true;
        else if (a == QStringLiteral("--windowed"))
            opts.windowed = true;
        else if (a == QStringLiteral("--hide-cursor"))
            opts.hideCursor = true;
        else if (a == QStringLiteral("--no-frame"))
            opts.frame = false;
        else if (a.startsWith(QStringLiteral("--scale="))) {
            bool ok = false;
            int v = a.mid(8).toInt(&ok);
            if (ok && v >= 80 && v <= 200) opts.scalePercent = v;
        }
    }
    // Kiosk 隐式含全屏+隐藏光标+无边框
    if (opts.kiosk) {
        opts.fullscreen = true;
        opts.hideCursor = true;
        opts.frame = false;
    }
    // 默认策略：未显式选模式时，根据 QPA 平台自动判断
    if (!opts.fullscreen && !opts.kiosk && !opts.windowed) {
        const QString platform = qEnvironmentVariable("QT_QPA_PLATFORM");
        if (platform.startsWith(QStringLiteral("eglfs")) ||
            platform.startsWith(QStringLiteral("linuxfb")) ||
            platform.startsWith(QStringLiteral("vnc")) ||
            platform.startsWith(QStringLiteral("directfb"))) {
            opts.fullscreen = true;
            opts.hideCursor = true;
            opts.frame = false;
        } else {
            opts.windowed = true;
        }
    }
    if (opts.fullscreen) opts.frame = false;
    return opts;
}

void applyQtAttributesBeforeApp(const LaunchOptions &opts) {
    // 高 DPI PassThrough：设计稿 1024x600 = 实际屏幕像素
    QGuiApplication::setHighDpiScaleFactorRoundingPolicy(
        Qt::HighDpiScaleFactorRoundingPolicy::PassThrough);

    // 触摸合成：避免长按触发右键菜单
    QGuiApplication::setAttribute(Qt::AA_SynthesizeMouseForUnhandledTouchEvents, true);
    QGuiApplication::setAttribute(Qt::AA_SynthesizeTouchForUnhandledMouseEvents, false);

    // 抗锯齿 + 垂直同步，玻璃拟态不闪
    QSurfaceFormat fmt;
    fmt.setSamples(4);
    fmt.setSwapInterval(1);
    fmt.setDepthBufferSize(16);
    QSurfaceFormat::setDefaultFormat(fmt);

    // Basic 样式：与现有 Controls.Basic 一致，减少外部主题影响
    QQuickStyle::setStyle(QStringLiteral("Basic"));

    Q_UNUSED(opts);
}

void applyAppOptionsAfterApp(QGuiApplication &app, const LaunchOptions &opts) {
    if (opts.hideCursor) {
        app.setOverrideCursor(QCursor(Qt::BlankCursor));
    }
    // 触摸屏应用不希望长按时弹出系统菜单/文本选中
    Q_UNUSED(app);
}

// 把固定 1024x600 的设计稿映射到屏幕：
//   - 若屏幕正好 1024x600：1:1，centerIn = screen
//   - 若屏幕更大（1280x800 等）：保持 1024x600 居中，或者 scalePercent 放大
//   - 若屏幕更小：按屏幕短边等比缩小（防止裁剪）
QRect computeWindowGeometry(const QScreen *screen, const LaunchOptions &opts) {
    const QRect avail = screen->geometry();
    if (opts.fullscreen || opts.kiosk) return avail;

    qreal scale = qreal(opts.scalePercent) / 100.0;
    qreal w = opts.fixedSize.width() * scale;
    qreal h = opts.fixedSize.height() * scale;

    // 屏幕更小时按比例 fit
    if (w > avail.width() || h > avail.height()) {
        const qreal f = qMin(qreal(avail.width()) / w, qreal(avail.height()) / h);
        w *= f;
        h *= f;
    }
    const int x = avail.x() + (avail.width() - int(w)) / 2;
    const int y = avail.y() + (avail.height() - int(h)) / 2;
    return QRect(x, y, int(w), int(h));
}

}  // namespace

int main(int argc, char *argv[]) {
    const LaunchOptions opts = parseArgs(argc, argv);
    applyQtAttributesBeforeApp(opts);

    QGuiApplication app(argc, argv);
    app.setApplicationName(QStringLiteral("HRI"));
    app.setApplicationDisplayName(QStringLiteral("HRI 主控台"));
    app.setOrganizationName(QStringLiteral("rak"));
    applyAppOptionsAfterApp(app, opts);

    AppController controller;
    const QString host = qEnvironmentVariable("HRI_BACKEND_HOST",
                                              QStringLiteral("localhost"));
    const quint16 port = quint16(qEnvironmentVariableIntValue("HRI_BACKEND_PORT") > 0
                                     ? quint16(qEnvironmentVariableIntValue("HRI_BACKEND_PORT"))
                                     : quint16(8500));
    controller.connectBackend(host, port);

    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty(QStringLiteral("app"), &controller);
    engine.rootContext()->setContextProperty(
        QStringLiteral("__hri_kiosk"), QVariant::fromValue<bool>(opts.kiosk));
    engine.rootContext()->setContextProperty(
        QStringLiteral("__hri_fullscreen"), QVariant::fromValue<bool>(opts.fullscreen));
    engine.rootContext()->setContextProperty(
        QStringLiteral("__hri_scale"), QVariant::fromValue<qreal>(
                          qreal(opts.scalePercent) / 100.0));

    const QUrl url(QStringLiteral("qrc:/qt/qml/com/hri/app/main.qml"));
    engine.load(url);
    if (engine.rootObjects().isEmpty()) {
        qFatal("QML 加载失败: %s", qUtf8Printable(url.toString()));
    }

    // 给根窗口套上全屏/无边框/几何
    auto *window = qobject_cast<QQuickWindow *>(engine.rootObjects().first());
    if (window) {
        const QRect geo = computeWindowGeometry(window->screen(), opts);
        window->setGeometry(geo);
        if (!opts.frame) {
            window->setFlags(window->flags() | Qt::FramelessWindowHint);
        }
        window->setTitle(QStringLiteral("HRI"));
        // 避免屏保/息屏：需要 X11/Wayland/EGLFS 各自的外部配合
        window->setScreen(qGuiApp->primaryScreen());
        if (opts.kiosk) {
            window->setVisibility(QWindow::FullScreen);
        } else if (opts.fullscreen) {
            window->setVisibility(QWindow::FullScreen);
        }
        // 保证在最前（独占桌面）
        window->raise();
        window->requestActivate();
    }

    return app.exec();
}
