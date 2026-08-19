#pragma once

#include <QByteArray>
#include <QObject>
#include <QString>
#include <QUrl>

class QTcpSocket;

// 极简 WebSocket 客户端 (仅依赖 Qt Network, 不需要 Qt WebSockets 模块)
// 用途: 接收后端 WS 推送的 JSON 事件, 自动应答 ping。
// 只实现本场景需要的子集: 收文本帧 / ping / close, 发 pong。
class WsClient : public QObject {
    Q_OBJECT
public:
    explicit WsClient(QObject *parent = nullptr);
    ~WsClient() override;

    void open(const QUrl &url);
    void close();
    bool isOpen() const;

signals:
    void connected();
    void disconnected();
    void textReceived(const QString &text);
    void errorOccurred(const QString &message);

private slots:
    void onConnected();
    void onReadyRead();
    void onDisconnected();
    void onError();

private:
    bool parseHandshake();
    void parseFrames();
    void sendFrame(quint8 opcode, const QByteArray &payload);

    QTcpSocket *m_sock;
    QByteArray m_buf;
    bool m_handshakeDone = false;
    bool m_closed = true;
};
