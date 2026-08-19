#include "WsClient.h"

#include <QDataStream>
#include <QHostAddress>
#include <QRandomGenerator>
#include <QTcpSocket>

WsClient::WsClient(QObject *parent)
    : QObject(parent), m_sock(new QTcpSocket(this)) {
    connect(m_sock, &QTcpSocket::connected, this, &WsClient::onConnected);
    connect(m_sock, &QTcpSocket::readyRead, this, &WsClient::onReadyRead);
    connect(m_sock, &QTcpSocket::disconnected, this, &WsClient::onDisconnected);
    connect(m_sock, &QTcpSocket::errorOccurred, this, &WsClient::onError);
}

WsClient::~WsClient() { close(); }

bool WsClient::isOpen() const { return !m_closed; }

void WsClient::open(const QUrl &url) {
    m_buf.clear();
    m_handshakeDone = false;
    m_closed = false;
    m_sock->connectToHost(url.host(), url.port(80));
}

void WsClient::close() {
    if (m_sock->state() != QAbstractSocket::UnconnectedState)
        m_sock->abort();
    m_closed = true;
    m_buf.clear();
}

void WsClient::onConnected() {
    // 生成随机的 Sec-WebSocket-Key (16 随机字节 base64)
    QByteArray key(16, 0);
    for (int i = 0; i < key.size(); ++i)
        key[i] = char(QRandomGenerator::global()->bounded(256));
    QString path = m_sock->peerName(); // unused; keep host for Host header
    Q_UNUSED(path)
    const QString host = m_sock->peerAddress().toString();
    const quint16 port = m_sock->peerPort();
    QByteArray req;
    req += "GET /ws HTTP/1.1\r\n";
    req += "Host: " + (host == "127.0.0.1" ? QStringLiteral("localhost") : host).toUtf8()
           + ":" + QByteArray::number(port) + "\r\n";
    req += "Upgrade: websocket\r\n";
    req += "Connection: Upgrade\r\n";
    req += "Sec-WebSocket-Key: " + key.toBase64() + "\r\n";
    req += "Sec-WebSocket-Version: 13\r\n\r\n";
    m_sock->write(req);
}

void WsClient::onReadyRead() {
    m_buf += m_sock->readAll();
    if (!m_handshakeDone) {
        if (!parseHandshake())
            return;
    }
    parseFrames();
}

void WsClient::onDisconnected() {
    m_closed = true;
    emit disconnected();
}

void WsClient::onError() {
    m_closed = true;
    emit errorOccurred(m_sock->errorString());
    emit disconnected();
}

// 解析握手响应: 以 "\r\n\r\n" 结尾, 首行含 101 即成功
bool WsClient::parseHandshake() {
    int idx = m_buf.indexOf("\r\n\r\n");
    if (idx < 0)
        return false; // 等待更多数据
    QByteArray header = m_buf.left(idx + 4);
    m_buf.remove(0, header.size());
    if (!header.contains(" 101 ")) {
        m_sock->abort();
        emit errorOccurred(QString::fromUtf8("WebSocket 握手失败: ")
                           + QString::fromUtf8(header.left(40)));
        return false;
    }
    m_handshakeDone = true;
    emit connected();
    return true;
}

// 增量解析 WS 帧 (服务端→客户端帧未掩码)
void WsClient::parseFrames() {
    while (true) {
        if (m_buf.size() < 2)
            return;
        quint8 b0 = quint8(m_buf[0]);
        quint8 b1 = quint8(m_buf[1]);
        quint8 opcode = b0 & 0x0F;
        bool masked = (b1 & 0x80) != 0;
        quint64 len = b1 & 0x7F;
        int header = 2;

        if (len == 126) {
            if (m_buf.size() < 4)
                return;
            len = (quint16((quint8(m_buf[2])) << 8) | quint8(m_buf[3]));
            header = 4;
        } else if (len == 127) {
            if (m_buf.size() < 10)
                return;
            len = 0;
            for (int i = 2; i < 10; ++i)
                len = (len << 8) | quint8(m_buf[i]);
            header = 10;
        }

        QByteArray mask;
        if (masked) {
            if (m_buf.size() < header + 4)
                return;
            mask = m_buf.mid(header, 4);
            header += 4;
        }
        if (m_buf.size() < header + int(len))
            return;

        QByteArray payload = m_buf.mid(header, int(len));
        if (masked)
            for (int i = 0; i < payload.size(); ++i)
                payload[i] = char(quint8(payload[i]) ^ quint8(mask[i % 4]));
        m_buf.remove(0, header + int(len));

        switch (opcode) {
        case 0x1: // text
            emit textReceived(QString::fromUtf8(payload));
            break;
        case 0x9: // ping -> pong
            sendFrame(0xA, payload);
            break;
        case 0x8: // close
            close();
            emit disconnected();
            return;
        default: // binary / 其它: 忽略
            break;
        }
    }
}

// 发送单帧 (客户端→服务端必须掩码)
void WsClient::sendFrame(quint8 opcode, const QByteArray &payload) {
    QByteArray frame;
    frame.append(char(0x80 | opcode)); // FIN
    quint64 len = quint64(payload.size());
    if (len < 126) {
        frame.append(char(0x80 | len));
    } else if (len <= 0xFFFF) {
        frame.append(char(0x80 | 126));
        frame.append(char((len >> 8) & 0xFF));
        frame.append(char(len & 0xFF));
    } else {
        frame.append(char(0x80 | 127));
        for (int i = 7; i >= 0; --i)
            frame.append(char((len >> (8 * i)) & 0xFF));
    }
    char mask[4];
    for (int i = 0; i < 4; ++i)
        mask[i] = char(QRandomGenerator::global()->bounded(256));
    frame.append(mask, 4);
    for (int i = 0; i < payload.size(); ++i)
        frame.append(char(quint8(payload[i]) ^ quint8(mask[i % 4])));
    m_sock->write(frame);
}
