#include "gnsswindow.h"
#include "ui_gnsswindow.h"
#include <QtEndian>
#include <QDateTime>
#include <QMessageBox>

GNSSWindow::GNSSWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::GNSSWindow),
    m_socket(nullptr),
    m_timer(new QTimer(this))
{
    ui->setupUi(this);
    setupConnections();
}

GNSSWindow::~GNSSWindow()
{
    delete ui;
}

void GNSSWindow::setSocket(QTcpSocket *socket)
{
    if (m_socket) {
        m_socket->disconnect(this);
    }

    m_socket = socket;
    if (m_socket) {
        connect(m_socket, &QTcpSocket::readyRead, this, &GNSSWindow::onReadyRead);
        connect(m_socket, &QTcpSocket::disconnected, this, [this]() {
            QMessageBox::warning(this, "Disconnected", "Connection lost");
            this->close();
        });
    }
}

void GNSSWindow::setupConnections()
{
    connect(ui->sendButton, &QPushButton::clicked, this, &GNSSWindow::onSendButtonClicked);
    connect(ui->autoSendCheck, &QCheckBox::toggled, this, &GNSSWindow::onAutoSendToggled);
    connect(m_timer, &QTimer::timeout, this, &GNSSWindow::sendUbxNavPvt);
}

void GNSSWindow::onReadyRead()
{
    QByteArray data = m_socket->readAll();
    ui->teReceived->append("<< " + data.toHex(' '));
}

void GNSSWindow::onSendButtonClicked()
{
    quint8 msgClass = 0;
    quint8 msgId = 0;
    QByteArray payload;

    // Определяем выбранное сообщение
    QString selectedClass = ui->cbClass->currentText();
    QString selectedId = ui->cbId->currentText();

    if (selectedClass == "NAV (0x01)" && selectedId == "PVT (0x07)") {
        sendUbxNavPvt();
    }
    else if (selectedClass == "CFG (0x06)" && selectedId == "PRT (0x00)") {
        sendUbxCfgPrt();
    }
    else if (selectedClass == "MON (0x0A)" && selectedId == "VER (0x04)") {
        sendUbxMonVer();
    }
}

void GNSSWindow::onAutoSendToggled(bool checked)
{
    if (checked) {
        int rate = ui->rateSpin->value();
        m_timer->start(1000 / rate);
    } else {
        m_timer->stop();
    }
}

// CFG-PRT (Port Configuration)
void GNSSWindow::sendUbxCfgPrt()
{
    QByteArray payload(20, 0x00);

    // Port ID: 1 = UART1
    payload[0] = 0x01;

    // Reserved
    payload[1] = 0x00;

    // TxReady settings (disabled)
    qToLittleEndian<quint16>(0x0000, payload.data() + 2);

    // Mode: 8N1, no parity
    qToLittleEndian<quint32>(0x000008D0, payload.data() + 4);

    // BaudRate: 9600
    qToLittleEndian<quint32>(9600, payload.data() + 8);

    // InProtoMask: UBX + NMEA
    qToLittleEndian<quint16>(0x0003, payload.data() + 12);

    // OutProtoMask: UBX + NMEA
    qToLittleEndian<quint16>(0x0003, payload.data() + 14);

    // Flags
    qToLittleEndian<quint16>(0x0000, payload.data() + 16);

    // Reserved
    qToLittleEndian<quint16>(0x0000, payload.data() + 18);

    createUbxPacket(0x06, 0x00, payload);
}

// MON-VER (Receiver/Software Version)
void GNSSWindow::sendUbxMonVer()
{
    QByteArray payload;

    // Software version
    QString swVersion = "ROM CORE 3.01 (107888)";
    payload.append(swVersion.toUtf8());
    payload.append('\0');

    // Hardware version
    QString hwVersion = "00080000";
    payload.append(hwVersion.toUtf8());
    payload.append('\0');

    // Extension strings
    QString ext1 = "PROTVER=18.00";
    QString ext2 = "GPS;GLO;GAL;BDS";
    QString ext3 = "SBAS;IMES;QZSS";

    payload.append(ext1.toUtf8());
    payload.append('\0');
    payload.append(ext2.toUtf8());
    payload.append('\0');
    payload.append(ext3.toUtf8());
    payload.append('\0');

    createUbxPacket(0x0A, 0x04, payload);
}

// SEC-UNIQID (Unique Chip ID)
void GNSSWindow::sendUbxSecUniqid()
{
    QByteArray payload(5, 0x00);

    // Version
    payload[0] = 0x01;

    // Unique chip ID (example)
    payload[1] = 0x12;
    payload[2] = 0x34;
    payload[3] = 0x56;
    payload[4] = 0x78;

    createUbxPacket(0x27, 0x03, payload);
}

// NAV-STATUS (Receiver Navigation Status)
void GNSSWindow::sendUbxNavStatus()
{
    QByteArray payload(16, 0x00);

    // iTOW
    qToLittleEndian<quint32>(0, payload.data());

    // gpsFix: 3 = 3D fix
    payload[4] = 0x03;

    // Flags
    payload[5] = 0x01; // gpsFixOk

    // fixStat
    payload[6] = 0x00;

    // flags2
    payload[7] = 0x00;

    // ttff
    qToLittleEndian<quint32>(1000, payload.data() + 8); // 1 second

    // msss
    qToLittleEndian<quint32>(5000, payload.data() + 12); // 5 seconds

    createUbxPacket(0x01, 0x03, payload);
}

// CFG-ITFM (Jamming/Interference Monitor)
void GNSSWindow::sendUbxCfgItfm()
{
    QByteArray payload(8, 0x00);

    // config
    qToLittleEndian<quint32>(0x00000000, payload.data());

    // config2
    qToLittleEndian<quint32>(0x00000000, payload.data() + 4);

    createUbxPacket(0x06, 0x39, payload);
}

void GNSSWindow::sendUbxNavPvt()
{
    // Отправляем все служебные сообщения при первом вызове
    static bool firstRun = true;
    if (firstRun) {
        sendUbxMonVer();
        sendUbxSecUniqid();
        sendUbxCfgPrt();
        sendUbxNavStatus();
        sendUbxCfgItfm();
        firstRun = false;
    }

    // Код отправки NAV-PVT
    QByteArray payload(92, 0x00);

    // Fill PVT data
    QDateTime dt = QDateTime::currentDateTimeUtc();
    qToLittleEndian<quint32>(0, payload.data()); // iTOW
    qToLittleEndian<quint16>(dt.date().year(), payload.data() + 4);
    payload[6] = dt.date().month();
    payload[7] = dt.date().day();
    payload[8] = dt.time().hour();
    payload[9] = dt.time().minute();
    payload[10] = dt.time().second();
    payload[20] = 3; // 3D fix

    // Position data
    qToLittleEndian<qint32>(55*1e7, payload.data() + 24); // lat
    qToLittleEndian<qint32>(37*1e7, payload.data() + 28); // lon
    qToLittleEndian<qint32>(150*1000, payload.data() + 32); // height

    createUbxPacket(0x01, 0x07, payload); // NAV-PVT
}

void GNSSWindow::createUbxPacket(quint8 msgClass, quint8 msgId, const QByteArray &payload)
{
    QByteArray packet;
    packet.append('\xB5');
    packet.append('\x62');
    packet.append(msgClass);
    packet.append(msgId);

    quint16 length = payload.size();
    packet.append(length & 0xFF);
    packet.append((length >> 8) & 0xFF);
    packet.append(payload);

    // Calculate checksum
    quint8 ck_a = 0, ck_b = 0;
    for (int i = 2; i < packet.size(); i++) {
        ck_a += packet[i];
        ck_b += ck_a;
    }

    packet.append(ck_a);
    packet.append(ck_b);

    if (m_socket && m_socket->state() == QTcpSocket::ConnectedState) {
        m_socket->write(packet);
        ui->teReceived->append(">> " + packet.toHex(' '));
    }
}
