#include "gnsswindow.h"
#include "./ui_gnsswindow.h"
#include <QMessageBox>
#include <QDebug>
#include <QHash>
#include <QTime>
#include <QFile>

// Структура ключа для UBX-сообщения
struct UBXKey {
    quint8 classId;
    quint8 msgId;

    bool operator==(const UBXKey &other) const {
        return classId == other.classId && msgId == other.msgId;
    }
};

// Хэш-функция для QHash
inline uint qHash(const UBXKey &key, uint seed = 0) {
    return qHash((quint16(key.classId) << 8) | key.msgId, seed);
}

// Карта известных UBX сообщений
QHash<UBXKey, QString> createUbxMap() {
    QHash<UBXKey, QString> map;

    // UBX-ACK
    map.insert({0x05, 0x01}, "UBX-ACK-ACK");
    map.insert({0x05, 0x00}, "UBX-ACK-NAK");

    // UBX-CFG
    map.insert({0x06, 0x13}, "UBX-CFG-ANT");
    map.insert({0x06, 0x93}, "UBX-CFG-BATCH");
    map.insert({0x06, 0x09}, "UBX-CFG-CFG");
    map.insert({0x06, 0x06}, "UBX-CFG-DAT");
    map.insert({0x06, 0x69}, "UBX-CFG-GEOFENCE");
    map.insert({0x06, 0x3E}, "UBX-CFG-GNSS");
    map.insert({0x06, 0x02}, "UBX-CFG-INF");
    map.insert({0x06, 0x39}, "UBX-CFG-ITFM");
    map.insert({0x06, 0x47}, "UBX-CFG-LOGFILTER");
    map.insert({0x06, 0x01}, "UBX-CFG-MSG");
    map.insert({0x06, 0x24}, "UBX-CFG-NAV5");
    map.insert({0x06, 0x23}, "UBX-CFG-NAVX5");
    map.insert({0x06, 0x17}, "UBX-CFG-NMEA");
    map.insert({0x06, 0x1E}, "UBX-CFG-ODO");
    map.insert({0x06, 0x3B}, "UBX-CFG-PM2");
    map.insert({0x06, 0x86}, "UBX-CFG-PMS");
    map.insert({0x06, 0x00}, "UBX-CFG-PRT");
    map.insert({0x06, 0x57}, "UBX-CFG-PWR");
    map.insert({0x06, 0x08}, "UBX-CFG-RATE");
    map.insert({0x06, 0x34}, "UBX-CFG-RINV");
    map.insert({0x06, 0x04}, "UBX-CFG-RST");
    map.insert({0x06, 0x11}, "UBX-CFG-RXM");
    map.insert({0x06, 0x16}, "UBX-CFG-SBAS");
    map.insert({0x06, 0x31}, "UBX-CFG-TP5");
    map.insert({0x06, 0x1B}, "UBX-CFG-USB");
    map.insert({0x06, 0x8C}, "UBX-CFG-VALDEL");
    map.insert({0x06, 0x8B}, "UBX-CFG-VALGET");
    map.insert({0x06, 0x8A}, "UBX-CFG-VALSET");

    // UBX-INF
    map.insert({0x04, 0x04}, "UBX-INF-DEBUG");
    map.insert({0x04, 0x00}, "UBX-INF-ERROR");
    map.insert({0x04, 0x02}, "UBX-INF-NOTICE");
    map.insert({0x04, 0x03}, "UBX-INF-TEST");
    map.insert({0x04, 0x01}, "UBX-INF-WARNING");

    // UBX-LOG
    map.insert({0x21, 0x11}, "UBX-LOG-BATCH");
    map.insert({0x21, 0x07}, "UBX-LOG-CREATE");
    map.insert({0x21, 0x03}, "UBX-LOG-ERASE");
    map.insert({0x21, 0x0E}, "UBX-LOG-FINDTIME");
    map.insert({0x21, 0x08}, "UBX-LOG-INFO");
    map.insert({0x21, 0x09}, "UBX-LOG-RETRIEVE");
    map.insert({0x21, 0x10}, "UBX-LOG-RETRIEVEBATCH");
    map.insert({0x21, 0x0B}, "UBX-LOG-RETRIEVEPOS");
    map.insert({0x21, 0x0F}, "UBX-LOG-RETRIEVEPOSEXTRA");
    map.insert({0x21, 0x0D}, "UBX-LOG-RETRIEVESTRING");
    map.insert({0x21, 0x04}, "UBX-LOG-STRING");

    // UBX-MGA (Assistance)
    map.insert({0x13, 0x60}, "UBX-MGA-ACK");
    map.insert({0x13, 0x20}, "UBX-MGA-ANO");
    map.insert({0x13, 0x03}, "UBX-MGA-BDS");
    map.insert({0x13, 0x80}, "UBX-MGA-DBD");
    map.insert({0x13, 0x21}, "UBX-MGA-FLASH");
    map.insert({0x13, 0x02}, "UBX-MGA-GAL");
    map.insert({0x13, 0x06}, "UBX-MGA-GLO");
    map.insert({0x13, 0x00}, "UBX-MGA-GPS");
    map.insert({0x13, 0x40}, "UBX-MGA-INI");
    map.insert({0x13, 0x05}, "UBX-MGA-QZSS");

    // UBX-MON
    map.insert({0x0A, 0x32}, "UBX-MON-BATCH");
    map.insert({0x0A, 0x36}, "UBX-MON-COMMS");
    map.insert({0x0A, 0x28}, "UBX-MON-GNSS");
    map.insert({0x0A, 0x09}, "UBX-MON-HW");
    map.insert({0x0A, 0x0B}, "UBX-MON-HW2");
    map.insert({0x0A, 0x37}, "UBX-MON-HW3");
    map.insert({0x0A, 0x02}, "UBX-MON-IO");
    map.insert({0x0A, 0x06}, "UBX-MON-MSGPP");
    map.insert({0x0A, 0x27}, "UBX-MON-PATCH");
    map.insert({0x0A, 0x38}, "UBX-MON-RF");
    map.insert({0x0A, 0x07}, "UBX-MON-RXBUF");
    map.insert({0x0A, 0x21}, "UBX-MON-RXR");
    map.insert({0x0A, 0x31}, "UBX-MON-SPAN");
    map.insert({0x0A, 0x08}, "UBX-MON-TXBUF");
    map.insert({0x0A, 0x04}, "UBX-MON-VER");

    // UBX-NAV
    map.insert({0x01, 0x60}, "UBX-NAV-AOPSTATUS");
    map.insert({0x01, 0x22}, "UBX-NAV-CLOCK");
    map.insert({0x01, 0x36}, "UBX-NAV-COV");
    map.insert({0x01, 0x04}, "UBX-NAV-DOP");
    map.insert({0x01, 0x61}, "UBX-NAV-EOE");
    map.insert({0x01, 0x39}, "UBX-NAV-GEOFENCE");
    map.insert({0x01, 0x09}, "UBX-NAV-ODO");
    map.insert({0x01, 0x34}, "UBX-NAV-ORB");
    map.insert({0x01, 0x01}, "UBX-NAV-POSECEF");
    map.insert({0x01, 0x02}, "UBX-NAV-POSLLH");
    map.insert({0x01, 0x07}, "UBX-NAV-PVT");
    map.insert({0x01, 0x10}, "UBX-NAV-RESETODO");
    map.insert({0x01, 0x35}, "UBX-NAV-SAT");
    map.insert({0x01, 0x32}, "UBX-NAV-SBAS");
    map.insert({0x01, 0x43}, "UBX-NAV-SIG");
    map.insert({0x01, 0x42}, "UBX-NAV-SLAS");
    map.insert({0x01, 0x03}, "UBX-NAV-STATUS");
    map.insert({0x01, 0x24}, "UBX-NAV-TIMEBDS");
    map.insert({0x01, 0x25}, "UBX-NAV-TIMEGAL");
    map.insert({0x01, 0x23}, "UBX-NAV-TIMEGLO");
    map.insert({0x01, 0x20}, "UBX-NAV-TIMEGPS");
    map.insert({0x01, 0x26}, "UBX-NAV-TIMELS");
    map.insert({0x01, 0x27}, "UBX-NAV-TIMEQZSS");
    map.insert({0x01, 0x21}, "UBX-NAV-TIMEUTC");
    map.insert({0x01, 0x11}, "UBX-NAV-VELECEF");
    map.insert({0x01, 0x12}, "UBX-NAV-VELNED");

    // UBX-RXM
    map.insert({0x02, 0x14}, "UBX-RXM-MEASX");
    map.insert({0x02, 0x41}, "UBX-RXM-PMREQ");
    map.insert({0x02, 0x59}, "UBX-RXM-RLM");
    map.insert({0x02, 0x32}, "UBX-RXM-RTCM");
    map.insert({0x02, 0x13}, "UBX-RXM-SFRBX");

    // UBX-SEC
    map.insert({0x27, 0x03}, "UBX-SEC-UNIQID");

    // UBX-TIM
    map.insert({0x0d, 0x03}, "UBX-TIM-TM2");
    map.insert({0x0d, 0x01}, "UBX-TIM-TP");
    map.insert({0x0d, 0x06}, "UBX-TIM-VRFY");

    // UBX-UPD
    map.insert({0x09, 0x14}, "UBX-UPD-SOS");

    return map;
}

QHash<UBXKey, QString> ubxMap = createUbxMap();

QString getUbxMessageName(quint8 classId, quint8 msgId) {
    UBXKey key{classId, msgId};
    return ubxMap.contains(key) ? ubxMap.value(key) : "Неизвестное сообщение";
}

GNSSWindow::GNSSWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::GNSSWindow)
{
    ui->setupUi(this);

    // Инициализация сокета
    socket = new QTcpSocket(this);
    setupSocket();
    connectToServer();

    connect(ui->action_3, &QAction::triggered, this, []() {
        QMessageBox::information(nullptr, "О разработчике", "Разработчик: Семён Тихонов");
    });
    connect(ui->action_4, &QAction::triggered, this, []() {
        QMessageBox::information(nullptr, "О программе", "Версия: 1.0.0\nДата релиза: 15.06.2025");
    });
    connect(ui->action_2, &QAction::triggered, qApp, &QApplication::quit);

    connect(ui->pushBtnTrancfer, &QPushButton::clicked, this, &GNSSWindow::onSendBtnClicked);
}

GNSSWindow::~GNSSWindow()
{
    delete ui;
}

void GNSSWindow::setupSocket()
{
    connect(socket, &QTcpSocket::connected, this, [](){
        qDebug() << "Соединение установлено";
    });

    connect(socket, &QTcpSocket::readyRead, this, [this]() {
        QByteArray data = socket->readAll();
        qDebug() << "Приняты данные:" << data.toHex(' ');

        if (data.size() < 8) {
            return; // слишком короткий пакет
        }

        if (static_cast<quint8>(data[0]) != 0xB5 || static_cast<quint8>(data[1]) != 0x62) {
            qDebug() << "Некорректный пакет (нет UBX sync)";
            return;
        }

        quint8 msgClass = static_cast<quint8>(data[2]);
        quint8 msgId = static_cast<quint8>(data[3]);
        quint16 length = static_cast<quint8>(data[4]) | (static_cast<quint8>(data[5]) << 8);

        if (data.size() < 8 + length) {
            qDebug() << "Пакет неполный";
            return;
        }

        QByteArray payload = data.mid(6, length);

        ui->leClassReceiver->setText(QString("%1").arg(msgClass, 2, 16, QChar('0')).toUpper());
        ui->leIDReceiver->setText(QString("%1").arg(msgId, 2, 16, QChar('0')).toUpper());
        ui->tePayloadReceiver->setPlainText(payload.toHex(' ').toUpper());

        QString messageName = getUbxMessageName(msgClass, msgId);
        ui->leNameReceiver->setText(messageName);

        // Логирование принятых сообщений
        QString logEntry = QString("[%1] Получено: %2 (%3, %4) | Payload: %5")
            .arg(QTime::currentTime().toString("HH:mm:ss"))
            .arg(messageName)
            .arg(QString("0x%1").arg(msgClass, 2, 16, QLatin1Char('0')).toUpper())
            .arg(QString("0x%1").arg(msgId, 2, 16, QLatin1Char('0')).toUpper())
            .arg(QString::fromUtf8(payload.toHex(' ').toUpper()));
        ui->teLogs->append(logEntry);

        QFile file("test_log.txt");
        if (file.open(QIODevice::Append | QIODevice::Text)) {
            QTextStream out(&file);
            out << logEntry << "\n";
            file.close();
        }

    });

    connect(socket, QOverload<QAbstractSocket::SocketError>::of(&QTcpSocket::errorOccurred),
            this, [this](QAbstractSocket::SocketError){
                qDebug() << "Socket error:" << socket->errorString();
                QMessageBox::warning(this, "Socket Error", socket->errorString());
            });
}

void GNSSWindow::connectToServer()
{
    socket->connectToHost("127.0.0.1", 5000);
}

void GNSSWindow::onSendBtnClicked()
{
    // Считываем Class и ID из lineEdit как числа в hex
    bool okClass = false, okId = false;
    quint8 msgClass = ui->leClass->text().toUInt(&okClass, 16);
    quint8 msgId = ui->leID->text().toUInt(&okId, 16);

    if (!okClass || !okId) {
        QMessageBox::warning(this, "Ошибка", "Неверный формат Class или ID (ожидается hex)");
        return;
    }

    // Считываем payload из QTextEdit, ожидая hex-строку (без пробелов или с пробелами)
    QString payloadText = ui->tePayload->toPlainText().trimmed();

    // Уберём пробелы, переведём в QByteArray
    QByteArray payload = QByteArray::fromHex(payloadText.toUtf8());

    if (payload.isEmpty() && !payloadText.isEmpty()) {
        QMessageBox::warning(this, "Ошибка", "Payload должен содержать корректные hex-данные");
        return;
    }

    sendUBXPacket(msgClass, msgId, payload);
}

void GNSSWindow::sendUBXPacket(quint8 msgClass, quint8 msgId, const QByteArray &payload)
{
    QByteArray packet;
    packet.append(0xB5); // sync char 1
    packet.append(0x62); // sync char 2
    packet.append(msgClass);
    packet.append(msgId);

    quint16 len = payload.size();
    packet.append(len & 0xFF);         // length LSB
    packet.append((len >> 8) & 0xFF);  // length MSB
    packet.append(payload);

    // Checksum calculation
    quint8 ck_a = 0, ck_b = 0;
    for (int i = 2; i < packet.size(); ++i) {
        ck_a += static_cast<quint8>(packet[i]);
        ck_b += ck_a;
    }
    packet.append(ck_a);
    packet.append(ck_b);

     qDebug() << "Отправляем пакет:" << packet.toHex(' ');

    if (socket && socket->state() == QAbstractSocket::ConnectedState) {
        socket->write(packet);
        socket->flush();
        statusBar()->showMessage("UBX пакет отправлен", 2000);
    } else {
        QMessageBox::warning(this, "Ошибка", "Нет TCP-соединения");
    }
}

