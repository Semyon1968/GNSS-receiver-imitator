#include "gnsswindow.h"
#include "./ui_gnsswindow.h"
#include <QMessageBox>
#include <QDebug>
#include <QHash>
#include <QTime>
#include <QFile>
#include <QtEndian>
#include <QRegularExpression>
#include <QElapsedTimer>
#include <QTimer>

struct UBXKey
{
    quint8 classId;
    quint8 msgId;

    bool operator==(const UBXKey &other) const
    {
        return classId == other.classId && msgId == other.msgId;
    }
};

inline uint qHash(const UBXKey &key, uint seed = 0)
{
    return qHash((quint16(key.classId) << 8) | key.msgId, seed);
}

QHash<UBXKey, QString> createUbxMap()
{
    QHash<UBXKey, QString> map;

    // UBX-ACK
    map.insert({UBX_CLASS_ACK, UBX_ACK_ACK}, "UBX-ACK-ACK");
    map.insert({UBX_CLASS_ACK, UBX_ACK_NAK}, "UBX-ACK-NAK");

    // UBX-CFG
    map.insert({UBX_CLASS_CFG, 0x13}, "UBX-CFG-ANT");
    map.insert({UBX_CLASS_CFG, 0x93}, "UBX-CFG-BATCH");
    map.insert({UBX_CLASS_CFG, 0x09}, "UBX-CFG-CFG");
    map.insert({UBX_CLASS_CFG, 0x06}, "UBX-CFG-DAT");
    map.insert({UBX_CLASS_CFG, 0x69}, "UBX-CFG-GEOFENCE");
    map.insert({UBX_CLASS_CFG, UBX_CFG_GNSS}, "UBX-CFG-GNSS");
    map.insert({UBX_CLASS_CFG, 0x02}, "UBX-CFG-INF");
    map.insert({UBX_CLASS_CFG, 0x39}, "UBX-CFG-ITFM");
    map.insert({UBX_CLASS_CFG, 0x47}, "UBX-CFG-LOGFILTER");
    map.insert({UBX_CLASS_CFG, UBX_CFG_MSG}, "UBX-CFG-MSG");
    map.insert({UBX_CLASS_CFG, 0x24}, "UBX-CFG-NAV5");
    map.insert({UBX_CLASS_CFG, 0x23}, "UBX-CFG-NAVX5");
    map.insert({UBX_CLASS_CFG, 0x17}, "UBX-CFG-NMEA");
    map.insert({UBX_CLASS_CFG, 0x1E}, "UBX-CFG-ODO");
    map.insert({UBX_CLASS_CFG, 0x3B}, "UBX-CFG-PM2");
    map.insert({UBX_CLASS_CFG, 0x86}, "UBX-CFG-PMS");
    map.insert({UBX_CLASS_CFG, UBX_CFG_PRT}, "UBX-CFG-PRT");
    map.insert({UBX_CLASS_CFG, 0x57}, "UBX-CFG-PWR");
    map.insert({UBX_CLASS_CFG, UBX_CFG_RATE}, "UBX-CFG-RATE");
    map.insert({UBX_CLASS_CFG, 0x34}, "UBX-CFG-RINV");
    map.insert({UBX_CLASS_CFG, 0x04}, "UBX-CFG-RST");
    map.insert({UBX_CLASS_CFG, 0x11}, "UBX-CFG-RXM");
    map.insert({UBX_CLASS_CFG, 0x16}, "UBX-CFG-SBAS");
    map.insert({UBX_CLASS_CFG, 0x31}, "UBX-CFG-TP5");
    map.insert({UBX_CLASS_CFG, 0x1B}, "UBX-CFG-USB");
    map.insert({UBX_CLASS_CFG, UBX_CFG_VALDEL}, "UBX-CFG-VALDEL");
    map.insert({UBX_CLASS_CFG, UBX_CFG_VALGET}, "UBX-CFG-VALGET");
    map.insert({UBX_CLASS_CFG, UBX_CFG_VALSET}, "UBX-CFG-VALSET");

    // UBX-INF
    map.insert({UBX_CLASS_INF, UBX_INF_DEBUG}, "UBX-INF-DEBUG");
    map.insert({UBX_CLASS_INF, UBX_INF_ERROR}, "UBX-INF-ERROR");
    map.insert({UBX_CLASS_INF, UBX_INF_NOTICE}, "UBX-INF-NOTICE");
    map.insert({UBX_CLASS_INF, UBX_INF_TEST}, "UBX-INF-TEST");
    map.insert({UBX_CLASS_INF, UBX_INF_WARNING}, "UBX-INF-WARNING");

    // UBX-LOG
    map.insert({UBX_CLASS_LOG, 0x11}, "UBX-LOG-BATCH");
    map.insert({UBX_CLASS_LOG, UBX_LOG_CREATE}, "UBX-LOG-CREATE");
    map.insert({UBX_CLASS_LOG, UBX_LOG_ERASE}, "UBX-LOG-ERASE");
    map.insert({UBX_CLASS_LOG, 0x0E}, "UBX-LOG-FINDTIME");
    map.insert({UBX_CLASS_LOG, UBX_LOG_INFO}, "UBX-LOG-INFO");
    map.insert({UBX_CLASS_LOG, 0x09}, "UBX-LOG-RETRIEVE");
    map.insert({UBX_CLASS_LOG, 0x10}, "UBX-LOG-RETRIEVEBATCH");
    map.insert({UBX_CLASS_LOG, 0x0B}, "UBX-LOG-RETRIEVEPOS");
    map.insert({UBX_CLASS_LOG, 0x0F}, "UBX-LOG-RETRIEVEPOSEXTRA");
    map.insert({UBX_CLASS_LOG, 0x0D}, "UBX-LOG-RETRIEVESTRING");
    map.insert({UBX_CLASS_LOG, UBX_LOG_STRING}, "UBX-LOG-STRING");

    // UBX-MGA
    map.insert({UBX_CLASS_MGA, UBX_MGA_ACK}, "UBX-MGA-ACK");
    map.insert({UBX_CLASS_MGA, 0x20}, "UBX-MGA-ANO");
    map.insert({UBX_CLASS_MGA, UBX_MGA_BDS}, "UBX-MGA-BDS");
    map.insert({UBX_CLASS_MGA, 0x80}, "UBX-MGA-DBD");
    map.insert({UBX_CLASS_MGA, 0x21}, "UBX-MGA-FLASH");
    map.insert({UBX_CLASS_MGA, UBX_MGA_GAL}, "UBX-MGA-GAL");
    map.insert({UBX_CLASS_MGA, UBX_MGA_GLO}, "UBX-MGA-GLO");
    map.insert({UBX_CLASS_MGA, UBX_MGA_GPS}, "UBX-MGA-GPS");
    map.insert({UBX_CLASS_MGA, 0x40}, "UBX-MGA-INI");
    map.insert({UBX_CLASS_MGA, UBX_MGA_QZSS}, "UBX-MGA-QZSS");

    // UBX-MON
    map.insert({UBX_CLASS_MON, 0x32}, "UBX-MON-BATCH");
    map.insert({UBX_CLASS_MON, 0x36}, "UBX-MON-COMMS");
    map.insert({UBX_CLASS_MON, 0x28}, "UBX-MON-GNSS");
    map.insert({UBX_CLASS_MON, UBX_MON_HW}, "UBX-MON-HW");
    map.insert({UBX_CLASS_MON, UBX_MON_HW2}, "UBX-MON-HW2");
    map.insert({UBX_CLASS_MON, 0x37}, "UBX-MON-HW3");
    map.insert({UBX_CLASS_MON, UBX_MON_IO}, "UBX-MON-IO");
    map.insert({UBX_CLASS_MON, UBX_MON_MSGPP}, "UBX-MON-MSGPP");
    map.insert({UBX_CLASS_MON, 0x27}, "UBX-MON-PATCH");
    map.insert({UBX_CLASS_MON, 0x38}, "UBX-MON-RF");
    map.insert({UBX_CLASS_MON, 0x07}, "UBX-MON-RXBUF");
    map.insert({UBX_CLASS_MON, 0x21}, "UBX-MON-RXR");
    map.insert({UBX_CLASS_MON, 0x31}, "UBX-MON-SPAN");
    map.insert({UBX_CLASS_MON, 0x08}, "UBX-MON-TXBUF");
    map.insert({UBX_CLASS_MON, UBX_MON_VER}, "UBX-MON-VER");

    // UBX-NAV
    map.insert({UBX_CLASS_NAV, 0x60}, "UBX-NAV-AOPSTATUS");
    map.insert({UBX_CLASS_NAV, UBX_NAV_CLOCK}, "UBX-NAV-CLOCK");
    map.insert({UBX_CLASS_NAV, 0x36}, "UBX-NAV-COV");
    map.insert({UBX_CLASS_NAV, UBX_NAV_DOP}, "UBX-NAV-DOP");
    map.insert({UBX_CLASS_NAV, 0x61}, "UBX-NAV-EOE");
    map.insert({UBX_CLASS_NAV, 0x39}, "UBX-NAV-GEOFENCE");
    map.insert({UBX_CLASS_NAV, 0x09}, "UBX-NAV-ODO");
    map.insert({UBX_CLASS_NAV, 0x34}, "UBX-NAV-ORB");
    map.insert({UBX_CLASS_NAV, UBX_NAV_POSECEF}, "UBX-NAV-POSECEF");
    map.insert({UBX_CLASS_NAV, UBX_NAV_POSLLH}, "UBX-NAV-POSLLH");
    map.insert({UBX_CLASS_NAV, UBX_NAV_PVT}, "UBX-NAV-PVT");
    map.insert({UBX_CLASS_NAV, 0x10}, "UBX-NAV-RESETODO");
    map.insert({UBX_CLASS_NAV, UBX_NAV_SAT}, "UBX-NAV-SAT");
    map.insert({UBX_CLASS_NAV, 0x32}, "UBX-NAV-SBAS");
    map.insert({UBX_CLASS_NAV, UBX_NAV_SIG}, "UBX-NAV-SIG");
    map.insert({UBX_CLASS_NAV, 0x42}, "UBX-NAV-SLAS");
    map.insert({UBX_CLASS_NAV, UBX_NAV_STATUS}, "UBX-NAV-STATUS");
    map.insert({UBX_CLASS_NAV, 0x24}, "UBX-NAV-TIMEBDS");
    map.insert({UBX_CLASS_NAV, 0x25}, "UBX-NAV-TIMEGAL");
    map.insert({UBX_CLASS_NAV, 0x23}, "UBX-NAV-TIMEGLO");
    map.insert({UBX_CLASS_NAV, UBX_NAV_TIMEGPS}, "UBX-NAV-TIMEGPS");
    map.insert({UBX_CLASS_NAV, 0x26}, "UBX-NAV-TIMELS");
    map.insert({UBX_CLASS_NAV, 0x27}, "UBX-NAV-TIMEQZSS");
    map.insert({UBX_CLASS_NAV, UBX_NAV_TIMEUTC}, "UBX-NAV-TIMEUTC");
    map.insert({UBX_CLASS_NAV, UBX_NAV_VELECEF}, "UBX-NAV-VELECEF");
    map.insert({UBX_CLASS_NAV, UBX_NAV_VELNED}, "UBX-NAV-VELNED");

    // UBX-RXM
    map.insert({UBX_CLASS_RXM, UBX_RXM_MEASX}, "UBX-RXM-MEASX");
    map.insert({UBX_CLASS_RXM, 0x41}, "UBX-RXM-PMREQ");
    map.insert({UBX_CLASS_RXM, UBX_RXM_RLM}, "UBX-RXM-RLM");
    map.insert({UBX_CLASS_RXM, 0x32}, "UBX-RXM-RTCM");
    map.insert({UBX_CLASS_RXM, UBX_RXM_SFRBX}, "UBX-RXM-SFRBX");

    // UBX-SEC
    map.insert({UBX_CLASS_SEC, 0x03}, "UBX-SEC-UNIQID");

    // UBX-TIM
    map.insert({UBX_CLASS_TIM, UBX_TIM_TM2}, "UBX-TIM-TM2");
    map.insert({UBX_CLASS_TIM, UBX_TIM_TP}, "UBX-TIM-TP");
    map.insert({UBX_CLASS_TIM, UBX_TIM_VRFY}, "UBX-TIM-VRFY");

    // UBX-UPD
    map.insert({UBX_CLASS_UPD, 0x14}, "UBX-UPD-SOS");

    return map;
}

QHash<UBXKey, QString> ubxMap = createUbxMap();

QString getUbxMessageName(quint8 classId, quint8 msgId)
{
    UBXKey key{classId, msgId};
    return ubxMap.contains(key) ? ubxMap.value(key) : "Неизвестное сообщение";
}

GNSSWindow::GNSSWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::GNSSWindow)
{
    ui->setupUi(this);

    ui->labelModel->setText(currentModel);
    ui->labelSerialNum->setText(currentSerialNumber);

    ui->BoxUTCTime_2->addItem("Manual");
    ui->BoxUTCTime_2->addItem("Auto");
    ui->BoxUTCTime_2->setCurrentText("Auto");
    ui->leUTCTime_2->setEnabled(false);
    connect(ui->BoxUTCTime_2, &QComboBox::currentTextChanged, this, &GNSSWindow::onUTCTimeModeChanged);

    socket = new QTcpSocket(this);
    setupSocket();
    connectToServer();

    connect(ui->action_3, &QAction::triggered, this, []()
    {
        QMessageBox::information(nullptr, "Support", "Developer: Semyon Tikhonov\nMail: zigmen@vk.com");
    });
    connect(ui->action_4, &QAction::triggered, this, []()
    {
        QMessageBox::information(nullptr, "Program", "Version: 1.0.0\nRelease date: 15.06.2025");
    });
    connect(ui->action_2, &QAction::triggered, qApp, &QApplication::quit);
    connect(ui->cbRate, &QCheckBox::stateChanged, this, &GNSSWindow::onRateCheckboxChanged);
    connect(ui->pushBtnTrancfer, &QPushButton::clicked, this, &GNSSWindow::onSendBtnClicked);
}

void GNSSWindow::onUTCTimeModeChanged(const QString &mode)
{
    if (mode == "Manual")
    {
        ui->leUTCTime_2->setEnabled(true);
    } else if (mode == "Auto")
    {
        ui->leUTCTime_2->setEnabled(false);
    }
}

void GNSSWindow::setModel(const QString &model)
{
    currentModel = model;
    ui->labelModel->setText(model);
}

void GNSSWindow::setSerialNumber(const QString &serial)
{
    currentSerialNumber = serial;
    ui->labelSerialNum->setText(serial);
}

void GNSSWindow::setRate(const QString &rate)
{
    currentRate = rate;
    ui->labelRate->setText(rate);
}

void GNSSWindow::onRateCheckboxChanged(int state)
{
    if (state == Qt::Checked)
    {
        onSendRateClicked();
    }
    else
    {
        if (rateTimer && rateTimer->isActive())
        {
            rateTimer->stop();
            printedCount = 0;
            qDebug() << "Остановлена отправка с заданной частотой";
        }
    }
}

void GNSSWindow::onSendRateClicked()
{
    if (ui->cbRate->isChecked())
    {
        bool ok = false;
        currentRateInt = currentRate.toInt(&ok);

        if (!ok || currentRateInt <= 0)
        {
            qDebug() << "Неверное значение rate:" << currentRate;
            return;
        }

        int intervalMs = 60000 / currentRateInt;

        if (!rateTimer)
        {
            rateTimer = new QTimer(this);
            connect(rateTimer, &QTimer::timeout, this, [this]()
                    {
                        onSendBtnClicked();
                        printedCount++;
                        if (printedCount >= currentRateInt)
                        {
                            rateTimer->stop();
                            printedCount = 0;
                        }
                    });
        }

        printedCount = 0;
        rateTimer->start(intervalMs);
    }
}

GNSSWindow::~GNSSWindow()
{
    delete ui;
}

void GNSSWindow::setupSocket()
{
    connect(socket, &QTcpSocket::connected, this, []()
    {
        qDebug() << "Соединение установлено";
    });

    connect(socket, &QTcpSocket::readyRead, this, [this]()
    {
        receiveBuffer.append(socket->readAll());

        while (receiveBuffer.size() >= 8)
        {
            int startIdx = receiveBuffer.indexOf(QByteArray::fromHex("B562"));
            if (startIdx == -1)
            {
                receiveBuffer.clear();
                break;
            }
            if (startIdx > 0)
            {
                receiveBuffer.remove(0, startIdx);
            }

            if (receiveBuffer.size() < 8) break;

            quint8 msgClass = static_cast<quint8>(receiveBuffer[2]);
            quint8 msgId = static_cast<quint8>(receiveBuffer[3]);
            quint16 length = static_cast<quint8>(receiveBuffer[4]) | (static_cast<quint8>(receiveBuffer[5]) << 8);
            int totalSize = 6 + length + 2;

            if (receiveBuffer.size() < totalSize) break;

            QByteArray fullPacket = receiveBuffer.mid(0, totalSize);
            receiveBuffer.remove(0, totalSize);

            QByteArray payload = fullPacket.mid(6, length);
            quint8 ck_a = 0, ck_b = 0;
            for (int i = 2; i < 6 + length; ++i)
            {
                ck_a += static_cast<quint8>(fullPacket[i]);
                ck_b += ck_a;
            }

            if (ck_a != static_cast<quint8>(fullPacket[6 + length]) ||
                ck_b != static_cast<quint8>(fullPacket[6 + length + 1]))
            {
                qDebug() << "Ошибка контрольной суммы";
                continue;
            }

            clearGuiFields();
            qDebug() << "Принят пакет:" << fullPacket.toHex(' ').toUpper();

            if (msgClass == UBX_CLASS_NAV && msgId == UBX_NAV_PVT && payload.size() >= 92)
            {
                quint32 iTOW = qFromLittleEndian<quint32>((const uchar*)payload.constData());
                quint16 year = qFromLittleEndian<quint16>((const uchar*)(payload.constData() + 4));
                quint8 month = static_cast<quint8>(payload[6]);
                quint8 day = static_cast<quint8>(payload[7]);
                quint8 hour = static_cast<quint8>(payload[8]);
                quint8 minute = static_cast<quint8>(payload[9]);
                quint8 second = static_cast<quint8>(payload[10]);
                quint8 fixType = static_cast<quint8>(payload[20]);
                quint8 numSV = static_cast<quint8>(payload[23]);
                qint32 lon = qFromLittleEndian<qint32>((const uchar*)(payload.constData() + 24));
                qint32 lat = qFromLittleEndian<qint32>((const uchar*)(payload.constData() + 28));
                qint32 height = qFromLittleEndian<qint32>((const uchar*)(payload.constData() + 32));
                qint32 hMSL = qFromLittleEndian<qint32>((const uchar*)(payload.constData() + 36));
                qint32 groundSpeed = qFromLittleEndian<qint32>((const uchar*)(payload.constData() + 60));
                qint32 heading = qFromLittleEndian<qint32>((const uchar*)(payload.constData() + 64));

                QString utcTime = QString("%1-%2-%3 %4:%5:%6")
                    .arg(year, 4, 10, QChar('0'))
                    .arg(month, 2, 10, QChar('0'))
                    .arg(day, 2, 10, QChar('0'))
                    .arg(hour, 2, 10, QChar('0'))
                    .arg(minute, 2, 10, QChar('0'))
                    .arg(second, 2, 10, QChar('0'));

                double lonDeg = lon / 1e7;
                double latDeg = lat / 1e7;
                double speed_mps = groundSpeed / 1000.0;
                double heading_deg = heading / 1e5;
                double heightEllipsoid = height / 1000.0;
                double heightMSL = hMSL / 1000.0;

                ui->leUTCTime->setText(utcTime);
                ui->leFixType->setText(QString::number(fixType));
                ui->leNumSV->setText(QString::number(numSV));
                ui->leLatitude->setText(QString::number(latDeg, 'f', 7));
                ui->leLongitude->setText(QString::number(lonDeg, 'f', 7));
                ui->leHeight->setText(QString::number(heightEllipsoid, 'f', 2));
                ui->leHMSL->setText(QString::number(heightMSL, 'f', 2));
                ui->leSpeed->setText(QString::number(speed_mps, 'f', 2));
                ui->leHeading->setText(QString::number(heading_deg, 'f', 5));
                ui->tePayloadReceiver->setPlainText(payload.toHex(' ').toUpper());
            }
            else if (msgClass == UBX_CLASS_NAV && msgId == UBX_NAV_TIMEUTC && payload.size() >= 20)
            {
                quint32 iTOW = qFromLittleEndian<quint32>((const uchar*)payload.constData());
                quint16 year = qFromLittleEndian<quint16>((const uchar*)(payload.constData() + 12));
                quint8 month = static_cast<quint8>(payload[14]);
                quint8 day = static_cast<quint8>(payload[15]);
                quint8 hour = static_cast<quint8>(payload[16]);
                quint8 minute = static_cast<quint8>(payload[17]);
                quint8 second = static_cast<quint8>(payload[18]);

                QString utcTime = QString("%1-%2-%3 %4:%5:%6")
                    .arg(year, 4, 10, QChar('0'))
                    .arg(month, 2, 10, QChar('0'))
                    .arg(day, 2, 10, QChar('0'))
                    .arg(hour, 2, 10, QChar('0'))
                    .arg(minute, 2, 10, QChar('0'))
                    .arg(second, 2, 10, QChar('0'));

                ui->leUTCTime->setText(utcTime);
            }
            else if (msgClass == UBX_CLASS_INF)
            {
                QString type;
                switch (msgId)
                {
                    case UBX_INF_ERROR: type = "ERROR"; break;
                    case UBX_INF_WARNING: type = "WARNING"; break;
                    case UBX_INF_NOTICE: type = "NOTICE"; break;
                    case UBX_INF_TEST: type = "TEST"; break;
                    case UBX_INF_DEBUG: type = "DEBUG"; break;
                    default: type = "UNKNOWN"; break;
                }

                QString message = QString::fromUtf8(payload).trimmed();
                ui->leInfText->setText(message);
                ui->tePayloadReceiver->append(QString("UBX-INF-%1: %2").arg(type, message));

                QString logEntry = QString("[%1] UBX-INF-%2: %3")
                    .arg(QTime::currentTime().toString("HH:mm:ss"))
                    .arg(type)
                    .arg(message);
                qDebug() << logEntry;
            }

            ui->leClassReceiver->setText(QString("%1").arg(msgClass, 2, 16, QChar('0')).toUpper());
            ui->leIDReceiver->setText(QString("%1").arg(msgId, 2, 16, QChar('0')).toUpper());
            ui->leNameReceiver->setText(getUbxMessageName(msgClass, msgId));
            ui->tePayloadReceiver->setPlainText(payload.toHex(' ').toUpper());

            QString logEntry = QString("[%1] Получено: %2 (0x%3, 0x%4) | Payload: %5 | Model: %6 | Serial: %7")
                .arg(QTime::currentTime().toString("HH:mm:ss"))
                .arg(getUbxMessageName(msgClass, msgId))
                .arg(msgClass, 2, 16, QChar('0'))
                .arg(msgId, 2, 16, QChar('0'))
                .arg(QString(payload.toHex(' ').toUpper()))
                .arg(currentModel)
                .arg(currentSerialNumber);

                ui->teLogs->append(logEntry);

                QFile file("test_log.txt");
                if (file.open(QIODevice::Append | QIODevice::Text))
                {
                    QTextStream out(&file);
                    out << logEntry << "\n";
                    file.close();
                }
            }
        });

    connect(socket, QOverload<QAbstractSocket::SocketError>::of(&QTcpSocket::errorOccurred),
        this, [this](QAbstractSocket::SocketError)
        {
            qDebug() << "Socket error:" << socket->errorString();
            QMessageBox::warning(this, "Socket Error", socket->errorString());
    });
}

void GNSSWindow::clearGuiFields()
{
    ui->leUTCTime->clear();
    ui->leFixType->clear();
    ui->leNumSV->clear();
    ui->leLatitude->clear();
    ui->leLongitude->clear();
    ui->leHeight->clear();
    ui->leHMSL->clear();
    ui->leSpeed->clear();
    ui->leHeading->clear();
    ui->tePayloadReceiver->clear();
    ui->leClassReceiver->clear();
    ui->leIDReceiver->clear();
    ui->leNameReceiver->clear();
    ui->leStatus->clear();
    ui->leInfText->clear();
}

void GNSSWindow::onSendBtnClicked()
{
    bool okClass = false, okId = false;
    quint8 msgClass = ui->leClass->text().toUInt(&okClass, 16);
    quint8 msgId = ui->leID->text().toUInt(&okId, 16);

    if (!okClass || !okId)
    {
        QMessageBox::warning(this, "Ошибка", "Неверный формат Class или ID (hex)");
        return;
    }

    QByteArray payload;

    QDateTime dt;
    if (ui->BoxUTCTime_2->currentText() == "Auto")
    {
        dt = QDateTime::currentDateTimeUtc();
    } else {
        QString utcStr = ui->leUTCTime_2->text().trimmed();
        dt = QDateTime::fromString(utcStr, "yyyy-MM-dd HH:mm:ss");
        if (!dt.isValid())
        {
            QMessageBox::warning(this, "Ошибка", "Неверный формат даты/времени");
            return;
        }
    }

    quint16 year = dt.date().year();
    quint8 month = dt.date().month();
    quint8 day = dt.date().day();
    quint8 hour = dt.time().hour();
    quint8 minute = dt.time().minute();
    quint8 second = dt.time().second();

    if (msgClass == UBX_CLASS_NAV && msgId == UBX_NAV_PVT)
    {
        payload.fill(0, 92);

        quint32 iTOW = 0;
        qToLittleEndian(iTOW, (uchar*)payload.data());

        qToLittleEndian(year, (uchar*)(payload.data() + 4));
        payload[6] = month;
        payload[7] = day;
        payload[8] = hour;
        payload[9] = minute;
        payload[10] = second;
        payload[11] = 0x07;

        QString fixStr = ui->leFixType_2->text();
        if (!fixStr.trimmed().isEmpty())
        {
            payload[20] = fixStr.toUInt();
        }

        QString svStr = ui->leNumSV_2->text();
        if (!svStr.trimmed().isEmpty())
        {
            payload[23] = svStr.toUInt();
        }

        QString lonStr = ui->leLongitude_2->text();
        if (!lonStr.trimmed().isEmpty())
        {
            qint32 lon = static_cast<qint32>(lonStr.toDouble() * 1e7);
            qToLittleEndian(lon, (uchar*)(payload.data() + 24));
        }

        QString latStr = ui->leLatitude_2->text();
        if (!latStr.trimmed().isEmpty())
        {
            qint32 lat = static_cast<qint32>(latStr.toDouble() * 1e7);
            qToLittleEndian(lat, (uchar*)(payload.data() + 28));
        }

        QString hStr = ui->leHeight_2->text();
        if (!hStr.trimmed().isEmpty())
        {
            qint32 height = static_cast<qint32>(hStr.toDouble() * 1000);
            qToLittleEndian(height, (uchar*)(payload.data() + 32));
        }

        QString hmslStr = ui->leHMSL_2->text();
        if (!hmslStr.trimmed().isEmpty())
        {
            qint32 hMSL = static_cast<qint32>(hmslStr.toDouble() * 1000);
            qToLittleEndian(hMSL, (uchar*)(payload.data() + 36));
        }

        QString speedStr = ui->leSpeed_2->text();
        if (!speedStr.trimmed().isEmpty())
        {
            qint32 groundSpeed = static_cast<qint32>(speedStr.toDouble() * 1000.0);
            qToLittleEndian(groundSpeed, (uchar*)(payload.data() + 60));
        }

        QString headingStr = ui->leHeading_2->text();
        if (!headingStr.trimmed().isEmpty())
        {
            qint32 heading = static_cast<qint32>(headingStr.toDouble() * 1e5);
            qToLittleEndian(heading, (uchar*)(payload.data() + 64));
        }
    }
    else if (msgClass == UBX_CLASS_NAV && msgId == UBX_NAV_TIMEUTC)
    {
        payload.fill(0, 20);

        quint32 iTOW = 0;
        qToLittleEndian(iTOW, (uchar*)payload.data());

        qToLittleEndian(year, (uchar*)(payload.data() + 12));
        payload[14] = month;
        payload[15] = day;
        payload[16] = hour;
        payload[17] = minute;
        payload[18] = second;
        payload[19] = 0x07;

        QString fixStr = ui->leFixType_2->text();
        if (!fixStr.trimmed().isEmpty())
        {
            payload[16] = fixStr.toUInt();
        }
    }

    QString rawPayloadHex = ui->tePayload->toPlainText().trimmed().remove(' ');
    QByteArray rawPayload = QByteArray::fromHex(rawPayloadHex.toUtf8());

    if (msgClass == UBX_CLASS_INF && msgId <= UBX_INF_DEBUG)
    {
        QString infText = ui->leInfText_2->text().trimmed();
        if (infText.isEmpty())
        {
            QMessageBox::warning(this, "Ошибка", "Введите текст для UBX-INF сообщения");
            return;
        }

        QByteArray asciiText = infText.toUtf8();
        if (asciiText.size() > 255)
        {
            QMessageBox::warning(this, "Ошибка", "Максимум 255 байт в UBX-INF payload");
            return;
        }

        payload = asciiText;
    }

    QByteArray finalPayload = rawPayload.isEmpty() ? payload : rawPayload;

    if (msgClass == UBX_CLASS_NAV && msgId == UBX_NAV_PVT)
    {
        qToLittleEndian(year, (uchar*)(finalPayload.data() + 4));
        finalPayload[6] = month;
        finalPayload[7] = day;
        finalPayload[8] = hour;
        finalPayload[9] = minute;
        finalPayload[10] = second;
        finalPayload[11] = 0x07;

        QString fixStr = ui->leFixType_2->text();
        if (!fixStr.trimmed().isEmpty())
        {
            finalPayload[20] = fixStr.toUInt();
        }

        QString svStr = ui->leNumSV_2->text();
        if (!svStr.trimmed().isEmpty())
        {
            finalPayload[23] = svStr.toUInt();
        }

        QString lonStr = ui->leLongitude_2->text();
        if (!lonStr.trimmed().isEmpty())
        {
            qint32 lon = static_cast<qint32>(lonStr.toDouble() * 1e7);
            qToLittleEndian(lon, (uchar*)(finalPayload.data() + 24));
        }

        QString latStr = ui->leLatitude_2->text();
        if (!latStr.trimmed().isEmpty())
        {
            qint32 lat = static_cast<qint32>(latStr.toDouble() * 1e7);
            qToLittleEndian(lat, (uchar*)(finalPayload.data() + 28));
        }

        QString hStr = ui->leHeight_2->text();
        if (!hStr.trimmed().isEmpty())
        {
            qint32 height = static_cast<qint32>(hStr.toDouble() * 1000);
            qToLittleEndian(height, (uchar*)(finalPayload.data() + 32));
        }

        QString hmslStr = ui->leHMSL_2->text();
        if (!hmslStr.trimmed().isEmpty())
        {
            qint32 hMSL = static_cast<qint32>(hmslStr.toDouble() * 1000);
            qToLittleEndian(hMSL, (uchar*)(finalPayload.data() + 36));
        }

        QString speedStr = ui->leSpeed_2->text();
        if (!speedStr.trimmed().isEmpty())
        {
            qint32 groundSpeed = static_cast<qint32>(speedStr.toDouble() * 1000.0);
            qToLittleEndian(groundSpeed, (uchar*)(finalPayload.data() + 60));
        }

        QString headingStr = ui->leHeading_2->text();
        if (!headingStr.trimmed().isEmpty())
        {
            qint32 heading = static_cast<qint32>(headingStr.toDouble() * 1e5);
            qToLittleEndian(heading, (uchar*)(finalPayload.data() + 64));
        }
    }
    else if (msgClass == UBX_CLASS_NAV && msgId == UBX_NAV_TIMEUTC)
    {
        qToLittleEndian(year, (uchar*)(finalPayload.data() + 12));
        finalPayload[14] = month;
        finalPayload[15] = day;
        finalPayload[16] = hour;
        finalPayload[17] = minute;
        finalPayload[18] = second;
        finalPayload[19] = 0x07;

        QString fixStr = ui->leFixType_2->text();
        if (!fixStr.trimmed().isEmpty())
        {
            finalPayload[16] = fixStr.toUInt();
        }
    }

    if (ui->cbGetStatus->isChecked())
    {
        chkGetStatus();
    }

    QByteArray packet;
    packet.append(0xB5);
    packet.append(0x62);
    packet.append(msgClass);
    packet.append(msgId);

    quint16 len = static_cast<quint16>(finalPayload.size());
    packet.append(static_cast<char>(len & 0xFF));
    packet.append(static_cast<char>((len >> 8) & 0xFF));
    packet.append(finalPayload);

    quint8 ck_a = 0, ck_b = 0;
    for (int i = 2; i < packet.size(); ++i)
    {
        ck_a += static_cast<quint8>(packet[i]);
        ck_b += ck_a;
    }

    packet.append(ck_a);
    packet.append(ck_b);

    socket->write(packet);
}

void GNSSWindow::chkGetStatus()
{
    QTcpSocket *statusSocket = new QTcpSocket(this);

    connect(statusSocket, &QTcpSocket::connected, this, [statusSocket]()
    {
        statusSocket->write("GetStatus");
    });

    connect(statusSocket, &QTcpSocket::readyRead, this, [this, statusSocket]()
    {
        QByteArray data = statusSocket->readAll();

        int startIdx = data.indexOf(QByteArray::fromHex("B562"));
        if (startIdx == -1)
        {
            ui->leStatus->setText("Ошибка: UBX пакет не найден");
            statusSocket->deleteLater();
            return;
        }

        if (data.size() < startIdx + 8) return;

        quint8 msgClass = static_cast<quint8>(data[startIdx + 2]);
        quint8 msgId = static_cast<quint8>(data[startIdx + 3]);
        quint16 length = static_cast<quint8>(data[startIdx + 4]) | (static_cast<quint8>(data[startIdx + 5]) << 8);
        int totalSize = 6 + length + 2;

        if (data.size() < startIdx + totalSize) return;

        QByteArray payload = data.mid(startIdx + 6, length);

        quint8 ck_a = 0, ck_b = 0;
        for (int i = 2; i < 6 + length; ++i)
        {
            ck_a += static_cast<quint8>(data[startIdx + i]);
            ck_b += ck_a;
        }
        if (ck_a != static_cast<quint8>(data[startIdx + 6 + length]) ||
            ck_b != static_cast<quint8>(data[startIdx + 6 + length + 1]))
        {
            ui->leStatus->setText("Ошибка контрольной суммы");
            statusSocket->deleteLater();
            return;
        }

        QString statusResponse = QString::fromUtf8(payload).trimmed();
        ui->leStatus->setText(statusResponse);
        //коментарий для реализации проверки конкретного ubx пакета
        /*
        if (msgClass == UBX_CLASS_MON && msgId == UBX_MON_HW)
        {
            QString statusResponse = QString::fromUtf8(payload).trimmed();
            ui->leStatus->setText(statusResponse);
        }
        else
        {
            ui->leStatus->setText("Неожиданный пакет");
        }
        */
        statusSocket->disconnectFromHost();
        statusSocket->deleteLater();
        });

    connect(statusSocket, &QTcpSocket::errorOccurred, this, [this, statusSocket](QAbstractSocket::SocketError socketError)
    {
        Q_UNUSED(socketError);
        ui->leStatus->setText("Ошибка соединения: " + statusSocket->errorString());
        statusSocket->deleteLater();
    });

    statusSocket->connectToHost("127.0.0.1", 5000);
}

void GNSSWindow::connectToServer()
{
    socket->connectToHost("127.0.0.1", 5000);
    //порт для автопилота
    //socket->connectToHost("192.168.2.22", 23);
}

void GNSSWindow::sendUBXPacket(quint8 msgClass, quint8 msgId, const QByteArray &payload)
{
    QByteArray packet;
    packet.append(0xB5);
    packet.append(0x62);
    packet.append(msgClass);
    packet.append(msgId);
    quint16 len = payload.size();
    packet.append(len & 0xFF);
    packet.append((len >> 8) & 0xFF);
    packet.append(payload);

    quint8 ck_a = 0, ck_b = 0;
    for (int i = 2; i < packet.size(); ++i)
    {
        ck_a += static_cast<quint8>(packet[i]);
        ck_b += ck_a;
    }
    packet.append(ck_a);
    packet.append(ck_b);

    qDebug() << "Отправляем пакет:" << packet.toHex(' ');

    if (socket && socket->state() == QAbstractSocket::ConnectedState)
    {
        socket->write(packet);
        socket->flush();
        statusBar()->showMessage("UBX пакет отправлен", 2000);
    }
    else
    {
        QMessageBox::warning(this, "Ошибка", "Нет TCP-соединения");
    }
}
