#include "gnsswindow.h"
#include "ui_gnsswindow.h"
#include <QStandardItemModel>
#include <QLabel>
#include <QMessageBox>
#include <QDateTime>

GNSSWindow::GNSSWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::GNSSWindow),
    m_socket(nullptr),
    m_timer(new QTimer(this)),
    m_satModel(new QStandardItemModel(this)),
    m_antennaStatusLabel(new QLabel(this)),
    m_signalPlot(new QCustomPlot(this))
{
    ui->setupUi(this);

    // Инициализация UI компонентов
    setupSatelliteView();
    setupSignalPlot();

    // Добавление виджетов в интерфейс
    ui->antennaLayout->addWidget(m_antennaStatusLabel);
    ui->plotLayout->addWidget(m_signalPlot);

    // Настройка соединений
    setupConnections();
    registerHandlers();

    // Запрос начальных данных
    sendUbxMonVer();
    sendUbxCfgRate(1000, 1); // 1Hz navigation rate
    sendUbxCfgAnt(true); // Antenna power ON
}

// Исправленный метод initClassIdMapping
void GNSSWindow::initClassIdMapping()
{
    m_classIdMap.clear();

    // NAV класс (0x01)
    QMap<int, QString> navIds;
    navIds.insert(0x07, "PVT");
    navIds.insert(0x03, "STATUS");
    m_classIdMap.insert(0x01, navIds);

    // RXM класс (0x02)
    QMap<int, QString> rxmIds;
    rxmIds.insert(0x15, "MEASX");
    m_classIdMap.insert(0x02, rxmIds);

    // CFG класс (0x06)
    QMap<int, QString> cfgIds;
    cfgIds.insert(0x00, "PRT");
    cfgIds.insert(0x39, "ITFM");
    m_classIdMap.insert(0x06, cfgIds);

    // MON класс (0x0A)
    QMap<int, QString> monIds;
    monIds.insert(0x04, "VER");
    m_classIdMap.insert(0x0A, monIds);

    // SEC класс (0x27)
    QMap<int, QString> secIds;
    secIds.insert(0x03, "UNIQID");
    m_classIdMap.insert(0x27, secIds);

    // Заполняем comboBox классов
    ui->cbClass->clear();
    ui->cbClass->addItem("NAV (0x01)", 0x01);
    ui->cbClass->addItem("RXM (0x02)", 0x02);
    ui->cbClass->addItem("CFG (0x06)", 0x06);
    ui->cbClass->addItem("MON (0x0A)", 0x0A);
    ui->cbClass->addItem("SEC (0x27)", 0x27);
}

// Исправленный метод updateAvailableIds
void GNSSWindow::updateAvailableIds()
{
    int classId = ui->cbClass->currentData().toInt();

    ui->cbId->blockSignals(true);
    ui->cbId->clear();

    if (m_classIdMap.contains(classId)) {
        QMapIterator<int, QString> it(m_classIdMap[classId]);
        while (it.hasNext()) {
            it.next();
            ui->cbId->addItem(QString("%1 (0x%2)").arg(it.value()).arg(it.key(), 2, 16, QLatin1Char('0')), it.key());
        }
    }

    ui->cbId->blockSignals(false);

    if (ui->cbId->count() > 0) {
        ui->cbId->setCurrentIndex(0);
    }
}

GNSSWindow::~GNSSWindow()
{
    delete ui;
}

void GNSSWindow::setSocket(QTcpSocket *socket)
{
    if (m_socket) {
        m_socket->disconnect(this);
        //m_socket->deleteLater();
    }

    m_socket = socket;
    if (m_socket) {
        connect(m_socket, &QTcpSocket::readyRead, this, &GNSSWindow::onReadyRead);
        connect(m_socket, &QTcpSocket::disconnected, this, [this]() {
            QMessageBox::warning(this, "Disconnected", "Connection lost");
            this->close();
        });

        // Запрашиваем базовую информацию при подключении
        sendUbxMonVer();
        sendUbxCfgPrt();
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
    static QByteArray buffer;
    buffer += m_socket->readAll();
    qDebug() << "Raw data received, total buffer:" << buffer.toHex(' ');

    while(buffer.size() >= 8) {
        // Ищем начало сообщения
        int start = buffer.indexOf("\xB5\x62");
        if(start < 0) {
            qDebug() << "No UBX sync chars found, clearing buffer";
            buffer.clear();
            return;
        }

        if(start > 0) {
            qDebug() << "Discarding" << start << "bytes before sync chars";
            buffer.remove(0, start);
        }

        // Проверяем, достаточно ли данных для заголовка
        if(buffer.size() < 8) return;

        quint16 length = static_cast<quint8>(buffer[4]) | (static_cast<quint8>(buffer[5]) << 8);
        quint8 msgClass = static_cast<quint8>(buffer[2]);
        quint8 msgId = static_cast<quint8>(buffer[3]);

        // Проверяем, есть ли полное сообщение
        if(buffer.size() < 8 + length) {
            qDebug() << "Waiting for more data. Need:" << 8 + length
                     << "Have:" << buffer.size();
            return;
        }

        QByteArray message = buffer.left(8 + length);
        buffer.remove(0, 8 + length);

        // Парсим сообщение
        QByteArray payload;
        if(UbxParser::parseUbxMessage(message, msgClass, msgId, payload)) {
            qDebug() << "Valid UBX message:"
                     << "Class:" << QString("0x%1").arg(msgClass, 2, 16, QLatin1Char('0'))
                     << "ID:" << QString("0x%1").arg(msgId, 2, 16, QLatin1Char('0'))
                     << "Length:" << length;

            // Обработка сообщения...
            processUbxMessage(msgClass, msgId, payload);
        } else {
            qDebug() << "Invalid UBX message, skipping";
        }
    }
}

void GNSSWindow::registerHandlers()
{
    // NAV класс
    connect(&m_ubxParser, &UbxParser::navPvtReceived, this, &GNSSWindow::displayNavPvt);
    connect(&m_ubxParser, &UbxParser::navSatReceived, this, &GNSSWindow::processNavSat);
    connect(&m_ubxParser, &UbxParser::navStatusReceived, this, &GNSSWindow::displayNavStatus);

    // MON класс
    connect(&m_ubxParser, &UbxParser::monVerReceived, this, &GNSSWindow::displayMonVer);
    connect(&m_ubxParser, &UbxParser::monHwReceived, this, &GNSSWindow::processMonHw);

    // CFG класс
    connect(&m_ubxParser, &UbxParser::cfgPrtReceived, this, &GNSSWindow::displayCfgPrt);

    // Обработка ошибок
    connect(&m_ubxParser, &UbxParser::infErrorReceived, this, [this](const QString& msg) {
        appendToLog("INF-ERROR: " + msg, "error");
        QMessageBox::warning(this, "Receiver Error", msg);
    });
}

void GNSSWindow::processUbxMessage(quint8 msgClass, quint8 msgId, const QByteArray& payload)
{
    QString messageInfo;
    QString timestamp = QDateTime::currentDateTime().toString("[hh:mm:ss]");

    // Обработка ACK/NACK сообщений
    if (msgClass == 0x05) {
        UbxParser::AckPacket ack = UbxParser::parseAck(payload);
        QString ackType = (msgId == 0x01) ? "ACK" : "NACK";

        messageInfo = QString("%1 for %2 (0x%3) ID: 0x%4")
                          .arg(ackType)
                          .arg(getMessageName(ack.ackClass, ack.ackId))
                          .arg(ack.ackClass, 2, 16, QLatin1Char('0'))
                          .arg(ack.ackId, 2, 16, QLatin1Char('0'));

        appendToLog(messageInfo, "in");
        return;
    }

    // Обработка CFG-PRT ACK
    if (msgClass == 0x06 && msgId == 0x00 && payload.isEmpty()) {
        messageInfo = "CFG-PRT ACK received";
        appendToLog(messageInfo, "in");
        return;
    }

    // Обработка остальных сообщений
    switch(msgClass) {
    case 0x01: // NAV класс
        switch(msgId) {
        case 0x07: { // NAV-PVT
            UbxParser::NavPvt pvt = UbxParser::parseNavPvt(payload);
            displayNavPvt(pvt);
            messageInfo = QString("NAV-PVT: Lat=%1 Lon=%2 Fix=%3 Sats=%4")
                              .arg(pvt.lat/1e7, 0, 'f', 7)
                              .arg(pvt.lon/1e7, 0, 'f', 7)
                              .arg(pvt.fixType)
                              .arg(pvt.numSV);
            break;
        }
        case 0x03: { // NAV-STATUS
            UbxParser::NavStatus status = UbxParser::parseNavStatus(payload);
            displayNavStatus(status);
            messageInfo = QString("NAV-STATUS: Fix=%1 TTFF=%2ms")
                              .arg(status.fixType)
                              .arg(status.ttff);
            break;
        }
        default:
            messageInfo = QString("Unknown NAV message ID: 0x%1").arg(msgId, 2, 16, QLatin1Char('0'));
            break;
        }
        break;

    case 0x02: // RXM класс
        switch(msgId) {
        case 0x15: // RXM-MEASX
            messageInfo = "RXM-MEASX: Measurement data";
            break;
        default:
            messageInfo = QString("Unknown RXM message ID: 0x%1").arg(msgId, 2, 16, QLatin1Char('0'));
            break;
        }
        break;

    case 0x06: // CFG класс
        switch(msgId) {
        case 0x00: { // CFG-PRT
            UbxParser::CfgPrt prt = UbxParser::parseCfgPrt(payload);
            displayCfgPrt(prt);
            messageInfo = QString("CFG-PRT: Baud=%1 InProto=0x%2 OutProto=0x%3")
                              .arg(prt.baudRate)
                              .arg(prt.inProtoMask, 0, 16)
                              .arg(prt.outProtoMask, 0, 16);
            break;
        }
        case 0x39: // CFG-ITFM
            messageInfo = "CFG-ITFM: Jamming/interference config";
            break;
        default:
            messageInfo = QString("Unknown CFG message ID: 0x%1").arg(msgId, 2, 16, QLatin1Char('0'));
            break;
        }
        break;

    case 0x0A: // MON класс
        switch(msgId) {
        case 0x04: { // MON-VER
            UbxParser::MonVer ver = UbxParser::parseMonVer(payload);
            displayMonVer(ver);
            messageInfo = QString("MON-VER: SW=%1 HW=%2")
                              .arg(ver.swVersion)
                              .arg(ver.hwVersion);
            break;
        }
        default:
            messageInfo = QString("Unknown MON message ID: 0x%1").arg(msgId, 2, 16, QLatin1Char('0'));
            break;
        }
        break;

    case 0x27: // SEC класс
        switch(msgId) {
        case 0x03: // SEC-UNIQID
            messageInfo = "SEC-UNIQID: Unique chip ID received";
            break;
        default:
            messageInfo = QString("Unknown SEC message ID: 0x%1").arg(msgId, 2, 16, QLatin1Char('0'));
            break;
        }
        break;

    default:
        messageInfo = QString("Unknown message class: 0x%1").arg(msgClass, 2, 16, QLatin1Char('0'));
        break;
    }

    if (!messageInfo.isEmpty()) {
        appendToLog(messageInfo, "in");
    }
}

QString GNSSWindow::getMessageName(quint8 msgClass, quint8 msgId)
{
    switch(msgClass) {
    case 0x01: // NAV
        switch(msgId) {
        case 0x07: return "NAV-PVT";
        case 0x03: return "NAV-STATUS";
        default: return "NAV-UNKNOWN";
        }
    case 0x06: // CFG
        switch(msgId) {
        case 0x00: return "CFG-PRT";
        case 0x39: return "CFG-ITFM";
        default: return "CFG-UNKNOWN";
        }
    case 0x0A: return "MON";
    case 0x27: return "SEC";
    default: return QString("UNKNOWN (0x%1)").arg(msgClass, 2, 16, QLatin1Char('0'));
    }
}

void GNSSWindow::displayNavStatus(const UbxParser::NavStatus &data)
{
    qDebug() << "Updating UI with NAV-STATUS data:"
             << "Fix:" << data.fixType << "TTFF:" << data.ttff << "ms";

    ui->leFixType->setText(QString::number(data.fixType));
    ui->statusbar->showMessage(QString("Fix status: %1, TTFF: %2ms").arg(data.fixType).arg(data.ttff), 5000);
}

void GNSSWindow::displayCfgPrt(const UbxParser::CfgPrt &data)
{
    if (data.baudRate == 0) {
        qDebug() << "Received CFG-PRT ACK, requesting current config...";
        sendUbxCfgPrt(); // Запрашиваем актуальную конфигурацию
        return;
    }
    ui->statusbar->showMessage(
        QString("Port config: Baud=%1, InProto=0x%2, OutProto=0x%3")
            .arg(data.baudRate)
            .arg(data.inProtoMask, 0, 16)
            .arg(data.outProtoMask, 0, 16),
        5000);
}

void GNSSWindow::displayNavPvt(const UbxParser::NavPvt &data)
{
    qDebug() << "Updating UI with NAV-PVT data:"
             << "Lat:" << data.lat/1e7 << "Lon:" << data.lon/1e7
             << "Fix:" << data.fixType << "Sats:" << data.numSV;

    // Обновление полей позиции
    ui->leLatitude->setText(QString::number(data.lat / 1e7, 'f', 7));
    ui->leLongitude->setText(QString::number(data.lon / 1e7, 'f', 7));
    ui->leHeight->setText(QString::number(data.height / 1000.0, 'f', 2));
    ui->leSpeed->setText(QString::number(data.gSpeed / 1000.0, 'f', 2));
    ui->leFixType->setText(QString::number(data.fixType));
    ui->leSats->setText(QString::number(data.numSV));

    // Форматирование времени
    QString timeStr = QString("%1-%2-%3 %4:%5:%6 UTC")
                          .arg(data.year)
                          .arg(data.month, 2, 10, QChar('0'))
                          .arg(data.day, 2, 10, QChar('0'))
                          .arg(data.hour, 2, 10, QChar('0'))
                          .arg(data.min, 2, 10, QChar('0'))
                          .arg(data.sec, 2, 10, QChar('0'));
    ui->leTime->setText(timeStr);

    // Статусная строка
    QString status = QString("Position: Lat=%1 Lon=%2 Fix=%3D Sats=%4")
                         .arg(data.lat / 1e7, 0, 'f', 7)
                         .arg(data.lon / 1e7, 0, 'f', 7)
                         .arg(data.fixType)
                         .arg(data.numSV);
    ui->statusbar->showMessage(status, 5000);
}

UbxParser::NavSat UbxParser::parseNavSat(const QByteArray &payload) {
    NavSat result = {};
    if(payload.size() < 8) return result;

    result.iTOW = qFromLittleEndian<quint32>(payload.mid(0, 4));
    result.version = payload[4];
    result.numSvs = payload[5];

    const int satSize = 12; // Размер блока одного спутника
    for(int i = 0; i < result.numSvs && (8 + i*satSize + satSize) <= payload.size(); i++) {
        int offset = 8 + i*satSize;
        result.sats[i].gnssId = payload[offset];
        result.sats[i].svId = payload[offset+1];
        result.sats[i].cno = payload[offset+2];
        result.sats[i].elev = payload[offset+3];
        result.sats[i].azim = qFromLittleEndian<qint16>(payload.mid(offset+4, 2));
        result.sats[i].flags = qFromLittleEndian<quint32>(payload.mid(offset+8, 4));
    }
    return result;
}

UbxParser::MonHw UbxParser::parseMonHw(const QByteArray &payload) {
    MonHw result = {};
    if(payload.size() < 60) return result; // Минимальный размер структуры

    result.pinSel = qFromLittleEndian<quint32>(payload.mid(0, 4));
    result.pinBank = qFromLittleEndian<quint32>(payload.mid(4, 4));
    result.pinDir = qFromLittleEndian<quint32>(payload.mid(8, 4));
    result.pinVal = qFromLittleEndian<quint32>(payload.mid(12, 4));
    result.noisePerMS = qFromLittleEndian<quint32>(payload.mid(16, 4));
    result.agcCnt = qFromLittleEndian<quint32>(payload.mid(20, 4));
    result.aStatus = payload[24];
    result.aPower = payload[25];
    result.flags = payload[26];
    result.usedMask = qFromLittleEndian<quint32>(payload.mid(28, 4));
    result.jamInd = payload[56];
    result.pinIrq = qFromLittleEndian<quint32>(payload.mid(60, 4));
    result.pullH = qFromLittleEndian<quint32>(payload.mid(64, 4));
    result.pullL = qFromLittleEndian<quint32>(payload.mid(68, 4));

    return result;
}

void GNSSWindow::appendToLog(const QString &message, const QString &type)
{
    if(ui->actionPauseLog->isChecked())
        return;

    QString timestamp = QDateTime::currentDateTime().toString("[hh:mm:ss]");
    QString formattedMessage;

    if(type == "in") {
        formattedMessage = QString("<div style='color:#2E7D32;'><b>%1 IN:</b> %2</div>")
                               .arg(timestamp, message);
    }
    else if(type == "out") {
        // Теперь принимаем только данные без временной метки
        formattedMessage = QString("<div style='color:#1565C0;'><b>%1 OUT:</b> %2</div>")
                               .arg(timestamp, message);
    }
    else if(type == "system") {
        formattedMessage = QString("<div style='color:#7B1FA2;'><b>%1 SYS:</b> %2</div>")
                               .arg(timestamp, message);
    }
    else {
        formattedMessage = QString("<div>%1 %2</div>")
                               .arg(timestamp, message);
    }

    ui->teReceived->append(formattedMessage);

    if(!ui->actionPauseLog->isChecked()) {
        ui->teReceived->ensureCursorVisible();
    }
}

void GNSSWindow::displayMonVer(const UbxParser::MonVer &data)
{
    ui->leSwVer->setText(data.swVersion);
    ui->leHwVer->setText(data.hwVersion);
    ui->statusbar->showMessage(
        QString("Version: SW %1, HW %2").arg(data.swVersion).arg(data.hwVersion),
        5000);
}

void GNSSWindow::setupSatelliteView()
{
    // Настройка модели данных
    m_satModel->setHorizontalHeaderLabels({"System", "PRN", "CNR", "Elev.", "Azim."});

    // Настройка таблицы
    ui->tvSatellites->setModel(m_satModel);
    ui->tvSatellites->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui->tvSatellites->verticalHeader()->hide();
    ui->tvSatellites->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
}

void GNSSWindow::processNavSat(const UbxParser::NavSat &sat)
{
    m_satModel->removeRows(0, m_satModel->rowCount());

    for(int i = 0; i < sat.numSvs; i++) {
        QString system;
        switch(sat.sats[i].gnssId) {
        case 0: system = "GPS"; break;
        case 1: system = "SBAS"; break;
        case 2: system = "GAL"; break;
        case 3: system = "BEID"; break;
        case 4: system = "IMES"; break;
        case 5: system = "QZSS"; break;
        case 6: system = "GLON"; break;
        default: system = "UNK"; break;
        }

        QList<QStandardItem*> items;
        items << new QStandardItem(system);
        items << new QStandardItem(QString::number(sat.sats[i].svId));
        items << new QStandardItem(QString::number(sat.sats[i].cno));
        items << new QStandardItem(QString::number(sat.sats[i].elev));
        items << new QStandardItem(QString::number(sat.sats[i].azim));

        // Цвет в зависимости от уровня сигнала
        if(sat.sats[i].cno > 30) {
            for(auto item : items) {
                item->setBackground(QBrush(QColor(200,255,200)));
            }
        }

        m_satModel->appendRow(items);
    }

    updateSignalPlot();
}

void GNSSWindow::updateSignalPlot()
{
    m_signalPlot->clearGraphs();

    QMap<QString, QVector<double>> cnrData;
    for(int i = 0; i < m_satModel->rowCount(); i++) {
        QString system = m_satModel->item(i, 0)->text();
        double cnr = m_satModel->item(i, 2)->text().toDouble();
        cnrData[system].append(cnr);
    }

    int colorIndex = 0;
    const QList<QColor> colors = {Qt::blue, Qt::red, Qt::green, Qt::cyan, Qt::magenta};

    for(auto it = cnrData.begin(); it != cnrData.end(); ++it) {
        m_signalPlot->addGraph();
        m_signalPlot->graph()->setName(it.key());

        QVector<double> xValues;
        for(int i = 0; i < it.value().size(); i++) {
            xValues.append(i+1);
        }

        m_signalPlot->graph()->setData(xValues, it.value());
        m_signalPlot->graph()->setBrush(QBrush(colors[colorIndex++ % colors.size()]));
    }

    m_signalPlot->xAxis->setLabel("Satellite");
    m_signalPlot->yAxis->setLabel("CNR (dB-Hz)");
    m_signalPlot->rescaleAxes();
    m_signalPlot->replot();
}

void GNSSWindow::processMonHw(const UbxParser::MonHw &hw)
{
    QString status;
    if(hw.aPower == 1) {
        status = "Active (Powered)";
        m_antennaStatusLabel->setPixmap(QPixmap(":/icons/antenna_green.png"));
    } else {
        status = "Inactive";
        m_antennaStatusLabel->setPixmap(QPixmap(":/icons/antenna_red.png"));
    }

    QString info = QString("Antenna: %1\nJamming: %2%\nNoise: %3")
                       .arg(status)
                       .arg(hw.jamInd)
                       .arg(hw.noisePerMS);

    ui->lbAntennaStatus->setText(info);
    appendToLog(QString("MON-HW: %1").arg(info), "status");
}

void GNSSWindow::sendUbxCfgRate(quint16 measRate, quint16 navRate) {
    QByteArray payload(6, 0x00);
    qToLittleEndian<quint16>(measRate, payload.data());
    qToLittleEndian<quint16>(navRate, payload.data()+2);
    qToLittleEndian<quint16>(1, payload.data()+4); // GPS time reference

    createUbxPacket(0x06, 0x08, payload);
    appendToLog(QString("Set rates - Meas: %1ms, Nav: %2 cycles").arg(measRate).arg(navRate), "config");
}

void GNSSWindow::sendUbxCfgAnt(bool enablePower) {
    QByteArray payload(4, 0x00);
    quint16 flags = enablePower ? 0x0001 : 0x0000;
    qToLittleEndian<quint16>(flags, payload.data());

    createUbxPacket(0x06, 0x13, payload);
    appendToLog(QString("Antenna power %1").arg(enablePower ? "ON" : "OFF"), "config");
}

void GNSSWindow::onSendButtonClicked()
{
    // Получаем выбранные Class и ID
    quint8 msgClass = static_cast<quint8>(ui->cbClass->currentData().toInt());
    quint8 msgId = static_cast<quint8>(ui->cbId->currentData().toInt());

    // Отправка в зависимости от выбранного типа сообщения
    switch(msgClass) {
    case 0x01: // NAV
        if (msgId == 0x07) sendUbxNavPvt();
        else if (msgId == 0x03) sendUbxNavStatus();
        break;
    case 0x06: // CFG
        if (msgId == 0x00) sendUbxCfgPrt();
        else if (msgId == 0x39) sendUbxCfgItfm();
        break;
    case 0x0A: // MON
        if (msgId == 0x04) sendUbxMonVer();
        break;
    case 0x27: // SEC
        if (msgId == 0x03) sendUbxSecUniqid();
        break;
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

void GNSSWindow::on_btnClearLog_clicked()
{
    ui->teReceived->clear();
    appendToLog("Log cleared");
}

void GNSSWindow::onConnectionStatusChanged(bool connected)
{
    QString status = connected ? "Connected" : "Disconnected";
    ui->statusbar->showMessage(status, 3000);

    if (!connected) {
        m_timer->stop();
        ui->autoSendCheck->setChecked(false);
    }
}

void GNSSWindow::saveLogToFile()
{
    QString fileName = QFileDialog::getSaveFileName(this, "Save Log", "", "Text Files (*.txt);;All Files (*)");
    if (fileName.isEmpty())
        return;

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        return;

    QTextStream out(&file);
    out << ui->teReceived->toPlainText();
    file.close();
}

void GNSSWindow::clearLog()
{
    ui->teReceived->clear();
    appendToLog("Log cleared", "system");
}

void GNSSWindow::pauseLog(bool paused)
{
    if (paused) {
        ui->statusbar->showMessage("Log paused", 2000);
    } else {
        ui->teReceived->ensureCursorVisible();
        ui->statusbar->showMessage("Log resumed", 2000);
    }
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

void GNSSWindow::setupSignalPlot()
{
    m_signalPlot = new QCustomPlot(this);

    // Basic plot setup
    m_signalPlot->xAxis->setLabel("Satellite");
    m_signalPlot->yAxis->setLabel("CNR (dB-Hz)");
    m_signalPlot->legend->setVisible(true);

    // Enable antialiasing
    m_signalPlot->setAntialiasedElements(QCP::aeAll);

    // Make axis rects' left and right axes transfer ranges to each other:
    connect(m_signalPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), m_signalPlot->xAxis2, SLOT(setRange(QCPRange)));
    connect(m_signalPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), m_signalPlot->yAxis2, SLOT(setRange(QCPRange)));

    // Initial empty plot
    m_signalPlot->addGraph();
    m_signalPlot->replot();
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
    // Формируем заголовок UBX
    packet.append('\xB5');  // Sync char 1
    packet.append('\x62');  // Sync char 2
    packet.append(msgClass);
    packet.append(msgId);

    // Добавляем длину payload
    quint16 length = payload.size();
    packet.append(length & 0xFF);
    packet.append((length >> 8) & 0xFF);

    // Добавляем сам payload
    packet.append(payload);

    // Вычисляем контрольную сумму
    quint8 ck_a = 0, ck_b = 0;
    for(int i = 2; i < packet.size(); i++) {
        ck_a += static_cast<quint8>(packet[i]);
        ck_b += ck_a;
    }
    packet.append(ck_a);
    packet.append(ck_b);

    if(m_socket && m_socket->state() == QTcpSocket::ConnectedState) {
        // Отправляем пакет
        m_socket->write(packet);

        // Форматируем hex-вывод
        QString hexStr;
        const int bytesPerLine = 16;

        // Добавляем заголовок
        for(int i = 0; i < 6 && i < packet.size(); ++i) {
            hexStr += QString("%1 ").arg(static_cast<quint8>(packet[i]), 2, 16, QLatin1Char('0')).toUpper();
        }

        // Добавляем payload с переносами строк
        if(packet.size() > 6) {
            hexStr += "\n    ";
            for(int i = 6; i < packet.size(); ++i) {
                hexStr += QString("%1 ").arg(static_cast<quint8>(packet[i]), 2, 16, QLatin1Char('0')).toUpper();
                if((i - 5) % bytesPerLine == 0 && i != packet.size() - 1) {
                    hexStr += "\n    ";
                }
            }
        }

        // Удаляем последний пробел и добавляем в лог
        hexStr = hexStr.trimmed();
        appendToLog(hexStr, "out");

        qDebug() << "Sent UBX packet - Class: 0x" << QString::number(msgClass, 16).toUpper()
                 << "ID: 0x" << QString::number(msgId, 16).toUpper()
                 << "Size:" << packet.size() << "bytes";
    }
}
