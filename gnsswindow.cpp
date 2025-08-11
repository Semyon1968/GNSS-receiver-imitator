#include "gnsswindow.h"
#include "ui_gnsswindow.h"
#include <QStandardItemModel>
#include <QLabel>
#include <QMessageBox>
#include <QDateTime>
#include "dialog.h"
#include "ubxparser.h"

GNSSWindow::GNSSWindow(Dialog* parentDialog, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::GNSSWindow),
    m_parentDialog(parentDialog),
    m_socket(nullptr),
    m_pvtTimer(new QTimer(this)),      // Timer for NAV-PVT
    m_statusTimer(new QTimer(this)),   // Timer for NAV-STATUS
    m_initTimer(new QTimer(this)),     // Initialization timer
    m_ackTimeoutTimer(new QTimer(this)), // ACK wait timer
    m_utcTimer(new QTimer(this)),
    m_initializationComplete(false),
    m_waitingForAck(false)
{
    ui->setupUi(this);

    // Setup timer intervals (default 1 Hz)
    m_pvtTimer->setInterval(1000);     // NAV-PVT
    m_statusTimer->setInterval(1000);  // NAV-STATUS
    m_initTimer->setSingleShot(true);
    m_initTimer->setInterval(15000);   // 15 sec initialization timeout
    m_ackTimeoutTimer->setSingleShot(true);
    m_ackTimeoutTimer->setInterval(3000); // 3 sec ACK timeout

    // Connect timer signals
    connect(m_pvtTimer, &QTimer::timeout, this, &GNSSWindow::sendUbxNavPvt);
    connect(m_statusTimer, &QTimer::timeout, this, &GNSSWindow::sendUbxNavStatus);

    connect(m_initTimer, &QTimer::timeout, this, [this]() {
        if (!m_initializationComplete) {
            appendToLog("Configuration timeout - proceeding without ACK", "warning");
            m_initializationComplete = true;
            // Start both timers on timeout
            int rate = ui->rateSpin->value();
            m_pvtTimer->start(1000 / rate);
            m_statusTimer->start(1000 / rate);
            ui->autoSendCheck->setChecked(true);
        }
    });

    connect(m_ackTimeoutTimer, &QTimer::timeout, this, [this]() {
        if (m_waitingForAck) {
            appendToLog("ACK timeout - configuration may be incomplete", "error");
            m_waitingForAck = false;
        }
    });

    // Menu signals
    connect(ui->actionSaveLog, &QAction::triggered, this, &GNSSWindow::onActionSaveLogTriggered);
    connect(ui->actionClearLog, &QAction::triggered, this, &GNSSWindow::onActionClearLogTriggered);
    connect(ui->actionAbout, &QAction::triggered, this, &GNSSWindow::onActionAboutTriggered);

    // Connect message selection signals
    connect(ui->cbClass, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &GNSSWindow::updateAvailableIds);
    connect(ui->cbId, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &GNSSWindow::onClassIdChanged);

    // Connect UI elements
    connect(ui->sendButton, &QPushButton::clicked,
            this, &GNSSWindow::onSendButtonClicked);
    connect(ui->autoSendCheck, &QCheckBox::toggled,
            this, &GNSSWindow::onAutoSendToggled);
    connect(ui->btnClearLog, &QPushButton::clicked,
            this, &GNSSWindow::on_btnClearLog_clicked);

    // Real time signal
    connect(m_utcTimer, &QTimer::timeout, this, &GNSSWindow::updateUTCTime);
    m_utcTimer->start(1000);
    updateUTCTime();

    // Initialize message system
    initClassIdMapping();
    updateAvailableIds();
    registerHandlers();

    // Connect to parent dialog
    if (m_parentDialog) {
        connect(m_parentDialog, &Dialog::logMessage,
                this, &GNSSWindow::appendToLog);
        connect(m_parentDialog, &Dialog::connectionStatusChanged,
                this, &GNSSWindow::onConnectionStatusChanged);
    }

    // Setup fields
    setupNavSatFields();
    setupMonRfFields();
    setupNavPvtFields();
    setupNavStatusFields();

    // Initial state
    appendToLog("GNSS Window initialized successfully", "system");
    qDebug() << "GNSSWindow initialized at" << QDateTime::currentDateTime().toString("hh:mm:ss");

    //ui->autoSendCheck->setChecked(false);
    ui->statusbar->showMessage("Waiting for connection...", 3000);
    ui->leChipId->setText("0x00000000");
}

void GNSSWindow::initClassIdMapping() {
    m_classIdMap.clear();

    // NAV class (0x01)
    QMap<int, QString> navIds;
    navIds.insert(0x01, "POSECEF");
    navIds.insert(0x02, "POSLLH");
    navIds.insert(0x03, "STATUS");
    navIds.insert(0x04, "DOP");
    navIds.insert(0x05, "ATT");
    navIds.insert(0x06, "SOL");
    navIds.insert(0x07, "PVT");
    navIds.insert(0x35, "SAT");
    navIds.insert(0x24, "DYNMODEL");
    m_classIdMap.insert(0x01, navIds);

    // RXM class (0x02)
    QMap<int, QString> rxmIds;
    rxmIds.insert(0x15, "MEASX");
    rxmIds.insert(0x32, "RAWX");
    m_classIdMap.insert(0x02, rxmIds);

    // MON class (0x0A)
    QMap<int, QString> monIds;
    monIds.insert(0x04, "VER");
    monIds.insert(0x09, "HW");
    monIds.insert(0x0B, "IO");
    monIds.insert(0x38, "RF");
    m_classIdMap.insert(0x0A, monIds);

    // CFG class (0x06)
    QMap<int, QString> cfgIds;
    cfgIds.insert(0x00, "PRT");
    cfgIds.insert(0x01, "MSG");
    cfgIds.insert(0x13, "ANT");
    cfgIds.insert(0x24, "DYNMODEL");
    cfgIds.insert(0x39, "ITFM");
    m_classIdMap.insert(0x06, cfgIds);

    // SEC class (0x27)
    QMap<int, QString> secIds;
    secIds.insert(0x03, "UNIQID");
    m_classIdMap.insert(0x27, secIds);

    // Fill class comboBox
    ui->cbClass->clear();
    ui->cbClass->addItem("NAV (0x01)", 0x01);
    ui->cbClass->addItem("RXM (0x02)", 0x02);
    ui->cbClass->addItem("CFG (0x06)", 0x06);
    ui->cbClass->addItem("MON (0x0A)", 0x0A);
    ui->cbClass->addItem("SEC (0x27)", 0x27);

    updateAvailableIds(); // Initialize ID list
}

void GNSSWindow::updateAvailableIds() {
    int classId = ui->cbClass->currentData().toInt();

    ui->cbId->blockSignals(true);
    ui->cbId->clear();

    if (m_classIdMap.contains(classId)) {
        const QMap<int, QString>& ids = m_classIdMap[classId];
        QMapIterator<int, QString> it(ids);
        while (it.hasNext()) {
            it.next();
            ui->cbId->addItem(QString("%1 (0x%2)")
                                  .arg(it.value())
                                  .arg(it.key(), 2, 16, QLatin1Char('0')),
                              it.key());
        }
    }

    ui->cbId->blockSignals(false);
    onClassIdChanged(); // Update displayed fields
}

GNSSWindow::~GNSSWindow() {
    delete ui;
}

void GNSSWindow::setSocket(QTcpSocket *socket) {
    // Disconnect old socket if exists
    if (m_socket) {
        m_socket->disconnect(this);
    }

    m_socket = socket;

    if (m_socket) {
        // Main data reading connection
        connect(m_socket, &QTcpSocket::readyRead, this, &GNSSWindow::onReadyRead);

        // Disconnection handler
        connect(m_socket, &QTcpSocket::disconnected, this, [this]() {
            appendToLog("Disconnected from host", "system");
            m_timer->stop();
            m_socket = nullptr;
        });

        // Error handler
        connect(m_socket, &QTcpSocket::errorOccurred, this, &GNSSWindow::onError);

        appendToLog("Socket connected and configured", "debug");
    } else {
        appendToLog("Socket pointer is null!", "error");
    }
}

void GNSSWindow::onError(QAbstractSocket::SocketError error) {
    Q_UNUSED(error);
    QString errorMsg = m_socket ? m_socket->errorString() : "Socket not initialized";
    appendToLog("Socket error: " + errorMsg, "error");

    // Additional error handling
    m_timer->stop();
    m_initializationComplete = false;
    m_waitingForAck = false;

    if (m_socket && m_socket->state() != QAbstractSocket::UnconnectedState) {
        m_socket->disconnectFromHost();
    }

    ui->statusbar->showMessage("Connection error: " + errorMsg, 5000);
}

void GNSSWindow::updateUTCTime()
{
    QDateTime utcTime = QDateTime::currentDateTimeUtc();
    QString timeString = utcTime.toString("yyyy-MM-dd hh:mm:ss UTC");
    ui->leUTCTimeView->setText(timeString);
}

void GNSSWindow::onActionSaveLogTriggered()
{
    QString fileName = QFileDialog::getSaveFileName(this, "Save Log", "", "Text Files (*.txt);;All Files (*)");
    if (fileName.isEmpty()) return;

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "Error", "Could not save file");
        return;
    }

    QTextStream out(&file);
    out << ui->teReceived->toPlainText();
    file.close();
}

void GNSSWindow::onActionClearLogTriggered()
{
    ui->teReceived->clear();
    appendToLog("Log cleared by user", "system");
}

void GNSSWindow::onActionAboutTriggered()
{
    QMessageBox::about(this, "About GNSS Simulator",
                       "Application simulates GNSS receiver\n"
                       "Version: 1.0\n"
                       "Uses the u-blox UBX protocol");
}

void GNSSWindow::setupNavSatFields() {
    // Set default values
    ui->sbSatVersion->setValue(1);
    ui->sbNumSatsSat->setValue(10);
    ui->cbQualityInd->setCurrentIndex(4); // Code locked
    ui->cbHealth->setCurrentIndex(1);    // Healthy
    ui->dsbPrResMin->setValue(-2.0);
    ui->dsbPrResMax->setValue(2.0);
    ui->cbSvUsed->setChecked(true);
    ui->cbDiffCorr->setChecked(false);
    ui->cbSmoothed->setChecked(false);
    ui->cbOrbitSource->setCurrentIndex(1); // Ephemeris
}

void GNSSWindow::onClassIdChanged() {
    hideAllParameterFields();

    int classId = ui->cbClass->currentData().toInt();
    int msgId = ui->cbId->currentData().toInt();

    switch(classId) {
    case 0x01: // NAV
        if (msgId == 0x07) { // PVT
            setupNavPvtFields();
            ui->gbNavPvtFields->setVisible(true);
        } else if (msgId == 0x03) { // STATUS
            setupNavStatusFields();
            ui->gbNavStatusFields->setVisible(true);
        } else if (msgId == 0x35) { // SAT
            setupNavSatFields();
            ui->gbNavSatFields->setVisible(true);
        }
        break;
    case 0x0A: // MON
        if (msgId == 0x38) { // RF
            setupMonRfFields();
            ui->gbMonRfFields->setVisible(true);
        }
        break;
    }

    if (!ui->gbNavPvtFields->isVisible() &&
        !ui->gbNavStatusFields->isVisible() &&
        !ui->gbMonRfFields->isVisible() &&
        !ui->gbNavSatFields->isVisible()) {
        ui->tePayload->setVisible(true);
    }
}

void GNSSWindow::hideAllParameterFields() {
    ui->gbNavPvtFields->setVisible(false);
    ui->gbNavStatusFields->setVisible(false);
    ui->gbMonRfFields->setVisible(false);
    ui->gbNavSatFields->setVisible(false);
    ui->tePayload->setVisible(false);
}

void GNSSWindow::setupConnections() {
    // Initialize timer (not started yet)
    m_timer = new QTimer(this);
    m_initTimer->start(5000); // 5 sec initialization timeout

    connect(m_timer, &QTimer::timeout, this, &GNSSWindow::sendUbxNavPvt);
    sendInitialConfiguration();
}

void GNSSWindow::sendInitialConfiguration() {
    if (m_waitingForAck) return;

    m_waitingForAck = true;
    m_ackTimeoutTimer->start();
    m_initTimer->start(20000);

    // Main configuration
    sendUbxCfgPrtResponse();
    sendUbxCfgMsg(0x01, 0x07, 1);  // NAV-PVT
    sendUbxCfgMsg(0x01, 0x03, 1);  // NAV-STATUS
    sendUbxCfgMsg(0x0A, 0x28, 1);  // MON-RF
    sendUbxCfgRate(1000, 1);       // 1Hz
    sendUbxCfgDynModel(4);         // Automotive
    sendUbxCfgAntSettings(true, true, true); // Antenna settings
    sendUbxCfgItfm(true);          // Enable interference detection

    // Delayed info requests
    QTimer::singleShot(500, this, &GNSSWindow::sendUbxMonVer);
    QTimer::singleShot(1000, this, &GNSSWindow::sendUbxMonHw);
    QTimer::singleShot(1500, this, &GNSSWindow::sendUbxSecUniqidReq);
}

void GNSSWindow::onReadyRead() {
    if (!m_socket) {
        qCritical() << "onReadyRead: Socket is null!";
        return;
    }

    // 1. Read all available data
    QByteArray newData = m_socket->readAll();
    if (newData.isEmpty()) {
        qWarning() << "Received empty data packet";
        return;
    }

    qDebug() << "Received" << newData.size() << "bytes of raw data:" << newData.toHex(' ');

    // 2. Add data to buffer
    static QByteArray buffer;
    buffer.append(newData);

    qDebug() << "Total buffer size:" << buffer.size() << "bytes";
    appendToLog(QString("Received %1 bytes (total buffer: %2)")
                    .arg(newData.size())
                    .arg(buffer.size()), "in");

    // 3. Process all complete messages in buffer
    while (buffer.size() >= 8) { // Minimum UBX message size (without payload)
        // 3.1. Find sync bytes
        int startPos = buffer.indexOf("\xB5\x62");
        if (startPos < 0) {
            qDebug() << "No UBX sync chars found, clearing buffer";
            buffer.clear();
            return;
        }

        // 3.2. Remove garbage before sync bytes
        if (startPos > 0) {
            qDebug() << "Discarding" << startPos << "bytes before sync chars";
            buffer.remove(0, startPos);
            continue;
        }

        // 3.3. Check if we have enough data for header
        if (buffer.size() < 8) {
            qDebug() << "Waiting for more data (header incomplete)";
            return;
        }

        // 3.4. Extract payload length
        quint16 length = static_cast<quint8>(buffer[4]) | (static_cast<quint8>(buffer[5]) << 8);
        quint8 msgClass = static_cast<quint8>(buffer[2]);
        quint8 msgId = static_cast<quint8>(buffer[3]);

        qDebug() << "Potential message - Class:" << QString("0x%1").arg(msgClass, 2, 16, QLatin1Char('0'))
                 << "ID:" << QString("0x%1").arg(msgId, 2, 16, QLatin1Char('0'))
                 << "Length:" << length;

        // 3.5. Check if we have complete message
        int totalMessageSize = 8 + length; // Header + payload
        if (buffer.size() < totalMessageSize) {
            qDebug() << "Waiting for more data. Need:" << totalMessageSize
                     << "Have:" << buffer.size();
            return;
        }

        // 3.6. Extract complete message
        QByteArray message = buffer.left(totalMessageSize);
        buffer.remove(0, totalMessageSize);

        qDebug() << "Processing message of size:" << message.size() << "bytes";
        appendToLog(QString("Processing UBX: Class=0x%1 ID=0x%2 (%3 bytes)")
                        .arg(msgClass, 2, 16, QLatin1Char('0'))
                        .arg(msgId, 2, 16, QLatin1Char('0'))
                        .arg(message.size()), "debug");

        // 3.7. Parse message
        QByteArray payload;
        if (UbxParser::parseUbxMessage(message, msgClass, msgId, payload)) {
            processUbxMessage(msgClass, msgId, payload);
        } else {
            qWarning() << "Failed to parse UBX message";
            appendToLog("Failed to parse UBX message", "error");
        }
    }
}

void GNSSWindow::registerHandlers() {
    // NAV class
    connect(&m_ubxParser, &UbxParser::navPvtReceived, this, &GNSSWindow::displayNavPvt);
    connect(&m_ubxParser, &UbxParser::navStatusReceived, this, &GNSSWindow::displayNavStatus);

    // MON class
    connect(&m_ubxParser, &UbxParser::monVerReceived, this, &GNSSWindow::displayMonVer);
    connect(&m_ubxParser, &UbxParser::monHwReceived, this, &GNSSWindow::processMonHw);
    connect(&m_ubxParser, &UbxParser::monRfReceived, this, &GNSSWindow::displayMonRf);

    // CFG class
    connect(&m_ubxParser, &UbxParser::cfgPrtReceived, this, &GNSSWindow::displayCfgPrt);

    // Error handling
    connect(&m_ubxParser, &UbxParser::infErrorReceived, this, [this](const QString& msg) {
        appendToLog("INF-ERROR: " + msg, "error");
        QMessageBox::warning(this, "Receiver Error", msg);
    });

    connect(&m_ubxParser, &UbxParser::secUniqidReceived, this, [this](const UbxParser::SecUniqid &data) {
        ui->leChipId->setText(QString("0x%1").arg(data.uniqueId, 8, 16, QLatin1Char('0')));
        appendToLog(QString("SEC-UNIQID: 0x%1").arg(data.uniqueId, 8, 16, QLatin1Char('0')), "info");
    });
}

void GNSSWindow::sendUbxCfgItfm(bool enableDetection) {
    QByteArray payload(8, 0x00);
    quint32 config = enableDetection ? 0x00000001 : 0x00000000;
    qToLittleEndian<quint32>(config, payload.data());
    qToLittleEndian<quint32>(0x00000000, payload.data()+4);

    createUbxPacket(0x06, 0x39, payload);
    appendToLog(QString("CFG-ITFM: Jamming detection %1")
                    .arg(enableDetection ? "enabled" : "disabled"), "config");
}

void GNSSWindow::setupMonRfFields()
{
    // Set default values
    ui->sbRfVersion->setValue(0);
    ui->sbRfBlocks->setValue(1);
    ui->dsbRfNoise->setValue(50.0);
    ui->dsbRfAgc->setValue(75.0);
    ui->cbRfJamState->setCurrentIndex(1); // OK
    ui->cbRfAntStatus->setCurrentIndex(2); // OK
    ui->cbRfAntPower->setCurrentIndex(1); // ON
    ui->sbRfCwSuppression->setValue(0);
}

void GNSSWindow::processCfgValGet(const QByteArray &payload) {
    if(payload.size() >= 8) {
        quint32 key = qFromLittleEndian<quint32>(payload.mid(4, 4).constData());
        QByteArray response = payload.left(4);
        response.append(0x01); // Example value

        createUbxPacket(0x06, 0x8B, response);
        appendToLog(QString("CFG-VALGET response for key 0x%1").arg(key, 8, 16, QLatin1Char('0')), "config");
    }
}

void GNSSWindow::processCfgValSet(const QByteArray &payload) {
    sendUbxAck(0x06, 0x8A);
    appendToLog("CFG-VALSET processed", "config");
}

void GNSSWindow::sendUbxMonRf()
{
    // Проверка соединения
    if (!m_socket || m_socket->state() != QAbstractSocket::ConnectedState) {
        appendToLog("Error: No active connection to send MON-RF", "error");
        return;
    }

    // Подготовка данных
    const int numBlocks = ui->sbRfBlocks->value();
    const int payloadSize = 4 + 24 * numBlocks;
    QByteArray payload(payloadSize, 0x00);

    appendToLog(QString("Preparing MON-RF message with %1 blocks (%2 bytes total)")
                    .arg(numBlocks).arg(payloadSize), "debug");

    // Заполнение заголовка
    payload[0] = static_cast<quint8>(ui->sbRfVersion->value());
    payload[1] = static_cast<quint8>(numBlocks);
    // reserved1[2] уже инициализированы нулями

    appendToLog(QString("Header: version=%1, nBlocks=%2")
                    .arg(payload[0]).arg(payload[1]), "debug");

    // Заполнение каждого RF блока
    for (int i = 0; i < numBlocks; i++) {
        int offset = 4 + i * 24;

        // Базовые параметры
        payload[offset] = static_cast<quint8>(i); // blockId
        payload[offset+1] = static_cast<quint8>(ui->cbRfJamState->currentIndex()); // flags
        payload[offset+2] = static_cast<quint8>(ui->cbRfAntStatus->currentIndex()); // antStatus
        payload[offset+3] = static_cast<quint8>(ui->cbRfAntPower->currentIndex()); // antPower

        // POST status (0 = no errors)
        qToLittleEndian<quint32>(0x00000000, payload.data() + offset + 4);

        // Noise и AGC (конвертация в scaled values)
        quint16 noisePerMS = static_cast<quint16>(ui->dsbRfNoise->value() * 100);
        quint16 agcCnt = static_cast<quint16>(ui->dsbRfAgc->value() * 100);
        qToLittleEndian<quint16>(noisePerMS, payload.data() + offset + 12);
        qToLittleEndian<quint16>(agcCnt, payload.data() + offset + 14);

        // CW suppression
        payload[offset+16] = static_cast<quint8>(ui->sbRfCwSuppression->value());

        // I/Q параметры
        payload[offset+17] = static_cast<qint8>(0);    // ofsI
        payload[offset+18] = static_cast<quint8>(128); // magI
        payload[offset+19] = static_cast<qint8>(0);    // ofsQ
        payload[offset+20] = static_cast<quint8>(128); // magQ

        // Логирование содержимого блока
        appendToLog(QString("Block %1: antId=%2, jamState=%3, antStatus=%4, antPower=%5, noise=%6, agc=%7")
                        .arg(i)
                        .arg(payload[offset])
                        .arg(payload[offset+1])
                        .arg(payload[offset+2])
                        .arg(payload[offset+3])
                        .arg(noisePerMS)
                        .arg(agcCnt), "debug");
    }

    // Логирование полного payload перед отправкой
    appendToLog("MON-RF payload: " + payload.toHex(' '), "debug");

    // Создание и отправка пакета
    createUbxPacket(0x0A, 0x38, payload);
    appendToLog(QString("MON-RF message sent (%1 bytes)").arg(payloadSize + 8), "out");
}

void GNSSWindow::sendUbxCfgDynModel(quint8 model) {
    QByteArray payload(4, 0x00);
    payload[0] = model;
    createUbxPacket(0x06, 0x24, payload);
    appendToLog(QString("CFG-DYNMODEL: Model=%1").arg(model), "config");
}

void GNSSWindow::sendUbxCfgAntSettings(bool openDet, bool shortDet, bool recover) {
    QByteArray payload(4, 0x00);
    quint16 flags = 0;

    if(openDet) flags |= 0x0001;
    if(shortDet) flags |= 0x0002;
    if(recover) flags |= 0x0004;

    qToLittleEndian<quint16>(flags, payload.data());
    createUbxPacket(0x06, 0x13, payload);

    appendToLog(QString("CFG-ANT: OpenDet=%1, ShortDet=%2, Recover=%3")
                    .arg(openDet).arg(shortDet).arg(recover), "config");
}

void GNSSWindow::sendUbxSecUniqidReq() {
    createUbxPacket(0x27, 0x03, QByteArray());
    appendToLog("Requested SEC-UNIQID", "config");
}

void GNSSWindow::sendUbxCfgMsg(quint8 msgClass, quint8 msgId, quint8 rate) {
    QByteArray payload;
    payload.append(msgClass);
    payload.append(msgId);
    payload.append(rate);

    createUbxPacket(0x06, 0x01, payload);
    qDebug() << "CFG-MSG: Class=" << QString("0x%1").arg(msgClass, 2, 16, QLatin1Char('0'))
             << "ID=" << QString("0x%1").arg(msgId, 2, 16, QLatin1Char('0'))
             << "Rate=" << rate;
}

void GNSSWindow::processUbxMessage(quint8 msgClass, quint8 msgId, const QByteArray& payload) {
    QString messageInfo;
    QString timestamp = QDateTime::currentDateTime().toString("[hh:mm:ss.zzz]");

    // Log incoming message
    qDebug().nospace() << timestamp << " Processing UBX message: "
                       << "Class=0x" << QString("%1").arg(msgClass, 2, 16, QLatin1Char('0')).toUpper()
                       << " ID=0x" << QString("%1").arg(msgId, 2, 16, QLatin1Char('0')).toUpper()
                       << " Size=" << payload.size() << " bytes";

    // Process ACK/NACK messages
    if (msgClass == 0x05) {
        processAckNack(msgId, payload);
        return;
    }

    // Process CFG messages
    if (msgClass == 0x06) {
        processCfgMessages(msgId, payload);
        return;
    }

    // Process info requests
    if (processInfoRequests(msgClass, msgId)) {
        return;
    }

    // Process main messages by class
    switch (msgClass) {
    case 0x01: messageInfo = processNavMessages(msgId, payload); break;
    case 0x02: messageInfo = processRxmMessages(msgId, payload); break;
    case 0x04: processInfMessages(msgId, payload); return;
    case 0x0A: messageInfo = processMonMessages(msgId, payload); break;
    case 0x27: messageInfo = processSecMessages(msgId, payload); break;
    default:
        messageInfo = QString("Unknown message class: 0x%1").arg(msgClass, 2, 16, QLatin1Char('0'));
        qWarning() << messageInfo;
        break;
    }

    if (!messageInfo.isEmpty()) {
        appendToLog(messageInfo, "in");
    }
}

void GNSSWindow::processAckNack(quint8 msgId, const QByteArray& payload) {
    UbxParser::AckPacket ack = m_ubxParser.parseAck(payload);
    QString ackType = (msgId == 0x01) ? "ACK" : "NACK";

    QString message = QString("%1 for %2 (0x%3) ID: 0x%4")
                          .arg(ackType)
                          .arg(getMessageName(ack.ackClass, ack.ackId))
                          .arg(ack.ackClass, 2, 16, QLatin1Char('0'))
                          .arg(ack.ackId, 2, 16, QLatin1Char('0'));

    if (msgId == 0x01 && ack.ackClass == 0x06) {
        if (ack.ackId == 0x00 || ack.ackId == 0x01 || ack.ackId == 0x08 || ack.ackId == 0x13) {
            completeInitialization();
        }
    }

    appendToLog(message, "in");
}

void GNSSWindow::completeInitialization() {
    m_initializationComplete = true;
    m_waitingForAck = false;
    m_ackTimeoutTimer->stop();
    m_initTimer->stop();

    int rate = ui->rateSpin->value();
    m_pvtTimer->start(1000 / rate);
    m_statusTimer->start(1000 / rate);

    ui->autoSendCheck->setChecked(true);

    appendToLog("Configuration complete. Starting NAV-PVT and NAV-STATUS.", "system");
}

void GNSSWindow::processCfgMessages(quint8 msgId, const QByteArray& payload) {
    QString message;

    switch (msgId) {
    case 0x00: // CFG-PRT
        sendUbxCfgPrtResponse();
        sendInitialConfiguration();
        return;
    case 0x8A: // CFG-VALGET
        processCfgValGet(payload);
        break;
    case 0x8B: // CFG-VALSET
        processCfgValSet(payload);
        break;
    case 0x39: // CFG-ITFM
        sendUbxAck(0x06, 0x39);
        sendUbxCfgItfm(true);
        break;
    default:
        message = QString("Unknown CFG message ID: 0x%1").arg(msgId, 2, 16, QLatin1Char('0'));
        qWarning() << message;
        break;
    }

    if (!message.isEmpty()) {
        appendToLog(message, "in");
    }
}

bool GNSSWindow::processInfoRequests(quint8 msgClass, quint8 msgId) {
    if (msgClass == 0x0A && msgId == 0x04) { // MON-VER
        sendUbxMonVer();
        return true;
    }
    if (msgClass == 0x0A && msgId == 0x09) { // MON-HW
        sendUbxMonHw();
        return true;
    }
    if (msgClass == 0x27 && msgId == 0x03) { // SEC-UNIQID
        sendUbxSecUniqid();
        return true;
    }
    return false;
}

QString GNSSWindow::processNavMessages(quint8 msgId, const QByteArray& payload) {
    switch (msgId) {
    case 0x07: { // NAV-PVT
        UbxParser::NavPvt pvt = UbxParser::parseNavPvt(payload);
        displayNavPvt(pvt);
        return QString("NAV-PVT: Lat=%1 Lon=%2 Fix=%3 Sats=%4")
            .arg(pvt.lat/1e7, 0, 'f', 7)
            .arg(pvt.lon/1e7, 0, 'f', 7)
            .arg(pvt.fixType)
            .arg(pvt.numSV);
    }
    case 0x03: { // NAV-STATUS
        UbxParser::NavStatus status = UbxParser::parseNavStatus(payload);
        displayNavStatus(status);
        return QString("NAV-STATUS: Fix=%1 TTFF=%2ms")
            .arg(status.fixType)
            .arg(status.ttff);
    }
    case 0x35: { // NAV-SAT
        UbxParser::NavSat sat = UbxParser::parseNavSat(payload);
        return QString("NAV-SAT: Version=%1 SVs=%2")
            .arg(sat.version)
            .arg(sat.numSvs);
    }
    default:
        return QString("Unknown NAV message ID: 0x%1").arg(msgId, 2, 16, QLatin1Char('0'));
    }
}

QString GNSSWindow::processRxmMessages(quint8 msgId, const QByteArray& payload) {
    switch (msgId) {
    case 0x15: // RXM-MEASX
        qDebug() << "RXM-MEASX measurement data received";
        return "RXM-MEASX: Measurement data received";
    case 0x32: // RXM-RAWX
        qDebug() << "RXM-RAWX raw measurement data received";
        return "RXM-RAWX: Raw measurement data received";
    default:
        return QString("Unknown RXM message ID: 0x%1").arg(msgId, 2, 16, QLatin1Char('0'));
    }
}

QString GNSSWindow::processMonMessages(quint8 msgId, const QByteArray& payload) {
    switch (msgId) {
    case 0x04: { // MON-VER
        UbxParser::MonVer ver = UbxParser::parseMonVer(payload);
        displayMonVer(ver);
        return QString("MON-VER: SW=%1 HW=%2")
            .arg(ver.swVersion)
            .arg(ver.hwVersion);
    }
    case 0x09: { // MON-HW
        UbxParser::MonHw hw = UbxParser::parseMonHw(payload);
        processMonHw(hw);
        return QString("MON-HW: Antenna=%1 Jamming=%2%")
            .arg(hw.aPower ? "ON" : "OFF")
            .arg(hw.jamInd);
    }
    case 0x0B: // MON-IO
        qDebug() << "MON-IO I/O system status received";
        return "MON-IO: I/O system status";
    case 0x28: { // MON-RF
        UbxParser::MonRf rf = UbxParser::parseMonRf(payload);
        displayMonRf(rf);
        return QString("MON-RF: %1 RF blocks").arg(rf.nBlocks);
    }
    default:
        return QString("Unknown MON message ID: 0x%1").arg(msgId, 2, 16, QLatin1Char('0'));
    }
}

QString GNSSWindow::processSecMessages(quint8 msgId, const QByteArray& payload) {
    switch (msgId) {
    case 0x03: { // SEC-UNIQID
        UbxParser::SecUniqid uniqid = UbxParser::parseSecUniqid(payload);
        emit secUniqidReceived(uniqid);
        return QString("SEC-UNIQID: 0x%1").arg(uniqid.uniqueId, 8, 16, QLatin1Char('0'));
    }
    case 0x04: // SEC-SESSION
        qDebug() << "SEC-SESSION session management message received";
        return "SEC-SESSION: Session management";
    default:
        return QString("Unknown SEC message ID: 0x%1").arg(msgId, 2, 16, QLatin1Char('0'));
    }
}

void GNSSWindow::processInfMessages(quint8 msgId, const QByteArray& payload) {
    switch (msgId) {
    case 0x00: // INF-ERROR
        emit infErrorReceived(QString::fromLatin1(payload));
        break;
    case 0x01: // INF-WARNING
        appendToLog("INF-WARNING: " + QString::fromLatin1(payload), "warning");
        break;
    case 0x02: // INF-NOTICE
        appendToLog("INF-NOTICE: " + QString::fromLatin1(payload), "info");
        break;
    default:
        appendToLog(QString("Unknown INF message ID: 0x%1").arg(msgId, 2, 16, QLatin1Char('0')), "warning");
        break;
    }
}

QString GNSSWindow::getMessageName(quint8 msgClass, quint8 msgId) {
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

void GNSSWindow::displayNavStatus(const UbxParser::NavStatus &data) {
    qDebug() << "Updating UI with NAV-STATUS data:"
             << "Fix:" << data.fixType << "TTFF:" << data.ttff << "ms";

    ui->statusbar->showMessage(QString("Fix status: %1, TTFF: %2ms").arg(data.fixType).arg(data.ttff), 5000);
}

void GNSSWindow::displayMonRf(const UbxParser::MonRf &data)
{
    QString info = QString("MON-RF: Version=%1 Blocks=%2")
                       .arg(data.version)
                       .arg(data.nBlocks);

    for(int i = 0; i < data.nBlocks && i < 4; i++) {
        const auto& block = data.blocks[i];
        QString jammingState;
        switch(block.flags & 0x03) {
        case 0: jammingState = "Unknown"; break;
        case 1: jammingState = "OK"; break;
        case 2: jammingState = "Warning"; break;
        case 3: jammingState = "Critical"; break;
        }

        QString antStatus;
        switch(block.antStatus) {
        case 0: antStatus = "INIT"; break;
        case 1: antStatus = "DONTKNOW"; break;
        case 2: antStatus = "OK"; break;
        case 3: antStatus = "SHORT"; break;
        case 4: antStatus = "OPEN"; break;
        default: antStatus = QString("Unknown (%1)").arg(block.antStatus); break;
        }

        QString antPower;
        switch(block.antPower) {
        case 0: antPower = "OFF"; break;
        case 1: antPower = "ON"; break;
        case 2: antPower = "DONTKNOW"; break;
        default: antPower = QString("Unknown (%1)").arg(block.antPower); break;
        }

        info += QString("\nBlock %1 (ID=%2):\n"
                        "  JammingState=%3\n"
                        "  AntStatus=%4\n"
                        "  AntPower=%5\n"
                        "  POSTStatus=0x%6\n"
                        "  Noise=%7 dB\n"
                        "  AGC=%8%%\n"
                        "  CWSuppression=%9\n"
                        "  I/Q: ofsI=%10 magI=%11 ofsQ=%12 magQ=%13")
                    .arg(i)
                    .arg(block.antId)
                    .arg(jammingState)
                    .arg(antStatus)
                    .arg(antPower)
                    .arg(block.postStatus, 8, 16, QLatin1Char('0'))
                    .arg(block.noisePerMS / 100.0, 0, 'f', 2)
                    .arg(block.agcCnt / 100.0, 0, 'f', 2)
                    .arg(block.cwSuppression)
                    .arg(block.ofsI)
                    .arg(block.magI)
                    .arg(block.ofsQ)
                    .arg(block.magQ);
    }

    appendToLog(info, "status");
}

void GNSSWindow::displayCfgPrt(const UbxParser::CfgPrt &data) {
    if (data.baudRate == 0) {
        qDebug() << "Received CFG-PRT ACK, requesting current config...";
        sendUbxCfgPrtResponse();
        return;
    }
    ui->statusbar->showMessage(
        QString("Port config: Baud=%1, InProto=0x%2, OutProto=0x%3")
            .arg(data.baudRate)
            .arg(data.inProtoMask, 0, 16)
            .arg(data.outProtoMask, 0, 16),
        5000);
}

void GNSSWindow::displayNavPvt(const UbxParser::NavPvt &data) {
    // Status bar
    QString status = QString("Position: Lat=%1 Lon=%2 Fix=%3D Sats=%4")
                         .arg(data.lat / 1e7, 0, 'f', 7)
                         .arg(data.lon / 1e7, 0, 'f', 7)
                         .arg(data.fixType)
                         .arg(data.numSV);
    ui->statusbar->showMessage(status, 5000);
}

void GNSSWindow::displayMonVer(const UbxParser::MonVer &data) {
    ui->statusbar->showMessage(
        QString("Version: SW %1, HW %2").arg(data.swVersion).arg(data.hwVersion),
        5000);
}

void GNSSWindow::appendToLog(const QString &message, const QString &type) {
    if (!ui || !ui->actionPauseLog || ui->actionPauseLog->isChecked()) {
        return;
    }

    QString timestamp = QDateTime::currentDateTime().toString("[hh:mm:ss]");
    QString formattedMessage;

    if (type.compare("in", Qt::CaseInsensitive) == 0) {
        formattedMessage = QString("<div style='color:#2E7D32;'><b>%1 IN:</b> %2</div>")
                               .arg(timestamp, message.toHtmlEscaped());
    }
    else if (type.compare("out", Qt::CaseInsensitive) == 0) {
        formattedMessage = QString("<div style='color:#1565C0;'><b>%1 OUT:</b> %2</div>")
                               .arg(timestamp, message.toHtmlEscaped());
    }
    else if (type.compare("system", Qt::CaseInsensitive) == 0) {
        formattedMessage = QString("<div style='color:#7B1FA2;'><b>%1 SYS:</b> %2</div>")
                               .arg(timestamp, message.toHtmlEscaped());
    }
    else if (type.compare("error", Qt::CaseInsensitive) == 0) {
        formattedMessage = QString("<div style='color:#C62828;'><b>%1 ERR:</b> %2</div>")
                               .arg(timestamp, message.toHtmlEscaped());
    }
    else {
        formattedMessage = QString("<div style='color:#000000;'>%1 %2</div>")
                               .arg(timestamp, message.toHtmlEscaped());
    }

    if (ui && ui->teReceived) {
        if (ui->teReceived->document()->lineCount() > 10000) {
            ui->teReceived->clear();
            ui->teReceived->append("<div style='color:#7B1FA2;'><b>Log cleared automatically</b></div>");
        }

        ui->teReceived->append(formattedMessage);

        if (!ui->actionPauseLog->isChecked()) {
            QTextCursor cursor = ui->teReceived->textCursor();
            cursor.movePosition(QTextCursor::End);
            ui->teReceived->setTextCursor(cursor);
        }
    }

    qDebug().noquote() << QString("%1 %2: %3")
                              .arg(timestamp)
                              .arg(type.toUpper())
                              .arg(message);
}

void GNSSWindow::processMonHw(const UbxParser::MonHw &hw) {
    QString status = hw.aPower == 1 ? "Active (Powered)" : "Inactive";
    QString info = QString("Antenna: %1\nJamming: %2%\nNoise: %3\nAGC: %4")
                       .arg(status)
                       .arg(hw.jamInd)
                       .arg(hw.noisePerMS)
                       .arg(hw.agcCnt);

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

void GNSSWindow::sendUbxNavSat() {
    QByteArray payload;
    QRandomGenerator *generator = QRandomGenerator::global();

    // Header (8 bytes)
    payload.append("\x00\x00\x00\x00", 4); // iTOW (fill later)
    payload.append(static_cast<char>(ui->sbSatVersion->value())); // version
    payload.append(static_cast<char>(ui->sbNumSatsSat->value())); // numSvs
    payload.append("\x00\x00", 2); // reserved

    // Get UI settings
    quint8 qualityInd = static_cast<quint8>(ui->cbQualityInd->currentIndex());
    quint8 health = static_cast<quint8>(ui->cbHealth->currentIndex());
    bool svUsed = ui->cbSvUsed->isChecked();
    bool diffCorr = ui->cbDiffCorr->isChecked();
    bool smoothed = ui->cbSmoothed->isChecked();
    quint8 orbitSource = static_cast<quint8>(ui->cbOrbitSource->currentIndex());
    double prResMin = ui->dsbPrResMin->value();
    double prResMax = ui->dsbPrResMax->value();

    // Add satellites (12 bytes each)
    for(int i = 0; i < ui->sbNumSatsSat->value(); i++) {
        // Basic satellite info
        payload.append(static_cast<char>(1)); // gnssId (GPS)
        payload.append(static_cast<char>(i+1)); // svId
        payload.append(static_cast<char>(35 + generator->bounded(20))); // cno (35-55 dBHz)
        payload.append(static_cast<char>(30 + generator->bounded(50))); // elev (30-80 deg)

        // Azimuth
        qint16 azim = generator->bounded(360);
        payload.append(static_cast<char>(azim & 0xFF));
        payload.append(static_cast<char>((azim >> 8) & 0xFF));

        // Pseudorange residual (prRes) in 0.1m units
        double prResMeters = prResMin + generator->bounded(prResMax - prResMin);
        qint16 prRes = static_cast<qint16>(prResMeters * 10); // convert to 0.1m units
        payload.append(static_cast<char>(prRes & 0xFF));
        payload.append(static_cast<char>((prRes >> 8) & 0xFF));

        // Flags (4 bytes)
        quint32 flags = 0;

        // Set flags from UI
        flags |= (qualityInd & 0x07) << 0;  // qualityInd (bits 0-2)
        flags |= (svUsed ? 1 : 0) << 3;     // svUsed (bit 3)
        flags |= (health & 0x03) << 4;      // health (bits 4-5)
        flags |= (diffCorr ? 1 : 0) << 6;   // diffCorr (bit 6)
        flags |= (smoothed ? 1 : 0) << 7;   // smoothed (bit 7)
        flags |= (orbitSource & 0x07) << 8; // orbitSource (bits 8-10)

        // Set derived flags
        if (orbitSource == 1) flags |= 1 << 11; // ephAvail if ephemeris
        flags |= 1 << 12; // almAvail (always available)

        // Add flags to payload
        payload.append(static_cast<char>((flags >> 0) & 0xFF));
        payload.append(static_cast<char>((flags >> 8) & 0xFF));
        payload.append(static_cast<char>((flags >> 16) & 0xFF));
        payload.append(static_cast<char>((flags >> 24) & 0xFF));
    }

    // Update iTOW (GPS week time in ms)
    QDateTime currentTime = QDateTime::currentDateTimeUtc();
    quint32 iTOW = static_cast<quint32>(currentTime.toMSecsSinceEpoch() % (7 * 24 * 60 * 60 * 1000));
    qToLittleEndian<quint32>(iTOW, payload.data());

    createUbxPacket(0x01, 0x35, payload);
    appendToLog("Sent NAV-SAT message", "out");
}

void GNSSWindow::onSendButtonClicked() {
    quint8 msgClass = static_cast<quint8>(ui->cbClass->currentData().toInt());
    quint8 msgId = static_cast<quint8>(ui->cbId->currentData().toInt());

    switch(msgClass) {
    case 0x01: // NAV
        if (msgId == 0x07) sendUbxNavPvt();
        else if (msgId == 0x03) sendUbxNavStatus();
        else if (msgId == 0x35) sendUbxNavSat();
        break;
    case 0x06: // CFG
        if (msgId == 0x00) sendUbxCfgPrtResponse();
        else if (msgId == 0x39) sendUbxCfgItfm();
        break;
    case 0x0A: // MON
        if (msgId == 0x04) sendUbxMonVer();
        else if (msgId == 0x38) sendUbxMonRf();
        break;
    case 0x27: // SEC
        if (msgId == 0x03) sendUbxSecUniqid();
        break;
    }
}

void GNSSWindow::onAutoSendToggled(bool checked) {
    if (checked && !m_initializationComplete) {
        QMessageBox::warning(this, "Warning",
                             "Cannot start sending - initialization not complete");
        ui->autoSendCheck->setChecked(false);
        return;
    }

    if (checked) {
        int rate = ui->rateSpin->value();
        m_pvtTimer->start(1000 / rate);
        m_statusTimer->start(1000 / rate);
    } else {
        m_pvtTimer->stop();
        m_statusTimer->stop();
    }
}

void GNSSWindow::sendUbxCfgPrtResponse() {
    QByteArray payload(20, 0x00);

    // UART1 configuration
    payload[0] = 0x01; // Port ID: UART1
    qToLittleEndian<quint32>(0x000008D0, payload.data() + 4); // Mode: 8N1, no parity
    qToLittleEndian<quint32>(115200, payload.data() + 8); // BaudRate: 115200
    qToLittleEndian<quint16>(0x0003, payload.data() + 12); // inProtoMask: UBX + NMEA
    qToLittleEndian<quint16>(0x0003, payload.data() + 14); // outProtoMask: UBX + NMEA

    createUbxPacket(0x06, 0x00, payload);
    appendToLog("Sent UART1 configuration", "config");
}

void GNSSWindow::sendUbxMonVer() {
    QByteArray payload;
    payload.append("ROM CORE 3.01 (107888)");
    payload.append('\0');
    payload.append("00080000");
    payload.append('\0');
    payload.append("PROTVER=18.00");
    payload.append('\0');
    payload.append("GPS;GLO;GAL;BDS");
    payload.append('\0');
    payload.append("SBAS;IMES;QZSS");
    payload.append('\0');

    createUbxPacket(0x0A, 0x04, payload);
    appendToLog("Sent version information", "config");
}

void GNSSWindow::sendUbxSecUniqid() {
    QByteArray payload(5, 0x00);
    payload[0] = 0x01; // Version
    payload[1] = 0x12; // Unique chip ID
    payload[2] = 0x34;
    payload[3] = 0x56;
    payload[4] = 0x78;

    createUbxPacket(0x27, 0x03, payload);
    appendToLog("Sent SEC-UNIQID with valid chip ID", "config");
    ui->leChipId->setText("0x12345678");
}

void GNSSWindow::sendUbxNavStatus() {
    QByteArray payload(16, 0x00);
    QDateTime currentTime = QDateTime::currentDateTime();
    quint32 iTOW = static_cast<quint32>(currentTime.toMSecsSinceEpoch() % (7 * 24 * 60 * 60 * 1000));

    quint8 fixType = static_cast<quint8>(ui->sbFixTypeStatus->value());
    quint32 ttff = static_cast<quint32>(ui->sbTtff->value());

    qToLittleEndian<quint32>(iTOW, payload.data()); // iTOW
    payload[4] = fixType; // fixType
    qToLittleEndian<quint32>(ttff, payload.data() + 8); // ttff

    createUbxPacket(0x01, 0x03, payload);
    appendToLog("Sent NAV-STATUS", "out");
}

void GNSSWindow::on_btnClearLog_clicked() {
    ui->teReceived->clear();
    appendToLog("Log cleared");
}

void GNSSWindow::onConnectionStatusChanged(bool connected) {
    QString status = connected ? "Connected" : "Disconnected";
    ui->statusbar->showMessage(status, 3000);

    if (!connected) {
        m_timer->stop();
        ui->autoSendCheck->setChecked(false);
    }
}

void GNSSWindow::saveLogToFile() {
    QString fileName = QFileDialog::getSaveFileName(this, "Save Log", "", "Text Files (*.txt);;All Files (*)");
    if (fileName.isEmpty()) return;

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;

    QTextStream out(&file);
    out << ui->teReceived->toPlainText();
    file.close();
}

void GNSSWindow::clearLog() {
    ui->teReceived->clear();
    appendToLog("Log cleared", "system");
}

void GNSSWindow::pauseLog(bool paused) {
    if (paused) {
        ui->statusbar->showMessage("Log paused", 2000);
    } else {
        ui->teReceived->ensureCursorVisible();
        ui->statusbar->showMessage("Log resumed", 2000);
    }
}

void GNSSWindow::sendUbxCfgItfm() {
    QByteArray payload(8, 0x00);
    qToLittleEndian<quint32>(0x00000000, payload.data());
    qToLittleEndian<quint32>(0x00000000, payload.data() + 4);

    createUbxPacket(0x06, 0x39, payload);
    appendToLog("Sent CFG-ITFM with default config (jamming detection disabled)", "config");
}

void GNSSWindow::sendUbxAck(quint8 msgClass, quint8 msgId) {
    QByteArray payload;
    payload.append(static_cast<char>(msgClass));
    payload.append(static_cast<char>(msgId));

    createUbxPacket(0x05, 0x01, payload);
    appendToLog(QString("Sent ACK for Class=0x%1 ID=0x%2")
                    .arg(msgClass, 2, 16, QLatin1Char('0'))
                    .arg(msgId, 2, 16, QLatin1Char('0')),
                "config");
}

void GNSSWindow::setupNavPvtFields() {
    ui->gbNavPvtFields->setVisible(true);
    ui->gbNavStatusFields->setVisible(false);
    ui->tePayload->setVisible(false);

    // Set default values
    ui->dsbLat->setValue(55.7522200);
    ui->dsbLon->setValue(37.6155600);
    ui->dsbHeight->setValue(150.0);
    ui->dsbSpeed->setValue(0.0);
    ui->dsbHeading->setValue(0.0);
    ui->sbNumSats->setValue(10);
    ui->dsbVelN->setValue(0.0);
    ui->dsbVelE->setValue(0.0);
    ui->dsbVelU->setValue(0.0);
    ui->dsbRmsPos->setValue(1.0);
    ui->dsbRmsVel->setValue(0.1);
    ui->dsbPdop->setValue(1.5);
}

void GNSSWindow::setupNavStatusFields() {
    ui->gbNavPvtFields->setVisible(false);
    ui->gbNavStatusFields->setVisible(true);
    ui->tePayload->setVisible(false);

    // Set default values
    ui->sbFixTypeStatus->setValue(3); // 3D fix
    ui->sbTtff->setValue(5000); // 5 seconds
}

void GNSSWindow::sendUbxNack(quint8 msgClass, quint8 msgId) {
    QByteArray payload;
    payload.append(static_cast<char>(msgClass));
    payload.append(static_cast<char>(msgId));

    createUbxPacket(0x05, 0x00, payload);
    appendToLog(QString("Sent NACK for Class=0x%1 ID=0x%2")
                    .arg(msgClass, 2, 16, QLatin1Char('0'))
                    .arg(msgId, 2, 16, QLatin1Char('0')),
                "error");
}

void GNSSWindow::sendUbxCfgPrt() {
    QByteArray payload(20, 0x00);

    // Port ID: 1 = UART1
    payload[0] = 0x01;
    payload[1] = 0x00; // Reserved

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
    appendToLog("Sent CFG-PRT configuration", "config");
}

void GNSSWindow::sendUbxMonHw() {
    QByteArray payload(60, 0x00);
    qToLittleEndian<quint32>(0x00000000, payload.data());      // pinSel
    qToLittleEndian<quint32>(0x00000000, payload.data() + 4);  // pinBank
    qToLittleEndian<quint32>(0x00000000, payload.data() + 8);  // pinDir
    qToLittleEndian<quint32>(0x00000000, payload.data() + 12); // pinVal
    qToLittleEndian<quint16>(50, payload.data() + 16);  // noisePerMS (50)
    qToLittleEndian<quint16>(120, payload.data() + 18); // agcCnt (120)
    payload[20] = 0x01; // aStatus (1 = OK)
    payload[21] = 0x01; // aPower (1 = powered)
    payload[22] = 0x01; // flags (1 = antenna supervised)
    payload[45] = 0x00; // jamInd (0%)
    qToLittleEndian<quint32>(0x00000000, payload.data() + 48); // pinIrq
    qToLittleEndian<quint32>(0x00000000, payload.data() + 52); // pullH
    qToLittleEndian<quint32>(0x00000000, payload.data() + 56); // pullL

    createUbxPacket(0x0A, 0x09, payload);
    appendToLog("Sent MON-HW with valid status (0x01)", "config");
}

void GNSSWindow::sendUbxNavPvt() {
    QByteArray payload(92, 0x00);
    QDateTime currentTime = QDateTime::currentDateTime();
    quint32 iTOW = static_cast<quint32>(currentTime.toMSecsSinceEpoch() % (7 * 24 * 60 * 60 * 1000));

    // Get values from UI
    qint32 lat = static_cast<qint32>(ui->dsbLat->value() * 1e7);
    qint32 lon = static_cast<qint32>(ui->dsbLon->value() * 1e7);
    qint32 height = static_cast<qint32>(ui->dsbHeight->value() * 1000);
    quint32 gSpeed = static_cast<quint32>(ui->dsbSpeed->value() * 1000);
    quint32 headMot = static_cast<quint32>(ui->dsbHeading->value() * 1e5);
    quint8 fixType = static_cast<quint8>(ui->sbNumSats->value() > 0 ? 3 : 0);
    quint8 numSV = static_cast<quint8>(ui->sbNumSats->value());
    qint32 velN = static_cast<qint32>(ui->dsbVelN->value() * 1000);
    qint32 velE = static_cast<qint32>(ui->dsbVelE->value() * 1000);
    qint32 velD = static_cast<qint32>(ui->dsbVelU->value() * 1000);
    quint32 hAcc = static_cast<quint32>(ui->dsbRmsPos->value() * 1000);
    quint32 vAcc = static_cast<quint32>(ui->dsbRmsPos->value() * 1000);
    quint32 sAcc = static_cast<quint32>(ui->dsbRmsVel->value() * 1000);
    quint16 pDOP = static_cast<quint16>(ui->dsbPdop->value() * 100);

    // Fill payload
    qToLittleEndian<quint32>(iTOW, payload.data()); // iTOW
    qToLittleEndian<quint16>(currentTime.date().year(), payload.data() + 4);
    payload[6] = static_cast<quint8>(currentTime.date().month());
    payload[7] = static_cast<quint8>(currentTime.date().day());
    payload[8] = static_cast<quint8>(currentTime.time().hour());
    payload[9] = static_cast<quint8>(currentTime.time().minute());
    payload[10] = static_cast<quint8>(currentTime.time().second());
    payload[11] = 0x07; // valid: date, time, fully resolved

    qToLittleEndian<qint32>(lat, payload.data() + 24); // lon
    qToLittleEndian<qint32>(lon, payload.data() + 28); // lat
    qToLittleEndian<qint32>(height, payload.data() + 32); // height
    qToLittleEndian<qint32>(velN, payload.data() + 48); // velN
    qToLittleEndian<qint32>(velE, payload.data() + 52); // velE
    qToLittleEndian<qint32>(velD, payload.data() + 56); // velD
    qToLittleEndian<quint32>(gSpeed, payload.data() + 60); // gSpeed
    qToLittleEndian<quint32>(headMot, payload.data() + 64); // headMot
    qToLittleEndian<quint32>(hAcc, payload.data() + 40); // hAcc
    qToLittleEndian<quint32>(vAcc, payload.data() + 44); // vAcc
    qToLittleEndian<quint32>(sAcc, payload.data() + 72); // sAcc
    qToLittleEndian<quint16>(pDOP, payload.data() + 80); // pDOP
    payload[20] = fixType; // fixType
    payload[23] = numSV;   // numSV

    createUbxPacket(0x01, 0x07, payload);
    appendToLog("Sent NAV-PVT", "out");
}

void GNSSWindow::createUbxPacket(quint8 msgClass, quint8 msgId, const QByteArray &payload)
{
    if (!m_socket) {
        appendToLog("Error: Socket not initialized", "error");
        return;
    }

    if (m_socket->state() != QAbstractSocket::ConnectedState) {
        appendToLog(QString("Error: Socket not connected (state: %1)").arg(m_socket->state()), "error");
        return;
    }

    QByteArray packet;
    packet.append('\xB5'); // Sync char 1
    packet.append('\x62'); // Sync char 2
    packet.append(static_cast<char>(msgClass));
    packet.append(static_cast<char>(msgId));

    quint16 length = payload.size();
    packet.append(static_cast<char>(length & 0xFF));
    packet.append(static_cast<char>((length >> 8) & 0xFF));
    packet.append(payload);

    // Calculate checksum
    quint8 ck_a = 0, ck_b = 0;
    for (int i = 2; i < packet.size(); i++) {
        quint8 byte = static_cast<quint8>(packet[i]);
        ck_a += byte;
        ck_b += ck_a;
    }
    packet.append(ck_a);
    packet.append(ck_b);

    // Логирование деталей пакета
    appendToLog(QString("UBX Packet: Class=0x%1, ID=0x%2, Length=%3, Checksum=0x%4 0x%5")
                    .arg(msgClass, 2, 16, QLatin1Char('0'))
                    .arg(msgId, 2, 16, QLatin1Char('0'))
                    .arg(length)
                    .arg(ck_a, 2, 16, QLatin1Char('0'))
                    .arg(ck_b, 2, 16, QLatin1Char('0')), "debug");

    // Отправка пакета
    qint64 bytesWritten = m_socket->write(packet);
    if (bytesWritten == -1) {
        appendToLog(QString("Write error: %1").arg(m_socket->errorString()), "error");
    } else if (bytesWritten != packet.size()) {
        appendToLog(QString("Partial write: %1/%2 bytes").arg(bytesWritten).arg(packet.size()), "warning");
    } else {
        appendToLog(QString("Successfully sent %1 bytes").arg(bytesWritten), "debug");
    }

    if (!m_socket->flush()) {
        appendToLog(QString("Flush failed: %1").arg(m_socket->errorString()), "warning");
    }
}
