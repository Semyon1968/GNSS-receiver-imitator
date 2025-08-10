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
    m_pvtTimer(new QTimer(this)),      // Таймер для NAV-PVT
    m_statusTimer(new QTimer(this)),   // Таймер для NAV-STATUS
    m_initTimer(new QTimer(this)),     // Таймер инициализации
    m_ackTimeoutTimer(new QTimer(this)), // Таймер ожидания ACK
    m_initializationComplete(false),
    m_waitingForAck(false)
{
    ui->setupUi(this);

    // Настройка интервалов таймеров (по умолчанию 1 Гц)
    m_pvtTimer->setInterval(1000);     // NAV-PVT
    m_statusTimer->setInterval(1000);  // NAV-STATUS
    m_initTimer->setSingleShot(true);
    m_initTimer->setInterval(15000);   // 15 секунд на инициализацию
    m_ackTimeoutTimer->setSingleShot(true);
    m_ackTimeoutTimer->setInterval(3000); // 3 секунды на ожидание ACK

    // Подключение сигналов таймеров
    connect(m_pvtTimer, &QTimer::timeout, this, &GNSSWindow::sendUbxNavPvt);
    connect(m_statusTimer, &QTimer::timeout, this, &GNSSWindow::sendUbxNavStatus);

    connect(m_initTimer, &QTimer::timeout, this, [this]() {
        if (!m_initializationComplete) {
            appendToLog("Configuration timeout - proceeding without ACK", "warning");
            m_initializationComplete = true;
            // Запускаем оба таймера при таймауте
            int rate = ui->rateSpin->value();
            m_pvtTimer->start(1000 / rate);
            m_statusTimer->start(1000 / rate);
        }
    });

    connect(m_ackTimeoutTimer, &QTimer::timeout, this, [this]() {
        if (m_waitingForAck) {
            appendToLog("ACK timeout - configuration may be incomplete", "error");
            m_waitingForAck = false;
        }
    });

    // Подключение сигналов выбора сообщений
    connect(ui->cbClass, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &GNSSWindow::updateAvailableIds);
    connect(ui->cbId, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &GNSSWindow::onClassIdChanged);

    // Подключение UI элементов
    connect(ui->sendButton, &QPushButton::clicked,
            this, &GNSSWindow::onSendButtonClicked);
    connect(ui->autoSendCheck, &QCheckBox::toggled,
            this, &GNSSWindow::onAutoSendToggled);
    connect(ui->btnClearLog, &QPushButton::clicked,
            this, &GNSSWindow::on_btnClearLog_clicked);

    // Инициализация системы сообщений
    initClassIdMapping();
    updateAvailableIds();
    registerHandlers();

    // Подключение к родительскому диалогу
    if (m_parentDialog) {
        connect(m_parentDialog, &Dialog::logMessage,
                this, &GNSSWindow::appendToLog);
        connect(m_parentDialog, &Dialog::connectionStatusChanged,
                this, &GNSSWindow::onConnectionStatusChanged);
    }

    // Настройка полей
    setupNavSatFields();
    setupMonRfFields();
    setupNavPvtFields();
    setupNavStatusFields();

    // Начальное состояние
    appendToLog("GNSS Window initialized successfully", "system");
    qDebug() << "GNSSWindow initialized at" << QDateTime::currentDateTime().toString("hh:mm:ss");

    ui->autoSendCheck->setChecked(false);
    ui->statusbar->showMessage("Waiting for connection...", 3000);
    ui->leChipId->setText("0x00000000");
}

void GNSSWindow::initClassIdMapping()
{
    m_classIdMap.clear();

    // NAV класс (0x01)
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

    // RXM класс (0x02)
    QMap<int, QString> rxmIds;
    rxmIds.insert(0x15, "MEASX");
    rxmIds.insert(0x32, "RAWX");
    m_classIdMap.insert(0x02, rxmIds);

    // MON класс (0x0A)
    QMap<int, QString> monIds;
    monIds.insert(0x04, "VER");
    monIds.insert(0x09, "HW");
    monIds.insert(0x0B, "IO");
    monIds.insert(0x28, "RF");
    m_classIdMap.insert(0x0A, monIds);

    // CFG класс (0x06)
    QMap<int, QString> cfgIds;
    cfgIds.insert(0x00, "PRT");
    cfgIds.insert(0x01, "MSG");
    cfgIds.insert(0x13, "ANT");
    cfgIds.insert(0x24, "DYNMODEL");
    cfgIds.insert(0x39, "ITFM");
    m_classIdMap.insert(0x06, cfgIds);

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

    updateAvailableIds(); // Инициализируем список ID
}

void GNSSWindow::updateAvailableIds()
{
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
    onClassIdChanged(); // Обновляем отображаемые поля
}

GNSSWindow::~GNSSWindow()
{
    delete ui;
}

void GNSSWindow::setSocket(QTcpSocket *socket)
{
    // Отключаем старые соединения, если сокет уже был установлен
    if (m_socket) {
        m_socket->disconnect(this); // Отключаем все сигналы от старого сокета
    }

    m_socket = socket;

    if (m_socket) {
        // Основное соединение для чтения данных
        connect(m_socket, &QTcpSocket::readyRead, this, &GNSSWindow::onReadyRead);

        // Соединение для обработки отключения
        connect(m_socket, &QTcpSocket::disconnected, this, [this]() {
            appendToLog("Disconnected from host", "system");
            m_timer->stop();
            m_socket = nullptr;
        });

        // Соединение для обработки ошибок (добавляем здесь!)
        connect(m_socket, &QTcpSocket::errorOccurred, this, &GNSSWindow::onError);

        appendToLog("Socket connected and configured", "debug");
    } else {
        appendToLog("Socket pointer is null!", "error");
    }
}

void GNSSWindow::onError(QAbstractSocket::SocketError error)
{
    Q_UNUSED(error);
    QString errorMsg = m_socket ? m_socket->errorString() : "Socket not initialized";
    appendToLog("Socket error: " + errorMsg, "error");

    // Дополнительные действия при ошибке
    m_timer->stop();
    m_initializationComplete = false;
    m_waitingForAck = false;

    if (m_socket && m_socket->state() != QAbstractSocket::UnconnectedState) {
        m_socket->disconnectFromHost();
    }

    ui->statusbar->showMessage("Connection error: " + errorMsg, 5000);
}

void GNSSWindow::setupNavSatFields()
{
    // Установка значений по умолчанию
    ui->sbSatVersion->setValue(1);
    ui->sbNumSatsSat->setValue(10);
}

void GNSSWindow::onClassIdChanged()
{
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
        if (msgId == 0x28) { // RF
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

void GNSSWindow::hideAllParameterFields()
{
    ui->gbNavPvtFields->setVisible(false);
    ui->gbNavStatusFields->setVisible(false);
    ui->gbMonRfFields->setVisible(false);
    ui->gbNavSatFields->setVisible(false);
    ui->tePayload->setVisible(false);
}

void GNSSWindow::setupConnections()
{
    // Инициализация таймера (но пока не запускаем)
    m_timer = new QTimer(this);
    m_initTimer->start(5000); // Таймаут 5 секунд на инициализацию

    connect(m_timer, &QTimer::timeout, this, &GNSSWindow::sendUbxNavPvt);

    // Отправляем начальную конфигурацию
    sendInitialConfiguration();

    // Другие соединения...
    connect(ui->sendButton, &QPushButton::clicked, this, &GNSSWindow::onSendButtonClicked);
    connect(ui->autoSendCheck, &QCheckBox::toggled, this, &GNSSWindow::onAutoSendToggled);
}

void GNSSWindow::sendInitialConfiguration() {
    if (m_waitingForAck) return;

    m_waitingForAck = true;
    m_ackTimeoutTimer->start();
    m_initTimer->start(20000);

    // Основная конфигурация
    sendUbxCfgPrtResponse();
    sendUbxCfgMsg(0x01, 0x07, 1);  // NAV-PVT
    sendUbxCfgMsg(0x01, 0x03, 1);  // NAV-STATUS (добавлено)
    sendUbxCfgMsg(0x0A, 0x28, 1);  // MON-RF
    sendUbxCfgRate(1000, 1);       // 1Hz
    sendUbxCfgDynModel(4);         // Automotive
    sendUbxCfgAntSettings(true, true, true); // Antenna settings
    sendUbxCfgItfm(true);          // Включить обнаружение помех

    // Информационные запросы с задержкой
    QTimer::singleShot(500, this, &GNSSWindow::sendUbxMonVer);
    QTimer::singleShot(1000, this, &GNSSWindow::sendUbxMonHw);
    QTimer::singleShot(1500, this, &GNSSWindow::sendUbxSecUniqidReq);
}

void GNSSWindow::onReadyRead()
{
    if (!m_socket) {
        qCritical() << "onReadyRead: Socket is null!";
        return;
    }

    // 1. Чтение всех доступных данных
    QByteArray newData = m_socket->readAll();
    if (newData.isEmpty()) {
        qWarning() << "Received empty data packet";
        return;
    }

    qDebug() << "Received" << newData.size() << "bytes of raw data:" << newData.toHex(' ');

    // 2. Добавление данных в буфер
    static QByteArray buffer; // Статический буфер для накопления данных между вызовами
    buffer.append(newData);

    qDebug() << "Total buffer size:" << buffer.size() << "bytes";
    appendToLog(QString("Received %1 bytes (total buffer: %2)")
                    .arg(newData.size())
                    .arg(buffer.size()), "in");

    // 3. Обработка всех полных сообщений в буфере
    while (buffer.size() >= 8) { // Минимальный размер UBX сообщения (без payload)
        // 3.1. Поиск синхробайтов
        int startPos = buffer.indexOf("\xB5\x62");
        if (startPos < 0) {
            qDebug() << "No UBX sync chars found, clearing buffer";
            buffer.clear();
            return;
        }

        // 3.2. Удаляем мусор перед синхробайтами
        if (startPos > 0) {
            qDebug() << "Discarding" << startPos << "bytes before sync chars";
            buffer.remove(0, startPos);
            continue;
        }

        // 3.3. Проверяем, достаточно ли данных для заголовка
        if (buffer.size() < 8) {
            qDebug() << "Waiting for more data (header incomplete)";
            return;
        }

        // 3.4. Извлекаем длину payload
        quint16 length = static_cast<quint8>(buffer[4]) | (static_cast<quint8>(buffer[5]) << 8);
        quint8 msgClass = static_cast<quint8>(buffer[2]);
        quint8 msgId = static_cast<quint8>(buffer[3]);

        qDebug() << "Potential message - Class:" << QString("0x%1").arg(msgClass, 2, 16, QLatin1Char('0'))
                 << "ID:" << QString("0x%1").arg(msgId, 2, 16, QLatin1Char('0'))
                 << "Length:" << length;

        // 3.5. Проверяем, есть ли полное сообщение
        int totalMessageSize = 8 + length; // Заголовок + payload
        if (buffer.size() < totalMessageSize) {
            qDebug() << "Waiting for more data. Need:" << totalMessageSize
                     << "Have:" << buffer.size();
            return;
        }

        // 3.6. Извлекаем полное сообщение
        QByteArray message = buffer.left(totalMessageSize);
        buffer.remove(0, totalMessageSize);

        qDebug() << "Processing message of size:" << message.size() << "bytes";
        appendToLog(QString("Processing UBX: Class=0x%1 ID=0x%2 (%3 bytes)")
                        .arg(msgClass, 2, 16, QLatin1Char('0'))
                        .arg(msgId, 2, 16, QLatin1Char('0'))
                        .arg(message.size()), "debug");

        // 3.7. Парсим сообщение
        QByteArray payload;
        if (UbxParser::parseUbxMessage(message, msgClass, msgId, payload)) {
            processUbxMessage(msgClass, msgId, payload);
        } else {
            qWarning() << "Failed to parse UBX message";
            appendToLog("Failed to parse UBX message", "error");
        }
    }
}

void GNSSWindow::registerHandlers()
{
    // NAV класс
    connect(&m_ubxParser, &UbxParser::navPvtReceived, this, &GNSSWindow::displayNavPvt);
    connect(&m_ubxParser, &UbxParser::navStatusReceived, this, &GNSSWindow::displayNavStatus);

    // MON класс
    connect(&m_ubxParser, &UbxParser::monVerReceived, this, &GNSSWindow::displayMonVer);
    connect(&m_ubxParser, &UbxParser::monHwReceived, this, &GNSSWindow::processMonHw);
    connect(&m_ubxParser, &UbxParser::monRfReceived, this, &GNSSWindow::displayMonRf);

    // CFG класс
    connect(&m_ubxParser, &UbxParser::cfgPrtReceived, this, &GNSSWindow::displayCfgPrt);

    // Обработка ошибок
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
    // Установим значения по умолчанию
    ui->sbRfVersion->setValue(0);
    ui->dsbRfNoise->setValue(50.0);
    ui->dsbRfAgc->setValue(120.0);
    ui->sbRfJamInd->setValue(0);
}

void GNSSWindow::displayMonRf(const UbxParser::MonRf &data)
{
    QString info = QString("MON-RF: Version=%1 Blocks=%2")
                       .arg(data.version)
                       .arg(data.nBlocks);

    for(int i = 0; i < data.nBlocks && i < 4; i++) {
        info += QString("\nBlock %1: AntID=%2 Noise=%3 AGC=%4 Jam=%5")
                    .arg(i)
                    .arg(data.blocks[i].antId)
                    .arg(data.blocks[i].noisePerMS)
                    .arg(data.blocks[i].agcCnt)
                    .arg(data.blocks[i].jamInd);
    }

    appendToLog(info, "status");
}

void GNSSWindow::processCfgValGet(const QByteArray &payload) {
    if(payload.size() >= 8) {
        quint32 key = qFromLittleEndian<quint32>(payload.mid(4, 4).constData());
        QByteArray response = payload.left(4);
        response.append(0x01); // Пример значения

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
    QByteArray payload(4 + 16, 0x00); // Header + 1 block

    // Version and number of blocks (fixed to 1 block)
    payload[0] = static_cast<quint8>(ui->sbRfVersion->value());
    payload[1] = 1; // We'll send 1 block
    payload[2] = 0; // Reserved
    payload[3] = 0; // Reserved

    // Fill block data
    int offset = 4;
    payload[offset] = 0; // Antenna ID
    payload[offset+1] = 0; // Flags

    float noise = static_cast<float>(ui->dsbRfNoise->value());
    float agc = static_cast<float>(ui->dsbRfAgc->value());

    qToLittleEndian<float>(noise, payload.data() + offset + 4);
    qToLittleEndian<float>(agc, payload.data() + offset + 8);

    payload[offset+12] = static_cast<quint8>(ui->sbRfJamInd->value());

    createUbxPacket(0x0A, 0x38, payload);
    appendToLog("Sent MON-RF message", "out");
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

void GNSSWindow::sendUbxCfgMsg(quint8 msgClass, quint8 msgId, quint8 rate)
{
    QByteArray payload;
    payload.append(msgClass);
    payload.append(msgId);
    payload.append(rate);

    createUbxPacket(0x06, 0x01, payload);
    qDebug() << "CFG-MSG: Class=" << QString("0x%1").arg(msgClass, 2, 16, QLatin1Char('0'))
             << "ID=" << QString("0x%1").arg(msgId, 2, 16, QLatin1Char('0'))
             << "Rate=" << rate;
}

void GNSSWindow::processUbxMessage(quint8 msgClass, quint8 msgId, const QByteArray& payload)
{
    QString messageInfo;
    QString timestamp = QDateTime::currentDateTime().toString("[hh:mm:ss.zzz]");

    // 1. Логирование входящего сообщения
    qDebug().nospace() << timestamp << " Processing UBX message: "
                       << "Class=0x" << QString("%1").arg(msgClass, 2, 16, QLatin1Char('0')).toUpper()
                       << " ID=0x" << QString("%1").arg(msgId, 2, 16, QLatin1Char('0')).toUpper()
                       << " Size=" << payload.size() << " bytes";

    qDebug() << "Payload hex:" << payload.toHex(' ');

    // 2. Обработка ACK/NACK сообщений
    if (msgClass == 0x05) {
        processAckNack(msgId, payload);
        return;
    }

    // 3. Обработка CFG сообщений
    if (msgClass == 0x06) {
        processCfgMessages(msgId, payload);
        return;
    }

    // 4. Обработка информационных запросов
    if (processInfoRequests(msgClass, msgId)) {
        return;
    }

    // 5. Обработка основных сообщений по классам
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

    // 6. Логирование результата обработки
    if (!messageInfo.isEmpty()) {
        appendToLog(messageInfo, "in");
    }
}

// Вспомогательные методы для обработки разных типов сообщений:

void GNSSWindow::processAckNack(quint8 msgId, const QByteArray& payload)
{
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

void GNSSWindow::completeInitialization()
{
    m_initializationComplete = true;
    m_waitingForAck = false;
    m_ackTimeoutTimer->stop();
    m_initTimer->stop();

    int rate = ui->rateSpin->value();
    m_pvtTimer->start(1000 / rate);
    m_statusTimer->start(1000 / rate);

    appendToLog("Configuration complete. Starting NAV-PVT and NAV-STATUS.", "system");
}

void GNSSWindow::processCfgMessages(quint8 msgId, const QByteArray& payload)
{
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

bool GNSSWindow::processInfoRequests(quint8 msgClass, quint8 msgId)
{
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

QString GNSSWindow::processNavMessages(quint8 msgId, const QByteArray& payload)
{
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

QString GNSSWindow::processRxmMessages(quint8 msgId, const QByteArray& payload)
{
    switch (msgId) {
    case 0x15: { // RXM-MEASX
        qDebug() << "RXM-MEASX measurement data received";
        return "RXM-MEASX: Measurement data received";
    }

    case 0x32: { // RXM-RAWX
        qDebug() << "RXM-RAWX raw measurement data received";
        return "RXM-RAWX: Raw measurement data received";
    }

    default:
        return QString("Unknown RXM message ID: 0x%1").arg(msgId, 2, 16, QLatin1Char('0'));
    }
}

QString GNSSWindow::processMonMessages(quint8 msgId, const QByteArray& payload)
{
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

    case 0x0B: { // MON-IO
        qDebug() << "MON-IO I/O system status received";
        return "MON-IO: I/O system status";
    }

    case 0x28: { // MON-RF
        UbxParser::MonRf rf = UbxParser::parseMonRf(payload);
        displayMonRf(rf);
        return QString("MON-RF: %1 RF blocks").arg(rf.nBlocks);
    }

    default:
        return QString("Unknown MON message ID: 0x%1").arg(msgId, 2, 16, QLatin1Char('0'));
    }
}

QString GNSSWindow::processSecMessages(quint8 msgId, const QByteArray& payload)
{
    switch (msgId) {
    case 0x03: { // SEC-UNIQID
        UbxParser::SecUniqid uniqid = UbxParser::parseSecUniqid(payload);
        emit secUniqidReceived(uniqid);
        return QString("SEC-UNIQID: 0x%1").arg(uniqid.uniqueId, 8, 16, QLatin1Char('0'));
    }

    case 0x04: { // SEC-SESSION
        qDebug() << "SEC-SESSION session management message received";
        return "SEC-SESSION: Session management";
    }

    default:
        return QString("Unknown SEC message ID: 0x%1").arg(msgId, 2, 16, QLatin1Char('0'));
    }
}

void GNSSWindow::processInfMessages(quint8 msgId, const QByteArray& payload)
{
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
        sendUbxCfgPrtResponse(); // Запрашиваем актуальную конфигурацию
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

void GNSSWindow::appendToLog(const QString &message, const QString &type)
{
    // 1. Проверка на паузу лога
    if (!ui || !ui->actionPauseLog || ui->actionPauseLog->isChecked()) {
        return;
    }

    // 2. Подготовка временной метки
    QString timestamp = QDateTime::currentDateTime().toString("[hh:mm:ss]");
    QString formattedMessage;

    // 3. Форматирование сообщения в зависимости от типа
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

    // 4. Проверка и добавление в лог
    if (ui && ui->teReceived) {
        // 4.1. Ограничение размера лога (не более 10000 строк)
        if (ui->teReceived->document()->lineCount() > 10000) {
            ui->teReceived->clear();
            ui->teReceived->append("<div style='color:#7B1FA2;'><b>Log cleared automatically</b></div>");
        }

        // 4.2. Добавление сообщения
        ui->teReceived->append(formattedMessage);

        // 4.3. Автопрокрутка если не стоит пауза
        if (!ui->actionPauseLog->isChecked()) {
            QTextCursor cursor = ui->teReceived->textCursor();
            cursor.movePosition(QTextCursor::End);
            ui->teReceived->setTextCursor(cursor);
        }
    }

    // 5. Дублирование в консоль для отладки
    qDebug().noquote() << QString("%1 %2: %3")
                              .arg(timestamp)
                              .arg(type.toUpper())
                              .arg(message);
}

void GNSSWindow::displayMonVer(const UbxParser::MonVer &data)
{
    ui->leSwVer->setText(data.swVersion);
    ui->leHwVer->setText(data.hwVersion);
    ui->statusbar->showMessage(
        QString("Version: SW %1, HW %2").arg(data.swVersion).arg(data.hwVersion),
        5000);
}

void GNSSWindow::processMonHw(const UbxParser::MonHw &hw)
{
    QString status;
    if(hw.aPower == 1) {
        status = "Active (Powered)";
    } else {
        status = "Inactive";
    }

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

void GNSSWindow::sendUbxNavSat()
{
    QByteArray payload;
    QRandomGenerator *generator = QRandomGenerator::global();

    // Заголовок (8 байт)
    payload.append("\x00\x00\x00\x00", 4); // iTOW (заполним позже)
    payload.append(static_cast<char>(ui->sbSatVersion->value())); // version
    payload.append(static_cast<char>(ui->sbNumSatsSat->value())); // numSvs
    payload.append("\x00\x00", 2); // reserved

    // Добавляем спутники (по 12 байт каждый)
    for(int i = 0; i < ui->sbNumSatsSat->value(); i++) {
        payload.append(static_cast<char>(1)); // gnssId (GPS)
        payload.append(static_cast<char>(i+1)); // svId
        payload.append(static_cast<char>(35 + generator->bounded(20))); // cno (35-55 dBHz)
        payload.append(static_cast<char>(30 + generator->bounded(50))); // elev (30-80 deg)
        qint16 azim = generator->bounded(360);
        payload.append(static_cast<char>(azim & 0xFF));
        payload.append(static_cast<char>((azim >> 8) & 0xFF));
        payload.append("\x00\x00\x00\x01", 4); // flags (используется)
    }

    // Обновляем iTOW (время недели GPS в мс)
    QDateTime currentTime = QDateTime::currentDateTimeUtc(); // Используем UTC время
    quint32 iTOW = static_cast<quint32>(currentTime.toMSecsSinceEpoch() % (7 * 24 * 60 * 60 * 1000));
    qToLittleEndian<quint32>(iTOW, payload.data());

    createUbxPacket(0x01, 0x35, payload);
    appendToLog("Sent NAV-SAT message", "out");
}

void GNSSWindow::onSendButtonClicked()
{
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
        m_statusTimer->start(1000 / rate); // Запускаем второй таймер
    } else {
        m_pvtTimer->stop();
        m_statusTimer->stop(); // Останавливаем оба таймера
    }
}

void GNSSWindow::sendUbxCfgPrtResponse()
{
    QByteArray payload(20, 0x00);

    // UART1 configuration
    payload[0] = 0x01; // Port ID: UART1

    // Mode: 8N1, no parity
    qToLittleEndian<quint32>(0x000008D0, payload.data() + 4);

    // BaudRate: 115200
    qToLittleEndian<quint32>(115200, payload.data() + 8);

    // Protocols: UBX + NMEA
    qToLittleEndian<quint16>(0x0003, payload.data() + 12); // inProtoMask
    qToLittleEndian<quint16>(0x0003, payload.data() + 14); // outProtoMask

    createUbxPacket(0x06, 0x00, payload);
    appendToLog("Sent UART1 configuration", "config");
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

    // SW Version
    payload.append("ROM CORE 3.01 (107888)");
    payload.append('\0');

    // HW Version
    payload.append("00080000");
    payload.append('\0');

    // Extensions
    payload.append("PROTVER=18.00");
    payload.append('\0');
    payload.append("GPS;GLO;GAL;BDS");
    payload.append('\0');
    payload.append("SBAS;IMES;QZSS");
    payload.append('\0');

    createUbxPacket(0x0A, 0x04, payload);
    appendToLog("Sent version information", "config");
}

// SEC-UNIQID (Unique Chip ID)
void GNSSWindow::sendUbxSecUniqid()
{
    QByteArray payload(5, 0x00);

    // Version
    payload[0] = 0x01;

    // Unique chip ID
    payload[1] = 0x12;
    payload[2] = 0x34;
    payload[3] = 0x56;
    payload[4] = 0x78;

    createUbxPacket(0x27, 0x03, payload);
    appendToLog("Sent SEC-UNIQID with valid chip ID", "config");

    ui->leChipId->setText("0x12345678");
}

// NAV-STATUS (Receiver Navigation Status)
void GNSSWindow::sendUbxNavStatus()
{
    QByteArray payload(16, 0x00);

    // Получаем текущее время
    QDateTime currentTime = QDateTime::currentDateTime();
    quint32 iTOW = static_cast<quint32>(currentTime.toMSecsSinceEpoch() % (7 * 24 * 60 * 60 * 1000)); // Время недели в мс

    // Заполняем поля из UI
    quint8 fixType = static_cast<quint8>(ui->sbFixTypeStatus->value());
    quint32 ttff = static_cast<quint32>(ui->sbTtff->value());

    // Заполняем payload
    qToLittleEndian<quint32>(iTOW, payload.data()); // iTOW
    payload[4] = fixType; // fixType
    qToLittleEndian<quint32>(ttff, payload.data() + 8); // ttff

    createUbxPacket(0x01, 0x03, payload);
    appendToLog("Sent NAV-STATUS", "out");
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
    // Создаем payload размером 8 байт (как указано в спецификации UBX-CFG-ITFM)
    QByteArray payload(8, 0x00);

    // Записываем config (первые 4 байта)
    qToLittleEndian<quint32>(0x00000000, payload.data());

    // Записываем config2 (последние 4 байта) - резерв, должен быть 0
    qToLittleEndian<quint32>(0x00000000, payload.data() + 4);

    // Формируем и отправляем UBX пакет
    createUbxPacket(0x06, 0x39, payload);

    appendToLog("Sent CFG-ITFM with default config (jamming detection disabled)", "config");

    qDebug() << "CFG-ITFM packet sent with payload:" << payload.toHex(' ');
}

void GNSSWindow::sendUbxAck(quint8 msgClass, quint8 msgId)
{
    QByteArray payload;
    payload.append(static_cast<char>(msgClass));
    payload.append(static_cast<char>(msgId));

    createUbxPacket(0x05, 0x01, payload); // 0x05,0x01 - это ACK

    appendToLog(QString("Sent ACK for Class=0x%1 ID=0x%2")
                    .arg(msgClass, 2, 16, QLatin1Char('0'))
                    .arg(msgId, 2, 16, QLatin1Char('0')),
                "config");

    qDebug() << "ACK sent for message Class:" << QString("0x%1").arg(msgClass, 2, 16, QLatin1Char('0'))
             << "ID:" << QString("0x%1").arg(msgId, 2, 16, QLatin1Char('0'));
}

void GNSSWindow::setupNavPvtFields()
{
    ui->gbNavPvtFields->setVisible(true);
    ui->gbNavStatusFields->setVisible(false);
    ui->tePayload->setVisible(false);

    // Установим значения по умолчанию
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

void GNSSWindow::setupNavStatusFields()
{
    ui->gbNavPvtFields->setVisible(false);
    ui->gbNavStatusFields->setVisible(true);
    ui->tePayload->setVisible(false);

    // Установим значения по умолчанию
    ui->sbFixTypeStatus->setValue(3); // 3D fix
    ui->sbTtff->setValue(5000); // 5 секунд
}

void GNSSWindow::sendUbxNack(quint8 msgClass, quint8 msgId)
{
    QByteArray payload;
    payload.append(static_cast<char>(msgClass));
    payload.append(static_cast<char>(msgId));

    createUbxPacket(0x05, 0x00, payload); // 0x05,0x00 - это NACK

    appendToLog(QString("Sent NACK for Class=0x%1 ID=0x%2")
                    .arg(msgClass, 2, 16, QLatin1Char('0'))
                    .arg(msgId, 2, 16, QLatin1Char('0')),
                "error");
}

void GNSSWindow::sendUbxMonHw()
{
    QByteArray payload(60, 0x00); // Exactly 60 bytes for MON-HW

    // Fill in all required fields
    qToLittleEndian<quint32>(0x00000000, payload.data());      // pinSel
    qToLittleEndian<quint32>(0x00000000, payload.data() + 4);  // pinBank
    qToLittleEndian<quint32>(0x00000000, payload.data() + 8);  // pinDir
    qToLittleEndian<quint32>(0x00000000, payload.data() + 12); // pinVal

    // Noise and AGC - set realistic values
    qToLittleEndian<quint16>(50, payload.data() + 16);  // noisePerMS (50)
    qToLittleEndian<quint16>(120, payload.data() + 18); // agcCnt (120)

    // Critical status fields
    payload[20] = 0x01; // aStatus (1 = OK)
    payload[21] = 0x01; // aPower (1 = powered)
    payload[22] = 0x01; // flags (1 = antenna supervised)
    payload[23] = 0x00; // reserved1

    // usedMask (4 bytes)
    payload[24] = 0x00;
    payload[25] = 0x00;
    payload[26] = 0x00;
    payload[27] = 0x00;

    // VP (interference) array (17 bytes)
    for(int i = 28; i < 45; i++) {
        payload[i] = 0x00;
    }

    payload[45] = 0x00; // jamInd (0%)
    payload[46] = 0x00; // reserved2[0]
    payload[47] = 0x00; // reserved2[1]

    qToLittleEndian<quint32>(0x00000000, payload.data() + 48); // pinIrq
    qToLittleEndian<quint32>(0x00000000, payload.data() + 52); // pullH
    qToLittleEndian<quint32>(0x00000000, payload.data() + 56); // pullL

    createUbxPacket(0x0A, 0x09, payload);
    appendToLog("Sent MON-HW with valid status (0x01)", "config");
}

void GNSSWindow::sendUbxNavPvt()
{
    QByteArray payload(92, 0x00);

    // Получаем текущее время
    QDateTime currentTime = QDateTime::currentDateTime();
    quint32 iTOW = static_cast<quint32>(currentTime.toMSecsSinceEpoch() % (7 * 24 * 60 * 60 * 1000)); // Время недели в мс

    // Заполняем поля из UI
    qint32 lat = static_cast<qint32>(ui->dsbLat->value() * 1e7);
    qint32 lon = static_cast<qint32>(ui->dsbLon->value() * 1e7);
    qint32 height = static_cast<qint32>(ui->dsbHeight->value() * 1000);
    quint32 gSpeed = static_cast<quint32>(ui->dsbSpeed->value() * 1000);
    quint32 headMot = static_cast<quint32>(ui->dsbHeading->value() * 1e5);
    quint8 fixType = static_cast<quint8>(ui->sbNumSats->value() > 0 ? 3 : 0); // 3 = 3D fix, если есть спутники
    quint8 numSV = static_cast<quint8>(ui->sbNumSats->value());
    qint32 velN = static_cast<qint32>(ui->dsbVelN->value() * 1000);
    qint32 velE = static_cast<qint32>(ui->dsbVelE->value() * 1000);
    qint32 velD = static_cast<qint32>(ui->dsbVelU->value() * 1000);
    quint32 hAcc = static_cast<quint32>(ui->dsbRmsPos->value() * 1000);
    quint32 vAcc = static_cast<quint32>(ui->dsbRmsPos->value() * 1000);
    quint32 sAcc = static_cast<quint32>(ui->dsbRmsVel->value() * 1000);
    quint16 pDOP = static_cast<quint16>(ui->dsbPdop->value() * 100);

    // Заполняем payload
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
    // 1. Проверка состояния сокета
    if (!m_socket) {
        qCritical() << "[UBX] Socket is null! Cannot send packet.";
        appendToLog("Error: Socket not initialized", "error");
        return;
    }

    if (m_socket->state() != QAbstractSocket::ConnectedState) {
        qWarning() << "[UBX] Socket not connected. Current state:" << m_socket->state();
        appendToLog(QString("Error: Socket state %1").arg(m_socket->state()), "error");
        return;
    }

    // 2. Проверка входных параметров
    if (msgClass > 0xFF || msgId > 0xFF) {
        qWarning() << "[UBX] Invalid class or ID (class:" << msgClass << "id:" << msgId << ")";
        return;
    }

    // 3. Формирование пакета
    QByteArray packet;

    // 3.1. Синхробайты
    packet.append('\xB5'); // Sync char 1
    packet.append('\x62'); // Sync char 2

    // 3.2. Заголовок
    packet.append(static_cast<char>(msgClass));
    packet.append(static_cast<char>(msgId));

    // 3.3. Длина payload (little-endian)
    quint16 length = payload.size();
    packet.append(static_cast<char>(length & 0xFF));
    packet.append(static_cast<char>((length >> 8) & 0xFF));

    // 3.4. Payload
    packet.append(payload);

    // 3.5. Расчет контрольной суммы
    quint8 ck_a = 0, ck_b = 0;
    for (int i = 2; i < packet.size(); i++) {
        quint8 byte = static_cast<quint8>(packet[i]);
        ck_a += byte;
        ck_b += ck_a;
    }
    packet.append(ck_a);
    packet.append(ck_b);

    // 4. Подробное логирование
    QString logMessage = QString("[UBX] Prepared packet: Class=0x%1 ID=0x%2 Length=%3 (%4 bytes total)")
                             .arg(msgClass, 2, 16, QLatin1Char('0'))
                             .arg(msgId, 2, 16, QLatin1Char('0'))
                             .arg(length)
                             .arg(packet.size());

    // Детальное логирование содержимого
    qDebug().noquote() << logMessage;
    qDebug() << "Sync bytes:" << packet.mid(0, 2).toHex();
    qDebug() << "Header:" << packet.mid(2, 2).toHex()
             << "(Class=0x" << QString::number(msgClass, 16)
             << " ID=0x" << QString::number(msgId, 16) << ")";
    qDebug() << "Length:" << packet.mid(4, 2).toHex() << "(" << length << "bytes)";

    if (length > 0) {
        qDebug() << "Payload:" << payload.toHex(' ');
    } else {
        qDebug() << "Payload: [empty]";
    }

    qDebug() << "Checksum:" << QString("0x%1 0x%2").arg(ck_a, 2, 16, QLatin1Char('0'))
                                   .arg(ck_b, 2, 16, QLatin1Char('0'));
    qDebug() << "Full packet:" << packet.toHex(' ');

    appendToLog(logMessage, "debug");
    appendToLog(QString("Packet details: %1 bytes payload, checksum=0x%2 0x%3")
                    .arg(length)
                    .arg(ck_a, 2, 16, QLatin1Char('0'))
                    .arg(ck_b, 2, 16, QLatin1Char('0')),
                "debug");

    // 5. Отправка пакета
    qint64 bytesWritten = m_socket->write(packet);

    if (bytesWritten == -1) {
        QString errorMsg = QString("[UBX] Write error: %1").arg(m_socket->errorString());
        qCritical() << errorMsg;
        appendToLog(errorMsg, "error");
    }
    else if (bytesWritten != packet.size()) {
        QString warnMsg = QString("[UBX] Partial write: %1/%2 bytes")
                              .arg(bytesWritten).arg(packet.size());
        qWarning() << warnMsg;
        appendToLog(warnMsg, "warning");
    }
    else {
        QString successMsg = QString("[UBX] Successfully sent %1 bytes (Class=0x%2 ID=0x%3)")
                                 .arg(bytesWritten)
                                 .arg(msgClass, 2, 16, QLatin1Char('0'))
                                 .arg(msgId, 2, 16, QLatin1Char('0'));
        qDebug() << successMsg;
        appendToLog(successMsg, "out");

        // Принудительная отправка буфера с проверкой
        if (!m_socket->flush()) {
            QString flushError = QString("[UBX] Flush failed: %1").arg(m_socket->errorString());
            qWarning() << flushError;
            appendToLog(flushError, "warning");
        }
    }
}
