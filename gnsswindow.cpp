#include "gnsswindow.h"
#include "ui_gnsswindow.h"
#include <QStandardItemModel>
#include <QLabel>
#include <QMessageBox>
#include <QDateTime>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include "dialog.h"
#include "ubxparser.h"
#include "ubxdefs.h"

GNSSWindow::GNSSWindow(Dialog* parentDialog, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::GNSSWindow),
    m_parentDialog(parentDialog),
    m_socket(nullptr),
    m_pvtTimer(new QTimer(this)),
    m_statusTimer(new QTimer(this)),
    m_initTimer(new QTimer(this)),
    m_ackTimeoutTimer(new QTimer(this)),
    m_utcTimer(new QTimer(this)),
    m_initializationComplete(false),
    m_waitingForAck(false) {
    ui->setupUi(this);

    // Setup timer intervals (default 1 Hz)
    m_pvtTimer->setInterval(1000);
    m_statusTimer->setInterval(1000);
    m_initTimer->setSingleShot(true);
    m_initTimer->setInterval(15000);   // 15 sec
    m_ackTimeoutTimer->setSingleShot(true);
    m_ackTimeoutTimer->setInterval(3000); // 3 sec

    // Setup fields
    initClassIdMapping();
    updateAvailableIds();
    initializeAllFields();
    m_firstInitialization = false;
    hideAllParameterFields();

    // Connections
    setupConnections();

    // Real time signal
    m_utcTimer->start(1000);
    updateUTCTime();

    // Initial state
    appendToLog("GNSS Window initialized successfully", "system");
    qDebug() << "GNSSWindow initialized at" << QDateTime::currentDateTime().toString("hh:mm:ss");
    ui->statusbar->showMessage("Waiting for connection...", 3000);
}

void GNSSWindow::initClassIdMapping() {
    m_classIdMap.clear();

    QMap<int, QString> navIds;
    navIds.insert(UBX_NAV_STATUS, "STATUS");
    navIds.insert(UBX_NAV_PVT, "PVT");
    navIds.insert(UBX_NAV_TIMEUTC, "TIMEUTC");
    navIds.insert(UBX_NAV_SAT, "SAT");
    m_classIdMap.insert(UBX_CLASS_NAV, navIds);

    QMap<int, QString> monIds;
    monIds.insert(UBX_MON_VER, "VER");
    monIds.insert(UBX_MON_HW, "HW");
    monIds.insert(UBX_MON_RF, "RF");
    m_classIdMap.insert(UBX_CLASS_MON, monIds);

    QMap<int, QString> cfgIds;
    cfgIds.insert(UBX_CFG_PRT, "PRT");
    cfgIds.insert(UBX_CFG_MSG, "MSG");
    cfgIds.insert(UBX_CFG_ANT, "ANT");
    cfgIds.insert(UBX_CFG_NAV5, "NAV5");
    cfgIds.insert(UBX_CFG_ITFM, "ITFM");
    cfgIds.insert(UBX_CFG_RATE, "RATE");
    cfgIds.insert(UBX_CFG_VALGET, "VALGET");
    cfgIds.insert(UBX_CFG_VALSET, "VALSET");
    m_classIdMap.insert(UBX_CLASS_CFG, cfgIds);

    QMap<int, QString> secIds;
    secIds.insert(UBX_SEC_UNIQID, "UNIQID");
    m_classIdMap.insert(UBX_CLASS_SEC, secIds);

    QMap<int, QString> infIds;
    infIds.insert(UBX_INF_DEBUG, "DEBUG");
    infIds.insert(UBX_INF_ERROR, "ERROR");
    infIds.insert(UBX_INF_NOTICE, "NOTICE");
    infIds.insert(UBX_INF_TEST, "TEST");
    infIds.insert(UBX_INF_WARNING, "WARNING");
    m_classIdMap.insert(UBX_CLASS_INF, infIds);

    QMap<int, QString> ackIds;
    ackIds.insert(UBX_ACK_ACK, "ACK");
    ackIds.insert(UBX_ACK_NAK, "NAK");
    m_classIdMap.insert(UBX_CLASS_ACK, ackIds);

    // Fill class comboBox
    ui->cbClass->clear();
    ui->cbClass->addItem("NAV (0x01)", UBX_CLASS_NAV);
    ui->cbClass->addItem("CFG (0x06)", UBX_CLASS_CFG);
    ui->cbClass->addItem("MON (0x0A)", UBX_CLASS_MON);
    ui->cbClass->addItem("SEC (0x27)", UBX_CLASS_SEC);
    ui->cbClass->addItem("INF (0x04)", UBX_CLASS_INF);
    ui->cbClass->addItem("ACK (0x05)", UBX_CLASS_ACK);

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

QJsonObject GNSSWindow::getCurrentSettings() const {
    QJsonObject settings;

    // Save auto-send checkboxes
    QJsonObject autoSendSettings;
    autoSendSettings["navPvt"] = ui->cbAutoSendNavPvt->isChecked();
    autoSendSettings["navStatus"] = ui->cbAutoSendNavStatus->isChecked();
    autoSendSettings["navSat"] = ui->cbAutoSendNavSat->isChecked();
    autoSendSettings["navTimeUTC"] = ui->cbAutoSendNavTimeUTC->isChecked();
    autoSendSettings["monVer"] = ui->cbAutoSendMonVer->isChecked();
    autoSendSettings["monHw"] = ui->cbAutoSendMonHw->isChecked();
    autoSendSettings["monRf"] = ui->cbAutoSendMonRf->isChecked();
    autoSendSettings["cfgPrt"] = ui->cbAutoSendCfgPrt->isChecked();
    autoSendSettings["cfgItfm"] = ui->cbAutoSendCfgItfm->isChecked();
    autoSendSettings["cfgNav5"] = ui->cbAutoSendCfgNav5->isChecked();
    autoSendSettings["cfgRate"] = ui->cbAutoSendCfgRate->isChecked();
    autoSendSettings["cfgValget"] = ui->cbAutoSendCfgValget->isChecked();
    autoSendSettings["cfgValset"] = ui->cbAutoSendCfgValset->isChecked();
    autoSendSettings["cfgAnt"] = ui->cbAutoSendCfgAnt->isChecked();
    autoSendSettings["infDebug"] = ui->cbAutoSendInfDebug->isChecked();
    autoSendSettings["infError"] = ui->cbAutoSendInfError->isChecked();
    autoSendSettings["infWarning"] = ui->cbAutoSendInfWarning->isChecked();
    autoSendSettings["infNotice"] = ui->cbAutoSendInfNotice->isChecked();
    autoSendSettings["infTest"] = ui->cbAutoSendInfTest->isChecked();
    autoSendSettings["secUniqid"] = ui->cbAutoSendSecUniqid->isChecked();
    settings["autoSend"] = autoSendSettings;

    // Save rate
    settings["rate"] = ui->rateSpin->value();

    // Save NAV-PVT settings
    QJsonObject navPvtSettings;
    navPvtSettings["lat"] = ui->dsbLat->value();
    navPvtSettings["lon"] = ui->dsbLon->value();
    navPvtSettings["height"] = ui->dsbHeight->value();
    navPvtSettings["speed"] = ui->dsbSpeed->value();
    navPvtSettings["heading"] = ui->dsbHeading->value();
    navPvtSettings["numSats"] = ui->sbNumSats->value();
    navPvtSettings["velN"] = ui->dsbVelN->value();
    navPvtSettings["velE"] = ui->dsbVelE->value();
    navPvtSettings["velU"] = ui->dsbVelU->value();
    navPvtSettings["rmsPos"] = ui->dsbRmsPos->value();
    navPvtSettings["rmsVel"] = ui->dsbRmsVel->value();
    navPvtSettings["pdop"] = ui->dsbPdop->value();
    settings["navPvt"] = navPvtSettings;

    // Save NAV-STATUS settings
    QJsonObject navStatusSettings;
    navStatusSettings["fixType"] = ui->sbFixTypeStatus->value();
    navStatusSettings["ttff"] = ui->sbTtff->value();
    settings["navStatus"] = navStatusSettings;

    // Save NAV-SAT settings
    QJsonObject navSatSettings;
    navSatSettings["version"] = ui->sbSatVersion->value();
    navSatSettings["numSats"] = ui->sbNumSatsSat->value();
    navSatSettings["qualityInd"] = ui->cbQualityInd->currentIndex();
    navSatSettings["health"] = ui->cbHealth->currentIndex();
    navSatSettings["prResMin"] = ui->dsbPrResMin->value();
    navSatSettings["prResMax"] = ui->dsbPrResMax->value();
    navSatSettings["svUsed"] = ui->cbSvUsed->isChecked();
    navSatSettings["diffCorr"] = ui->cbDiffCorr->isChecked();
    navSatSettings["smoothed"] = ui->cbSmoothed->isChecked();
    navSatSettings["orbitSource"] = ui->cbOrbitSource->currentIndex();
    settings["navSat"] = navSatSettings;

    // Save NAV-TIMEUTC settings
    QJsonObject navTimeUtcSettings;
    navTimeUtcSettings["tAcc"] = ui->sbTimeUtcTAcc->value();
    navTimeUtcSettings["nano"] = ui->sbTimeUtcNano->value();
    navTimeUtcSettings["valid"] = ui->cbTimeUtcValid->currentIndex();
    navTimeUtcSettings["standard"] = ui->cbTimeUtcStandard->currentIndex();
    settings["navTimeUtc"] = navTimeUtcSettings;

    // Save MON-VER settings
    QJsonObject monVerSettings;
    monVerSettings["swVersion"] = ui->leSwVersion->text();
    monVerSettings["hwVersion"] = ui->leHwVersion->text();
    monVerSettings["extensions"] = ui->teExtensions->toPlainText();
    settings["monVer"] = monVerSettings;

    // Save MON-HW settings
    QJsonObject monHwSettings;
    monHwSettings["noise"] = ui->sbHwNoise->value();
    monHwSettings["agc"] = ui->sbHwAgc->value();
    monHwSettings["antStatus"] = ui->cbHwAntStatus->currentIndex();
    monHwSettings["antPower"] = ui->cbHwAntPower->currentIndex();
    monHwSettings["jamming"] = ui->cbHwJamming->currentIndex();
    monHwSettings["cwSuppression"] = ui->sbHwCwSuppression->value();
    settings["monHw"] = monHwSettings;

    // Save MON-RF settings
    QJsonObject monRfSettings;
    monRfSettings["version"] = ui->sbRfVersion->value();
    monRfSettings["blocks"] = ui->sbRfBlocks->value();
    monRfSettings["noise"] = ui->dsbRfNoise->value();
    monRfSettings["agc"] = ui->dsbRfAgc->value();
    monRfSettings["jamState"] = ui->cbRfJamState->currentIndex();
    monRfSettings["antStatus"] = ui->cbRfAntStatus->currentIndex();
    monRfSettings["antPower"] = ui->cbRfAntPower->currentIndex();
    monRfSettings["cwSuppression"] = ui->sbRfCwSuppression->value();
    settings["monRf"] = monRfSettings;

    // Save CFG-PRT settings
    QJsonObject cfgPrtSettings;
    cfgPrtSettings["portId"] = ui->cbPortId->currentIndex();
    cfgPrtSettings["baudRate"] = ui->cbBaudRate->currentText();
    cfgPrtSettings["inUbx"] = ui->cbInUbx->isChecked();
    cfgPrtSettings["inNmea"] = ui->cbInNmea->isChecked();
    cfgPrtSettings["outUbx"] = ui->cbOutUbx->isChecked();
    cfgPrtSettings["outNmea"] = ui->cbOutNmea->isChecked();
    settings["cfgPrt"] = cfgPrtSettings;

    // Save CFG-ITFM settings
    QJsonObject cfgItfmSettings;
    cfgItfmSettings["bbThreshold"] = ui->sbBbThreshold->value();
    cfgItfmSettings["cwThreshold"] = ui->sbCwThreshold->value();
    cfgItfmSettings["enable"] = ui->cbEnable->isChecked();
    cfgItfmSettings["antSetting"] = ui->cbAntSetting->currentIndex();
    cfgItfmSettings["enable2"] = ui->cbEnable2->isChecked();
    settings["cfgItfm"] = cfgItfmSettings;

    // Save CFG-NAV5 settings
    QJsonObject cfgNav5Settings;
    cfgNav5Settings["dynModel"] = ui->cbDynModel->currentIndex();
    cfgNav5Settings["fixMode"] = ui->cbFixMode->currentIndex();
    cfgNav5Settings["fixedAlt"] = ui->dsbFixedAlt->value();
    cfgNav5Settings["minElev"] = ui->sbMinElev->value();
    cfgNav5Settings["pdop"] = ui->dsbPDOP->value();
    cfgNav5Settings["tdop"] = ui->dsbTDOP->value();
    cfgNav5Settings["pAcc"] = ui->dsbPAcc->value();
    cfgNav5Settings["tAcc"] = ui->dsbTAcc->value();
    cfgNav5Settings["staticHoldThresh"] = ui->dsbStaticHoldThresh->value();
    cfgNav5Settings["dgnssTimeout"] = ui->sbDgnssTimeout->value();
    cfgNav5Settings["cnoThresh"] = ui->sbCnoThresh->value();
    cfgNav5Settings["cnoThreshNumSVs"] = ui->sbCnoThreshNumSVs->value();
    cfgNav5Settings["utcStandard"] = ui->cbUtcStandard->currentIndex();
    cfgNav5Settings["staticHoldMaxDist"] = ui->dsbStaticHoldMaxDist->value();
    settings["cfgNav5"] = cfgNav5Settings;

    // Save CFG-RATE settings
    QJsonObject cfgRateSettings;
    cfgRateSettings["measRate"] = ui->sbMeasRate->value();
    cfgRateSettings["navRate"] = ui->sbNavRate->value();
    cfgRateSettings["timeRef"] = ui->cbTimeRef->currentIndex();
    settings["cfgRate"] = cfgRateSettings;

    // Save CFG-VALGET settings
    QJsonObject cfgValgetSettings;
    cfgValgetSettings["version"] = ui->sbValgetVersion->value();
    cfgValgetSettings["layer"] = ui->cbValgetLayer->currentIndex();
    cfgValgetSettings["position"] = ui->sbValgetPosition->value();
    cfgValgetSettings["keys"] = ui->leValgetKeys->text();
    settings["cfgValget"] = cfgValgetSettings;

    // Save CFG-VALSET settings
    QJsonObject cfgValsetSettings;
    cfgValsetSettings["version"] = ui->sbValsetVersion->value();
    cfgValsetSettings["ram"] = ui->cbValsetRam->isChecked();
    cfgValsetSettings["bbr"] = ui->cbValsetBbr->isChecked();
    cfgValsetSettings["flash"] = ui->cbValsetFlash->isChecked();
    cfgValsetSettings["keysValues"] = ui->leValsetKeysValues->text();
    settings["cfgValset"] = cfgValsetSettings;

    // Save CFG-ANT settings
    QJsonObject cfgAntSettings;
    cfgAntSettings["supplyCtrl"] = ui->cbAntSupplyCtrl->isChecked();
    cfgAntSettings["shortDetect"] = ui->cbAntShortDetect->isChecked();
    cfgAntSettings["openDetect"] = ui->cbAntOpenDetect->isChecked();
    cfgAntSettings["powerDown"] = ui->cbAntPowerDown->isChecked();
    cfgAntSettings["autoRecover"] = ui->cbAntAutoRecover->isChecked();
    cfgAntSettings["switchPin"] = ui->sbAntSwitchPin->value();
    cfgAntSettings["shortPin"] = ui->sbAntShortPin->value();
    cfgAntSettings["openPin"] = ui->sbAntOpenPin->value();
    cfgAntSettings["reconfig"] = ui->cbAntReconfig->isChecked();
    settings["cfgAnt"] = cfgAntSettings;

    // Save INF-DEBUG settings
    QJsonObject infDebugSettings;
    infDebugSettings["message"] = ui->teInfDebugMessage->toPlainText();
    settings["infDebug"] = infDebugSettings;

    // Save INF-ERROR settings
    QJsonObject infErrorSettings;
    infErrorSettings["message"] = ui->teInfErrorMessage->toPlainText();
    settings["infError"] = infErrorSettings;

    // Save INF-WARNING settings
    QJsonObject infWarningSettings;
    infWarningSettings["message"] = ui->teInfWarningMessage->toPlainText();
    settings["infWarning"] = infWarningSettings;

    // Save INF-NOTICE settings
    QJsonObject infNoticeSettings;
    infNoticeSettings["message"] = ui->teInfNoticeMessage->toPlainText();
    settings["infNotice"] = infNoticeSettings;

    // Save INF-TEST settings
    QJsonObject infTestSettings;
    infTestSettings["message"] = ui->teInfTestMessage->toPlainText();
    settings["infTest"] = infTestSettings;

    // Save SEC-UNIQID settings
    QJsonObject secUniqidSettings;
    secUniqidSettings["version"] = ui->sbUniqidVersion->value();
    secUniqidSettings["chipId"] = ui->leChipId->text();
    settings["secUniqid"] = secUniqidSettings;

    return settings;
}

void GNSSWindow::saveSettings(const QString &filename) {
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly)) {
        QMessageBox::warning(this, "Error", "Could not save settings file");
        return;
    }

    QJsonObject settings = getCurrentSettings();
    QJsonDocument doc(settings);
    file.write(doc.toJson());
    file.close();

    appendToLog(QString("Settings saved to %1").arg(filename), "system");
}

void GNSSWindow::loadSettings(const QString &filename) {
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly)) {
        QMessageBox::warning(this, "Error", "Could not load settings file");
        return;
    }

    QByteArray data = file.readAll();
    file.close();

    QJsonDocument doc = QJsonDocument::fromJson(data);
    if (doc.isNull()) {
        QMessageBox::warning(this, "Error", "Invalid settings file format");
        return;
    }

    applySettings(doc.object());
    appendToLog(QString("Settings loaded from %1").arg(filename), "system");
}

void GNSSWindow::applySettings(const QJsonObject &settings)
{
    // 1. Блокируем сигналы всех виджетов
    QList<QWidget*> widgets = findChildren<QWidget*>();
    foreach (QWidget* widget, widgets) {
        widget->blockSignals(true);
    }

    // 2. Применяем общие настройки
    if (settings.contains("general")) {
        QJsonObject general = settings["general"].toObject();
        ui->autoSendCheck->setChecked(general["autoSendEnabled"].toBool());
        ui->rateSpin->setValue(general["updateRate"].toInt(1));
    }

    // 3. Применяем настройки автоотправки
    if (settings.contains("autoSend")) {
        QJsonObject autoSend = settings["autoSend"].toObject();
        auto applyCheckbox = [&](const QString& key, QCheckBox* checkbox) {
            if (autoSend.contains(key)) checkbox->setChecked(autoSend[key].toBool());
        };

        applyCheckbox("navPvt", ui->cbAutoSendNavPvt);
        applyCheckbox("navStatus", ui->cbAutoSendNavStatus);
        applyCheckbox("navSat", ui->cbAutoSendNavSat);
        applyCheckbox("navTimeUTC", ui->cbAutoSendNavTimeUTC);
        applyCheckbox("monVer", ui->cbAutoSendMonVer);
        applyCheckbox("monHw", ui->cbAutoSendMonHw);
        applyCheckbox("monRf", ui->cbAutoSendMonRf);
        applyCheckbox("cfgPrt", ui->cbAutoSendCfgPrt);
        applyCheckbox("cfgItfm", ui->cbAutoSendCfgItfm);
        applyCheckbox("cfgNav5", ui->cbAutoSendCfgNav5);
        applyCheckbox("cfgRate", ui->cbAutoSendCfgRate);
        applyCheckbox("cfgValget", ui->cbAutoSendCfgValget);
        applyCheckbox("cfgValset", ui->cbAutoSendCfgValset);
        applyCheckbox("cfgAnt", ui->cbAutoSendCfgAnt);
        applyCheckbox("infDebug", ui->cbAutoSendInfDebug);
        applyCheckbox("infError", ui->cbAutoSendInfError);
        applyCheckbox("infWarning", ui->cbAutoSendInfWarning);
        applyCheckbox("infNotice", ui->cbAutoSendInfNotice);
        applyCheckbox("infTest", ui->cbAutoSendInfTest);
        applyCheckbox("secUniqid", ui->cbAutoSendSecUniqid);
    }

    // 4. Применяем параметры для каждого типа сообщений
    auto applyNavPvtSettings = [&](const QJsonObject& obj) {
        ui->dsbLat->setValue(obj["lat"].toDouble(55.7522200));
        ui->dsbLon->setValue(obj["lon"].toDouble(37.6155600));
        ui->dsbHeight->setValue(obj["height"].toDouble(150.0));
        ui->dsbSpeed->setValue(obj["speed"].toDouble(0.0));
        ui->dsbHeading->setValue(obj["heading"].toDouble(0.0));
        ui->sbNumSats->setValue(obj["numSats"].toInt(10));
        ui->dsbVelN->setValue(obj["velN"].toDouble(0.0));
        ui->dsbVelE->setValue(obj["velE"].toDouble(0.0));
        ui->dsbVelU->setValue(obj["velU"].toDouble(0.0));
        ui->dsbRmsPos->setValue(obj["rmsPos"].toDouble(1.0));
        ui->dsbRmsVel->setValue(obj["rmsVel"].toDouble(0.1));
        ui->dsbPdop->setValue(obj["pdop"].toDouble(1.5));
    };

    auto applyNavStatusSettings = [&](const QJsonObject& obj) {
        ui->sbFixTypeStatus->setValue(obj["fixType"].toInt(3));
        ui->sbTtff->setValue(obj["ttff"].toInt(5000));
    };

    auto applyNavSatSettings = [&](const QJsonObject& obj) {
        ui->sbSatVersion->setValue(obj["version"].toInt(1));
        ui->sbNumSatsSat->setValue(obj["numSats"].toInt(10));
        ui->cbQualityInd->setCurrentIndex(obj["qualityInd"].toInt(4));
        ui->cbHealth->setCurrentIndex(obj["health"].toInt(1));
        ui->dsbPrResMin->setValue(obj["prResMin"].toDouble(-2.0));
        ui->dsbPrResMax->setValue(obj["prResMax"].toDouble(2.0));
        ui->cbSvUsed->setChecked(obj["svUsed"].toBool(true));
        ui->cbDiffCorr->setChecked(obj["diffCorr"].toBool(false));
        ui->cbSmoothed->setChecked(obj["smoothed"].toBool(false));
        ui->cbOrbitSource->setCurrentIndex(obj["orbitSource"].toInt(1));
    };

    auto applyMonVerSettings = [&](const QJsonObject& obj) {
        ui->leSwVersion->setText(obj["swVersion"].toString("ROM CORE 3.01 (107888)"));
        ui->leHwVersion->setText(obj["hwVersion"].toString("00080000"));
        ui->teExtensions->setPlainText(obj["extensions"].toString("PROTVER=18.00\nGPS;GLO;GAL;BDS\nSBAS;IMES;QZSS"));
    };

    auto applyMonHwSettings = [&](const QJsonObject& obj) {
        ui->sbHwNoise->setValue(obj["noise"].toInt(50));
        ui->sbHwAgc->setValue(obj["agc"].toInt(75));
        ui->cbHwAntStatus->setCurrentIndex(obj["antStatus"].toInt(2)); // 2 = OK
        ui->cbHwAntPower->setCurrentIndex(obj["antPower"].toInt(1));   // 1 = ON
        ui->cbHwJamming->setCurrentIndex(obj["jamming"].toInt(1));     // 1 = OK
        ui->sbHwCwSuppression->setValue(obj["cwSuppression"].toInt(0));
    };

    auto applyMonRfSettings = [&](const QJsonObject& obj) {
        ui->sbRfVersion->setValue(obj["version"].toInt(0));
        ui->sbRfBlocks->setValue(obj["blocks"].toInt(1));
        ui->dsbRfNoise->setValue(obj["noise"].toDouble(50.0));
        ui->dsbRfAgc->setValue(obj["agc"].toDouble(75.0));
        ui->cbRfJamState->setCurrentIndex(obj["jamState"].toInt(1));   // 1 = OK
        ui->cbRfAntStatus->setCurrentIndex(obj["antStatus"].toInt(2)); // 2 = OK
        ui->cbRfAntPower->setCurrentIndex(obj["antPower"].toInt(1));   // 1 = ON
        ui->sbRfCwSuppression->setValue(obj["cwSuppression"].toInt(0));
    };

    auto applyCfgPrtSettings = [&](const QJsonObject& obj) {
        ui->cbPortId->setCurrentIndex(obj["portId"].toInt(0)); // 0 = UART1
        ui->cbBaudRate->setCurrentText(obj["baudRate"].toString("115200"));
        ui->cbInUbx->setChecked(obj["inUbx"].toBool(true));
        ui->cbInNmea->setChecked(obj["inNmea"].toBool(true));
        ui->cbOutUbx->setChecked(obj["outUbx"].toBool(true));
        ui->cbOutNmea->setChecked(obj["outNmea"].toBool(true));
    };

    auto applyCfgItfmSettings = [&](const QJsonObject& obj) {
        ui->sbBbThreshold->setValue(obj["bbThreshold"].toInt(0));
        ui->sbCwThreshold->setValue(obj["cwThreshold"].toInt(0));
        ui->cbEnable->setChecked(obj["enable"].toBool(true));
        ui->cbAntSetting->setCurrentIndex(obj["antSetting"].toInt(0));
        ui->cbEnable2->setChecked(obj["enable2"].toBool(false));
    };

    auto applyCfgNav5Settings = [&](const QJsonObject& obj) {
        ui->cbDynModel->setCurrentIndex(obj["dynModel"].toInt(4));      // 4 = Automotive
        ui->cbFixMode->setCurrentIndex(obj["fixMode"].toInt(2));        // 2 = Auto 2D/3D
        ui->dsbFixedAlt->setValue(obj["fixedAlt"].toDouble(0.0));
        ui->sbMinElev->setValue(obj["minElev"].toInt(5));
        ui->dsbPDOP->setValue(obj["pdop"].toDouble(2.5));
        ui->dsbTDOP->setValue(obj["tdop"].toDouble(2.5));
        ui->dsbPAcc->setValue(obj["pAcc"].toDouble(0.0));
        ui->dsbTAcc->setValue(obj["tAcc"].toDouble(0.0));
        ui->dsbStaticHoldThresh->setValue(obj["staticHoldThresh"].toDouble(0.0));
        ui->sbDgnssTimeout->setValue(obj["dgnssTimeout"].toInt(60));
        ui->sbCnoThresh->setValue(obj["cnoThresh"].toInt(0));
        ui->sbCnoThreshNumSVs->setValue(obj["cnoThreshNumSVs"].toInt(0));
        ui->cbUtcStandard->setCurrentIndex(obj["utcStandard"].toInt(0)); // 0 = Automatic
        ui->dsbStaticHoldMaxDist->setValue(obj["staticHoldMaxDist"].toDouble(0.0));
    };

    auto applyCfgRateSettings = [&](const QJsonObject& obj) {
        ui->sbMeasRate->setValue(obj["measRate"].toInt(1000));  // 1 Hz
        ui->sbNavRate->setValue(obj["navRate"].toInt(1));
        ui->cbTimeRef->setCurrentIndex(obj["timeRef"].toInt(0)); // 0 = UTC
    };

    auto applyCfgValgetSettings = [&](const QJsonObject& obj) {
        ui->sbValgetVersion->setValue(obj["version"].toInt(0));
        ui->cbValgetLayer->setCurrentIndex(obj["layer"].toInt(0)); // 0 = RAM
        ui->sbValgetPosition->setValue(obj["position"].toInt(0));
        ui->leValgetKeys->setText(obj["keys"].toString("0x00000000"));
    };

    auto applyCfgValsetSettings = [&](const QJsonObject& obj) {
        ui->sbValsetVersion->setValue(obj["version"].toInt(0));
        ui->cbValsetRam->setChecked(obj["ram"].toBool(true));
        ui->cbValsetBbr->setChecked(obj["bbr"].toBool(false));
        ui->cbValsetFlash->setChecked(obj["flash"].toBool(false));
        ui->leValsetKeysValues->setText(obj["keysValues"].toString("0x00000000=0x00000000"));
    };

    auto applyCfgAntSettings = [&](const QJsonObject& obj) {
        ui->cbAntSupplyCtrl->setChecked(obj["supplyCtrl"].toBool(true));
        ui->cbAntShortDetect->setChecked(obj["shortDetect"].toBool(true));
        ui->cbAntOpenDetect->setChecked(obj["openDetect"].toBool(true));
        ui->cbAntPowerDown->setChecked(obj["powerDown"].toBool(true));
        ui->cbAntAutoRecover->setChecked(obj["autoRecover"].toBool(true));
        ui->sbAntSwitchPin->setValue(obj["switchPin"].toInt(0));
        ui->sbAntShortPin->setValue(obj["shortPin"].toInt(1));
        ui->sbAntOpenPin->setValue(obj["openPin"].toInt(2));
        ui->cbAntReconfig->setChecked(obj["reconfig"].toBool(false));
    };

    auto applyInfDebugSettings = [&](const QJsonObject& obj) {
        ui->teInfDebugMessage->setPlainText(obj["message"].toString("Debug message"));
    };

    auto applyInfErrorSettings = [&](const QJsonObject& obj) {
        ui->teInfErrorMessage->setPlainText(obj["message"].toString("Error message"));
    };

    auto applyInfWarningSettings = [&](const QJsonObject& obj) {
        ui->teInfWarningMessage->setPlainText(obj["message"].toString("Warning message"));
    };

    auto applyInfNoticeSettings = [&](const QJsonObject& obj) {
        ui->teInfNoticeMessage->setPlainText(obj["message"].toString("Notice message"));
    };

    auto applyInfTestSettings = [&](const QJsonObject& obj) {
        ui->teInfTestMessage->setPlainText(obj["message"].toString("Test message"));
    };

    auto applySecUniqidSettings = [&](const QJsonObject& obj) {
        ui->sbUniqidVersion->setValue(obj["version"].toInt(1));
        ui->leChipId->setText(obj["chipId"].toString("12345678"));
    };

    auto applyIfExists = [&](const QString& key, auto handler) {
        if (settings.contains(key)) handler(settings[key].toObject());
    };

    applyIfExists("navPvt", applyNavPvtSettings);
    applyIfExists("navStatus", applyNavStatusSettings);
    applyIfExists("navSat", applyNavSatSettings);
    applyIfExists("monVer", applyMonVerSettings);
    applyIfExists("monHw", applyMonHwSettings);
    applyIfExists("monRf", applyMonRfSettings);
    applyIfExists("cfgPrt", applyCfgPrtSettings);
    applyIfExists("cfgItfm", applyCfgItfmSettings);
    applyIfExists("cfgNav5", applyCfgNav5Settings);
    applyIfExists("cfgRate", applyCfgRateSettings);
    applyIfExists("cfgValget", applyCfgValgetSettings);
    applyIfExists("cfgValset", applyCfgValsetSettings);
    applyIfExists("cfgAnt", applyCfgAntSettings);
    applyIfExists("infDebug", applyInfDebugSettings);
    applyIfExists("infError", applyInfErrorSettings);
    applyIfExists("infWarning", applyInfWarningSettings);
    applyIfExists("infNotice", applyInfNoticeSettings);
    applyIfExists("infTest", applyInfTestSettings);
    applyIfExists("secUniqid", applySecUniqidSettings);

    foreach (QWidget* widget, widgets) {
        widget->blockSignals(false);
    }

    updateAvailableIds();
    onClassIdChanged();

    onAutoSendToggled(ui->autoSendCheck->isChecked());

    ui->statusbar->showMessage(tr("Settings applied successfully"), 3000);
    appendToLog("All settings applied from configuration", "system");
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

void GNSSWindow::handleInitTimeout()
{
    if (!m_initializationComplete) {
        appendToLog("Configuration timeout - proceeding without ACK", "warning");
        m_initializationComplete = true;

        if (m_socket && m_socket->state() == QAbstractSocket::ConnectedState) {
            int rate = ui->rateSpin->value();
            m_pvtTimer->start(1000 / rate);
            m_statusTimer->start(1000 / rate);
            ui->autoSendCheck->setChecked(true);
        }
    }
}

void GNSSWindow::handleAckTimeout()
{
    if (m_waitingForAck) {
        appendToLog("ACK timeout - configuration may be incomplete", "error");
        m_waitingForAck = false;
    }
}

void GNSSWindow::handleSocketDisconnected()
{
    appendToLog("Disconnected from host", "system");
    m_timer->stop();
    m_socket = nullptr;
}

void GNSSWindow::handleSocketError(QAbstractSocket::SocketError error)
{
    Q_UNUSED(error);
    QString errorMsg = m_socket ? m_socket->errorString() : "Socket not initialized";
    appendToLog("Socket error: " + errorMsg, "error");

    m_timer->stop();
    m_initializationComplete = false;
    m_waitingForAck = false;

    if (m_socket && m_socket->state() != QAbstractSocket::UnconnectedState) {
        m_socket->disconnectFromHost();
    }

    ui->statusbar->showMessage("Connection error: " + errorMsg, 5000);
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

void GNSSWindow::updateUTCTime() {
    QDateTime utcTime = QDateTime::currentDateTimeUtc();
    QString timeString = utcTime.toString("yyyy-MM-dd hh:mm:ss UTC");
    ui->leUTCTimeView->setText(timeString);
}

void GNSSWindow::onActionSaveLogTriggered() {
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

void GNSSWindow::onActionClearLogTriggered() {
    ui->teReceived->clear();
    appendToLog("Log cleared by user", "system");
}

void GNSSWindow::onActionAboutTriggered() {
    QMessageBox::about(this, "About GNSS Simulator",
                       "Application simulates GNSS receiver\n"
                       "Version: 1.0\n"
                       "Uses the u-blox UBX protocol");
}

void GNSSWindow::setupMonRfFields() {
    ui->gbMonRfFields->setVisible(true);
    ui->tePayload->setVisible(false);

    if (m_firstInitialization) {
        ui->sbRfVersion->setValue(0);
        ui->sbRfBlocks->setValue(1);
        ui->dsbRfNoise->setValue(50.0);
        ui->dsbRfAgc->setValue(75.0);
        ui->cbRfJamState->setCurrentIndex(1); // OK
        ui->cbRfAntStatus->setCurrentIndex(2); // OK
        ui->cbRfAntPower->setCurrentIndex(1); // ON
        ui->sbRfCwSuppression->setValue(0);
    }
}

void GNSSWindow::setupCfgValsetFields() {
    ui->gbCfgValsetFields->setVisible(true);
    ui->tePayload->setVisible(false);

    if (m_firstInitialization) {
        ui->sbValsetVersion->setValue(0);
        ui->cbValsetRam->setChecked(true);
        ui->cbValsetBbr->setChecked(false);
        ui->cbValsetFlash->setChecked(false);
        ui->leValsetKeysValues->setText("0x00000000=0x00000000");
    }
}

void GNSSWindow::setupCfgValgetFields() {
    ui->gbCfgValgetFields->setVisible(true);
    ui->tePayload->setVisible(false);

    if (m_firstInitialization) {
        ui->sbValgetVersion->setValue(0);
        ui->cbValgetLayer->setCurrentIndex(0); // RAM layer
        ui->sbValgetPosition->setValue(0);
        ui->leValgetKeys->setText("0x00000000");
    }
}

void GNSSWindow::setupMonVerFields() {
    ui->gbMonVerFields->setVisible(true);
    ui->tePayload->setVisible(false);

    if (m_firstInitialization) {
        ui->leSwVersion->setText("ROM CORE 3.01 (107888)");
        ui->leHwVersion->setText("00080000");
        ui->teExtensions->setPlainText("PROTVER=18.00\nGPS;GLO;GAL;BDS\nSBAS;IMES;QZSS");
    }
}

void GNSSWindow::setupMonHwFields() {
    ui->gbMonHwFields->setVisible(true);
    ui->tePayload->setVisible(false);

    if (m_firstInitialization) {
        ui->sbHwNoise->setValue(50);
        ui->sbHwAgc->setValue(75);
        ui->cbHwAntStatus->setCurrentIndex(2); // OK
        ui->cbHwAntPower->setCurrentIndex(1); // ON
        ui->cbHwJamming->setCurrentIndex(1); // OK
        ui->sbHwCwSuppression->setValue(0);
    }
}

void GNSSWindow::setupSecUniqidFields() {
    ui->gbSecUniqidFields->setVisible(true);
    ui->tePayload->setVisible(false);

    if (m_firstInitialization) {
        ui->sbUniqidVersion->setValue(1);
        ui->leChipId->setText("12345678");
    }
}

void GNSSWindow::setupNavTimeUtcFields() {
    ui->gbNavTimeUtcFields->setVisible(true);
    ui->tePayload->setVisible(false);

    if (m_firstInitialization) {
        ui->sbTimeUtcTAcc->setValue(100000);
        ui->sbTimeUtcNano->setValue(0);
        ui->cbTimeUtcValid->setCurrentIndex(2); // Valid UTC
        ui->cbTimeUtcStandard->setCurrentIndex(4); // BIPM
    }
}

void GNSSWindow::setupNavSatFields() {
    ui->gbNavSatFields->setVisible(true);
    ui->tePayload->setVisible(false);

    if (m_firstInitialization) {
    ui->sbSatVersion->setValue(1);
    ui->sbNumSatsSat->setValue(10);
    ui->cbQualityInd->setCurrentIndex(4);
    ui->cbHealth->setCurrentIndex(1);
    ui->dsbPrResMin->setValue(-2.0);
    ui->dsbPrResMax->setValue(2.0);
    ui->cbSvUsed->setChecked(true);
    ui->cbDiffCorr->setChecked(false);
    ui->cbSmoothed->setChecked(false);
    ui->cbOrbitSource->setCurrentIndex(1);
    }
}

void GNSSWindow::setupInfDebugFields() {
    ui->gbInfDebugFields->setVisible(true);
    ui->tePayload->setVisible(false);

    if (m_firstInitialization) {
    ui->teInfDebugMessage->setPlainText("Debug message");
    }
}

void GNSSWindow::setupInfErrorFields() {
    ui->gbInfErrorFields->setVisible(true);
    ui->tePayload->setVisible(false);

    if (m_firstInitialization) {
    ui->teInfErrorMessage->setPlainText("Error message");
    }
}

void GNSSWindow::setupInfWarningFields() {
    ui->gbInfWarningFields->setVisible(true);
    ui->tePayload->setVisible(false);

    if (m_firstInitialization) {
    ui->teInfWarningMessage->setPlainText("Warning message");
    }
}

void GNSSWindow::setupInfNoticeFields() {
    ui->gbInfNoticeFields->setVisible(true);
    ui->tePayload->setVisible(false);

    if (m_firstInitialization) {
    ui->teInfNoticeMessage->setPlainText("Notice message");
    }
}

void GNSSWindow::setupInfTestFields() {
    ui->gbInfTestFields->setVisible(true);
    ui->tePayload->setVisible(false);

    if (m_firstInitialization) {
    ui->teInfTestMessage->setPlainText("Test message");
    }
}

void GNSSWindow::setupCfgAntFields() {
    ui->gbCfgAntFields->setVisible(true);
    ui->tePayload->setVisible(false);

    if (m_firstInitialization) {
    ui->cbAntSupplyCtrl->setChecked(true);
    ui->cbAntShortDetect->setChecked(true);
    ui->cbAntOpenDetect->setChecked(true);
    ui->cbAntPowerDown->setChecked(true);
    ui->cbAntAutoRecover->setChecked(true);
    ui->sbAntSwitchPin->setValue(0);
    ui->sbAntShortPin->setValue(1);
    ui->sbAntOpenPin->setValue(2);
    ui->cbAntReconfig->setChecked(false);
    }
}

void GNSSWindow::setupCfgNav5Fields() {
    ui->gbCfgNav5Fields->setVisible(true);
    ui->tePayload->setVisible(false);

    if (m_firstInitialization) {
    ui->cbDynModel->setCurrentIndex(4);
    ui->cbFixMode->setCurrentIndex(2);
    ui->dsbFixedAlt->setValue(0.0);
    ui->sbMinElev->setValue(5);
    ui->dsbPDOP->setValue(2.5);
    ui->dsbTDOP->setValue(2.5);
    ui->dsbPAcc->setValue(0.0);
    ui->dsbTAcc->setValue(0.0);
    ui->dsbStaticHoldThresh->setValue(0.0);
    ui->sbDgnssTimeout->setValue(60);
    ui->sbCnoThresh->setValue(0);
    ui->sbCnoThreshNumSVs->setValue(0);
    ui->cbUtcStandard->setCurrentIndex(0);
    ui->dsbStaticHoldMaxDist->setValue(0.0);
    }
}

void GNSSWindow::setupCfgRateFields() {
    ui->gbCfgRateFields->setVisible(true);
    ui->tePayload->setVisible(false);

    if (m_firstInitialization) {
    ui->sbMeasRate->setValue(1000);  // 1 Hz
    ui->sbNavRate->setValue(1);
    ui->cbTimeRef->setCurrentIndex(0); // UTC time
    }
}

void GNSSWindow::sendUbxInfDebug() {
    if (!m_socket || m_socket->state() != QAbstractSocket::ConnectedState) {
        appendToLog("Error: No active connection to send INF-DEBUG", "error");
        return;
    }

    QString debugMessage = ui->teInfDebugMessage->toPlainText();
    QByteArray payload = debugMessage.toLatin1();

    createUbxPacket(UBX_CLASS_INF, UBX_INF_DEBUG, payload);
    appendToLog(QString("INF-DEBUG sent: %1").arg(debugMessage), "out");
}

void GNSSWindow::sendUbxInfError() {
    if (!m_socket || m_socket->state() != QAbstractSocket::ConnectedState) {
        appendToLog("Error: No active connection to send INF-ERROR", "error");
        return;
    }

    QString message = ui->teInfErrorMessage->toPlainText();
    QByteArray payload = message.toLatin1();

    createUbxPacket(UBX_CLASS_INF, UBX_INF_ERROR, payload);
    appendToLog(QString("INF-ERROR sent: %1").arg(message), "out");
}

void GNSSWindow::sendUbxInfNotice() {
    if (!m_socket || m_socket->state() != QAbstractSocket::ConnectedState) {
        appendToLog("Error: No active connection to send INF-NOTICE", "error");
        return;
    }

    QString message = ui->teInfNoticeMessage->toPlainText();
    QByteArray payload = message.toLatin1();

    createUbxPacket(UBX_CLASS_INF, UBX_INF_NOTICE, payload);
    appendToLog(QString("INF-NOTICE sent: %1").arg(message), "out");
}

void GNSSWindow::sendUbxInfTest() {
    if (!m_socket || m_socket->state() != QAbstractSocket::ConnectedState) {
        appendToLog("Error: No active connection to send INF-TEST", "error");
        return;
    }

    QString message = ui->teInfTestMessage->toPlainText();
    QByteArray payload = message.toLatin1();

    createUbxPacket(UBX_CLASS_INF, UBX_INF_TEST, payload);
    appendToLog(QString("INF-TEST sent: %1").arg(message), "out");
}

void GNSSWindow::sendUbxInfWarning() {
    if (!m_socket || m_socket->state() != QAbstractSocket::ConnectedState) {
        appendToLog("Error: No active connection to send INF-WARNING", "error");
        return;
    }

    QString message = ui->teInfWarningMessage->toPlainText();
    QByteArray payload = message.toLatin1();

    createUbxPacket(UBX_CLASS_INF, UBX_INF_WARNING, payload);
    appendToLog(QString("INF-WARNING sent: %1").arg(message), "out");
}

void GNSSWindow::sendUbxCfgRate() {
    if (!m_socket || m_socket->state() != QAbstractSocket::ConnectedState) {
        appendToLog("Error: No active connection to send CFG-RATE", "error");
        return;
    }

    quint16 measRate = static_cast<quint16>(ui->sbMeasRate->value());
    quint16 navRate = static_cast<quint16>(ui->sbNavRate->value());
    quint16 timeRef = static_cast<quint16>(ui->cbTimeRef->currentIndex());

    QByteArray payload(6, 0x00);

    qToLittleEndian<quint16>(measRate, payload.data());

    qToLittleEndian<quint16>(navRate, payload.data() + 2);

    qToLittleEndian<quint16>(timeRef, payload.data() + 4);

    createUbxPacket(UBX_CLASS_CFG, UBX_CFG_RATE, payload);
    appendToLog(QString("CFG-RATE sent: MeasRate=%1ms, NavRate=%2, TimeRef=%3")
                    .arg(measRate)
                    .arg(navRate)
                    .arg(timeRef), "config");
}

void GNSSWindow::onClassIdChanged()
{
    if (!m_fieldsInitialized) {
        initializeAllFields();
    }
    showFieldsForCurrentSelection();
}

void GNSSWindow::hideAllParameterFields() {
    ui->gbNavPvtFields->setVisible(false);
    ui->gbNavStatusFields->setVisible(false);
    ui->gbNavTimeUtcFields->setVisible(false);
    ui->gbMonRfFields->setVisible(false);
    ui->gbNavSatFields->setVisible(false);
    ui->gbCfgPrtFields->setVisible(false);
    ui->gbMonVerFields->setVisible(false);
    ui->gbCfgRateFields->setVisible(false);
    ui->gbMonHwFields->setVisible(false);
    ui->gbCfgValsetFields->setVisible(false);
    ui->gbSecUniqidFields->setVisible(false);
    ui->gbCfgAntFields->setVisible(false);
    ui->gbCfgItfmFields->setVisible(false);
    ui->gbInfDebugFields->setVisible(false);
    ui->gbInfErrorFields->setVisible(false);
    ui->gbInfWarningFields->setVisible(false);
    ui->gbInfNoticeFields->setVisible(false);
    ui->gbInfTestFields->setVisible(false);
    ui->gbCfgValgetFields->setVisible(false);
    ui->gbCfgNav5Fields->setVisible(false);
    ui->tePayload->setVisible(false);
}

void GNSSWindow::initializeAllFields()
{
    if (m_fieldsInitialized) return;

    setupNavPvtFields();
    setupNavStatusFields();
    setupNavSatFields();
    setupNavTimeUtcFields();
    setupMonVerFields();
    setupMonHwFields();
    setupMonRfFields();
    setupCfgPrtFields();
    setupCfgItfmFields();
    setupCfgNav5Fields();
    setupCfgRateFields();
    setupCfgValgetFields();
    setupCfgValsetFields();
    setupCfgAntFields();
    setupInfDebugFields();
    setupInfErrorFields();
    setupInfWarningFields();
    setupInfNoticeFields();
    setupInfTestFields();
    setupSecUniqidFields();

    m_fieldsInitialized = true;
}

void GNSSWindow::showFieldsForCurrentSelection()
{
    hideAllParameterFields();

    int classId = ui->cbClass->currentData().toInt();
    int msgId = ui->cbId->currentData().toInt();

    QWidget* groupToShow = nullptr;
    bool showPayloadEditor = false;

    switch(classId) {
    case UBX_CLASS_NAV:
        switch(msgId) {
        case UBX_NAV_PVT:
            groupToShow = ui->gbNavPvtFields;
            break;
        case UBX_NAV_STATUS:
            groupToShow = ui->gbNavStatusFields;
            break;
        case UBX_NAV_SAT:
            groupToShow = ui->gbNavSatFields;
            break;
        case UBX_NAV_TIMEUTC:
            groupToShow = ui->gbNavTimeUtcFields;
            break;
        default:
            showPayloadEditor = true;
            break;
        }
        break;

    case UBX_CLASS_MON:
        switch(msgId) {
        case UBX_MON_VER:
            groupToShow = ui->gbMonVerFields;
            break;
        case UBX_MON_HW:
            groupToShow = ui->gbMonHwFields;
            break;
        case UBX_MON_RF:
            groupToShow = ui->gbMonRfFields;
            break;
        default:
            showPayloadEditor = true;
            break;
        }
        break;

    case UBX_CLASS_CFG:
        switch(msgId) {
        case UBX_CFG_PRT:
            groupToShow = ui->gbCfgPrtFields;
            break;
        case UBX_CFG_ITFM:
            groupToShow = ui->gbCfgItfmFields;
            break;
        case UBX_CFG_NAV5:
            groupToShow = ui->gbCfgNav5Fields;
            break;
        case UBX_CFG_RATE:
            groupToShow = ui->gbCfgRateFields;
            break;
        case UBX_CFG_VALGET:
            groupToShow = ui->gbCfgValgetFields;
            break;
        case UBX_CFG_VALSET:
            groupToShow = ui->gbCfgValsetFields;
            break;
        case UBX_CFG_ANT:
            groupToShow = ui->gbCfgAntFields;
            break;
        default:
            showPayloadEditor = true;
            break;
        }
        break;

    case UBX_CLASS_INF:
        switch(msgId) {
        case UBX_INF_DEBUG:
            groupToShow = ui->gbInfDebugFields;
            break;
        case UBX_INF_ERROR:
            groupToShow = ui->gbInfErrorFields;
            break;
        case UBX_INF_WARNING:
            groupToShow = ui->gbInfWarningFields;
            break;
        case UBX_INF_NOTICE:
            groupToShow = ui->gbInfNoticeFields;
            break;
        case UBX_INF_TEST:
            groupToShow = ui->gbInfTestFields;
            break;
        default:
            showPayloadEditor = true;
            break;
        }
        break;

    case UBX_CLASS_SEC:
        if (msgId == UBX_SEC_UNIQID) {
            groupToShow = ui->gbSecUniqidFields;
        } else {
            showPayloadEditor = true;
        }
        break;

    default:
        showPayloadEditor = true;
        break;
    }

    if (groupToShow) {
        groupToShow->setVisible(true);
    } else if (showPayloadEditor) {
        ui->tePayload->setVisible(true);
    }

    QString statusMsg = QString("Showing fields for: %1-%2")
                            .arg(ui->cbClass->currentText())
                            .arg(ui->cbId->currentText());
    appendToLog(statusMsg, "debug");
}

void GNSSWindow::setupConnections()
{
    // Timer connections
    m_timer = new QTimer(this);
    connect(m_timer, &QTimer::timeout, this, &GNSSWindow::sendUbxNavPvt);
    connect(m_pvtTimer, &QTimer::timeout, this, &GNSSWindow::sendUbxNavPvt);
    connect(m_statusTimer, &QTimer::timeout, this, &GNSSWindow::sendUbxNavStatus);
    connect(m_initTimer, &QTimer::timeout, this, &GNSSWindow::handleInitTimeout);
    connect(m_ackTimeoutTimer, &QTimer::timeout, this, &GNSSWindow::handleAckTimeout);
    connect(m_utcTimer, &QTimer::timeout, this, &GNSSWindow::updateUTCTime);

    // Menu connections
    connect(ui->actionSaveSettings, &QAction::triggered,
            this, &GNSSWindow::onActionSaveSettingsTriggered);
    connect(ui->actionLoadSettings, &QAction::triggered,
            this, &GNSSWindow::onActionLoadSettingsTriggered);
    connect(ui->actionSaveLog, &QAction::triggered,
            this, &GNSSWindow::onActionSaveLogTriggered);
    connect(ui->actionClearLog, &QAction::triggered,
            this, &GNSSWindow::onActionClearLogTriggered);
    connect(ui->actionAbout, &QAction::triggered,
            this, &GNSSWindow::onActionAboutTriggered);
    connect(ui->actionExit, &QAction::triggered,
            this, &QMainWindow::close);

    // UI
    connect(ui->cbClass, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &GNSSWindow::updateAvailableIds);
    connect(ui->cbId, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &GNSSWindow::onClassIdChanged);
    connect(ui->sendButton, &QPushButton::clicked,
            this, &GNSSWindow::onSendButtonClicked);
    connect(ui->autoSendCheck, &QCheckBox::toggled,
            this, &GNSSWindow::onAutoSendToggled);
    connect(ui->btnClearLog, &QPushButton::clicked,
            this, &GNSSWindow::on_btnClearLog_clicked);

    // Autosend
    connect(ui->cbAutoSendNavPvt, &QCheckBox::toggled, this, &GNSSWindow::onAutoSendNavPvtToggled);
    connect(ui->cbAutoSendNavStatus, &QCheckBox::toggled, this, &GNSSWindow::onAutoSendNavStatusToggled);
    connect(ui->cbAutoSendNavSat, &QCheckBox::toggled, this, &GNSSWindow::onAutoSendNavSatToggled);
    connect(ui->cbAutoSendNavTimeUTC, &QCheckBox::toggled, this, &GNSSWindow::onAutoSendNavTimeUtcToggled);
    connect(ui->cbAutoSendMonVer, &QCheckBox::toggled, this, &GNSSWindow::onAutoSendMonVerToggled);
    connect(ui->cbAutoSendMonHw, &QCheckBox::toggled, this, &GNSSWindow::onAutoSendMonHwToggled);
    connect(ui->cbAutoSendMonRf, &QCheckBox::toggled, this, &GNSSWindow::onAutoSendMonRfToggled);
    connect(ui->cbAutoSendCfgPrt, &QCheckBox::toggled, this, &GNSSWindow::onAutoSendCfgPrtToggled);
    connect(ui->cbAutoSendCfgItfm, &QCheckBox::toggled, this, &GNSSWindow::onAutoSendCfgItfmToggled);
    connect(ui->cbAutoSendCfgNav5, &QCheckBox::toggled, this, &GNSSWindow::onAutoSendCfgNav5Toggled);
    connect(ui->cbAutoSendCfgRate, &QCheckBox::toggled, this, &GNSSWindow::onAutoSendCfgRateToggled);
    connect(ui->cbAutoSendCfgValget, &QCheckBox::toggled, this, &GNSSWindow::onAutoSendCfgValgetToggled);
    connect(ui->cbAutoSendCfgValset, &QCheckBox::toggled, this, &GNSSWindow::onAutoSendCfgValsetToggled);
    connect(ui->cbAutoSendCfgAnt, &QCheckBox::toggled, this, &GNSSWindow::onAutoSendCfgAntToggled);
    connect(ui->cbAutoSendInfDebug, &QCheckBox::toggled, this, &GNSSWindow::onAutoSendInfDebugToggled);
    connect(ui->cbAutoSendInfError, &QCheckBox::toggled, this, &GNSSWindow::onAutoSendInfErrorToggled);
    connect(ui->cbAutoSendInfWarning, &QCheckBox::toggled, this, &GNSSWindow::onAutoSendInfWarningToggled);
    connect(ui->cbAutoSendInfNotice, &QCheckBox::toggled, this, &GNSSWindow::onAutoSendInfNoticeToggled);
    connect(ui->cbAutoSendInfTest, &QCheckBox::toggled, this, &GNSSWindow::onAutoSendInfTestToggled);
    connect(ui->cbAutoSendSecUniqid, &QCheckBox::toggled, this, &GNSSWindow::onAutoSendSecUniqidToggled);

    // UBX parsers
    registerHandlers();

    if (m_parentDialog) {
        connect(m_parentDialog, &Dialog::logMessage,
                this, &GNSSWindow::appendToLog);
        connect(m_parentDialog, &Dialog::connectionStatusChanged,
                this, &GNSSWindow::onConnectionStatusChanged);
    }
}

void GNSSWindow::stopAllAutoSendTimers()
{
    if (m_pvtTimer && m_pvtTimer->isActive()) {
        m_pvtTimer->stop();
    }
    if (m_statusTimer && m_statusTimer->isActive()) {
        m_statusTimer->stop();
    }

    QList<QTimer*> allTimers = findChildren<QTimer*>();
    foreach (QTimer* timer, allTimers) {
        if (timer->isActive() && timer != m_utcTimer) {
            timer->stop();
        }
    }


    QList<QCheckBox*> autoSendCheckBoxes = findChildren<QCheckBox*>(QRegularExpression("cbAutoSend.*"));
    foreach (QCheckBox* checkBox, autoSendCheckBoxes) {
        checkBox->setChecked(false);
    }
}

void GNSSWindow::onActionSaveSettingsTriggered()
{
    QString fileName = QFileDialog::getSaveFileName(this,
                                                    "Save Settings",
                                                    "",
                                                    "JSON Files (*.json);;All Files (*)");

    if (!fileName.isEmpty()) {
        if (!fileName.endsWith(".json", Qt::CaseInsensitive)) {
            fileName += ".json";
        }
        saveSettings(fileName);
    }
}

void GNSSWindow::onActionLoadSettingsTriggered()
{
    QString fileName = QFileDialog::getOpenFileName(
        this,
        tr("Load Settings"),
        QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation),
        tr("JSON Settings (*.json);;All Files (*)")
        );

    if (fileName.isEmpty()) {
        return;
    }

    QJsonObject backupSettings = getCurrentSettings();

    try {
        stopAllAutoSendTimers();

        QFile file(fileName);
        if (!file.open(QIODevice::ReadOnly)) {
            throw std::runtime_error(tr("Could not open settings file").toStdString());
        }

        QJsonParseError parseError;
        QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &parseError);
        file.close();

        if (parseError.error != QJsonParseError::NoError) {
            throw std::runtime_error(tr("Invalid JSON format: %1").arg(parseError.errorString()).toStdString());
        }

        if (doc.isNull() || !doc.isObject()) {
            throw std::runtime_error(tr("Invalid settings file format").toStdString());
        }

        m_settingsLoaded = true;
        m_firstInitialization = false;

        applySettings(doc.object());

        if (ui->autoSendCheck->isChecked()) {
            int rate = ui->rateSpin->value();
            if (rate > 0) {
                m_pvtTimer->start(1000 / rate);
                m_statusTimer->start(1000 / rate);
            }
        }

        appendToLog(tr("Settings successfully loaded from %1").arg(fileName), "system");
        ui->statusbar->showMessage(tr("Settings loaded successfully"), 3000);

    } catch (const std::exception& e) {
        m_settingsLoaded = false;
        applySettings(backupSettings);

        QMessageBox::warning(this, tr("Error"), tr(e.what()));
        appendToLog(tr("Failed to load settings: %1").arg(e.what()), "error");

        if (ui->autoSendCheck->isChecked()) {
            int rate = ui->rateSpin->value();
            if (rate > 0) {
                m_pvtTimer->start(1000 / rate);
                m_statusTimer->start(1000 / rate);
            }
        }
    }
}

void GNSSWindow::sendInitialConfiguration() {
    if (m_settingsLoaded || !m_socket || m_socket->state() != QAbstractSocket::ConnectedState) {
        return;
    }

    if (m_waitingForAck) {
        appendToLog("Configuration already in progress", "warning");
        return;
    }

    appendToLog("Starting initial configuration", "system");

    m_waitingForAck = true;
    m_ackTimeoutTimer->start();
    m_initTimer->start(20000);

    // Main configuration
    sendUbxCfgPrtResponse();
    sendUbxCfgMsg(UBX_CLASS_NAV, UBX_NAV_PVT, 1);
    sendUbxCfgMsg(UBX_CLASS_NAV, UBX_NAV_STATUS, 1);
    sendUbxCfgMsg(UBX_CLASS_MON, UBX_MON_RF, 1);
    sendUbxCfgRate();
    sendUbxCfgNav5();
    sendUbxCfgAnt();
    sendUbxCfgItfm();

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

    // Read all available data
    QByteArray newData = m_socket->readAll();
    if (newData.isEmpty()) {
        qWarning() << "Received empty data packet";
        return;
    }

    qDebug() << "Received" << newData.size() << "bytes of raw data:" << newData.toHex(' ');

    // Add data to member buffer
    m_receiveBuffer.append(newData);

    qDebug() << "Total buffer size:" << m_receiveBuffer.size() << "bytes";
    appendToLog(QString("Received %1 bytes (total buffer: %2)")
                    .arg(newData.size())
                    .arg(m_receiveBuffer.size()), "in");

    // Process all complete messages in buffer
    while (m_receiveBuffer.size() >= 8) { // Minimum UBX message size (without payload)
        // 3.1. Find sync bytes
        int startPos = m_receiveBuffer.indexOf("\xB5\x62");
        if (startPos < 0) {
            qDebug() << "No UBX sync chars found, clearing buffer";
            m_receiveBuffer.clear();
            return;
        }

        // Remove garbage before sync bytes
        if (startPos > 0) {
            qDebug() << "Discarding" << startPos << "bytes before sync chars";
            m_receiveBuffer.remove(0, startPos);
            continue;
        }

        if (m_receiveBuffer.size() < 8) {
            qDebug() << "Waiting for more data (header incomplete)";
            return;
        }

        // Extract payload length
        quint16 length = static_cast<quint8>(m_receiveBuffer[4]) |
                         (static_cast<quint8>(m_receiveBuffer[5]) << 8);
        quint8 msgClass = static_cast<quint8>(m_receiveBuffer[2]);
        quint8 msgId = static_cast<quint8>(m_receiveBuffer[3]);

        // Check if we have complete message
        int totalMessageSize = 8 + length; // Header + payload
        if (m_receiveBuffer.size() < totalMessageSize) {
            qDebug() << "Waiting for more data. Need:" << totalMessageSize
                     << "Have:" << m_receiveBuffer.size();
            return;
        }

        // Extract complete message
        QByteArray message = m_receiveBuffer.left(totalMessageSize);
        m_receiveBuffer.remove(0, totalMessageSize);

        // Parse and process message
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
    connect(&m_ubxParser, &UbxParser::navPvtReceived, this, &GNSSWindow::displayNavPvt);
    connect(&m_ubxParser, &UbxParser::navStatusReceived, this, &GNSSWindow::displayNavStatus);

    connect(&m_ubxParser, &UbxParser::monVerReceived, this, &GNSSWindow::displayMonVer);
    connect(&m_ubxParser, &UbxParser::monHwReceived, this, &GNSSWindow::processMonHw);
    connect(&m_ubxParser, &UbxParser::monRfReceived, this, &GNSSWindow::displayMonRf);

    connect(&m_ubxParser, &UbxParser::cfgPrtReceived, this, &GNSSWindow::displayCfgPrt);

    connect(&m_ubxParser, &UbxParser::infErrorReceived, this, [this](const QString& msg) {
        appendToLog("INF-ERROR: " + msg, "error");
        QMessageBox::warning(this, "Receiver Error", msg);
    });
}

void GNSSWindow::sendUbxSecUniqidReq() {
    createUbxPacket(UBX_CLASS_SEC, UBX_SEC_UNIQID, QByteArray());
    appendToLog("Requested SEC-UNIQID", "config");
}

void GNSSWindow::sendUbxCfgItfm() {
    if (!m_socket || m_socket->state() != QAbstractSocket::ConnectedState) {
        appendToLog("Error: No active connection to send CFG-ITFM", "error");
        return;
    }

    QByteArray payload(8, 0x00);

    // Forms config word with UI elements
    quint32 config = 0;
    config |= (ui->sbBbThreshold->value() & 0x0F);        // BB Threshold (0-3)
    config |= (ui->sbCwThreshold->value() & 0x1F) << 4;   // CW Threshold (4-8)
    config |= 0x16B156 << 9;                             // Algoritmic bytes
    if (ui->cbEnable->isChecked()) {
        config |= 0x80000000;                            // Turn On byte (31)
    }
    qToLittleEndian<quint32>(config, payload.data());

    // Forms config2 word with UI elements
    quint32 config2 = 0;
    config2 |= 0x31E;                                    // Fixed general bytes
    config2 |= (ui->cbAntSetting->currentIndex() & 0x03) << 12;
    if (ui->cbEnable2->isChecked()) {
        config2 |= 0x00004000;                           // Aux band
    }
    qToLittleEndian<quint32>(config2, payload.data() + 4);

    createUbxPacket(UBX_CLASS_CFG, UBX_CFG_ITFM, payload);
    appendToLog(QString("CFG-ITFM sent: BB=%1, CW=%2, Enable=%3")
                    .arg(ui->sbBbThreshold->value())
                    .arg(ui->sbCwThreshold->value())
                    .arg(ui->cbEnable->isChecked() ? "ON" : "OFF"), "out");
}

void GNSSWindow::sendUbxCfgValset() {
    if (!m_socket || m_socket->state() != QAbstractSocket::ConnectedState) {
        appendToLog("Error: No active connection to send CFG-VALSET", "error");
        return;
    }

    // Gets UI parametrs
    quint8 version = static_cast<quint8>(ui->sbValsetVersion->value());
    quint8 layers = 0;
    if (ui->cbValsetRam->isChecked()) layers |= 0x01;
    if (ui->cbValsetBbr->isChecked()) layers |= 0x02;
    if (ui->cbValsetFlash->isChecked()) layers |= 0x04;

    QString kvPairsStr = ui->leValsetKeysValues->text();

    // Parse key=values
    QList<QPair<quint32, quint32>> kvPairs;
    QStringList pairs = kvPairsStr.split(',', Qt::SkipEmptyParts);
    for (const QString &pair : pairs) {
        QStringList kv = pair.split('=', Qt::SkipEmptyParts);
        if (kv.size() == 2) {
            bool keyOk, valueOk;
            quint32 key = kv[0].trimmed().toUInt(&keyOk, 16);
            quint32 value = kv[1].trimmed().toUInt(&valueOk, 16);
            if (keyOk && valueOk) {
                kvPairs.append(qMakePair(key, value));
            }
        }
    }

    if (kvPairs.isEmpty()) {
        appendToLog("Error: No valid key-value pairs provided for CFG-VALSET", "error");
        return;
    }

    // Forming payload
    QByteArray payload;
    payload.append(static_cast<char>(version));
    payload.append(static_cast<char>(layers));
    payload.append('\0'); // reserved1
    payload.append('\0'); // reserved2

    // Add key-values
    for (const auto &kv : kvPairs) {
        quint32 key = kv.first;
        quint32 value = kv.second;

        payload.append(static_cast<char>(key & 0xFF));
        payload.append(static_cast<char>((key >> 8) & 0xFF));
        payload.append(static_cast<char>((key >> 16) & 0xFF));
        payload.append(static_cast<char>((key >> 24) & 0xFF));

        payload.append(static_cast<char>(value & 0xFF));
        payload.append(static_cast<char>((value >> 8) & 0xFF));
        payload.append(static_cast<char>((value >> 16) & 0xFF));
        payload.append(static_cast<char>((value >> 24) & 0xFF));
    }

    createUbxPacket(UBX_CLASS_CFG, UBX_CFG_VALSET, payload);
    appendToLog(QString("CFG-VALSET sent: Version=%1, Layers=0x%2, %3 key-value pairs")
                    .arg(version)
                    .arg(layers, 2, 16, QLatin1Char('0'))
                    .arg(kvPairs.size()), "config");
}

void GNSSWindow::sendUbxCfgValGet() {
    if (!m_socket || m_socket->state() != QAbstractSocket::ConnectedState) {
        appendToLog("Error: No active connection to send CFG-VALGET", "error");
        return;
    }

    // Gets parameters from UI
    quint8 version = static_cast<quint8>(ui->sbValgetVersion->value());
    quint8 layer = static_cast<quint8>(ui->cbValgetLayer->currentIndex());
    quint16 position = static_cast<quint16>(ui->sbValgetPosition->value());
    QString keysStr = ui->leValgetKeys->text();

    // Parse keys (format: "0x12345678,0x87654321")
    QList<quint32> keys;
    QStringList keyList = keysStr.split(',', Qt::SkipEmptyParts);
    for (const QString &keyStr : keyList) {
        bool ok;
        quint32 key = keyStr.trimmed().toUInt(&ok, 16);
        if (ok) {
            keys.append(key);
        }
    }

    if (keys.isEmpty()) {
        appendToLog("Error: No valid keys provided for CFG-VALGET", "error");
        return;
    }

    // Forming payload (version + lauer + position + keys)
    QByteArray payload;
    payload.append(static_cast<char>(version));
    payload.append(static_cast<char>(layer));
    payload.append(static_cast<char>(position & 0xFF));
    payload.append(static_cast<char>((position >> 8) & 0xFF));

    for (quint32 key : keys) {
        payload.append(static_cast<char>(key & 0xFF));
        payload.append(static_cast<char>((key >> 8) & 0xFF));
        payload.append(static_cast<char>((key >> 16) & 0xFF));
        payload.append(static_cast<char>((key >> 24) & 0xFF));
    }

    createUbxPacket(UBX_CLASS_CFG, UBX_CFG_VALGET, payload);
    appendToLog(QString("CFG-VALGET sent: Version=%1, Layer=%2, Position=%3, Keys=%4")
                    .arg(version)
                    .arg(layer)
                    .arg(position)
                    .arg(keysStr), "config");
}

void GNSSWindow::sendUbxNavTimeUtc() {
    QByteArray payload(20, 0x00);
    QDateTime currentTime = QDateTime::currentDateTimeUtc();

    // iTOW
    quint32 iTOW = static_cast<quint32>(currentTime.toMSecsSinceEpoch() % (7 * 24 * 60 * 60 * 1000));
    qToLittleEndian<quint32>(iTOW, payload.data());

    // tAcc
    quint32 tAcc = static_cast<quint32>(ui->sbTimeUtcTAcc->value());
    qToLittleEndian<quint32>(tAcc, payload.data() + 4);

    // nano
    qint32 nano = static_cast<qint32>(ui->sbTimeUtcNano->value());
    qToLittleEndian<qint32>(nano, payload.data() + 8);

    // Date and time UTC
    qToLittleEndian<quint16>(currentTime.date().year(), payload.data() + 12);
    payload[14] = static_cast<quint8>(currentTime.date().month());
    payload[15] = static_cast<quint8>(currentTime.date().day());
    payload[16] = static_cast<quint8>(currentTime.time().hour());
    payload[17] = static_cast<quint8>(currentTime.time().minute());
    payload[18] = static_cast<quint8>(currentTime.time().second());

    // Valid flags
    quint8 validFlags = 0;
    switch(ui->cbTimeUtcValid->currentIndex()) {
    case 0: validFlags |= 0x01; break; // Valid TOW
    case 1: validFlags |= 0x02; break; // Valid WKN
    case 2: validFlags |= 0x04; break; // Valid UTC
    case 3: validFlags |= 0x08; break; // Authenticated
    }

    // UTC standart (4)
    quint8 utcStandard = static_cast<quint8>(ui->cbTimeUtcStandard->currentIndex());
    validFlags |= (utcStandard << 4);

    // Write flags
    payload[19] = validFlags;

    createUbxPacket(UBX_CLASS_NAV, UBX_NAV_TIMEUTC, payload);
    appendToLog("Sent NAV-TIMEUTC", "out");
}

void GNSSWindow::processCfgValGet(const QByteArray& payload) {
    if (payload.size() < 4) {
        appendToLog("CFG-VALGET response too short", "error");
        return;
    }

    quint8 version = static_cast<quint8>(payload[0]);
    quint8 layer = static_cast<quint8>(payload[1]);

    QString message = QString("CFG-VALGET response: Version=%1, Layer=%2").arg(version).arg(layer);

    // Processing key-value (evry 8 bytes: 4 bytes key, 4 bytes value)
    for (int i = 4; i + 7 < payload.size(); i += 8) {
        quint32 key = qFromLittleEndian<quint32>(payload.mid(i, 4).constData());
        quint32 value = qFromLittleEndian<quint32>(payload.mid(i + 4, 4).constData());

        message += QString("\nKey: 0x%1, Value: 0x%2").arg(key, 8, 16, QLatin1Char('0'))
                       .arg(value, 8, 16, QLatin1Char('0'));
    }

    appendToLog(message, "in");
}

void GNSSWindow::processCfgValSet(const QByteArray &payload) {
    sendUbxAck(UBX_CLASS_CFG, UBX_CFG_VALSET);
    appendToLog("CFG-VALSET processed", "config");
}

void GNSSWindow::sendUbxMonRf() {
    if (!m_socket || m_socket->state() != QAbstractSocket::ConnectedState) {
        appendToLog("Error: No active connection to send MON-RF", "error");
        return;
    }

    // Processing data
    const int numBlocks = ui->sbRfBlocks->value();
    const int payloadSize = 4 + 24 * numBlocks;
    QByteArray payload(payloadSize, 0x00);

    appendToLog(QString("Preparing MON-RF message with %1 blocks (%2 bytes total)")
                    .arg(numBlocks).arg(payloadSize), "debug");

    // Header
    payload[0] = static_cast<quint8>(ui->sbRfVersion->value());
    payload[1] = static_cast<quint8>(numBlocks);

    appendToLog(QString("Header: version=%1, nBlocks=%2")
                    .arg(payload[0]).arg(payload[1]), "debug");

    // Write each RF blok
    for (int i = 0; i < numBlocks; i++) {
        int offset = 4 + i * 24;

        // base parameters
        payload[offset] = static_cast<quint8>(i); // blockId
        payload[offset+1] = static_cast<quint8>(ui->cbRfJamState->currentIndex()); // flags
        payload[offset+2] = static_cast<quint8>(ui->cbRfAntStatus->currentIndex()); // antStatus
        payload[offset+3] = static_cast<quint8>(ui->cbRfAntPower->currentIndex()); // antPower

        // POST status (0 = no errors)
        qToLittleEndian<quint32>(0x00000000, payload.data() + offset + 4);

        // Noise and AGC (scaled values)
        quint16 noisePerMS = static_cast<quint16>(ui->dsbRfNoise->value() * 100);
        quint16 agcCnt = static_cast<quint16>(ui->dsbRfAgc->value() * 100);
        qToLittleEndian<quint16>(noisePerMS, payload.data() + offset + 12);
        qToLittleEndian<quint16>(agcCnt, payload.data() + offset + 14);

        // CW suppression
        payload[offset+16] = static_cast<quint8>(ui->sbRfCwSuppression->value());

        // I/Q parameters
        payload[offset+17] = static_cast<qint8>(0);    // ofsI
        payload[offset+18] = static_cast<quint8>(128); // magI
        payload[offset+19] = static_cast<qint8>(0);    // ofsQ
        payload[offset+20] = static_cast<quint8>(128); // magQ

        // Log blocks
        appendToLog(QString("Block %1: antId=%2, jamState=%3, antStatus=%4, antPower=%5, noise=%6, agc=%7")
                        .arg(i)
                        .arg(payload[offset])
                        .arg(payload[offset+1])
                        .arg(payload[offset+2])
                        .arg(payload[offset+3])
                        .arg(noisePerMS)
                        .arg(agcCnt), "debug");
    }

    // Log payload before sending
    appendToLog("MON-RF payload: " + payload.toHex(' '), "debug");

    // Create and sending packet
    createUbxPacket(UBX_CLASS_MON, UBX_MON_RF, payload);
    appendToLog(QString("MON-RF message sent (%1 bytes)").arg(payloadSize + 8), "out");
}

void GNSSWindow::sendUbxSecUniqid() {
    if (!m_socket || m_socket->state() != QAbstractSocket::ConnectedState) {
        appendToLog("Error: No active connection to send SEC-UNIQID", "error");
        return;
    }

    QByteArray payload(9, 0x00);

    payload[0] = static_cast<quint8>(ui->sbUniqidVersion->value());

    QString chipIdStr = ui->leChipId->text();
    bool ok;
    quint32 chipId = chipIdStr.toUInt(&ok, 16);
    if (!ok) {
        appendToLog("Invalid Chip ID format (must be hex)", "error");
        return;
    }

    for (int i = 0; i < 5; i++) {
        payload[4 + i] = static_cast<quint8>((chipId >> (8 * (4 - i))) & 0xFF);
    }

    createUbxPacket(UBX_CLASS_SEC, UBX_SEC_UNIQID, payload);
    appendToLog(QString("SEC-UNIQID sent: Version=%1, ChipID=0x%2")
                    .arg(ui->sbUniqidVersion->value())
                    .arg(chipIdStr), "out");
}

void GNSSWindow::sendUbxCfgMsg(quint8 msgClass, quint8 msgId, quint8 rate) {
    bool useLegacyMethod = true;

    if (m_protocolVersion > 23.01f) {
        useLegacyMethod = false;
    }

    if (!useLegacyMethod) {
        // Use UBX-CFG-VALSET for modern receivers
        QByteArray payload;
        payload.append(0x01); // Version 1
        payload.append(0x07); // RAM + BBR + Flash layers
        payload.append('\0'); // Reserved
        payload.append('\0'); // Reserved

        // Key for message rate configuration
        quint32 key = 0x20910000 | (msgClass << 8) | msgId;
        payload.append(static_cast<char>(key & 0xFF));
        payload.append(static_cast<char>((key >> 8) & 0xFF));
        payload.append(static_cast<char>((key >> 16) & 0xFF));
        payload.append(static_cast<char>((key >> 24) & 0xFF));

        // Value (rate)
        payload.append(static_cast<char>(rate));
        payload.append('\0'); // Padding
        payload.append('\0'); // Padding
        payload.append('\0'); // Padding

        createUbxPacket(UBX_CLASS_CFG, UBX_CFG_VALSET, payload); // UBX-CFG-VALSET
        appendToLog(QString("CFG-VALSET for MSG: Class=0x%1 ID=0x%2 Rate=%3")
                        .arg(msgClass, 2, 16, QLatin1Char('0'))
                        .arg(msgId, 2, 16, QLatin1Char('0'))
                        .arg(rate), "config");
    } else {
        // Legacy method for older receivers
        QByteArray payload;
        payload.append(static_cast<char>(msgClass));
        payload.append(static_cast<char>(msgId));
        payload.append(static_cast<char>(rate));

        createUbxPacket(UBX_CLASS_CFG, UBX_CFG_MSG, payload);
        appendToLog(QString("CFG-MSG: Class=0x%1 ID=0x%2 Rate=%3")
                        .arg(msgClass, 2, 16, QLatin1Char('0'))
                        .arg(msgId, 2, 16, QLatin1Char('0'))
                        .arg(rate), "config");
    }
}

void GNSSWindow::processUbxMessage(quint8 msgClass, quint8 msgId, const QByteArray& payload) {
    QString messageInfo;
    QString timestamp = QDateTime::currentDateTime().toString("[hh:mm:ss.zzz]");

    // Log incoming message
    qDebug().nospace() << timestamp << " Processing UBX message: "
                       << "Class=0x" << QString("%1").arg(msgClass, 2, 16, QLatin1Char('0')).toUpper()
                       << " ID=0x" << QString("%1").arg(msgId, 2, 16, QLatin1Char('0')).toUpper()
                       << " Size=" << payload.size() << " bytes";

    if (msgClass == UBX_CLASS_ACK) {
        if (payload.size() >= 2) {
            quint8 ackedClass = static_cast<quint8>(payload[0]);
            quint8 ackedId = static_cast<quint8>(payload[1]);

            if (msgId == UBX_ACK_ACK) {
                appendToLog(QString("ACK received for Class=0x%1 ID=0x%2")
                                .arg(ackedClass, 2, 16, QLatin1Char('0'))
                                .arg(ackedId, 2, 16, QLatin1Char('0')),
                            "in");

                // Handle configuration completion if this was for a CFG message
                if (ackedClass == UBX_CLASS_CFG) {
                    completeInitialization();
                }
            }
            else if (msgId == UBX_ACK_NAK) {
                appendToLog(QString("NACK received for Class=0x%1 ID=0x%2")
                                .arg(ackedClass, 2, 16, QLatin1Char('0'))
                                .arg(ackedId, 2, 16, QLatin1Char('0')),
                            "error");
            }
        }
        return;
    }

    // Process CFG messages
    if (msgClass == UBX_CLASS_CFG) {
        processCfgMessages(msgId, payload);
        return;
    }

    // Process info requests
    if (processInfoRequests(msgClass, msgId)) {
        return;
    }

    // Process main messages by class
    switch (msgClass) {
    case UBX_CLASS_NAV: messageInfo = processNavMessages(msgId, payload); break;
    case UBX_CLASS_INF: processInfMessages(msgId, payload); return;
    case UBX_CLASS_MON: messageInfo = processMonMessages(msgId, payload); break;
    case UBX_CLASS_SEC: messageInfo = processSecMessages(msgId, payload); break;
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
    if (payload.size() < 2) {
        appendToLog("Invalid ACK/NAK payload size", "error");
        return;
    }

    quint8 ackedClass = static_cast<quint8>(payload[0]);
    quint8 ackedId = static_cast<quint8>(payload[1]);

    if (msgId == UBX_ACK_ACK) {
        appendToLog(QString("ACK received for %1 (0x%2) ID: 0x%3")
                        .arg(getMessageName(ackedClass, ackedId))
                        .arg(ackedClass, 2, 16, QLatin1Char('0'))
                        .arg(ackedId, 2, 16, QLatin1Char('0')),
                    "in");
    }
    else if (msgId == UBX_ACK_NAK) {
        appendToLog(QString("NACK received for %1 (0x%2) ID: 0x%3")
                        .arg(getMessageName(ackedClass, ackedId))
                        .arg(ackedClass, 2, 16, QLatin1Char('0'))
                        .arg(ackedId, 2, 16, QLatin1Char('0')),
                    "error");
    }
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
    case UBX_CFG_PRT:
        sendUbxCfgPrtResponse();
        sendInitialConfiguration();
        return;
    case UBX_CFG_VALSET:
        processCfgValGet(payload);
        break;
    case UBX_CFG_VALGET:
        processCfgValGet(payload);
        break;
    case UBX_CFG_ITFM:
        sendUbxAck(UBX_CLASS_CFG, UBX_CFG_ITFM);
        sendUbxCfgItfm();
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
    if (msgClass == UBX_CLASS_MON && msgId == UBX_MON_VER) {
        sendUbxMonVer();
        return true;
    }
    if (msgClass == UBX_CLASS_MON && msgId == UBX_MON_HW) {
        sendUbxMonHw();
        return true;
    }
    if (msgClass == UBX_CLASS_SEC && msgId == UBX_SEC_UNIQID) {
        sendUbxSecUniqid();
        return true;
    }
    return false;
}

QString GNSSWindow::processNavMessages(quint8 msgId, const QByteArray& payload) {
    switch (msgId) {
    case UBX_NAV_PVT: {
        UbxParser::NavPvt pvt = UbxParser::parseNavPvt(payload);
        displayNavPvt(pvt);
        return QString("NAV-PVT: Lat=%1 Lon=%2 Fix=%3 Sats=%4")
            .arg(pvt.lat/1e7, 0, 'f', 7)
            .arg(pvt.lon/1e7, 0, 'f', 7)
            .arg(pvt.fixType)
            .arg(pvt.numSV);
    }
    case UBX_NAV_STATUS: {
        UbxParser::NavStatus status = UbxParser::parseNavStatus(payload);
        displayNavStatus(status);
        return QString("NAV-STATUS: Fix=%1 TTFF=%2ms")
            .arg(status.fixType)
            .arg(status.ttff);
    }
    case UBX_NAV_SAT: {
        UbxParser::NavSat sat = UbxParser::parseNavSat(payload);
        return QString("NAV-SAT: Version=%1 SVs=%2")
            .arg(sat.version)
            .arg(sat.numSvs);
    }
    case UBX_NAV_TIMEUTC: {
        return QString("NAV-TimeUTC");
    }
    default:
        return QString("Unknown NAV message ID: 0x%1").arg(msgId, 2, 16, QLatin1Char('0'));
    }
}

QString GNSSWindow::processMonMessages(quint8 msgId, const QByteArray& payload) {
    switch (msgId) {
    case UBX_MON_VER: {
        UbxParser::MonVer ver = UbxParser::parseMonVer(payload);
        displayMonVer(ver);
        return QString("MON-VER: SW=%1 HW=%2")
            .arg(ver.swVersion)
            .arg(ver.hwVersion);
    }
    case UBX_MON_HW: {
        UbxParser::MonHw hw = UbxParser::parseMonHw(payload);
        processMonHw(hw);
        return QString("MON-HW: Antenna=%1 Jamming=%2%")
            .arg(hw.aPower ? "ON" : "OFF")
            .arg(hw.jamInd);
    }
    case UBX_MON_RF: {
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
    case UBX_SEC_UNIQID: {
        UbxParser::SecUniqid uniqid = UbxParser::parseSecUniqid(payload);
        emit secUniqidReceived(uniqid);
        return QString("SEC-UNIQID: 0x%1").arg(uniqid.uniqueId, 8, 16, QLatin1Char('0'));
    }
    default:
        return QString("Unknown SEC message ID: 0x%1").arg(msgId, 2, 16, QLatin1Char('0'));
    }
}

void GNSSWindow::processInfMessages(quint8 msgId, const QByteArray& payload) {
    switch (msgId) {
    case UBX_INF_ERROR:
        emit infErrorReceived(QString::fromLatin1(payload));
        break;
    case UBX_INF_WARNING:
        appendToLog("INF-WARNING: " + QString::fromLatin1(payload), "warning");
        break;
    case UBX_INF_NOTICE:
        appendToLog("INF-NOTICE: " + QString::fromLatin1(payload), "info");
        break;
    case UBX_INF_TEST:
        appendToLog("INF-TEST: " + QString::fromLatin1(payload), "test");
        break;
    case UBX_INF_DEBUG:
        appendToLog("INF-DEBUG: " + QString::fromLatin1(payload), "debug");
        break;
    default:
        appendToLog(QString("Unknown INF message ID: 0x%1").arg(msgId, 2, 16, QLatin1Char('0')), "warning");
        break;
    }
}

QString GNSSWindow::getMessageName(quint8 msgClass, quint8 msgId) {
    switch(msgClass) {
    case UBX_CLASS_NAV:
        switch(msgId) {
        case UBX_NAV_PVT: return "NAV-PVT";
        case UBX_NAV_STATUS: return "NAV-STATUS";
        case UBX_NAV_SAT: return "NAV-SAT";
        case UBX_NAV_TIMEUTC: return "NAV-TIMEUTC";
        default: return QString("NAV-UNKNOWN (0x%1)").arg(msgId, 2, 16, QLatin1Char('0'));
        }
    case UBX_CLASS_INF:
        switch(msgId) {
        case UBX_INF_ERROR: return "INF-ERROR";
        case UBX_INF_WARNING: return "INF-WARNING";
        case UBX_INF_NOTICE: return "INF-NOTICE";
        case UBX_INF_TEST: return "INF-TEST";
        case UBX_INF_DEBUG: return "INF-DEBUG";
        default: return QString("INF-UNKNOWN (0x%1)").arg(msgId, 2, 16, QLatin1Char('0'));
        }
    case UBX_CLASS_ACK:
        switch(msgId) {
        case UBX_ACK_ACK: return "ACK-ACK";
        case UBX_ACK_NAK: return "ACK-NAK";
        default: return QString("ACK-UNKNOWN (0x%1)").arg(msgId, 2, 16, QLatin1Char('0'));
        }
    case UBX_CLASS_CFG:
        switch(msgId) {
        case UBX_CFG_PRT: return "CFG-PRT";
        case UBX_CFG_MSG: return "CFG-MSG";
        case UBX_CFG_RATE: return "CFG-RATE";
        case UBX_CFG_ANT: return "CFG-ANT";
        case UBX_CFG_NAV5: return "CFG-NAV5";
        case UBX_CFG_ITFM: return "CFG-ITFM";
        case UBX_CFG_VALSET: return "CFG-VALSET";
        case UBX_CFG_VALGET: return "CFG-VALGET";
        default: return QString("CFG-UNKNOWN (0x%1)").arg(msgId, 2, 16, QLatin1Char('0'));
        }
    case UBX_CLASS_MON:
        switch(msgId) {
        case UBX_MON_VER: return "MON-VER";
        case UBX_MON_HW: return "MON-HW";
        case UBX_MON_RF: return "MON-RF";
        default: return QString("MON-UNKNOWN (0x%1)").arg(msgId, 2, 16, QLatin1Char('0'));
        }
    case UBX_CLASS_SEC:
        switch(msgId) {
        case UBX_SEC_UNIQID: return "SEC-UNIQID";
        default: return QString("SEC-UNKNOWN (0x%1)").arg(msgId, 2, 16, QLatin1Char('0'));
        }
    default:
        return QString("UNKNOWN (Class: 0x%1, ID: 0x%2)")
            .arg(msgClass, 2, 16, QLatin1Char('0'))
            .arg(msgId, 2, 16, QLatin1Char('0'));
    }
}

void GNSSWindow::displayNavStatus(const UbxParser::NavStatus &data) {
    qDebug() << "Updating UI with NAV-STATUS data:"
             << "Fix:" << data.fixType << "TTFF:" << data.ttff << "ms";

    ui->statusbar->showMessage(QString("Fix status: %1, TTFF: %2ms").arg(data.fixType).arg(data.ttff), 5000);
}

void GNSSWindow::displayMonRf(const UbxParser::MonRf &data) {
    QString info = QString("MON-RF: Version=%1, Blocks=%2")
                       .arg(data.version)
                       .arg(data.nBlocks);

    for (int i = 0; i < data.nBlocks && i < 4; i++) {
        const auto& block = data.blocks[i];

        QString jammingState;
        switch (block.flags & 0x03) {
        case JAMMING_UNKNOWN: jammingState = "Unknown"; break;
        case JAMMING_OK: jammingState = "OK"; break;
        case JAMMING_WARNING: jammingState = "Warning"; break;
        case JAMMING_CRITICAL: jammingState = "Critical"; break;
        }

        QString antStatus;
        switch (block.antStatus) {
        case ANT_STATUS_INIT: antStatus = "INIT"; break;
        case ANT_STATUS_DONTKNOW: antStatus = "DONTKNOW"; break;
        case ANT_STATUS_OK: antStatus = "OK"; break;
        case ANT_STATUS_SHORT: antStatus = "SHORT"; break;
        case ANT_STATUS_OPEN: antStatus = "OPEN"; break;
        }

        QString antPower;
        switch (block.antPower) {
        case ANT_POWER_OFF: antPower = "OFF"; break;
        case ANT_POWER_ON: antPower = "ON"; break;
        case ANT_POWER_UNKNOWN: antPower = "UNKNOWN"; break;
        }

        info += QString("\nBlock %1: Jamming=%2, Antenna=%3, Power=%4")
                    .arg(i)
                    .arg(jammingState)
                    .arg(antStatus)
                    .arg(antPower);
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

    ui->leSwVersion->setText(data.swVersion);
    ui->leHwVersion->setText(data.hwVersion);
    ui->teExtensions->setPlainText(data.extensions.join("\n"));

    QRegularExpression re("PROTVER=(\\d+\\.\\d+)");
    QRegularExpressionMatch match = re.match(data.swVersion);
    if (match.hasMatch()) {
        m_protocolVersion = match.captured(1).toFloat();
        appendToLog(QString("Protocol version detected: %1").arg(m_protocolVersion), "system");
    }

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

void GNSSWindow::sendUbxCfgAnt() {
    if (!m_socket || m_socket->state() != QAbstractSocket::ConnectedState) {
        appendToLog("Error: No active connection to send CFG-ANT", "error");
        return;
    }

    QByteArray payload(4, 0x00);

    // Build flags
    quint16 flags = 0;
    if (ui->cbAntSupplyCtrl->isChecked()) flags |= 0x0001;
    if (ui->cbAntShortDetect->isChecked()) flags |= 0x0002;
    if (ui->cbAntOpenDetect->isChecked()) flags |= 0x0004;
    if (ui->cbAntPowerDown->isChecked()) flags |= 0x0008;
    if (ui->cbAntAutoRecover->isChecked()) flags |= 0x0010;

    // Build pins configuration
    quint16 pins = 0;
    pins |= (ui->sbAntSwitchPin->value() & 0x1F);
    pins |= ((ui->sbAntShortPin->value() & 0x1F) << 5);
    pins |= ((ui->sbAntOpenPin->value() & 0x1F) << 10);
    if (ui->cbAntReconfig->isChecked()) pins |= 0x8000;

    // Write to payload
    qToLittleEndian<quint16>(flags, payload.data());
    qToLittleEndian<quint16>(pins, payload.data() + 2);

    createUbxPacket(UBX_CLASS_CFG, UBX_CFG_ANT, payload);
    appendToLog(QString("CFG-ANT sent: flags=0x%1, pins=0x%2")
                    .arg(flags, 4, 16, QLatin1Char('0'))
                    .arg(pins, 4, 16, QLatin1Char('0')), "config");
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

    createUbxPacket(UBX_CLASS_NAV, UBX_NAV_SAT, payload);
    appendToLog("Sent NAV-SAT message", "out");
}

void GNSSWindow::onSendButtonClicked() {
    quint8 msgClass = static_cast<quint8>(ui->cbClass->currentData().toInt());
    quint8 msgId = static_cast<quint8>(ui->cbId->currentData().toInt());

    switch(msgClass) {
    case UBX_CLASS_NAV:
        if (msgId == UBX_NAV_PVT) sendUbxNavPvt();
        else if (msgId == UBX_NAV_STATUS) sendUbxNavStatus();
        else if (msgId == UBX_NAV_SAT) sendUbxNavSat();
        else if (msgId == UBX_NAV_TIMEUTC) sendUbxNavTimeUtc();
        break;
    case UBX_CLASS_CFG:
        if (msgId == UBX_CFG_PRT) sendUbxCfgPrt();
        else if (msgId == UBX_CFG_ITFM) sendUbxCfgItfm();
        else if (msgId == UBX_CFG_NAV5) sendUbxCfgNav5();
        else if (msgId == UBX_CFG_RATE) sendUbxCfgRate();
        else if (msgId == UBX_CFG_VALGET) sendUbxCfgValGet();
        else if (msgId == UBX_CFG_VALSET) sendUbxCfgValset();
        break;
    case UBX_CLASS_INF:
        if (msgId == UBX_INF_ERROR) sendUbxInfError();
        else if (msgId == UBX_INF_WARNING) sendUbxInfWarning();
        else if (msgId == UBX_INF_NOTICE) sendUbxInfNotice();
        else if (msgId == UBX_INF_TEST) sendUbxInfTest();
        else if (msgId == UBX_INF_DEBUG) sendUbxInfDebug();
        break;
    case UBX_CLASS_MON:
        if (msgId == UBX_MON_VER) sendUbxMonVer();
        else if (msgId == UBX_MON_HW) sendUbxMonHw();
        else if (msgId == UBX_MON_RF) sendUbxMonRf();
        break;
    case UBX_CLASS_SEC:
        if (msgId == UBX_SEC_UNIQID) sendUbxSecUniqid();
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

void GNSSWindow::onAutoSendNavPvtToggled(bool checked) {
    if (checked) {
        QTimer::singleShot(0, this, &GNSSWindow::sendUbxNavPvt);
        m_pvtTimer->start(1000 / ui->rateSpin->value());
    } else {
        m_pvtTimer->stop();
    }
}

void GNSSWindow::onAutoSendNavStatusToggled(bool checked) {
    if (checked) {
        QTimer::singleShot(0, this, &GNSSWindow::sendUbxNavStatus);
        m_statusTimer->start(1000 / ui->rateSpin->value());
    } else {
        m_statusTimer->stop();
    }
}

void GNSSWindow::onAutoSendNavSatToggled(bool checked) {
    static QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &GNSSWindow::sendUbxNavSat);
    if (checked) {
        QTimer::singleShot(0, this, &GNSSWindow::sendUbxNavSat);
        timer->start(1000 / ui->rateSpin->value());
    } else {
        timer->stop();
    }
}

void GNSSWindow::onAutoSendNavTimeUtcToggled(bool checked) {
    static QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &GNSSWindow::sendUbxNavTimeUtc);
    if (checked) {
        QTimer::singleShot(0, this, &GNSSWindow::sendUbxNavTimeUtc);
        timer->start(1000 / ui->rateSpin->value());
    } else {
        timer->stop();
    }
}

void GNSSWindow::onAutoSendMonVerToggled(bool checked) {
    static QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &GNSSWindow::sendUbxMonVer);
    if (checked) {
        QTimer::singleShot(0, this, &GNSSWindow::sendUbxMonVer);
        timer->start(5000); // Every 5 seconds
    } else {
        timer->stop();
    }
}

void GNSSWindow::onAutoSendMonHwToggled(bool checked) {
    static QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &GNSSWindow::sendUbxMonHw);
    if (checked) {
        QTimer::singleShot(0, this, &GNSSWindow::sendUbxMonHw);
        timer->start(5000); // Every 5 seconds
    } else {
        timer->stop();
    }
}

void GNSSWindow::onAutoSendMonRfToggled(bool checked) {
    static QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &GNSSWindow::sendUbxMonRf);
    if (checked) {
        QTimer::singleShot(0, this, &GNSSWindow::sendUbxMonRf);
        timer->start(1000 / ui->rateSpin->value());
    } else {
        timer->stop();
    }
}

void GNSSWindow::onAutoSendCfgPrtToggled(bool checked) {
    static QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &GNSSWindow::sendUbxCfgPrt);
    if (checked) {
        QTimer::singleShot(0, this, &GNSSWindow::sendUbxCfgPrt);
        timer->start(10000); // Every 10 seconds
    } else {
        timer->stop();
    }
}

void GNSSWindow::onAutoSendCfgItfmToggled(bool checked) {
    static QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &GNSSWindow::sendUbxCfgItfm);
    if (checked) {
        QTimer::singleShot(0, this, &GNSSWindow::sendUbxCfgItfm);
        timer->start(10000); // Every 10 seconds
    } else {
        timer->stop();
    }
}

void GNSSWindow::onAutoSendCfgNav5Toggled(bool checked) {
    static QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &GNSSWindow::sendUbxCfgNav5);
    if (checked) {
        QTimer::singleShot(0, this, &GNSSWindow::sendUbxCfgNav5);
        timer->start(10000); // Every 10 seconds
    } else {
        timer->stop();
    }
}

void GNSSWindow::onAutoSendCfgRateToggled(bool checked) {
    static QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &GNSSWindow::sendUbxCfgRate);
    if (checked) {
        QTimer::singleShot(0, this, &GNSSWindow::sendUbxCfgRate);
        timer->start(10000); // Every 10 seconds
    } else {
        timer->stop();
    }
}

void GNSSWindow::onAutoSendCfgValgetToggled(bool checked) {
    static QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &GNSSWindow::sendUbxCfgValGet);
    if (checked) {
        QTimer::singleShot(0, this, &GNSSWindow::sendUbxCfgValGet);
        timer->start(5000); // Every 5 seconds
    } else {
        timer->stop();
    }
}

void GNSSWindow::onAutoSendCfgValsetToggled(bool checked) {
    static QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &GNSSWindow::sendUbxCfgValset);
    if (checked) {
        QTimer::singleShot(0, this, &GNSSWindow::sendUbxCfgValset);
        timer->start(5000); // Every 5 seconds
    } else {
        timer->stop();
    }
}

void GNSSWindow::onAutoSendCfgAntToggled(bool checked) {
    static QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &GNSSWindow::sendUbxCfgAnt);
    if (checked) {
        QTimer::singleShot(0, this, &GNSSWindow::sendUbxCfgAnt);
        timer->start(10000); // Every 10 seconds
    } else {
        timer->stop();
    }
}

void GNSSWindow::onAutoSendInfDebugToggled(bool checked) {
    static QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &GNSSWindow::sendUbxInfDebug);
    if (checked) {
        QTimer::singleShot(0, this, &GNSSWindow::sendUbxInfDebug);
        timer->start(2000); // Every 2 seconds
    } else {
        timer->stop();
    }
}

void GNSSWindow::onAutoSendInfErrorToggled(bool checked) {
    static QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &GNSSWindow::sendUbxInfError);
    if (checked) {
        QTimer::singleShot(0, this, &GNSSWindow::sendUbxInfError);
        timer->start(5000); // Every 5 seconds
    } else {
        timer->stop();
    }
}

void GNSSWindow::onAutoSendInfWarningToggled(bool checked) {
    static QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &GNSSWindow::sendUbxInfWarning);
    if (checked) {
        QTimer::singleShot(0, this, &GNSSWindow::sendUbxInfWarning);
        timer->start(3000); // Every 3 seconds
    } else {
        timer->stop();
    }
}

void GNSSWindow::onAutoSendInfNoticeToggled(bool checked) {
    static QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &GNSSWindow::sendUbxInfNotice);
    if (checked) {
        QTimer::singleShot(0, this, &GNSSWindow::sendUbxInfNotice);
        timer->start(4000); // Every 4 seconds
    } else {
        timer->stop();
    }
}

void GNSSWindow::onAutoSendInfTestToggled(bool checked) {
    static QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &GNSSWindow::sendUbxInfTest);
    if (checked) {
        QTimer::singleShot(0, this, &GNSSWindow::sendUbxInfTest);
        timer->start(3000); // Every 3 seconds
    } else {
        timer->stop();
    }
}

void GNSSWindow::onAutoSendSecUniqidToggled(bool checked) {
    static QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &GNSSWindow::sendUbxSecUniqid);
    if (checked) {
        QTimer::singleShot(0, this, &GNSSWindow::sendUbxSecUniqid);
        timer->start(10000); // Every 10 seconds
    } else {
        timer->stop();
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

    createUbxPacket(UBX_CLASS_CFG, UBX_CFG_PRT, payload);
    appendToLog("Sent CFG-PRT", "config");
}

void GNSSWindow::sendUbxCfgNav5() {
    if (!m_socket || m_socket->state() != QAbstractSocket::ConnectedState) {
        appendToLog("Error: No active connection to send CFG-NAV5", "error");
        return;
    }

    QByteArray payload(36, 0x00);

    // Set mask - enable all parameters
    quint16 mask = 0x05FF; // Enable all parameters except reserved bits
    qToLittleEndian<quint16>(mask, payload.data());

    // Dynamic model
    payload[2] = static_cast<quint8>(ui->cbDynModel->currentIndex());

    // Fix mode
    payload[3] = static_cast<quint8>(ui->cbFixMode->currentIndex() + 1); // Fix modes start at 1

    // Fixed altitude (cm)
    qint32 fixedAlt = static_cast<qint32>(ui->dsbFixedAlt->value() * 100);
    qToLittleEndian<qint32>(fixedAlt, payload.data() + 4);

    // Fixed altitude variance (0.0001 m^2)
    quint32 fixedAltVar = 10000; // Default 1 m^2
    qToLittleEndian<quint32>(fixedAltVar, payload.data() + 8);

    // Minimum elevation
    payload[12] = static_cast<quint8>(ui->sbMinElev->value());

    // PDOP mask (0.1 units)
    quint16 pDop = static_cast<quint16>(ui->dsbPDOP->value() * 10);
    qToLittleEndian<quint16>(pDop, payload.data() + 14);

    // TDOP mask (0.1 units)
    quint16 tDop = static_cast<quint16>(ui->dsbTDOP->value() * 10);
    qToLittleEndian<quint16>(tDop, payload.data() + 16);

    // Position accuracy mask (m)
    quint16 pAcc = static_cast<quint16>(ui->dsbPAcc->value());
    qToLittleEndian<quint16>(pAcc, payload.data() + 18);

    // Time accuracy mask (m)
    quint16 tAcc = static_cast<quint16>(ui->dsbTAcc->value());
    qToLittleEndian<quint16>(tAcc, payload.data() + 20);

    // Static hold threshold (cm/s)
    payload[22] = static_cast<quint8>(ui->dsbStaticHoldThresh->value());

    // DGNSS timeout (s)
    payload[23] = static_cast<quint8>(ui->sbDgnssTimeout->value());

    // CNO threshold and number of SVs
    payload[24] = static_cast<quint8>(ui->sbCnoThreshNumSVs->value());
    payload[25] = static_cast<quint8>(ui->sbCnoThresh->value());

    // Static hold max distance (m)
    quint16 staticHoldMaxDist = static_cast<quint16>(ui->dsbStaticHoldMaxDist->value());
    qToLittleEndian<quint16>(staticHoldMaxDist, payload.data() + 28);

    // UTC standard
    payload[30] = static_cast<quint8>(ui->cbUtcStandard->currentIndex());

    createUbxPacket(UBX_CLASS_CFG, UBX_CFG_NAV5, payload);
    appendToLog("Sent CFG-NAV5 configuration", "config");
}

void GNSSWindow::sendUbxMonVer() {
    QByteArray payload;

    // Software version (30 bytes, null-terminated)
    QString swVersion = ui->leSwVersion->text();
    QByteArray swVersionBytes = swVersion.left(29).toLatin1(); // Ensure it fits in 30 bytes with null terminator
    payload.append(swVersionBytes);
    payload.append('\0');
    while (payload.size() < 30) payload.append('\0'); // Pad to 30 bytes

    // Hardware version (10 bytes, null-terminated)
    QString hwVersion = ui->leHwVersion->text();
    QByteArray hwVersionBytes = hwVersion.left(9).toLatin1(); // Ensure it fits in 10 bytes with null terminator
    payload.append(hwVersionBytes);
    payload.append('\0');
    while (payload.size() < 40) payload.append('\0'); // Pad to 40 bytes

    // Extensions (each 30 bytes, null-terminated)
    QStringList extensions = ui->teExtensions->toPlainText().split('\n', Qt::SkipEmptyParts);
    for (const QString &ext : extensions) {
        QByteArray extBytes = ext.left(29).toLatin1(); // Ensure it fits in 30 bytes with null terminator
        payload.append(extBytes);
        payload.append('\0');
        while (payload.size() % 30 != 0) payload.append('\0'); // Pad to multiple of 30 bytes
    }

    createUbxPacket(UBX_CLASS_MON, UBX_MON_VER, payload);
    appendToLog(QString("Sent MON-VER: SW=%1, HW=%2, %3 extensions")
                    .arg(swVersion)
                    .arg(hwVersion)
                    .arg(extensions.size()), "config");
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

    createUbxPacket(UBX_CLASS_NAV, UBX_NAV_STATUS, payload);
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
        clearReceiveBuffer();
        m_timer->stop();
        ui->autoSendCheck->setChecked(false);
    }
}

void GNSSWindow::clearReceiveBuffer() {
    m_receiveBuffer.clear();
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

void GNSSWindow::sendUbxAck(quint8 msgClass, quint8 msgId) {
    QByteArray payload;
    payload.append(static_cast<char>(msgClass));
    payload.append(static_cast<char>(msgId));

    createUbxPacket(UBX_CLASS_ACK, UBX_ACK_ACK, payload);
    appendToLog(QString("Sent ACK for Class=0x%1 ID=0x%2")
                    .arg(msgClass, 2, 16, QLatin1Char('0'))
                    .arg(msgId, 2, 16, QLatin1Char('0')),
                "config");
}

void GNSSWindow::sendUbxNack(quint8 msgClass, quint8 msgId) {
    QByteArray payload;
    payload.append(static_cast<char>(msgClass));
    payload.append(static_cast<char>(msgId));

    createUbxPacket(UBX_CLASS_ACK, UBX_ACK_NAK, payload);
    appendToLog(QString("Sent NACK for Class=0x%1 ID=0x%2")
                    .arg(msgClass, 2, 16, QLatin1Char('0'))
                    .arg(msgId, 2, 16, QLatin1Char('0')),
                "error");
}
void GNSSWindow::setupNavPvtFields() {
    ui->gbNavPvtFields->setVisible(true);
    ui->tePayload->setVisible(false);

    if (m_firstInitialization) {
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
}

void GNSSWindow::setupNavStatusFields() {
    ui->gbNavStatusFields->setVisible(true);
    ui->tePayload->setVisible(false);

    if (m_firstInitialization) {
    ui->sbFixTypeStatus->setValue(3); // 3D fix
    ui->sbTtff->setValue(5000); // 5 seconds
    }
}

void GNSSWindow::setupCfgPrtFields() {
    ui->gbCfgPrtFields->setVisible(true);
    ui->tePayload->setVisible(false);

    if (m_firstInitialization) {
    ui->cbPortId->setCurrentIndex(0); // UART1 by default
    ui->cbBaudRate->setCurrentText("115200");
    ui->cbInUbx->setChecked(true);
    ui->cbInNmea->setChecked(true);
    ui->cbOutUbx->setChecked(true);
    ui->cbOutNmea->setChecked(true);
    }
}

void GNSSWindow::setupCfgItfmFields() {
    ui->gbCfgItfmFields->setVisible(true);
    ui->tePayload->setVisible(false);

    if (m_firstInitialization) {
    ui->sbBbThreshold->setValue(0);
    ui->sbCwThreshold->setValue(0);
    ui->cbEnable->setChecked(true);
    ui->cbAntSetting->setCurrentIndex(0);
    ui->cbEnable2->setChecked(false);
    }
}

void GNSSWindow::sendUbxCfgPrt() {
    if (!m_socket || m_socket->state() != QAbstractSocket::ConnectedState) {
        appendToLog("Error: No active connection to send CFG-PRT", "error");
        return;
    }

    QByteArray payload(20, 0x00);

    // Port ID
    quint8 portId = static_cast<quint8>(ui->cbPortId->currentIndex() + 1);
    payload[0] = portId;

    // Baud Rate
    quint32 baudRate = ui->cbBaudRate->currentText().toUInt();
    qToLittleEndian<quint32>(baudRate, payload.data() + 8);

    // Protocol masks
    quint16 inProtoMask = 0;
    if (ui->cbInUbx->isChecked()) inProtoMask |= 0x0001;
    if (ui->cbInNmea->isChecked()) inProtoMask |= 0x0002;
    if (ui->cbInRtcm->isChecked()) inProtoMask |= 0x0004;
    qToLittleEndian<quint16>(inProtoMask, payload.data() + 12);

    quint16 outProtoMask = 0;
    if (ui->cbOutUbx->isChecked()) outProtoMask |= 0x0001;
    if (ui->cbOutNmea->isChecked()) outProtoMask |= 0x0002;
    if (ui->cbOutRtcm->isChecked()) outProtoMask |= 0x0004;
    qToLittleEndian<quint16>(outProtoMask, payload.data() + 14);

    // Mode: 8N1, no parity (0x000008D0)
    qToLittleEndian<quint32>(0x000008D0, payload.data() + 4);

    createUbxPacket(UBX_CLASS_CFG, UBX_CFG_PRT, payload);
    appendToLog(QString("Sent CFG-PRT: Port=%1, Baud=%2, InProto=0x%3, OutProto=0x%4")
                    .arg(portId)
                    .arg(baudRate)
                    .arg(inProtoMask, 4, 16, QLatin1Char('0'))
                    .arg(outProtoMask, 4, 16, QLatin1Char('0')), "config");
}

void GNSSWindow::sendUbxMonHw() {
    QByteArray payload(60, 0x00);

    // Noise level (0-8191)
    quint16 noise = static_cast<quint16>(ui->sbHwNoise->value());
    qToLittleEndian<quint16>(noise, payload.data() + 16);

    // AGC (0-100% -> 0-8191)
    quint16 agc = static_cast<quint16>(ui->sbHwAgc->value() * 81.91);
    qToLittleEndian<quint16>(agc, payload.data() + 18);

    // Antenna status
    payload[20] = static_cast<quint8>(ui->cbHwAntStatus->currentIndex());

    // Antenna power
    payload[21] = static_cast<quint8>(ui->cbHwAntPower->currentIndex());

    // Flags
    quint8 flags = 0;
    flags |= (ui->cbHwJamming->currentIndex() << 2); // Jamming state in bits 2-3
    payload[22] = flags;

    // CW suppression
    payload[45] = static_cast<quint8>(ui->sbHwCwSuppression->value());

    // pinSel, pinBank, pinDir, pinVal, usedMask, VP, pinIrq, pullH, pullL

    createUbxPacket(UBX_CLASS_MON, UBX_MON_HW, payload);
    appendToLog(QString("MON-HW sent: Noise=%1, AGC=%2%, AntStatus=%3, AntPower=%4")
                    .arg(noise)
                    .arg(ui->sbHwAgc->value())
                    .arg(ui->cbHwAntStatus->currentText())
                    .arg(ui->cbHwAntPower->currentText()),
                "out");
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

    createUbxPacket(UBX_CLASS_NAV, UBX_NAV_PVT, payload);
    appendToLog("Sent NAV-PVT", "out");
}

void GNSSWindow::createUbxPacket(quint8 msgClass, quint8 msgId, const QByteArray &payload) {
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

    // Sending
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
