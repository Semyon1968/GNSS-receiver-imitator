#ifndef GNSSWINDOW_H
#define GNSSWINDOW_H

#include <QMainWindow>
#include <QTcpSocket>
#include <QTimer>
#include <QStandardItemModel>
#include <QLabel>
#include <QMessageBox>
#include <QMap>
#include "ubxparser.h"
#include "qcustomplot.h"
#include <QAbstractSocket>

class Dialog;

namespace Ui {
class GNSSWindow;
}

class GNSSWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit GNSSWindow(Dialog* parentDialog = nullptr, QWidget *parent = nullptr);
    ~GNSSWindow();
    void sendUbxCfgPrtResponse();
    void setSocket(QTcpSocket *socket);
    void setReceiveBuffer(QByteArray *receiveBuffer);
    void onConnectionStatusChanged(bool connected);

public slots:
    void appendToLog(const QString &message, const QString &type = "info");
    void sendUbxCfgItfm();

private slots:
    void onActionSaveSettingsTriggered();
    void onActionLoadSettingsTriggered();
    void handleInitTimeout();
    void handleAckTimeout();
    void handleSocketDisconnected();
    void handleSocketError(QAbstractSocket::SocketError error);
    void updateAvailableIds();
    void on_btnClearLog_clicked();
    void onReadyRead();
    void onSendButtonClicked();
    void onAutoSendToggled(bool checked);
    void sendUbxNavPvt();
    void sendUbxCfgPrt();
    void sendUbxMonVer();
    void sendUbxSecUniqid();
    void sendUbxNavStatus();
    void onClassIdChanged();
    void saveLogToFile();
    void clearLog();
    void pauseLog(bool paused);
    void sendUbxCfgMsg(quint8 msgClass, quint8 msgId, quint8 rate);
    void sendUbxSecUniqidReq();
    void onError(QAbstractSocket::SocketError error);
    void onActionSaveLogTriggered();
    void onActionClearLogTriggered();
    void onActionAboutTriggered();
    void onAutoSendNavPvtToggled(bool checked);
    void onAutoSendNavStatusToggled(bool checked);
    void onAutoSendNavSatToggled(bool checked);
    void onAutoSendNavTimeUtcToggled(bool checked);
    void onAutoSendMonVerToggled(bool checked);
    void onAutoSendMonHwToggled(bool checked);
    void onAutoSendMonRfToggled(bool checked);
    void onAutoSendCfgPrtToggled(bool checked);
    void onAutoSendCfgItfmToggled(bool checked);
    void onAutoSendCfgNav5Toggled(bool checked);
    void onAutoSendCfgRateToggled(bool checked);
    void onAutoSendCfgValgetToggled(bool checked);
    void onAutoSendCfgValsetToggled(bool checked);
    void onAutoSendCfgAntToggled(bool checked);
    void onAutoSendInfDebugToggled(bool checked);
    void onAutoSendInfErrorToggled(bool checked);
    void onAutoSendInfWarningToggled(bool checked);
    void onAutoSendInfNoticeToggled(bool checked);
    void onAutoSendInfTestToggled(bool checked);
    void onAutoSendSecUniqidToggled(bool checked);

signals:
    void secUniqidReceived(const UbxParser::SecUniqid &data);
    void infErrorReceived(const QString &msg);

private:
    Dialog* m_parentDialog;
    Ui::GNSSWindow *ui;
    QTimer *m_pvtTimer;
    QTimer *m_statusTimer;
    QTimer *m_ackTimeoutTimer;
    bool m_waitingForAck = false;
    QTimer *m_initTimer;
    float m_protocolVersion = 0.0f;
    QTcpSocket *m_socket;
    QTimer *m_timer;
    UbxParser m_ubxParser;
    QMap<quint8, QMap<int, QString>> m_classIdMap;
    QTimer *m_utcTimer;
    // QByteArray m_receiveBuffer;
    void updateUTCTime();
    void processAckNack(quint8 msgId, const QByteArray& payload);
    void completeInitialization();
    void processCfgMessages(quint8 msgId, const QByteArray& payload);
    bool processInfoRequests(quint8 msgClass, quint8 msgId);
    QString processNavMessages(quint8 msgId, const QByteArray& payload);
    QString processRxmMessages(quint8 msgId, const QByteArray& payload);
    QString processMonMessages(quint8 msgId, const QByteArray& payload);
    QString processSecMessages(quint8 msgId, const QByteArray& payload);
    void processInfMessages(quint8 msgId, const QByteArray& payload);
    QByteArray *m_receiveBuffer;
    void processBuffer();
    void setupMonRfFields();
    void displayMonRf(const UbxParser::MonRf &data);
    void sendUbxMonRf();
    void clearReceiveBuffer();
    bool m_initializationComplete = false;
    void setupNavPvtFields();
    void setupNavStatusFields();
    void setupNavSatFields();
    void setupCfgPrtFields();
    void setupMonVerFields();
    void setupSecUniqidFields();
    void setupCfgItfmFields();
    void setupNavTimeUtcFields();
    void setupMonHwFields();
    void setupCfgNav5Fields();
    void sendUbxCfgRate();
    void sendUbxCfgValGet();
    void setupCfgRateFields();
    void setupCfgValgetFields();
    void setupCfgValsetFields();
    void sendUbxCfgValset();
    void sendUbxCfgNav5();
    void setupInfDebugFields();
    void setupInfErrorFields();
    void setupInfWarningFields();
    void setupInfNoticeFields();
    void setupInfTestFields();
    void sendUbxInfDebug();
    void sendUbxInfError();
    void sendUbxInfNotice();
    void sendUbxInfTest();
    void sendUbxInfWarning();
    void sendUbxMonHw();
    void setupCfgAntFields();
    void sendUbxCfgAnt();
    void sendUbxNavTimeUtc();
    void sendUbxNavSat();
    void hideAllParameterFields();
    void sendUbxAck(quint8 msgClass, quint8 msgId);
    void sendUbxNack(quint8 msgClass, quint8 msgId);
    void sendInitialConfiguration();
    void setupConnections();
    void processCfgValGet(const QByteArray &payload);
    void processCfgValSet(const QByteArray &payload);
    void registerHandlers();
    void initClassIdMapping();
    void processUbxMessage(quint8 msgClass, quint8 msgId, const QByteArray& payload);
    void processMonHw(const UbxParser::MonHw &hw);
    void displayNavPvt(const UbxParser::NavPvt &data);
    void displayNavStatus(const UbxParser::NavStatus &data);
    void displayCfgPrt(const UbxParser::CfgPrt &data);
    void displayMonVer(const UbxParser::MonVer &data);
    void createUbxPacket(quint8 msgClass, quint8 msgId, const QByteArray &payload);
    QString getMessageName(quint8 msgClass, quint8 msgId);
    void saveSettings(const QString &filename);
    void loadSettings(const QString &filename);
    QJsonObject getCurrentSettings() const;
    void applySettings(const QJsonObject &settings);
    void setupUiFromSettings(const QJsonObject &settings);
    bool m_settingsLoaded = false;
    void stopAllAutoSendTimers();
    bool m_fieldsInitialized = false;
    void initializeAllFields();
    void showFieldsForCurrentSelection();
    bool m_firstInitialization = true;
};

#endif // GNSSWINDOW_H
