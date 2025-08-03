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

class GNSSWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit GNSSWindow(Dialog* parentDialog = nullptr, QWidget *parent = nullptr);
    ~GNSSWindow();
    void sendUbxCfgPrtResponse();
    void setSocket(QTcpSocket *socket);
    void onConnectionStatusChanged(bool connected);
    void sendUbxMonHw();

public slots:
    void appendToLog(const QString &message, const QString &type = "info");

private slots:
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
    void sendUbxCfgItfm();
    void sendUbxCfgRate(quint16 measRate, quint16 navRate);
    void sendUbxCfgAnt(bool enablePower);
    void saveLogToFile();
    void clearLog();
    void pauseLog(bool paused);
    void sendUbxCfgMsg(quint8 msgClass, quint8 msgId, quint8 rate);
    void sendUbxCfgDynModel(quint8 model);
    void sendUbxCfgAntSettings(bool openDet, bool shortDet, bool recover);
    void sendUbxSecUniqidReq();
    void onError(QAbstractSocket::SocketError error);

signals:
    void secUniqidReceived(const UbxParser::SecUniqid &data);

private:
    Dialog* m_parentDialog;
    Ui::GNSSWindow *ui;
    QTimer *m_ackTimeoutTimer;
    bool m_waitingForAck = false;
    QTimer *m_initTimer;
    QTcpSocket *m_socket;
    QTimer *m_timer;
    UbxParser m_ubxParser;
    QStandardItemModel *m_satModel;
    QLabel *m_antennaStatusLabel;
    QCustomPlot *m_signalPlot;
    QMap<quint8, QMap<int, QString>> m_classIdMap;
    bool m_initializationComplete = false;
    void sendUbxAck(quint8 msgClass, quint8 msgId);
    void sendUbxNack(quint8 msgClass, quint8 msgId);
    void sendInitialConfiguration();
    void setupConnections();
    void setupSatelliteView();
    void setupSignalPlot();
    void registerHandlers();
    void initClassIdMapping();
    void processUbxMessage(quint8 msgClass, quint8 msgId, const QByteArray& payload);
    void processNavSat(const UbxParser::NavSat &sat);
    void processMonHw(const UbxParser::MonHw &hw);
    void updateSignalPlot();
    void displayNavPvt(const UbxParser::NavPvt &data);
    void displayNavStatus(const UbxParser::NavStatus &data);
    void displayCfgPrt(const UbxParser::CfgPrt &data);
    void displayMonVer(const UbxParser::MonVer &data);
    void createUbxPacket(quint8 msgClass, quint8 msgId, const QByteArray &payload);
    QString getMessageName(quint8 msgClass, quint8 msgId);
};

#endif // GNSSWINDOW_H
