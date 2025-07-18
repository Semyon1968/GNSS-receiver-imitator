#ifndef GNSSWINDOW_H
#define GNSSWINDOW_H

#include <QMainWindow>
#include <QTcpSocket>

QT_BEGIN_NAMESPACE
namespace Ui {
class GNSSWindow;
}
QT_END_NAMESPACE

class GNSSWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit GNSSWindow(QWidget *parent = nullptr);
    ~GNSSWindow();
    void setModel(const QString &model);
    void setSerialNumber(const QString &serial);


private slots:
    void onSendBtnClicked();
    void clearGuiFields();
    void onUTCTimeModeChanged(const QString &mode);

private:
    Ui::GNSSWindow *ui;
    QTcpSocket *socket;
    QByteArray receiveBuffer;
    QString currentModel;
    QString currentSerialNumber;
    void setupSocket();
    void connectToServer();
    void sendUBXPacket(quint8 msgClass, quint8 msgId, const QByteArray &payload = {});
};

#endif // GNSSWINDOW_H
