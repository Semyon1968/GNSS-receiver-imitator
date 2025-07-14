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

private slots:
    void onSendBtnClicked();
    void clearGuiFields();

private:
    Ui::GNSSWindow *ui;
    QTcpSocket *socket;
    QByteArray receiveBuffer;
    void setupSocket();
    void connectToServer();
    void sendUBXPacket(quint8 msgClass, quint8 msgId, const QByteArray &payload = {});
};

#endif // GNSSWINDOW_H
