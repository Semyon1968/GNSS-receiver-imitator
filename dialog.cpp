#include "dialog.h"
#include "ui_dialog.h"
#include "gnsswindow.h"
#include <QMessageBox>
#include <QDebug>
#include <QTimer>

Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog),
    m_socket(new QTcpSocket(this)),
    m_gnssWindow(nullptr),
    m_connectionTimer(new QTimer(this))
{
    ui->setupUi(this);

    // Настройка таймера для соединения
    m_connectionTimer->setSingleShot(true);
    connect(m_connectionTimer, &QTimer::timeout, this, &Dialog::onConnectionTimeout);

    connect(m_socket, &QTcpSocket::connected, this, &Dialog::onConnected);
    connect(m_socket, &QTcpSocket::disconnected, this, &Dialog::onDisconnected);
    connect(m_socket, QOverload<QAbstractSocket::SocketError>::of(&QTcpSocket::errorOccurred),
            this, &Dialog::onError);
}

void Dialog::onConnectionTimeout()
{
    if(m_socket->state() == QAbstractSocket::ConnectingState) {
        m_socket->abort();
        QMessageBox::warning(this, "Timeout", "Connection timed out");
    }
}

void Dialog::onDisconnected()
{
    emit connectionStatusChanged(false);

    if (m_gnssWindow) {
        m_gnssWindow->deleteLater();
        m_gnssWindow = nullptr;
    }

    ui->statusLabel->setText("Disconnected");
    qDebug() << "Disconnected from host";
}

void Dialog::onConnected()
{
    m_connectionTimer->stop();
    emit connectionStatusChanged(true);

    qDebug() << "Connected to" << m_socket->peerName() << ":" << m_socket->peerPort();
    m_socket->write("\xB5\x62\x06\x00\x00\x00\x06\x18");

    if (!m_gnssWindow) {
        m_gnssWindow = new GNSSWindow();
        m_gnssWindow->setSocket(m_socket);
        connect(this, &Dialog::connectionStatusChanged, m_gnssWindow, &GNSSWindow::onConnectionStatusChanged);
        m_gnssWindow->show();
        this->hide();
    }
}

void Dialog::on_connectButton_clicked()
{
    QString host = ui->leIpAddress->text().trimmed();
    QString portStr = ui->lePort->text().trimmed();

    if (host.isEmpty() || portStr.isEmpty()) {
        QMessageBox::warning(this, "Error", "Please enter host and port");
        return;
    }

    bool ok;
    quint16 port = portStr.toUShort(&ok);
    if (!ok || port == 0) {
        QMessageBox::warning(this, "Error", "Invalid port number");
        return;
    }

    if (m_socket->state() != QAbstractSocket::UnconnectedState) {
        m_socket->disconnectFromHost();
        if (m_socket->state() != QAbstractSocket::UnconnectedState) {
            m_socket->waitForDisconnected(1000);
        }
    }

    ui->statusLabel->setText("Connecting...");
    m_socket->connectToHost(host, port);
    m_connectionTimer->start(5000); // 5 секунд таймаут
}

void Dialog::onError(QAbstractSocket::SocketError error)
{
    Q_UNUSED(error)
    m_connectionTimer->stop();

    QString errorMsg = m_socket->errorString();
    QMessageBox::critical(this, "Connection Error", errorMsg);
    ui->statusLabel->setText("Error: " + errorMsg);

    if (m_gnssWindow) {
        m_gnssWindow->close();
        m_gnssWindow->deleteLater();
        m_gnssWindow = nullptr;
    }
}

Dialog::~Dialog()
{
    m_connectionTimer->stop();

    if (m_socket->state() == QAbstractSocket::ConnectedState) {
        m_socket->disconnectFromHost();
        m_socket->waitForDisconnected(1000);
    }

    delete m_connectionTimer;
    delete ui;
}
