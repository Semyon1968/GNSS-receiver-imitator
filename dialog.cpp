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

    // Setup connection timer (10 sec timeout)
    m_connectionTimer->setSingleShot(true);
    m_connectionTimer->setInterval(10000);
    connect(m_connectionTimer, &QTimer::timeout, this, &Dialog::onConnectionTimeout);

    // Setup socket connections
    connect(m_socket, &QTcpSocket::connected, this, &Dialog::onConnected);
    connect(m_socket, &QTcpSocket::disconnected, this, &Dialog::onDisconnected);
    connect(m_socket, &QTcpSocket::stateChanged, this, [this](QAbstractSocket::SocketState state) {
        qDebug() << "Socket state changed to:" << state;
    });
    connect(m_socket, QOverload<QAbstractSocket::SocketError>::of(&QTcpSocket::errorOccurred),
            this, &Dialog::onError);
    connect(m_socket, &QTcpSocket::readyRead, this, [this]() {
        qDebug() << "Data available:" << m_socket->bytesAvailable() << "bytes";
    });

    // Set default UI values
    ui->leIpAddress->setText("192.168.2.22");
    ui->lePort->setText("40001");

    qDebug() << "Dialog initialized, socket state:" << m_socket->state();
}

void Dialog::onConnectionTimeout()
{
    if (m_socket->state() == QAbstractSocket::ConnectingState) {
        m_socket->abort();
        QMessageBox::warning(this, "Timeout", "Connection timed out");
    }
}

void Dialog::onDisconnected()
{
    if (m_gnssWindow) {
        m_gnssWindow->close();
    }

    qDebug() << "Disconnected from host";

    if (m_socket) {
        m_socket->abort();
    }
}

void Dialog::onConnected()
{
    m_connectionTimer->stop();

    if (!m_gnssWindow) {
        m_gnssWindow = new GNSSWindow(this);
        m_gnssWindow->setSocket(m_socket);
        m_gnssWindow->show();
        this->hide();
    }

    emit logMessage("Connected to autopilot", "system");
}

void Dialog::appendToLog(const QString &message, const QString &type)
{
    QString formattedMessage = QString("[%1] %2: %3")
                                   .arg(QDateTime::currentDateTime().toString("hh:mm:ss"))
                                   .arg(type.toUpper())
                                   .arg(message);

    qDebug() << formattedMessage;
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

    m_socket->connectToHost(host, port);
    m_connectionTimer->start(10000);
}

void Dialog::onError(QAbstractSocket::SocketError error)
{
    Q_UNUSED(error)
    m_connectionTimer->stop();

    QString errorMsg = m_socket->errorString();
    QMessageBox::critical(this, "Connection Error", errorMsg);

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
