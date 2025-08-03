#include "dialog.h"
#include "ui_dialog.h"
#include "gnsswindow.h"
#include <QMessageBox>
#include <QDebug>
#include <QTimer>

Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog),
    m_socket(new QTcpSocket(this)),  // Инициализация сокета
    m_gnssWindow(nullptr),
    m_connectionTimer(new QTimer(this))
{
    ui->setupUi(this);

    // Настройка текстового лога
    ui->logText->setReadOnly(true);
    ui->logText->setFont(QFont("Monospace", 10));

    // 1. Настройка таймера соединения
    m_connectionTimer->setSingleShot(true);
    m_connectionTimer->setInterval(10000); // 10 секунд
    connect(m_connectionTimer, &QTimer::timeout, this, &Dialog::onConnectionTimeout);

    // 2. Настройка соединений сокета
    connect(m_socket, &QTcpSocket::connected, this, &Dialog::onConnected);
    connect(m_socket, &QTcpSocket::disconnected, this, &Dialog::onDisconnected);
    connect(m_socket, &QTcpSocket::stateChanged, this, [this](QAbstractSocket::SocketState state) {
        qDebug() << "Socket state changed:" << state;
    });
    connect(m_socket, QOverload<QAbstractSocket::SocketError>::of(&QTcpSocket::errorOccurred),
            this, &Dialog::onError);
    connect(m_socket, &QTcpSocket::readyRead, this, [this]() {
        qDebug() << "Data available:" << m_socket->bytesAvailable() << "bytes";
    });

    // 3. Настройка UI
    ui->leIpAddress->setText("192.168.2.22");  // Значение по умолчанию
    ui->lePort->setText("40001");
    ui->statusLabel->setText("Disconnected");

    // 4. Дополнительные проверки
    qDebug() << "Dialog initialized, socket state:" << m_socket->state();
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
    if (m_gnssWindow) {
        m_gnssWindow->close();
    }

    ui->statusLabel->setText("Disconnected");
    qDebug() << "Disconnected from host";

    // Сбрасываем состояние сокета
    if (m_socket) {
        m_socket->abort();
    }
}

void Dialog::onConnected()
{
    m_connectionTimer->stop();

    if (!m_gnssWindow) {
        m_gnssWindow = new GNSSWindow(this); // Передаем this (Dialog) как parentDialog
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

    ui->logText->append(formattedMessage);
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

    ui->statusLabel->setText("Connecting...");
    m_socket->connectToHost(host, port);
    m_connectionTimer->start(10000); // 10 секунд таймаут
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
