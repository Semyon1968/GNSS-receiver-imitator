#include "dialog.h"
#include "ui_dialog.h"
#include "gnsswindow.h"
#include <QMessageBox>
#include <QDebug>

Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog),
    m_socket(new QTcpSocket(this)),
    m_gnssWindow(nullptr) // Инициализация
{
    ui->setupUi(this);

    connect(m_socket, &QTcpSocket::connected, this, &Dialog::onConnected);
    connect(m_socket, &QTcpSocket::disconnected, this, &Dialog::onDisconnected);
    connect(m_socket, QOverload<QAbstractSocket::SocketError>::of(&QTcpSocket::errorOccurred),
            this, &Dialog::onError);
}

// Добавьте новый слот
void Dialog::onDisconnected()
{
    if (m_gnssWindow) {
        m_gnssWindow->close();
        m_gnssWindow->deleteLater();
        m_gnssWindow = nullptr;
    }
}

void Dialog::onConnected()
{
    if (!m_gnssWindow) {
        m_gnssWindow = new GNSSWindow();
        m_gnssWindow->setSocket(m_socket);
        m_gnssWindow->show();
        this->hide(); // Лучше hide() чем close()
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

    // Отключаемся от предыдущего соединения если оно есть
    if (m_socket->state() != QAbstractSocket::UnconnectedState) {
        m_socket->disconnectFromHost();
        if (m_socket->state() != QAbstractSocket::UnconnectedState) {
            m_socket->waitForDisconnected();
        }
    }

    m_socket->connectToHost(host, port);
    qDebug() << "Connecting to" << host << ":" << port;
}

Dialog::~Dialog()
{
    // Отключаем все соединения
    m_socket->disconnect();

    // Удаляем дочерние объекты
    delete ui;
    delete m_socket;

    if (m_gnssWindow) {
        m_gnssWindow->deleteLater();
    }
}

void Dialog::onError(QAbstractSocket::SocketError error)
{
    Q_UNUSED(error)
    QMessageBox::critical(this, "Connection Error", m_socket->errorString());

    // Если окно GNSSWindow было создано, закрываем его
    if (m_gnssWindow) {
        m_gnssWindow->close();
        m_gnssWindow->deleteLater();
        m_gnssWindow = nullptr;
    }
}

