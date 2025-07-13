#include "gnsswindow.h"
#include "./ui_gnsswindow.h"
#include <QMessageBox>

GNSSWindow::GNSSWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::GNSSWindow)
{
    ui->setupUi(this);

    connect(ui->action_3, &QAction::triggered, this, []() {
        QMessageBox::information(nullptr, "О разработчике", "Разработчик: Семён Тихонов");
    });
    connect(ui->action_4, &QAction::triggered, this, []() {
        QMessageBox::information(nullptr, "Версия программы", "Версия: 1.0.0\nДата релиза: 15.06.2025");
    });

    // Выход из приложения
    connect(ui->action_2, &QAction::triggered, qApp, &QApplication::quit);

}

GNSSWindow::~GNSSWindow()
{
    delete ui;
}
