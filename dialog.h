#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include <QTcpSocket>
#include "ui_dialog.h"

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = nullptr);
    ~Dialog();

private slots:
    void on_ButtonEnter_clicked();
    void onConnected();

private:
    Ui::Dialog *ui;
    QTcpSocket *mSocket;

};

#endif // DIALOG_H
