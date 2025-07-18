//#include "gnsswindow.h"
#include "dialog.h"
#include <QApplication>
#include <QFile>
#include <QWidget>

/*
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    GNSSWindow w;
    w.show();
    return a.exec();
}
*/
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Dialog w;
    w.show();

    return a.exec();
}
