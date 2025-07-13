#include "gnsswindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    GNSSWindow w;
    w.show();
    return a.exec();
}
