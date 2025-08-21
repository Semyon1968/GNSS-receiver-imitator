#include "dialog.h"
#include <QApplication>
#include <QTranslator>
#include <QLocale>
#include <QFile>
#include <QDir>
#include <QLibraryInfo>

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    // Setup translations
    QTranslator translator;
    QString projectBaseDir;
    QString appDir = QCoreApplication::applicationDirPath();
    QDir currentDir(appDir);
    while (!currentDir.isRoot()) {
        if (QDir(currentDir.absoluteFilePath("translations")).exists()) {
            projectBaseDir = currentDir.absolutePath();
            break;
        }
        if (!currentDir.cdUp()) {
            break;
        }
    }
    QStringList searchPaths;
    if (!projectBaseDir.isEmpty()) {
        searchPaths << QDir::cleanPath(projectBaseDir + "/translations/gnss_simulator_ru.qm");
        qDebug() << "Project base directory found:" << projectBaseDir;
    }
    searchPaths << QDir::cleanPath(appDir + "/../translations/gnss_simulator_ru.qm")
                << QDir::cleanPath(appDir + "/../../translations/gnss_simulator_ru.qm")
                << ":/translations/gnss_simulator_ru.qm"
                << appDir + "/translations/gnss_simulator_ru.qm";
    bool loaded = false;
    for (const QString &path : searchPaths) {
        if (QFile::exists(path) && translator.load(path)) {
            app.installTranslator(&translator);
            qDebug() << "Successfully loaded translation from:" << path;
            loaded = true;
            break;
        }
    }
    if (!loaded) {
        qDebug() << "Failed to load translation. Searched in:";
        for (const QString &path : searchPaths) {
            qDebug() << "-" << path << "(exists:" << QFile::exists(path) << ")";
        }
    }

    Dialog dialog;
    dialog.show();
    return app.exec();
}
