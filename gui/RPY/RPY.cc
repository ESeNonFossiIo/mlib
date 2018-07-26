#include "lib_RPY.h"
#include <QApplication>
#include <QFile>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    MainWindow w;

    QFile qss("lib_RPY.qss");
    qss.open(QFile::ReadOnly);
    app.setStyleSheet(qss.readAll());
    qss.close();

    w.show();

    return app.exec();
}
