#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    qDebug() << "### in main function";
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
