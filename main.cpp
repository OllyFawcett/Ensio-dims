#include "mainwindow.h"
#include <QApplication>
#include <QFile>
#include <QDir>
#include <QFileInfo>



int main(int argc, char *argv[])
{

    QApplication a(argc, argv);

    //QFile styleFile("C:/Users/Olly/Downloads/Darkeum/Darkeum.qss");
    //styleFile.open( QFile::ReadOnly );

    // Apply the loaded stylesheet
    //QString style( styleFile.readAll() );
    //a.setStyleSheet( style );
    MainWindow w;
    //w.setStyleSheet(style);
    w.setWindowState(Qt::WindowMaximized);
    w.show();
    return a.exec();
}
