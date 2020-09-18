#include "mainwindow.h"
#include <QApplication>
#include <iostream>

int main(int argc, char *argv[])
{
    if(argc == 1)
    {
        std::cout << "Please add name of rrt package" << std::endl;
        return -1;
    }
    QApplication a(argc, argv);
    MainWindow w(argc, argv);
    w.show();

    w.changeSize();

    return a.exec();
}
