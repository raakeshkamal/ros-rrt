#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QAbstractButton>
#include <QColorDialog>
#include <QFileDialog>
#include <QProcess>
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include "canvaswidget.h"


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc, char *argv[],QWidget *parent = 0);
    ~MainWindow();
    void changeSize();

public slots:
    void updatePosLabel();
private slots:

    void on_brushSize_slider_sliderReleased();
    void on_brushSize_slider_sliderMoved(int position);
    void on_brushSize_slider_valueChanged(int value);

    void on_clearAll_clicked();

    void on_saveButton_clicked();

private:
    Ui::MainWindow *ui;
    QString yamlData[6];
    QString launchData[20];
    bool rrt,rrtStar,anytimeRRT;
    QString startX,startY;
    QString endX,endY;
protected:
    void resizeEvent(QResizeEvent* event) override;
};

#endif // MAINWINDOW_H
