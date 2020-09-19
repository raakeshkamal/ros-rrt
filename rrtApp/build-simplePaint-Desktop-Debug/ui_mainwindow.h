/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QWidget>
#include <canvaswidget.h>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    QSplitter *splitter;
    QLabel *brushSize;
    QWidget *widget;
    QHBoxLayout *horizontalLayout_4;
    QLabel *brushSize_label;
    QSlider *brushSize_slider;
    QLabel *Resize;
    QWidget *widget1;
    QHBoxLayout *horizontalLayout;
    QLabel *widthLabel;
    QLabel *cross;
    QLabel *heightLabel;
    QLabel *StartPos;
    QWidget *widget2;
    QHBoxLayout *horizontalLayout_2;
    QLabel *startY;
    QLabel *comma;
    QLabel *startX;
    QLabel *EndPos;
    QWidget *widget3;
    QHBoxLayout *horizontalLayout_3;
    QLabel *endX;
    QLabel *comma2;
    QLabel *endY;
    QLabel *stepSize;
    QLineEdit *stepSizeValue;
    QLabel *maxIter;
    QLineEdit *maxIterValue;
    QLabel *neighbouFactor;
    QLineEdit *neighbourFactorValue;
    QLabel *costToGoFactor;
    QLineEdit *costToGoFactorValue;
    QLabel *algoSpeed;
    QLineEdit *algoSpeedValue;
    QLabel *maxRuns;
    QLineEdit *maxRunsValue;
    QWidget *widget4;
    QHBoxLayout *horizontalLayout_6;
    QPushButton *saveButton;
    QPushButton *clearAll;
    CanvasWidget *canvasWidget;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(687, 676);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        centralWidget->setStyleSheet(QLatin1String("QPushButton {\n"
"min-width: 30;\n"
"max-width: 30;\n"
"min-height: 30;\n"
"max-height: 30;\n"
"border: 1 solid black;\n"
"}\n"
"\n"
"#colorIndicator {\n"
"min-width: 20;\n"
"max-width: 20;\n"
"min-height: 20;\n"
"max-height: 20;\n"
"}"));
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        splitter = new QSplitter(centralWidget);
        splitter->setObjectName(QStringLiteral("splitter"));
        splitter->setOrientation(Qt::Vertical);
        brushSize = new QLabel(splitter);
        brushSize->setObjectName(QStringLiteral("brushSize"));
        QFont font;
        font.setFamily(QStringLiteral("Ubuntu"));
        font.setPointSize(12);
        font.setBold(false);
        font.setItalic(false);
        font.setWeight(50);
        brushSize->setFont(font);
        splitter->addWidget(brushSize);
        widget = new QWidget(splitter);
        widget->setObjectName(QStringLiteral("widget"));
        horizontalLayout_4 = new QHBoxLayout(widget);
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        horizontalLayout_4->setContentsMargins(5, 0, 5, 0);
        brushSize_label = new QLabel(widget);
        brushSize_label->setObjectName(QStringLiteral("brushSize_label"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(brushSize_label->sizePolicy().hasHeightForWidth());
        brushSize_label->setSizePolicy(sizePolicy);
        brushSize_label->setMinimumSize(QSize(10, 0));

        horizontalLayout_4->addWidget(brushSize_label);

        brushSize_slider = new QSlider(widget);
        brushSize_slider->setObjectName(QStringLiteral("brushSize_slider"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(brushSize_slider->sizePolicy().hasHeightForWidth());
        brushSize_slider->setSizePolicy(sizePolicy1);
        brushSize_slider->setMinimum(1);
        brushSize_slider->setMaximum(40);
        brushSize_slider->setPageStep(5);
        brushSize_slider->setValue(5);
        brushSize_slider->setOrientation(Qt::Horizontal);

        horizontalLayout_4->addWidget(brushSize_slider);

        splitter->addWidget(widget);
        Resize = new QLabel(splitter);
        Resize->setObjectName(QStringLiteral("Resize"));
        QFont font1;
        font1.setPointSize(12);
        Resize->setFont(font1);
        splitter->addWidget(Resize);
        widget1 = new QWidget(splitter);
        widget1->setObjectName(QStringLiteral("widget1"));
        horizontalLayout = new QHBoxLayout(widget1);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        widthLabel = new QLabel(widget1);
        widthLabel->setObjectName(QStringLiteral("widthLabel"));

        horizontalLayout->addWidget(widthLabel);

        cross = new QLabel(widget1);
        cross->setObjectName(QStringLiteral("cross"));
        QSizePolicy sizePolicy2(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(cross->sizePolicy().hasHeightForWidth());
        cross->setSizePolicy(sizePolicy2);

        horizontalLayout->addWidget(cross);

        heightLabel = new QLabel(widget1);
        heightLabel->setObjectName(QStringLiteral("heightLabel"));

        horizontalLayout->addWidget(heightLabel);

        splitter->addWidget(widget1);
        StartPos = new QLabel(splitter);
        StartPos->setObjectName(QStringLiteral("StartPos"));
        splitter->addWidget(StartPos);
        widget2 = new QWidget(splitter);
        widget2->setObjectName(QStringLiteral("widget2"));
        horizontalLayout_2 = new QHBoxLayout(widget2);
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        startY = new QLabel(widget2);
        startY->setObjectName(QStringLiteral("startY"));

        horizontalLayout_2->addWidget(startY);

        comma = new QLabel(widget2);
        comma->setObjectName(QStringLiteral("comma"));

        horizontalLayout_2->addWidget(comma);

        startX = new QLabel(widget2);
        startX->setObjectName(QStringLiteral("startX"));

        horizontalLayout_2->addWidget(startX);

        splitter->addWidget(widget2);
        EndPos = new QLabel(splitter);
        EndPos->setObjectName(QStringLiteral("EndPos"));
        splitter->addWidget(EndPos);
        widget3 = new QWidget(splitter);
        widget3->setObjectName(QStringLiteral("widget3"));
        horizontalLayout_3 = new QHBoxLayout(widget3);
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(0, 0, 0, 0);
        endX = new QLabel(widget3);
        endX->setObjectName(QStringLiteral("endX"));

        horizontalLayout_3->addWidget(endX);

        comma2 = new QLabel(widget3);
        comma2->setObjectName(QStringLiteral("comma2"));

        horizontalLayout_3->addWidget(comma2);

        endY = new QLabel(widget3);
        endY->setObjectName(QStringLiteral("endY"));

        horizontalLayout_3->addWidget(endY);

        splitter->addWidget(widget3);
        stepSize = new QLabel(splitter);
        stepSize->setObjectName(QStringLiteral("stepSize"));
        splitter->addWidget(stepSize);
        stepSizeValue = new QLineEdit(splitter);
        stepSizeValue->setObjectName(QStringLiteral("stepSizeValue"));
        stepSizeValue->setMaximumSize(QSize(140, 30));
        splitter->addWidget(stepSizeValue);
        maxIter = new QLabel(splitter);
        maxIter->setObjectName(QStringLiteral("maxIter"));
        splitter->addWidget(maxIter);
        maxIterValue = new QLineEdit(splitter);
        maxIterValue->setObjectName(QStringLiteral("maxIterValue"));
        maxIterValue->setMaximumSize(QSize(140, 30));
        splitter->addWidget(maxIterValue);
        neighbouFactor = new QLabel(splitter);
        neighbouFactor->setObjectName(QStringLiteral("neighbouFactor"));
        splitter->addWidget(neighbouFactor);
        neighbourFactorValue = new QLineEdit(splitter);
        neighbourFactorValue->setObjectName(QStringLiteral("neighbourFactorValue"));
        neighbourFactorValue->setMaximumSize(QSize(140, 30));
        splitter->addWidget(neighbourFactorValue);
        costToGoFactor = new QLabel(splitter);
        costToGoFactor->setObjectName(QStringLiteral("costToGoFactor"));
        splitter->addWidget(costToGoFactor);
        costToGoFactorValue = new QLineEdit(splitter);
        costToGoFactorValue->setObjectName(QStringLiteral("costToGoFactorValue"));
        costToGoFactorValue->setMaximumSize(QSize(140, 30));
        splitter->addWidget(costToGoFactorValue);
        algoSpeed = new QLabel(splitter);
        algoSpeed->setObjectName(QStringLiteral("algoSpeed"));
        splitter->addWidget(algoSpeed);
        algoSpeedValue = new QLineEdit(splitter);
        algoSpeedValue->setObjectName(QStringLiteral("algoSpeedValue"));
        algoSpeedValue->setMaximumSize(QSize(140, 30));
        splitter->addWidget(algoSpeedValue);
        maxRuns = new QLabel(splitter);
        maxRuns->setObjectName(QStringLiteral("maxRuns"));
        splitter->addWidget(maxRuns);
        maxRunsValue = new QLineEdit(splitter);
        maxRunsValue->setObjectName(QStringLiteral("maxRunsValue"));
        maxRunsValue->setMaximumSize(QSize(140, 30));
        splitter->addWidget(maxRunsValue);
        widget4 = new QWidget(splitter);
        widget4->setObjectName(QStringLiteral("widget4"));
        horizontalLayout_6 = new QHBoxLayout(widget4);
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        horizontalLayout_6->setContentsMargins(0, 0, 0, 0);
        saveButton = new QPushButton(widget4);
        saveButton->setObjectName(QStringLiteral("saveButton"));
        saveButton->setStyleSheet(QLatin1String("min-width: 64;\n"
"max-width: 64;"));

        horizontalLayout_6->addWidget(saveButton);

        clearAll = new QPushButton(widget4);
        clearAll->setObjectName(QStringLiteral("clearAll"));
        clearAll->setMinimumSize(QSize(66, 32));
        clearAll->setMaximumSize(QSize(66, 32));
        clearAll->setStyleSheet(QLatin1String("min-width: 64;\n"
"max-width: 64;"));

        horizontalLayout_6->addWidget(clearAll);

        splitter->addWidget(widget4);

        gridLayout->addWidget(splitter, 0, 0, 1, 1);

        canvasWidget = new CanvasWidget(centralWidget);
        canvasWidget->setObjectName(QStringLiteral("canvasWidget"));
        canvasWidget->setEnabled(true);
        QSizePolicy sizePolicy3(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(canvasWidget->sizePolicy().hasHeightForWidth());
        canvasWidget->setSizePolicy(sizePolicy3);

        gridLayout->addWidget(canvasWidget, 0, 1, 1, 1);

        MainWindow->setCentralWidget(centralWidget);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Simple Paint", Q_NULLPTR));
        brushSize->setText(QApplication::translate("MainWindow", "Brush Size", Q_NULLPTR));
        brushSize_label->setText(QApplication::translate("MainWindow", "0", Q_NULLPTR));
        Resize->setText(QApplication::translate("MainWindow", "Width X Height", Q_NULLPTR));
        widthLabel->setText(QApplication::translate("MainWindow", "0", Q_NULLPTR));
        cross->setText(QApplication::translate("MainWindow", "X", Q_NULLPTR));
        heightLabel->setText(QApplication::translate("MainWindow", "0", Q_NULLPTR));
        StartPos->setText(QApplication::translate("MainWindow", "Start Pos", Q_NULLPTR));
        startY->setText(QApplication::translate("MainWindow", "0", Q_NULLPTR));
        comma->setText(QApplication::translate("MainWindow", ",", Q_NULLPTR));
        startX->setText(QApplication::translate("MainWindow", "0", Q_NULLPTR));
        EndPos->setText(QApplication::translate("MainWindow", "End Pos", Q_NULLPTR));
        endX->setText(QApplication::translate("MainWindow", "0", Q_NULLPTR));
        comma2->setText(QApplication::translate("MainWindow", ",", Q_NULLPTR));
        endY->setText(QApplication::translate("MainWindow", "0", Q_NULLPTR));
        stepSize->setText(QApplication::translate("MainWindow", "Step Size", Q_NULLPTR));
        maxIter->setText(QApplication::translate("MainWindow", "Max. Iterations", Q_NULLPTR));
        neighbouFactor->setText(QApplication::translate("MainWindow", "Neigbour Factor", Q_NULLPTR));
        costToGoFactor->setText(QApplication::translate("MainWindow", "Cost To Go Factor", Q_NULLPTR));
        algoSpeed->setText(QApplication::translate("MainWindow", "algo. Speed", Q_NULLPTR));
        maxRuns->setText(QApplication::translate("MainWindow", "Max. Runs", Q_NULLPTR));
        saveButton->setText(QApplication::translate("MainWindow", "Save", Q_NULLPTR));
        clearAll->setText(QApplication::translate("MainWindow", "Clear all", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
