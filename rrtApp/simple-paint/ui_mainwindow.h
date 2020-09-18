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
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include <canvaswidget.h>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    QVBoxLayout *verticalLayout;
    QLabel *brushSize;
    QHBoxLayout *horizontalLayout_4;
    QLabel *brushSize_label;
    QSlider *brushSize_slider;
    QLabel *Resize;
    QHBoxLayout *horizontalLayout;
    QLabel *widthLabel;
    QLabel *cross;
    QLabel *heightLabel;
    QLabel *StartPos;
    QHBoxLayout *horizontalLayout_2;
    QLabel *startY;
    QLabel *comma;
    QLabel *startX;
    QLabel *EndPos;
    QHBoxLayout *horizontalLayout_3;
    QLabel *endX;
    QLabel *comma2;
    QLabel *endY;
    QSpacerItem *verticalSpacer;
    QHBoxLayout *horizontalLayout_6;
    QPushButton *saveButton;
    QPushButton *clearAll;
    CanvasWidget *canvasWidget;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(666, 522);
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
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        brushSize = new QLabel(centralWidget);
        brushSize->setObjectName(QStringLiteral("brushSize"));
        QFont font;
        font.setFamily(QStringLiteral("Ubuntu"));
        font.setPointSize(12);
        font.setBold(false);
        font.setItalic(false);
        font.setWeight(50);
        brushSize->setFont(font);

        verticalLayout->addWidget(brushSize);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        horizontalLayout_4->setContentsMargins(5, 0, 5, -1);
        brushSize_label = new QLabel(centralWidget);
        brushSize_label->setObjectName(QStringLiteral("brushSize_label"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(brushSize_label->sizePolicy().hasHeightForWidth());
        brushSize_label->setSizePolicy(sizePolicy);
        brushSize_label->setMinimumSize(QSize(10, 0));

        horizontalLayout_4->addWidget(brushSize_label);

        brushSize_slider = new QSlider(centralWidget);
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


        verticalLayout->addLayout(horizontalLayout_4);

        Resize = new QLabel(centralWidget);
        Resize->setObjectName(QStringLiteral("Resize"));
        QFont font1;
        font1.setPointSize(12);
        Resize->setFont(font1);

        verticalLayout->addWidget(Resize);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        widthLabel = new QLabel(centralWidget);
        widthLabel->setObjectName(QStringLiteral("widthLabel"));

        horizontalLayout->addWidget(widthLabel);

        cross = new QLabel(centralWidget);
        cross->setObjectName(QStringLiteral("cross"));
        QSizePolicy sizePolicy2(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(cross->sizePolicy().hasHeightForWidth());
        cross->setSizePolicy(sizePolicy2);

        horizontalLayout->addWidget(cross);

        heightLabel = new QLabel(centralWidget);
        heightLabel->setObjectName(QStringLiteral("heightLabel"));

        horizontalLayout->addWidget(heightLabel);


        verticalLayout->addLayout(horizontalLayout);

        StartPos = new QLabel(centralWidget);
        StartPos->setObjectName(QStringLiteral("StartPos"));

        verticalLayout->addWidget(StartPos);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        startY = new QLabel(centralWidget);
        startY->setObjectName(QStringLiteral("startY"));

        horizontalLayout_2->addWidget(startY);

        comma = new QLabel(centralWidget);
        comma->setObjectName(QStringLiteral("comma"));

        horizontalLayout_2->addWidget(comma);

        startX = new QLabel(centralWidget);
        startX->setObjectName(QStringLiteral("startX"));

        horizontalLayout_2->addWidget(startX);


        verticalLayout->addLayout(horizontalLayout_2);

        EndPos = new QLabel(centralWidget);
        EndPos->setObjectName(QStringLiteral("EndPos"));

        verticalLayout->addWidget(EndPos);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        endX = new QLabel(centralWidget);
        endX->setObjectName(QStringLiteral("endX"));

        horizontalLayout_3->addWidget(endX);

        comma2 = new QLabel(centralWidget);
        comma2->setObjectName(QStringLiteral("comma2"));

        horizontalLayout_3->addWidget(comma2);

        endY = new QLabel(centralWidget);
        endY->setObjectName(QStringLiteral("endY"));

        horizontalLayout_3->addWidget(endY);


        verticalLayout->addLayout(horizontalLayout_3);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        horizontalLayout_6->setContentsMargins(-1, 0, -1, -1);
        saveButton = new QPushButton(centralWidget);
        saveButton->setObjectName(QStringLiteral("saveButton"));
        saveButton->setStyleSheet(QLatin1String("min-width: 64;\n"
"max-width: 64;"));

        horizontalLayout_6->addWidget(saveButton);

        clearAll = new QPushButton(centralWidget);
        clearAll->setObjectName(QStringLiteral("clearAll"));
        clearAll->setMinimumSize(QSize(66, 32));
        clearAll->setMaximumSize(QSize(66, 32));
        clearAll->setStyleSheet(QLatin1String("min-width: 64;\n"
"max-width: 64;"));

        horizontalLayout_6->addWidget(clearAll);


        verticalLayout->addLayout(horizontalLayout_6);


        gridLayout->addLayout(verticalLayout, 0, 0, 1, 1);

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
        Resize->setText(QApplication::translate("MainWindow", "Resize window", Q_NULLPTR));
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
        saveButton->setText(QApplication::translate("MainWindow", "Save", Q_NULLPTR));
        clearAll->setText(QApplication::translate("MainWindow", "Clear all", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
