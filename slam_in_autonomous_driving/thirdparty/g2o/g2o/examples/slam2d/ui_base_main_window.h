/********************************************************************************
** Form generated from reading UI file 'base_main_window.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_BASE_MAIN_WINDOW_H
#define UI_BASE_MAIN_WINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "slam2d_viewer.h"

QT_BEGIN_NAMESPACE

class Ui_BaseMainWindow
{
public:
    QAction *actionLoad;
    QAction *actionSave;
    QAction *actionQuit;
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout;
    QFrame *frame;
    QVBoxLayout *verticalLayout_2;
    QLabel *label;
    QSpinBox *spIterations;
    QCheckBox *cbCovariances;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout;
    QRadioButton *rbGauss;
    QRadioButton *rbLevenberg;
    QPushButton *btnInitialGuess;
    QPushButton *btnOptimize;
    QPushButton *btnQuit;
    QSpacerItem *verticalSpacer;
    g2o::Slam2DViewer *viewer;
    QMenuBar *menubar;
    QMenu *menuFile;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *BaseMainWindow)
    {
        if (BaseMainWindow->objectName().isEmpty())
            BaseMainWindow->setObjectName(QString::fromUtf8("BaseMainWindow"));
        BaseMainWindow->resize(800, 600);
        actionLoad = new QAction(BaseMainWindow);
        actionLoad->setObjectName(QString::fromUtf8("actionLoad"));
        actionSave = new QAction(BaseMainWindow);
        actionSave->setObjectName(QString::fromUtf8("actionSave"));
        actionQuit = new QAction(BaseMainWindow);
        actionQuit->setObjectName(QString::fromUtf8("actionQuit"));
        centralwidget = new QWidget(BaseMainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        horizontalLayout = new QHBoxLayout(centralwidget);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        frame = new QFrame(centralwidget);
        frame->setObjectName(QString::fromUtf8("frame"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(frame->sizePolicy().hasHeightForWidth());
        frame->setSizePolicy(sizePolicy);
        frame->setMinimumSize(QSize(150, 0));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        verticalLayout_2 = new QVBoxLayout(frame);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        label = new QLabel(frame);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout_2->addWidget(label);

        spIterations = new QSpinBox(frame);
        spIterations->setObjectName(QString::fromUtf8("spIterations"));
        spIterations->setMinimum(1);
        spIterations->setMaximum(100);
        spIterations->setValue(10);

        verticalLayout_2->addWidget(spIterations);

        cbCovariances = new QCheckBox(frame);
        cbCovariances->setObjectName(QString::fromUtf8("cbCovariances"));

        verticalLayout_2->addWidget(cbCovariances);

        groupBox = new QGroupBox(frame);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        verticalLayout = new QVBoxLayout(groupBox);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        rbGauss = new QRadioButton(groupBox);
        rbGauss->setObjectName(QString::fromUtf8("rbGauss"));
        rbGauss->setChecked(true);

        verticalLayout->addWidget(rbGauss);

        rbLevenberg = new QRadioButton(groupBox);
        rbLevenberg->setObjectName(QString::fromUtf8("rbLevenberg"));

        verticalLayout->addWidget(rbLevenberg);


        verticalLayout_2->addWidget(groupBox);

        btnInitialGuess = new QPushButton(frame);
        btnInitialGuess->setObjectName(QString::fromUtf8("btnInitialGuess"));

        verticalLayout_2->addWidget(btnInitialGuess);

        btnOptimize = new QPushButton(frame);
        btnOptimize->setObjectName(QString::fromUtf8("btnOptimize"));

        verticalLayout_2->addWidget(btnOptimize);

        btnQuit = new QPushButton(frame);
        btnQuit->setObjectName(QString::fromUtf8("btnQuit"));

        verticalLayout_2->addWidget(btnQuit);

        verticalSpacer = new QSpacerItem(20, 364, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer);


        horizontalLayout->addWidget(frame);

        viewer = new g2o::Slam2DViewer(centralwidget);
        viewer->setObjectName(QString::fromUtf8("viewer"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(1);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(viewer->sizePolicy().hasHeightForWidth());
        viewer->setSizePolicy(sizePolicy1);

        horizontalLayout->addWidget(viewer);

        BaseMainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(BaseMainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 800, 23));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        BaseMainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(BaseMainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        BaseMainWindow->setStatusBar(statusbar);

        menubar->addAction(menuFile->menuAction());
        menuFile->addAction(actionLoad);
        menuFile->addAction(actionSave);
        menuFile->addSeparator();
        menuFile->addAction(actionQuit);

        retranslateUi(BaseMainWindow);
        QObject::connect(btnQuit, SIGNAL(clicked()), BaseMainWindow, SLOT(close()));

        QMetaObject::connectSlotsByName(BaseMainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *BaseMainWindow)
    {
        BaseMainWindow->setWindowTitle(QApplication::translate("BaseMainWindow", "g2o SLAM2D Demo", nullptr));
        actionLoad->setText(QApplication::translate("BaseMainWindow", "Load", nullptr));
        actionSave->setText(QApplication::translate("BaseMainWindow", "Save", nullptr));
        actionQuit->setText(QApplication::translate("BaseMainWindow", "Quit", nullptr));
        label->setText(QApplication::translate("BaseMainWindow", "# Iterations", nullptr));
        cbCovariances->setText(QApplication::translate("BaseMainWindow", "Covariances", nullptr));
        groupBox->setTitle(QApplication::translate("BaseMainWindow", "Method", nullptr));
        rbGauss->setText(QApplication::translate("BaseMainWindow", "Gauss-Newton", nullptr));
        rbLevenberg->setText(QApplication::translate("BaseMainWindow", "Levenberg", nullptr));
        btnInitialGuess->setText(QApplication::translate("BaseMainWindow", "Initial Guess", nullptr));
        btnOptimize->setText(QApplication::translate("BaseMainWindow", "Optimize", nullptr));
        btnQuit->setText(QApplication::translate("BaseMainWindow", "Quit", nullptr));
        menuFile->setTitle(QApplication::translate("BaseMainWindow", "File", nullptr));
    } // retranslateUi

};

namespace Ui {
    class BaseMainWindow: public Ui_BaseMainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_BASE_MAIN_WINDOW_H
