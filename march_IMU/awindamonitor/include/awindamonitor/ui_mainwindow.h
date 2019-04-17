/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QListWidget>
#include <QtGui/QMainWindow>
#include <QtGui/QProgressBar>
#include <QtGui/QPushButton>
#include <QtGui/QTextBrowser>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QGroupBox *connectedMtwListGroupBox;
    QListWidget *connectedMtwList;
    QCheckBox *pitchToSelectCheckBox;
    QGroupBox *mtwPropertiesGroupBox;
    QLabel *batteryLevelCaptionLabel;
    QLabel *effUpdateRateCaptionLabel;
    QLabel *rssiCaptionLabel;
    QLabel *rssiLabel;
    QLabel *batteryLevelLabel;
    QLabel *effUpdateRateLabel;
    QLabel *rollLabel;
    QLabel *rollCaptionLabel;
    QLabel *yawCaptionLabel;
    QLabel *pitchLabel;
    QLabel *pitchCaptionLabel;
    QLabel *yawLabel;
    QGroupBox *stationPropertiesGroupBox;
    QLabel *updateRateCaptionLabel;
    QComboBox *allowedUpdateRatesComboBox;
    QLabel *channelCaptionLabel;
    QComboBox *channelComboBox;
    QLabel *stationIdCaptionLabel;
    QLabel *stationIdLabel;
    QPushButton *enableButton;
    QPushButton *startMeasurementButton;
    QPushButton *recordingButton;
    QLineEdit *logFilenameEdit;
    QLabel *logFilenameCaptionLabel;
    QProgressBar *flushingProgressBar;
    QLabel *flushingCaptionLabel;
    QGroupBox *loggingGroupBox;
    QTextBrowser *logWindow;
    QPushButton *clearLogPushButton;
    QGroupBox *stateMachineImageGroupBox;
    QLabel *stateDiagramLabel;
    QGroupBox *dockedMtwListGroupBox;
    QListWidget *dockedMtwList;
    QLabel *logoLabel;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(824, 348);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
        MainWindow->setSizePolicy(sizePolicy);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        connectedMtwListGroupBox = new QGroupBox(centralWidget);
        connectedMtwListGroupBox->setObjectName(QString::fromUtf8("connectedMtwListGroupBox"));
        connectedMtwListGroupBox->setGeometry(QRect(170, 170, 151, 171));
        connectedMtwList = new QListWidget(connectedMtwListGroupBox);
        connectedMtwList->setObjectName(QString::fromUtf8("connectedMtwList"));
        connectedMtwList->setGeometry(QRect(10, 20, 131, 111));
        connectedMtwList->setSortingEnabled(true);
        pitchToSelectCheckBox = new QCheckBox(connectedMtwListGroupBox);
        pitchToSelectCheckBox->setObjectName(QString::fromUtf8("pitchToSelectCheckBox"));
        pitchToSelectCheckBox->setGeometry(QRect(10, 140, 101, 20));
        mtwPropertiesGroupBox = new QGroupBox(centralWidget);
        mtwPropertiesGroupBox->setObjectName(QString::fromUtf8("mtwPropertiesGroupBox"));
        mtwPropertiesGroupBox->setGeometry(QRect(330, 170, 161, 171));
        batteryLevelCaptionLabel = new QLabel(mtwPropertiesGroupBox);
        batteryLevelCaptionLabel->setObjectName(QString::fromUtf8("batteryLevelCaptionLabel"));
        batteryLevelCaptionLabel->setGeometry(QRect(20, 30, 71, 16));
        batteryLevelCaptionLabel->setScaledContents(true);
        effUpdateRateCaptionLabel = new QLabel(mtwPropertiesGroupBox);
        effUpdateRateCaptionLabel->setObjectName(QString::fromUtf8("effUpdateRateCaptionLabel"));
        effUpdateRateCaptionLabel->setGeometry(QRect(20, 78, 91, 16));
        rssiCaptionLabel = new QLabel(mtwPropertiesGroupBox);
        rssiCaptionLabel->setObjectName(QString::fromUtf8("rssiCaptionLabel"));
        rssiCaptionLabel->setEnabled(true);
        rssiCaptionLabel->setGeometry(QRect(20, 54, 46, 16));
        rssiLabel = new QLabel(mtwPropertiesGroupBox);
        rssiLabel->setObjectName(QString::fromUtf8("rssiLabel"));
        rssiLabel->setGeometry(QRect(100, 54, 50, 13));
        rssiLabel->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        batteryLevelLabel = new QLabel(mtwPropertiesGroupBox);
        batteryLevelLabel->setObjectName(QString::fromUtf8("batteryLevelLabel"));
        batteryLevelLabel->setGeometry(QRect(100, 30, 50, 16));
        batteryLevelLabel->setLayoutDirection(Qt::LeftToRight);
        batteryLevelLabel->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        effUpdateRateLabel = new QLabel(mtwPropertiesGroupBox);
        effUpdateRateLabel->setObjectName(QString::fromUtf8("effUpdateRateLabel"));
        effUpdateRateLabel->setGeometry(QRect(100, 78, 50, 16));
        effUpdateRateLabel->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        rollLabel = new QLabel(mtwPropertiesGroupBox);
        rollLabel->setObjectName(QString::fromUtf8("rollLabel"));
        rollLabel->setGeometry(QRect(79, 102, 71, 20));
        rollLabel->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        rollCaptionLabel = new QLabel(mtwPropertiesGroupBox);
        rollCaptionLabel->setObjectName(QString::fromUtf8("rollCaptionLabel"));
        rollCaptionLabel->setGeometry(QRect(20, 102, 46, 16));
        yawCaptionLabel = new QLabel(mtwPropertiesGroupBox);
        yawCaptionLabel->setObjectName(QString::fromUtf8("yawCaptionLabel"));
        yawCaptionLabel->setGeometry(QRect(20, 150, 46, 16));
        pitchLabel = new QLabel(mtwPropertiesGroupBox);
        pitchLabel->setObjectName(QString::fromUtf8("pitchLabel"));
        pitchLabel->setGeometry(QRect(79, 126, 71, 20));
        pitchLabel->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        pitchCaptionLabel = new QLabel(mtwPropertiesGroupBox);
        pitchCaptionLabel->setObjectName(QString::fromUtf8("pitchCaptionLabel"));
        pitchCaptionLabel->setGeometry(QRect(20, 126, 46, 16));
        yawLabel = new QLabel(mtwPropertiesGroupBox);
        yawLabel->setObjectName(QString::fromUtf8("yawLabel"));
        yawLabel->setGeometry(QRect(79, 150, 71, 20));
        yawLabel->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        stationPropertiesGroupBox = new QGroupBox(centralWidget);
        stationPropertiesGroupBox->setObjectName(QString::fromUtf8("stationPropertiesGroupBox"));
        stationPropertiesGroupBox->setGeometry(QRect(10, 10, 151, 331));
        updateRateCaptionLabel = new QLabel(stationPropertiesGroupBox);
        updateRateCaptionLabel->setObjectName(QString::fromUtf8("updateRateCaptionLabel"));
        updateRateCaptionLabel->setEnabled(false);
        updateRateCaptionLabel->setGeometry(QRect(10, 120, 61, 20));
        allowedUpdateRatesComboBox = new QComboBox(stationPropertiesGroupBox);
        allowedUpdateRatesComboBox->setObjectName(QString::fromUtf8("allowedUpdateRatesComboBox"));
        allowedUpdateRatesComboBox->setEnabled(false);
        allowedUpdateRatesComboBox->setGeometry(QRect(80, 120, 61, 22));
        channelCaptionLabel = new QLabel(stationPropertiesGroupBox);
        channelCaptionLabel->setObjectName(QString::fromUtf8("channelCaptionLabel"));
        channelCaptionLabel->setEnabled(false);
        channelCaptionLabel->setGeometry(QRect(10, 50, 46, 13));
        channelComboBox = new QComboBox(stationPropertiesGroupBox);
        channelComboBox->setObjectName(QString::fromUtf8("channelComboBox"));
        channelComboBox->setEnabled(false);
        channelComboBox->setGeometry(QRect(90, 41, 51, 31));
        stationIdCaptionLabel = new QLabel(stationPropertiesGroupBox);
        stationIdCaptionLabel->setObjectName(QString::fromUtf8("stationIdCaptionLabel"));
        stationIdCaptionLabel->setGeometry(QRect(10, 20, 46, 13));
        stationIdLabel = new QLabel(stationPropertiesGroupBox);
        stationIdLabel->setObjectName(QString::fromUtf8("stationIdLabel"));
        stationIdLabel->setGeometry(QRect(30, 20, 61, 16));
        stationIdLabel->setScaledContents(false);
        enableButton = new QPushButton(stationPropertiesGroupBox);
        enableButton->setObjectName(QString::fromUtf8("enableButton"));
        enableButton->setEnabled(false);
        enableButton->setGeometry(QRect(10, 80, 131, 31));
        startMeasurementButton = new QPushButton(stationPropertiesGroupBox);
        startMeasurementButton->setObjectName(QString::fromUtf8("startMeasurementButton"));
        startMeasurementButton->setEnabled(false);
        startMeasurementButton->setGeometry(QRect(10, 150, 131, 31));
        recordingButton = new QPushButton(stationPropertiesGroupBox);
        recordingButton->setObjectName(QString::fromUtf8("recordingButton"));
        recordingButton->setEnabled(false);
        recordingButton->setGeometry(QRect(10, 220, 131, 31));
        logFilenameEdit = new QLineEdit(stationPropertiesGroupBox);
        logFilenameEdit->setObjectName(QString::fromUtf8("logFilenameEdit"));
        logFilenameEdit->setEnabled(false);
        logFilenameEdit->setGeometry(QRect(60, 190, 81, 21));
        logFilenameCaptionLabel = new QLabel(stationPropertiesGroupBox);
        logFilenameCaptionLabel->setObjectName(QString::fromUtf8("logFilenameCaptionLabel"));
        logFilenameCaptionLabel->setEnabled(false);
        logFilenameCaptionLabel->setGeometry(QRect(10, 190, 46, 21));
        flushingProgressBar = new QProgressBar(stationPropertiesGroupBox);
        flushingProgressBar->setObjectName(QString::fromUtf8("flushingProgressBar"));
        flushingProgressBar->setEnabled(false);
        flushingProgressBar->setGeometry(QRect(60, 260, 81, 23));
        flushingProgressBar->setValue(0);
        flushingProgressBar->setTextVisible(false);
        flushingProgressBar->setInvertedAppearance(true);
        flushingCaptionLabel = new QLabel(stationPropertiesGroupBox);
        flushingCaptionLabel->setObjectName(QString::fromUtf8("flushingCaptionLabel"));
        flushingCaptionLabel->setEnabled(false);
        flushingCaptionLabel->setGeometry(QRect(10, 260, 46, 20));
        loggingGroupBox = new QGroupBox(centralWidget);
        loggingGroupBox->setObjectName(QString::fromUtf8("loggingGroupBox"));
        loggingGroupBox->setGeometry(QRect(330, 10, 341, 141));
        logWindow = new QTextBrowser(loggingGroupBox);
        logWindow->setObjectName(QString::fromUtf8("logWindow"));
        logWindow->setGeometry(QRect(10, 20, 321, 101));
        clearLogPushButton = new QPushButton(loggingGroupBox);
        clearLogPushButton->setObjectName(QString::fromUtf8("clearLogPushButton"));
        clearLogPushButton->setGeometry(QRect(10, 125, 321, 10));
        stateMachineImageGroupBox = new QGroupBox(centralWidget);
        stateMachineImageGroupBox->setObjectName(QString::fromUtf8("stateMachineImageGroupBox"));
        stateMachineImageGroupBox->setGeometry(QRect(500, 170, 311, 171));
        stateDiagramLabel = new QLabel(stateMachineImageGroupBox);
        stateDiagramLabel->setObjectName(QString::fromUtf8("stateDiagramLabel"));
        stateDiagramLabel->setGeometry(QRect(10, 20, 291, 141));
        dockedMtwListGroupBox = new QGroupBox(centralWidget);
        dockedMtwListGroupBox->setObjectName(QString::fromUtf8("dockedMtwListGroupBox"));
        dockedMtwListGroupBox->setGeometry(QRect(170, 10, 151, 141));
        dockedMtwList = new QListWidget(dockedMtwListGroupBox);
        dockedMtwList->setObjectName(QString::fromUtf8("dockedMtwList"));
        dockedMtwList->setGeometry(QRect(10, 20, 131, 111));
        dockedMtwList->setSortingEnabled(true);
        logoLabel = new QLabel(centralWidget);
        logoLabel->setObjectName(QString::fromUtf8("logoLabel"));
        logoLabel->setGeometry(QRect(690, 20, 121, 131));
        MainWindow->setCentralWidget(centralWidget);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Awinda monitor", 0, QApplication::UnicodeUTF8));
        connectedMtwListGroupBox->setTitle(QApplication::translate("MainWindow", "Connected MTw list (0):", 0, QApplication::UnicodeUTF8));
        pitchToSelectCheckBox->setText(QApplication::translate("MainWindow", "Pitch to select", 0, QApplication::UnicodeUTF8));
        mtwPropertiesGroupBox->setTitle(QApplication::translate("MainWindow", "Selected MTw properties:", 0, QApplication::UnicodeUTF8));
        batteryLevelCaptionLabel->setText(QApplication::translate("MainWindow", "Battery level:", 0, QApplication::UnicodeUTF8));
        effUpdateRateCaptionLabel->setText(QApplication::translate("MainWindow", "Eff. update rate:", 0, QApplication::UnicodeUTF8));
        rssiCaptionLabel->setText(QApplication::translate("MainWindow", "RSSI:", 0, QApplication::UnicodeUTF8));
        rssiLabel->setText(QApplication::translate("MainWindow", "-", 0, QApplication::UnicodeUTF8));
        batteryLevelLabel->setText(QApplication::translate("MainWindow", "-", 0, QApplication::UnicodeUTF8));
        effUpdateRateLabel->setText(QApplication::translate("MainWindow", "-", 0, QApplication::UnicodeUTF8));
        rollLabel->setText(QApplication::translate("MainWindow", "-", 0, QApplication::UnicodeUTF8));
        rollCaptionLabel->setText(QApplication::translate("MainWindow", "Roll:", 0, QApplication::UnicodeUTF8));
        yawCaptionLabel->setText(QApplication::translate("MainWindow", "Yaw:", 0, QApplication::UnicodeUTF8));
        pitchLabel->setText(QApplication::translate("MainWindow", "-", 0, QApplication::UnicodeUTF8));
        pitchCaptionLabel->setText(QApplication::translate("MainWindow", "Pitch:", 0, QApplication::UnicodeUTF8));
        yawLabel->setText(QApplication::translate("MainWindow", "-", 0, QApplication::UnicodeUTF8));
        stationPropertiesGroupBox->setTitle(QApplication::translate("MainWindow", "Wireless master properties:", 0, QApplication::UnicodeUTF8));
        updateRateCaptionLabel->setText(QApplication::translate("MainWindow", "Update rate:", 0, QApplication::UnicodeUTF8));
        channelCaptionLabel->setText(QApplication::translate("MainWindow", "Channel:", 0, QApplication::UnicodeUTF8));
        channelComboBox->clear();
        channelComboBox->insertItems(0, QStringList()
         << QApplication::translate("MainWindow", "11", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "12", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "13", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "14", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "15", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "16", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "17", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "18", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "19", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "20", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "21", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "22", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "23", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "24", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("MainWindow", "25", 0, QApplication::UnicodeUTF8)
        );
        stationIdCaptionLabel->setText(QApplication::translate("MainWindow", "ID:", 0, QApplication::UnicodeUTF8));
        stationIdLabel->setText(QApplication::translate("MainWindow", "-", 0, QApplication::UnicodeUTF8));
        enableButton->setText(QApplication::translate("MainWindow", "Enable", 0, QApplication::UnicodeUTF8));
        startMeasurementButton->setText(QApplication::translate("MainWindow", "Start Measurement", 0, QApplication::UnicodeUTF8));
        recordingButton->setText(QApplication::translate("MainWindow", "Start recording", 0, QApplication::UnicodeUTF8));
        logFilenameEdit->setText(QApplication::translate("MainWindow", "logfile.mtb", 0, QApplication::UnicodeUTF8));
        logFilenameCaptionLabel->setText(QApplication::translate("MainWindow", "Filename:", 0, QApplication::UnicodeUTF8));
        flushingCaptionLabel->setText(QApplication::translate("MainWindow", "Flushing:", 0, QApplication::UnicodeUTF8));
        loggingGroupBox->setTitle(QApplication::translate("MainWindow", "What's going on:", 0, QApplication::UnicodeUTF8));
        clearLogPushButton->setText(QString());
        stateMachineImageGroupBox->setTitle(QApplication::translate("MainWindow", "State diagram:", 0, QApplication::UnicodeUTF8));
        stateDiagramLabel->setText(QApplication::translate("MainWindow", "<State diagram>", 0, QApplication::UnicodeUTF8));
        dockedMtwListGroupBox->setTitle(QApplication::translate("MainWindow", "Docked MTw list (0):", 0, QApplication::UnicodeUTF8));
        logoLabel->setText(QApplication::translate("MainWindow", "<Logo label>", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
