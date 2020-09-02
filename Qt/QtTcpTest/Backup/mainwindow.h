#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "serialtest.h"
#include "ipsetting.h"

#include <QMainWindow>
#include <QDebug>
#include <QTcpServer>
#include <QTcpSocket>
#include <QString>
#include <QByteArray>
#include <QSerialPort>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();


public slots:
    void Connection();
    void LogicalControlCenter(QString command);
    void start();
    void init();
    QString readSockRC();
    QString readSockPR();
    void sendSockRC(QString);
    void sendSockPR(QString);
    void sendBtnRC();
    void sendBtnPR();
    void stop();
//    void serialArduinoGrasp();
//    void serialArduinoRelease();
private slots:
    void serialArduino();
    void handleError(QSerialPort::SerialPortError error);
    void serialTest();
    void ipSetting();

private:
    void coordinateProcessing(QString);  // Get the v_distance and h_distance out of command qstring.
    void postureProcessing(QString);  // Get the gradient and direction out of command qstring.
    void serialCommunication(QByteArray);  // Send request to MCU gripper

private:
    Ui::MainWindow *ui;
    QTcpServer *server;
    QTcpSocket *sockRC;
    QTcpSocket *sockPR;
    float v_distance = 0.0;  // The robot end should move by this distance vertically ( in camera vision ).
    float h_distance = 0.0;  // The robot end should move by this distance horizontally ( in camera vision ).
//    QStringRef coordinate;
//    QStringRef posture;
    float gradient = 0.0;  // The robot end ( the last motor) should rotate by the gradient.
    int direction = 0;  // The direction of the target object.
    int numConnection = 0;
    bool stopSignal = false;  // true means the "Stop" button is clicked
    bool RCSetupFlag = false;  // Once the RC module sets up successfully and replies a status code "2", RCSetupFlag turns to true.
    QString current_ip = "127.0.0.1";


    QSerialPort serialPort;
    QByteArray m_readData;
    SerialTest *serialtest;
    IPSetting *set_ip;

};

#endif // MAINWINDOW_H
