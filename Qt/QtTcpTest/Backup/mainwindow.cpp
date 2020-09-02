#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QThread>
#include <QMessageBox>
#include <QProcess>
#include <QStringRef>
#include <QDir>

#define LBIP "127.0.0.1"  // loopback Internet Protocol

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    connect(&serialPort, &QSerialPort::readyRead, this, &MainWindow::serialArduino);
    connect(&serialPort, &QSerialPort::errorOccurred, this, &MainWindow::handleError);
    connect(ui->btnInit, SIGNAL(clicked(bool)), this, SLOT(init()));
    connect(ui->btnStart, SIGNAL(clicked(bool)), this, SLOT(start()));
    connect(ui->btnStop, SIGNAL(clicked(bool)), this, SLOT(stop()));
    connect(ui->actionSerial_Connection, SIGNAL(triggered()), this, SLOT(serialTest ()));
    connect(ui->actionIP_address, SIGNAL(triggered()), this, SLOT(ipSetting ()));

    // start the server
    server = new QTcpServer(this);
    connect(server, SIGNAL(newConnection()), this, SLOT(Connection()));
//    QString str_address = current_ip;
    QHostAddress hostaddress(current_ip);
    if(server->listen(hostaddress, 10000)){  //QHostAddress::Any
        qDebug() << "Server started!";
    }else{
        qDebug() << "Server could not start!";
    }

    qDebug() << "### end of constructor of MainWindow";
}

MainWindow::~MainWindow()
{

    delete ui;
}

/* Set up connections with Pattern Recognition module and Robot Control module.
 * 1. Connect with Robot Control module.
 * 2. Connect with Pattern Recognition module.
 */
void MainWindow::Connection (){
    ++numConnection;
    if(numConnection == 1){
        sockRC = server->nextPendingConnection ();
        qDebug() << "Connect to Robot Control Module";

        // read from socket and display in lineEdit
        connect (sockRC, SIGNAL(readyRead()), this, SLOT(readSockRC()));
        connect (ui->btnSendRC, SIGNAL(clicked(bool)), this, SLOT(sendBtnRC()));
        QMessageBox::warning (this, "Notification", "Connected to RC module.");
    }else if(numConnection == 2){
        sockPR = server->nextPendingConnection ();
        qDebug() << "Connect to Pattern Recognition Module";

        // read from socket and display in lineEdit
        connect (sockPR, SIGNAL(readyRead()), this, SLOT(readSockPR()));
        connect (ui->btnSendPR, SIGNAL(clicked(bool)), this, SLOT(sendBtnPR()));
        QMessageBox::warning (this, "Notification", "Connected to PR module.");
    }
}

QString MainWindow::readSockRC(){
//    QString recvMes = ui->lineEditRecvRC->text () + QString(sockRC->readAll());
    QString recvMes = QString(sockRC->readAll());
    ui->lineEditRecvRC->setText(recvMes);
    if(stopSignal == false){
        LogicalControlCenter(recvMes);
    }
    return recvMes;
}

QString MainWindow::readSockPR(){
//    QString recvMes = ui->lineEditRecvPR->text () + QString(sockPR->readAll());
    QString recvMes = QString(sockPR->readAll());
    ui->lineEditRecvPR->setText(recvMes);
    if(stopSignal == false){
        LogicalControlCenter(recvMes);
    }
    return recvMes;
}

void MainWindow::sendBtnRC(){
    QString sendMes = ui->lineEditSendRC->text ();
    sockRC->write (sendMes.toUtf8 ());
    sockRC->flush ();
    sockRC->waitForBytesWritten ();
}

void MainWindow::sendBtnPR(){
    QString sendMes = ui->lineEditSendPR->text ();
    sockPR->write (sendMes.toUtf8 ());
    sockPR->flush ();
    sockPR->waitForBytesWritten ();
}

void MainWindow::sendSockRC(QString sendMes){
    sockRC->write (sendMes.toUtf8 ());
    sockRC->flush ();
    sockRC->waitForBytesWritten ();
}

void MainWindow::sendSockPR(QString sendMes){
    sockPR->write (sendMes.toUtf8 ());
    sockPR->flush ();
    sockPR->waitForBytesWritten ();
}

/* Received Command Description:
 * "0": Invalid Command. Call stop() and stop the whole program.
 * "1": From PR, the object is right under the camera and PR returns the gradient and direction of the object.
 * "2": From RC, RC module sets up successfully. Set RCSetupFlag to true, so GUI start setting up PR module.
 * "3": From RC, RC module initiates successfully. GUI should start conveyer belt.
 * "4": From RC, RC module finished moving to object and requires PR redetect the object.
 * "5": From PR, the object is not right under the camera and PR returns the coordinates of the object.
 * "6": From PR, a plate is detected. Stop moving the conveyer belt.
 * "7": From PR, no component is detected. Wait 5s. Start detecting again.
 * "8": From PR, no plate is detected. Start conveyer belt. Then detect plate again.
 * "9": From PR, PR has set up successfully.
 * "a": From RC, start grasping.
 */
void MainWindow::LogicalControlCenter (QString command){
    QChar ctmp = command[0];

//    QThread::sleep (1);

    switch(ctmp.unicode () - 48){
    case 0 : this->stop(); break;
    case 1 : sendSockRC (command);  // Received "1" from PR. The RC should rotate by gradient and grab object.
             break;
    case 2 : this->RCSetupFlag = true;  // The robot control module initiate successfully and return status code "2".
             break;
    case 3 : //QMessageBox::warning (this, "Notification", "Start conveyer belt.");
             sendSockPR ("8");  // Received "3" from RC. Send "8" to PR to start recognition procedure.
             break;
    case 4 : //QMessageBox::warning (this, "Notification", "Finished moving to object. Redetect the object.");
             sendSockPR ("9");  // Received "4" from RC. Send "8" to PR to redetect the object.
             break;
    case 5 : sendSockRC (command);  // Received "5" from PR. The RC module should move to the object.
             break;
    case 6 : //QMessageBox::warning (this, "Notification", "A plate is detected. Stop conveyer belt.");
             sendSockPR ("9");
             break;
    case 7 : //QMessageBox::warning (this, "Notification", "No object is detected. Moving conveyer belt.");
             QThread::sleep (5);  // Wait for the current plate to leave by 5s.
             sendSockPR ("8");  // Start detecting again.
             break;
    case 8 : //QMessageBox::warning (this, "Notification", "No plate is detected.\nStart conveyer belt.");
             sendSockPR ("8");
             break;
    case 9 : // Received "9" from PR, which means PR has set up successfully.
             //QMessageBox::warning (this, "Notification", "PR module sets up successfully.");
             break;
    case 49: // Received 'a' from RC, which means it is time to grasp component.
             serialCommunication("1");
             sendSockRC("1");
             break;
    case 50: // Received 'b' from RC, which means it is time to release component.
             serialCommunication("2");
             sendSockRC("1");
             break;
    default: break;
    }
}

void MainWindow::init (){

    // Start Robot Control module
    QProcess pRC;
    //D:/IntegrationTest
    QString filenameRC = "C:/0Project/Integration/auboi5-sdk-for-windows-x64/x64/Debug/auboi5-sdk-for-windows-x64.exe";
    pRC.startDetached (filenameRC);

    // start pattern recognition module
    // wait for GUI module and RC moduel to establish connection.
    while(RCSetupFlag == false){
        QThread::msleep (500);
    }
    QProcess pPR;
    QString filenamePR;
    filenamePR = "C:/0Project/Python/PatternRecognitionModule.py";
    pPR.startDetached (filenamePR);
}

void MainWindow::start (){
    // start robot arm at first. Send a command "3" to RC module.
    sendSockRC ("3");
}

void MainWindow::stop (){
//    stopSignal = true;

    QThread::msleep (1000);
    // ending the working environment
    QString command = "0";
    sendSockPR (command);
    sendSockRC (command);
    numConnection = 0;
    RCSetupFlag = false;
    sockRC->close ();
    sockPR->close ();
}

void MainWindow::coordinateProcessing(QString command){
    // command e.g. "5 +0.123 +0.456"
    // The length of command is 15.
    QStringRef str_v(&command, 3, 5);
    QStringRef str_h(&command, 10, 5);
    this->v_distance = str_v.toFloat ();
    this->h_distance = str_h.toFloat ();
    if(command[2] == "-"){
        this->v_distance = -this->v_distance;
    }
    if(command[9] == "-"){
        this->h_distance = -this->h_distance;
    }
    qDebug() << "v_distance" << this->v_distance;
    qDebug() << "h_distance" << this->h_distance;
}

void MainWindow::postureProcessing(QString command){
    // command e.g. "1 +0.123 0"
    // The length of command is 10.
    QStringRef str_gra(&command, 3, 5);
    this->gradient = str_gra.toFloat ();
    this->direction = command[9].unicode () - 48;
    if(command[2] == "-"){
        this->gradient = -this->gradient;
    }
    qDebug() << "gradient" << this->gradient;
    qDebug() << "direction" << this->direction;
}

//void MainWindow::serialArduinoGrasp(){
//    serialPort.write ("1");  // 1 means start
//    serialPort.flush ();
//    QByteArray message = serialPort.readAll ();
//    serialPort.flush ();
//    if(message[0] == '1')
//        qDebug() << "Grasp done.";
//    else
//        qDebug() << "Failed to grasp.";
//}

//void MainWindow::serialArduinoRelease(){
//    serialPort.write ("0");  // 0 means start
//    serialPort.flush ();
//    QByteArray message = serialPort.readAll ();
//    serialPort.flush ();
//    if(message[0] == '1')
//        qDebug() << "Release done.";
//    else
//        qDebug() << "Failed to release.";
//}

void MainWindow::serialArduino(){
    QByteArray message = serialPort.readAll ();
    if(message[0] == '1'){  // 1 means successful
        sendSockRC ("1");
    }else if(message[0] == '0'){  // 0 means failure
        sendSockRC ("0");
    }
}

void MainWindow::handleError(QSerialPort::SerialPortError error){
    qDebug() << "Serial Port Communication error: " << serialPort.errorString();
}

void MainWindow::serialTest(){
    serialtest = new SerialTest(this);
    serialtest->setWindowTitle ("Serial Communication Test");
    serialtest->show();
}

void MainWindow::serialCommunication(QByteArray request){
    qDebug() << request;
    qDebug() << typeid (request).name ();
    serialPort.setPortName ("COM8");
    serialPort.setBaudRate (QSerialPort::Baud9600);
    if (!serialPort.open(QIODevice::WriteOnly)) {
        QMessageBox::warning (this, "Serial communication error", "Fail to open serial port while grasping");
        qDebug() << QObject::tr("Fail to open serial port while grasping");
    }
    qint64 num = serialPort.write (request, 1);
    serialPort.waitForBytesWritten ();
    QThread::msleep (1500);
    qDebug() << num;
    if(num == -1){
        qDebug() << "Fail to send request to MCU. The number of message sent to MCU: " << num;
    }
    if(serialPort.flush () == false){
        qDebug() << "Fail to flush serial port";
    }
    if(serialPort.isOpen ()){
        serialPort.close ();
    }else{
        qDebug() << "The serial port is not open";
    }
}

void MainWindow::ipSetting(){
    IPSetting set_ip(this);
    if(set_ip.exec () != QDialog::Accepted){
        QMessageBox::warning (this, "Error while setting IP", "Fail to set new IP.");
    }
    current_ip = set_ip.get_new_ip ();

    QDir di;
    di.cdUp();
    QFile file(di.path () + "/QtTutorialTcpTest/res/ip.txt");
    if(!file.open(QIODevice::WriteOnly)) {
        QMessageBox::information(this, "error", file.errorString());
    }

    QTextStream fout(&file);
    fout << current_ip << endl;
    file.close();
}
