#include "serial_test.h"
#include "ui_serial_test.h"

#include <QDebug>

Serial_Test::Serial_Test(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Serial_Test)
{
    ui->setupUi(this);
    connect(ui->portBtn_1, SIGNAL(click ()), this, SLOT(serialTest1 ()));
    connect(ui->portBtn_2, SIGNAL(click ()), this, SLOT(serialTest2 ()));
    connect(&serialPort1, &QSerialPort::readyRead, this, &Serial_Test::reply1);
    connect(&serialPort2, &QSerialPort::readyRead, this, &Serial_Test::reply2);
}

Serial_Test::~Serial_Test()
{
    delete ui;
}

void Serial_Test::serialTest1(){
    QString portName = ui->port_1->text ();
    const char command = (QChar((ui->command_1->text ())[0])).toLatin1 ();
    if(!(command == '0' || command == '1' || command == '2')){
        qDebug() << "illegal command in port 1";
        return;
    }
    serialPort1.setPortName (portName);
    serialPort1.setBaudRate (serialPortBaudRate);
    if(!serialPort1.open(QIODevice::ReadOnly)){
        qDebug() << "Fail to open serial port 1";
    }
    serialPort1.write(&command);
}

void Serial_Test::serialTest2(){
    QString portName = ui->port_2->text ();
    const char command = (QChar((ui->command_2->text ())[0])).toLatin1 ();
    if(!(command == '0' || command == '1' || command == '2')){
        qDebug() << "illegal command in port 2";
        return;
    }
    serialPort2.setPortName (portName);
    serialPort2.setBaudRate (serialPortBaudRate);
    if(!serialPort2.open(QIODevice::ReadOnly)){
        qDebug() << "Fail to open serial port 2";
    }
    serialPort2.write(&command);
}

void Serial_Test::reply1(){
    const QString rp = serialPort1.readAll ();
    ui->reply_1->setText (rp);
}

void Serial_Test::reply2(){
    const QString rp = serialPort2.readAll ();
    ui->reply_2->setText (rp);
}
