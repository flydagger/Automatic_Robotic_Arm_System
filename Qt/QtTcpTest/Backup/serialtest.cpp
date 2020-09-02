#include "serialtest.h"
#include "ui_serialtest.h"
#include <QMessageBox>

#include <QDebug>

SerialTest::SerialTest(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::SerialTest)
{
    ui->setupUi(this);
//    connect(ui->portBtn_1, SIGNAL(click ()), this, SLOT(serialTest1 ()));
//    connect(ui->portBtn_2, SIGNAL(click ()), this, SLOT(serialTest2 ()));
    connect(&serialPort1, &QSerialPort::readyRead, this, &SerialTest::reply1);
    connect(&serialPort2, &QSerialPort::readyRead, this, &SerialTest::reply2);
}

SerialTest::~SerialTest()
{
    if(serialPort1.isOpen ())
        serialPort1.close ();
    if(serialPort2.isOpen ())
        serialPort2.close ();
    delete ui;
}

void SerialTest::serialTest1(){
    QString portName = ui->port_1->text ();
//    QChar command = (QChar((ui->command_1->text ())[0])).toLatin1 ();
    QByteArray command;
    command.append (ui->command_1->text ());
    // 1 - clockwise, 2 - counterclockwise
    if(!(command[0] == '1' || command[0] == '2')){
        QMessageBox::warning (this, "Notification", "Illegal command. Command must be 1 or 2.");
        return;
    }
    QString step_number = ui->step_num->text ();
    bool f;
    int num_step = step_number.toInt (&f);
    if(!f){
        QMessageBox::warning (this, "Notification", "Not digits.");
        return;
    }
    if(num_step <= 0 || num_step > 99999){
        QMessageBox::warning (this, "Notification", "Illegal number of steps. [0, 99999]");
        ui->reply_1->setText ("Illegal number of steps");
        return;
    }
    int len_step_num = 5 - step_number.size ();
    while(len_step_num-- > 0){
        command.append ('0');
    }
    command.append (step_number);

    serialPort1.setPortName (portName);
    serialPort1.setBaudRate (serialPortBaudRate);
    if(!serialPort1.isOpen ()){
        if(!serialPort1.open(QIODevice::ReadWrite)){
            QMessageBox::warning (this, "Notification", "Fail to open seiral port connected with Stepper Motor.");
            return;
        }
    }
    serialPort1.write (command);
}

void SerialTest::serialTest2(){
    QString portName = ui->port_2->text ();
//    const char command = (QChar((ui->command_2->text ())[0])).toLatin1 ();
//    if(!(command == '1' || command == '2')){
//        qDebug() << "illegal command in port 2";
//        return;
//    }
    QString start_degree = ui->start_degree->text ();
    QString end_degree = ui->end_degree->text ();
    if(start_degree.toInt() < 0 || start_degree.toInt() > 180 || end_degree.toInt() < 0 || end_degree.toInt() > 180){
        QMessageBox::warning(this, "Notification", "Illegal degree. Degrees must between 0 and 180.");
    }
    serialPort2.setPortName (portName);
    serialPort2.setBaudRate (serialPortBaudRate);
    if(!serialPort2.isOpen ()){
        if(!serialPort2.open(QIODevice::ReadWrite)){
            QMessageBox::warning(this, "Notification", "Fail to open serial port 2.");
            return;
        }
    }
    int len_start_degree = 3 - start_degree.size ();
    int len_end_degree = 3 - end_degree.size ();
    QByteArray command;
    while(len_start_degree-- > 0){
        command.append ('0');
    }
    command.append (start_degree);
    while(len_end_degree-- > 0){
        command.append ('0');
    }
    command.append (end_degree);
//    qDebug() << command;
    serialPort2.write(command);
}

void SerialTest::reply1(){
    const QString rp = serialPort1.readAll ();
    ui->reply_1->setText (rp);
}

void SerialTest::reply2(){
    const QString rp = serialPort2.readAll ();
    ui->reply_2->setText (rp);
}
