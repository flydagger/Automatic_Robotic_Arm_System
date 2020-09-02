#ifndef SERIAL_TEST_H
#define SERIAL_TEST_H

#include <QWidget>
#include <QByteArray>
#include <QSerialPort>

namespace Ui {
class Serial_Test;
}

class Serial_Test : public QWidget
{
    Q_OBJECT

public:
    explicit Serial_Test(QWidget *parent = nullptr);
    ~Serial_Test();

private slots:
    void serialTest1();
    void serialTest2();
    void reply1();
    void reply2();

private:
    Ui::Serial_Test *ui;
//    QByteArray message;
    QSerialPort serialPort1;
    QSerialPort serialPort2;
    int serialPortBaudRate = QSerialPort::Baud9600;

};

#endif // SERIAL_TEST_H
