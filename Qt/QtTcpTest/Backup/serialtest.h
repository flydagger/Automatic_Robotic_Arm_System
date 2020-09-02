#ifndef SERIALTEST_H
#define SERIALTEST_H

#include <QMainWindow>
#include <QSerialPort>

namespace Ui {
class SerialTest;
}

class SerialTest : public QMainWindow
{
    Q_OBJECT

public:
    explicit SerialTest(QWidget *parent = nullptr);
    ~SerialTest();

public slots:
    void serialTest1();
    void serialTest2();
    void reply1();
    void reply2();

private:
    Ui::SerialTest *ui;
    QSerialPort serialPort1;
    QSerialPort serialPort2;
    int serialPortBaudRate = QSerialPort::Baud9600;
};

#endif // SERIALTEST_H
