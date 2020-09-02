#include "ipsetting.h"
#include "ui_ipsetting.h"
#include <QFile>
#include <QMessageBox>
#include <QTextStream>
#include <QDir>
#include <QDebug>

IPSetting::IPSetting(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::IPSetting)
{
    ui->setupUi(this);
    QDir di;
    di.cdUp();
    file_path = di.path () + "/QtTutorialTcpTest/res/ip.txt";
    qDebug() << file_path;
    QFile file(file_path);
    if(!file.open(QIODevice::ReadOnly)) {
        QMessageBox::information(this, "error", file.errorString());
    }

    QTextStream in(&file);
    ui->lineEdit_curIP->setText (in.readLine ());
    file.close();

    connect(ui->buttonBox_OK, SIGNAL (clicked(bool)), this, SLOT(set_new_ip ()));
}

IPSetting::~IPSetting()
{
    delete ui;
}

void IPSetting::set_new_ip(){
    new_ip = ui->lineEdit_newIP->text ();
}

QString IPSetting::get_new_ip(){
    return ui->lineEdit_newIP->text ();
}
