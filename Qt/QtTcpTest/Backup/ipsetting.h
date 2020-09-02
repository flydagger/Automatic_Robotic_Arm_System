#ifndef IPSETTING_H
#define IPSETTING_H

#include <QDialog>

namespace Ui {
class IPSetting;
}

class IPSetting : public QDialog
{
    Q_OBJECT

public:
    explicit IPSetting(QWidget *parent = nullptr);
    ~IPSetting();
    QString new_ip = "";
    QString get_new_ip();

private slots:
    void set_new_ip();

private:
    Ui::IPSetting *ui;
    QString file_path;
};

#endif // IPSETTING_H
