//#if 0
#ifndef TEST_H
#define TEST_H

#include <QObject>
#include <QTimer>

class Test : public QObject
{
    Q_OBJECT

public:
    explicit Test(QObject *parent = nullptr);
    void start();

signals:
    void speedChange(double speed);
    void batteryChange(int charge);

protected:
private slots:
    void updateSpeed();
    void updateBattery();

private:
    double speed;
    QTimer speedTimer;
    QTimer batteryTimer;
    bool increase;
    bool decrease;
    int charge;
};

#endif // TEST_H
//#endif
