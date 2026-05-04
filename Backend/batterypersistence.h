#ifndef BATTERYPERSISTENCE_H
#define BATTERYPERSISTENCE_H

class BatteryPersistence {
public:
    bool load(float *leftWh, float *rightWh);
    void save(float leftWh, float rightWh);

private:
    static constexpr int kSchemaVersion = 1;
};

#endif
