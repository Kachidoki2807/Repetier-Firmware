#ifndef _NUNCHUK_H
#define _NUNCHUK_H

#define NUNCHUK_DEVICE_ENABLE_ACCEL 0

class NunchukDeviceClass {
private:
    #define NUNCHUK_DEVICE_ADDR (0x52)

    byte state;
    millis_t nexttime;
    byte joyXcal;
    byte joyYcal;
    byte joyX;
    byte joyY;
#if NUNCHUK_DEVICE_ENABLE_ACCEL
    int  accelX;
    int  accelY;
    int  accelZ;
#endif
    bool btnZ;
    bool btnC;
    bool newData;
    bool calibrated;

protected:

public:
    NunchukDeviceClass() : state(0), nexttime(0), newData(false), calibrated(false) { };
    ~NunchukDeviceClass() {};

    void init();
    void loop();

    inline byte getJoyX()   { return joyX; };
    inline byte getJoyY()   { return joyY; };
#if NUNCHUK_DEVICE_ENABLE_ACCEL
    inline int  getAccelX() { return accelX; };
    inline int  getAccelY() { return accelY; };
    inline int  getAccelZ() { return accelZ; };
#endif
    inline bool getBtnZ()   { return btnZ; };
    inline bool getBtnC()   { return btnC; };
    bool hasNewData();
};

class NunchukClass {
private:
    typedef enum {
        INTERP_X_AXIS,
        INTERP_Y_AXIS,
        INTERP_Z_AXIS,

        MAX_INTERP_AXIS,
    } t_InterpAxis;

    int32_t moveX;
    int32_t moveY;
    int32_t moveZ;
    float   speed;
    bool    jogging;

    void interp(const t_InterpAxis axis, byte in, float &feedrate, int32_t &dir);
    void stopJog();
    void startJog();

protected:

public:
    void init();
    void loop();

    NunchukClass() : jogging(false) {};
    ~NunchukClass() {};
};

extern NunchukClass Nunchuk;

#endif
