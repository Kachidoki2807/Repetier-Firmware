#include <Wire.h>
#include "Repetier.h"

/****************************************************************************************
 NUNCHUK DEVICE CLASS
****************************************************************************************/

static NunchukDeviceClass NunchukDevice;

void NunchukDeviceClass::init() {
    Wire.begin();
}

void NunchukDeviceClass::loop() {
    // Tick management
    millis_t curtime = HAL::timeInMilliseconds();

    switch(state)
    {
        case 0U:     // Wait 100ms - Sampling interval
        case 2U:     // Wait 10ms  - Initializing Nunchuk
        case 4U:     // Wait 10ms  - Disabling encryption
        case 6U:     // Wait 5ms   - Data availability
        default:
            if(curtime < nexttime) {
                return;                     // Not ready yet
            } else {
                state++;
            }
            break;

        case 1U:
            // Init the Nunchuk and disable the encryption
            Wire.beginTransmission(NUNCHUK_DEVICE_ADDR);
            Wire.write(0xF0);
            Wire.write(0x55);
            Wire.endTransmission();
            nexttime = curtime + 10U;
            state++;
            break;
        
        case 3U:
            Wire.beginTransmission(NUNCHUK_DEVICE_ADDR);
            Wire.write(0xFB);
            Wire.write(0x00);
            Wire.endTransmission();
            nexttime = curtime + 10U;
            state++;
            break;

        case 5U:     // Start request
            Wire.beginTransmission(NUNCHUK_DEVICE_ADDR);
            Wire.write(0x00);
            Wire.endTransmission();
            nexttime = curtime + 5U;        // The data will be avaiable into 5ms
            state++;
            break;

        case 7U:     // Read the results
            byte index = 0;
            byte raw[6];

            Wire.requestFrom(NUNCHUK_DEVICE_ADDR, 6);

            while((Wire.available() > 0) && (index < 6)) {
                raw[index++] = Wire.read();
            }

            /* Data parsing */
            byte joyX   =   raw[0];
            byte joyY   =   raw[1];
#ifdef NUNCHUK_DEVICE_ENABLE_ACCEL
            int  accelX =  (raw[2] << 2) | ((raw[5] >> 2) & 0x03);
            int  accelY =  (raw[3] << 2) | ((raw[5] >> 4) & 0x03);
            int  accelZ =  (raw[4] << 2) | ((raw[5] >> 6) & 0x03);
#endif
            bool btnZ   = ((raw[5] >> 0) & 0x01) ^ 0x01;
            bool btnC   = ((raw[5] >> 1) & 0x01) ^ 0x01;

            if(this->joyX != joyX) {
                this->joyX = joyX;
                newData = true;
            }

            if(this->joyY != joyY) {
                this->joyY = joyY;
                newData = true;
            }

#ifdef NUNCHUK_DEVICE_ENABLE_ACCEL
            if(this->accelX != accelX) {
                this->accelX = accelX;
                newData = true;
            }

            if(this->accelY != accelY) {
                this->accelY = accelY;
                newData = true;
            }

            if(this->accelZ != accelZ) {
                this->accelZ = accelZ;
                newData = true;
            }
#endif

            if(this->btnZ != btnZ) {
                this->btnZ = btnZ;
                newData = true;
            }

            if(this->btnC != btnC) {
                this->btnC = btnC;
                newData = true;
            }

            nexttime = curtime + 100U;      // Next loop in 100ms
            state = 0U;
            break;
    }
}

bool NunchukDeviceClass::hasNewData() {
    bool newData = this->newData;

    this->newData = false;
    
    return newData;
}

/****************************************************************************************
 NUNCHUK CLASS
****************************************************************************************/

NunchukClass Nunchuk;

void NunchukClass::interp(const t_InterpAxis axis, byte in, float &feedrate, int32_t &moveLength) {
    #define INTERP_MIDDLE_VALUE (127u)
    #define INTERP_THRESHOLD     (16u)

    static const struct {
        int32_t below;
        int32_t above;
        float   feedrate;
    } moveLengthTable[MAX_INTERP_AXIS] = {
        [INTERP_X_AXIS] = { .below = -X_MAX_LENGTH, .above =  X_MAX_LENGTH, .feedrate = 50.0},
        [INTERP_Y_AXIS] = { .below =  Y_MAX_LENGTH, .above = -Y_MAX_LENGTH, .feedrate = 50.0},
        [INTERP_Z_AXIS] = { .below =  Z_MAX_LENGTH, .above = -Z_MAX_LENGTH, .feedrate = 10.0},
    };

    if(axis < MAX_INTERP_AXIS) {
        feedrate = moveLengthTable[axis].feedrate;
             if(in < (INTERP_MIDDLE_VALUE - INTERP_THRESHOLD)) { moveLength = moveLengthTable[axis].below; }
        else if(in > (INTERP_MIDDLE_VALUE + INTERP_THRESHOLD)) { moveLength = moveLengthTable[axis].above; }
        else                                                   { moveLength = 0; }    // Idle
    } else {
        feedrate = 50.0;
        moveLength = 0;
    }
}

void NunchukClass::stopJog() {
    if(jogging && (PrintLine::cur != NULL)) {
        PrintLine::cur->setXYMoveFinished();
        PrintLine::cur->setZMoveFinished();
        jogging = false;
    }
}

void NunchukClass::startJog() {
    jogging = true;
    bool nocheck = Printer::isNoDestinationCheck();
    Printer::setNoDestinationCheck(true);
    PrintLine::moveRelativeDistanceInStepsReal(Printer::axisStepsPerMM[X_AXIS] * moveX, /* int32_t x         */
                                               Printer::axisStepsPerMM[Y_AXIS] * moveY, /* int32_t y         */
                                               Printer::axisStepsPerMM[Z_AXIS] * moveZ, /* int32_t z         */
                                               0,                                       /* int32_t e         */
                                               speed,                                   /* float feedrate    */
                                               false,                                   /* bool waitEnd      */
                                               true);                                   /* bool pathOptimize */
    Printer::setNoDestinationCheck(nocheck);
}

void NunchukClass::init() {
    NunchukDevice.init();
}

void NunchukClass::loop() {
    NunchukDevice.loop();

    if(!NunchukDevice.hasNewData()) {
        return;                     // Not ready yet
    }

    bool enableXY = NunchukDevice.getBtnC();
    bool enableZ  = NunchukDevice.getBtnZ();

    // Emergency stop
    if(enableXY && enableZ) {
        stopJog();
        PrintLine::resetPathPlanner();
        Printer::kill(true);    // Stop moving and disable steppers
//        Printer::setOrigin(Printer::currentPosition[X_AXIS], Printer::currentPosition[Y_AXIS], Printer::currentPosition[Z_AXIS]);
        return;
    }

    int32_t moveX = 0;
    int32_t moveY = 0;
    int32_t moveZ = 0;
    float   speed = 0;
    
    if(enableXY) {
        float fx,fy;

        interp(INTERP_X_AXIS, NunchukDevice.getJoyX(), fx, moveX);
        interp(INTERP_Y_AXIS, NunchukDevice.getJoyY(), fy, moveY);
        speed = RMath::min(fx,fy);
    }

    if(enableZ) {
        interp(INTERP_Z_AXIS, NunchukDevice.getJoyY(), speed, moveZ);
    }

    bool newMove = false;

    // If something has changed, update the planner accordingly
    if(this->speed != speed) {
        this->speed = speed;
        newMove = true;
    }

    if(this->moveX != moveX) {
        this->moveX = moveX;
        newMove = true;
    }

    if(this->moveY != moveY) {
        this->moveY = moveY;
        newMove = true;
    }

    if(this->moveZ != moveZ) {
        this->moveZ = moveZ;
        newMove = true;
    }

    if(newMove) {
        stopJog();
        PrintLine::waitForXFreeLines(PRINTLINE_CACHE_SIZE, true);
        startJog();
    }
}
