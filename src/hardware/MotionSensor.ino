#include "configuration/configuration.h"
#include "FunctionalInterrupt.h"

class MotionSensor {
    protected:
    bool interruptTriggered;
    uint32_t min_interval_trigger_send = MOTION_MINIMUM_SEND_INTERVAL;
    uint32_t last_send_millis = 0;

    public:
    bool get_interrupt_triggered() {
        return interruptTriggered;
    }

    void set_interrupt_triggered(bool newState) {
        interruptTriggered = newState;
    }

    bool check_motion_send_status() {
        if((millis() - last_send_millis) >= MOTION_MINIMUM_SEND_INTERVAL && interruptTriggered){
            interruptTriggered = false;
            last_send_millis = millis();
            return true;
        }
        return false;
    }

    void init(std::function<void(void)> intRoutine) {
        pinMode(MOTION_PIN, INPUT_PULLDOWN);
        attachInterrupt(MOTION_PIN, intRoutine, RISING);
    }

    void loop() {
        return;
    }

    LengthArray buildSensorPacket() {
        last_send_millis = millis();

        static uint8_t txPacket[2];
        int bufSize = 0;
        unsigned char *tmpBuf;

        txPacket[bufSize++] = MOTION_START_BYTE;

        tmpBuf = (unsigned char*)(&interruptTriggered);
        txPacket[bufSize++] = tmpBuf[0];

        Serial.println("Returning Motion TX packet");

        LengthArray returnObject;
        returnObject.array = txPacket;
        returnObject.length = bufSize;

        set_interrupt_triggered(false);
        return returnObject;
    }
};