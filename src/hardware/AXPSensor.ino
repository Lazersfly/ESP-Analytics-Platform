/**
 * Init the AXP192 power manager chip
 * 
 * axp192 power 
    DCDC1 0.7-3.5V @ 1200mA max -> OLED  // If you turn this off you'll lose comms to the axp192 because the OLED and the axp192 share the same i2c bus, instead use ssd1306 sleep mode
    DCDC2 -> unused
    DCDC3 0.7-3.5V @ 700mA max -> ESP32 (keep this on!)
    LDO1 30mA -> charges GPS backup battery  // charges the tiny J13 battery by the GPS to power the GPS ram (for a couple of days), can not be turned off
    LDO2 200mA -> LORA
    LDO3 200mA -> GPS
 */

#include "configuration/configuration.h"
#include <Wire.h>
#include "axp20x.h"

AXP20X_Class axpClass;

class AXPSensor {
    protected:
    float batteryCurrent;
    float batteryVoltage;
    float batteryTemperature;
    uint32_t lastPacketMillis = 0;
    uint32_t selfSendInterval = AXP_DEFAULT_SEND_INTERVAL;

    public:
    void loop() {
        /*
        Loop method for AXP192 sensor, updates local sensor variables 

        Parameters:
        -----------
        None

        Returns:
        --------
        None
        */
        batteryCurrent = axpClass.getBattChargeCurrent() - axpClass.getBattDischargeCurrent();
        batteryVoltage = axpClass.getBattVoltage();
        batteryTemperature = axpClass.getTemp();
    }

    void eepromInit() {
        /*
        Reads our send interval variable from EEPROM into memory

        Parameters:
        -----------
        None

        Returns:
        --------
        None
        */
        selfSendInterval = EEPROM.readULong(AXP_EEPROM_ADDRESS);
    }

    void eepromReset() {
        /*
        Writes the default AXP send interval into EEPROM

        Parameters:
        -----------
        None

        Returns:
        --------
        None
        */
        EEPROM.writeULong(AXP_EEPROM_ADDRESS, AXP_DEFAULT_SEND_INTERVAL);
    }

    void init() {
        /*
        Init method for the AXP192 sensor
        Begins communications with AXP192 chip and sets power outputs

        Parameters:
        -----------
        None

        Returns:
        --------
        None
        */
        if (!axpClass.begin(Wire, AXP192_SLAVE_ADDRESS)) {
            Serial.println("AXP192 Begin PASS");
        } else {
            Serial.println("AXP192 Begin FAIL");
        }
        axpClass.setPowerOutPut(AXP192_LDO2, AXP202_ON);  // LORA radio
        axpClass.setPowerOutPut(AXP192_LDO3, AXP202_ON);  // GPS main power
        axpClass.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
        axpClass.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
        axpClass.setPowerOutPut(AXP192_DCDC1, AXP202_ON);

        axpClass.setDCDC1Voltage(3300);  // for the BME Power

        axpClass.adc1Enable(AXP202_BATT_CUR_ADC1, 1);
        axpClass.enableIRQ(AXP202_VBUS_REMOVED_IRQ | AXP202_VBUS_CONNECT_IRQ | AXP202_BATT_REMOVED_IRQ | AXP202_BATT_CONNECT_IRQ, 1);
        axpClass.clearIRQ();
    }

    int64_t calculateIntervalDelta() {
        /*
        Returns a delta in millis from now to the time when we are due to send a message
        Delta is negative if the next send time is in the future
        Delta is positive if the next send time is in the past

        Parameters:
        -----------
        None

        Returns:
        --------
        intervalDelta: int64_t
        */
        return (int64_t)millis() - (int64_t)(lastPacketMillis + selfSendInterval);
    }

    void saveSendInterval(uint32_t newInterval) {
        /*
        Sets a new send interval in millis to memory and EEPROM

        Parameters:
        -----------
        newInterval: uint32_t

        Returns:
        --------
        None
        */
        selfSendInterval = newInterval;
        EEPROM.writeULong(AXP_EEPROM_ADDRESS, newInterval);
    }

    void set_send_interval(uint8_t downlinkArray[], uint8_t len) {
    /*
    Called when we receieve a TTN downlink starting with "IA" (1: I for set interval, 2: A for AXP).
    If the downlink is of a valid length (>= 10), and ends with our RESET_CONFIRM_STRING,
    then we save the new interval into EEPROM

    Parameters:
    -----------
    downlinkArray: uint8_t[]
    len: unit8_t

    Returns:
    --------
    None
    */
    String downlinkString = reinterpret_cast<char *>(downlinkArray);
    if(downlinkString.length()>=10 && downlinkString.substring(len-4,len).equals(RESET_CONFIRM_STRING)){
        Serial.print("setting axp send interval to: "); Serial.println((uint32_t)downlinkString.substring(2,len-4).toInt());
        ttn_send(downlinkArray, len, LORAWAN_PORT, false);
        uint32_t tmp_interval = (uint32_t)downlinkString.substring(2,len-4).toInt();
        saveSendInterval(tmp_interval);
        EEPROM.commit();
    }else{
        Serial.print("set_axp_send_internal received string was too short: "); Serial.println(downlinkString);
    }
}

    LengthArray buildSensorPacket() {
        /*
        Returns a txPacket LengthArray containing current sensor values to be transmitted

        Parameters:
        -----------
        None

        Returns:
        --------
        txPacket: LengthArray (array: uint8_t*, length:uint8_t)
        */
        static uint8_t txPacket[13];

        int bufSize = 0;
        unsigned char *tmpBuf;

        txPacket[bufSize++] = AXP_START_BYTE;

        Serial.println("Returning AXP TX packet");

        tmpBuf = (unsigned char*)(&batteryVoltage);
        txPacket[bufSize++] = tmpBuf[0];
        txPacket[bufSize++] = tmpBuf[1];
        txPacket[bufSize++] = tmpBuf[2];
        txPacket[bufSize++] = tmpBuf[3];

        tmpBuf = (unsigned char*)(&batteryCurrent);
        txPacket[bufSize++] = tmpBuf[0];
        txPacket[bufSize++] = tmpBuf[1];
        txPacket[bufSize++] = tmpBuf[2];
        txPacket[bufSize++] = tmpBuf[3];

        tmpBuf = (unsigned char*)(&batteryTemperature);
        txPacket[bufSize++] = tmpBuf[0];
        txPacket[bufSize++] = tmpBuf[1];
        txPacket[bufSize++] = tmpBuf[2];
        txPacket[bufSize++] = tmpBuf[3];

        lastPacketMillis = millis();

        LengthArray returnObject;
        returnObject.array = txPacket;
        returnObject.length = bufSize;

        return returnObject;
    }
};