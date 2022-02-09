#include <TinyGPS++.h>
#include "configuration/configuration.h"

TinyGPSPlus _gps;
HardwareSerial _serial_gps(GPS_SERIAL_NUM);

class GPSSensor {
    protected:
    uint32_t LatitudeBinary;
    uint32_t LongitudeBinary;
    uint16_t altitudeGps;
    uint8_t hdopGps;
    uint8_t sats;
    uint32_t lastPacketMillis = 0;
    uint32_t selfSendInterval = GPS_DEFAULT_SEND_INTERVAL;

    public:

    float gps_latitude() {
        /*
        Returns current GPS latitude
        Returns 0 if GPS has no lock

        Parameters:
        -----------
        None

        Returns:
        --------
        latitude: float
        */
        return _gps.location.lat();
    }

    float gps_longitude() {
        /*
        Returns current GPS longitude
        Returns 0 if GPS has no lock

        Parameters:
        -----------
        None

        Returns:
        --------
        longitude: float
        */
        return _gps.location.lng();
    }

    float gps_altitude() {
        /*
        Returns current GPS altitude in feet
        Returns 0 if GPS has no lock

        Parameters:
        -----------
        None

        Returns:
        --------
        altitude: float
        */
        return _gps.altitude.feet();
    }

    float gps_course() {
        /*
        Returns current GPS course in degrees
        Returns 0 if GPS has no lock

        Parameters:
        -----------
        None

        Returns:
        --------
        course: float
        */
        return _gps.course.deg();
    }

    float gps_hdop() {
        /*
        Returns current GPS horizontal positioning error
        Returns 9999 if GPS has no lock

        Parameters:
        -----------
        None

        Returns:
        --------
        hdop: float
        */
        return _gps.hdop.hdop();
    }

    float gps_speed_mph() {
        /*
        Returns current GPS speed in MPH
        Returns 0 if GPS has no lock

        Parameters:
        -----------
        None

        Returns:
        --------
        speed: float
        */
        return _gps.speed.mph();
    }

    uint8_t gps_sats() {
        /*
        Returns current GPS satellite count

        Parameters:
        -----------
        None

        Returns:
        --------
        count: uint8_t
        */
        return _gps.satellites.value();
    }

    void init() {
        /*
        Begins serial communication with the NEO-M8N module

        Parameters:
        -----------
        None

        Returns:
        --------
        None
        */
        _serial_gps.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    }

    void eepromInit(){
        /*
        Reads our send interval variable from EEPROM into memory

        Parameters:
        -----------
        None

        Returns:
        --------
        None
        */
        selfSendInterval = EEPROM.readULong(GPS_EEPROM_ADDRESS);
    }

    void eepromReset(){
        /*
        Writes the default AXP send interval into EEPROM

        Parameters:
        -----------
        None

        Returns:
        --------
        None
        */
        EEPROM.writeULong(GPS_EEPROM_ADDRESS, GPS_DEFAULT_SEND_INTERVAL);
    }

    void set_send_interval(uint8_t downlinkArray[], uint8_t len) {
    /*
    Called when we receieve a TTN downlink starting with "IG" (1: I for set interval, 2: G for GPS).
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
        Serial.print("setting gps send interval to: "); Serial.println((uint32_t)downlinkString.substring(2,len-4).toInt());
        ttn_send(downlinkArray, len, LORAWAN_PORT, false);
        uint32_t tmp_interval = (uint32_t)downlinkString.substring(2,len-4).toInt();
        saveSendInterval(tmp_interval);
        EEPROM.commit();
    }else{
        Serial.print("set_gps_send_internal received string was too short: "); Serial.println(downlinkString);
    }
}

    static void loop() {
        /*
        Loop method for NEO-M8N GPS sensor, updates local sensor variables 

        Parameters:
        -----------
        None

        Returns:
        --------
        None
        */
        while (_serial_gps.available()) {
            _gps.encode(_serial_gps.read());
        }
    }

    int64_t calculateIntervalDelta(){
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

    void saveSendInterval(uint32_t newInterval){
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
        EEPROM.writeULong(GPS_EEPROM_ADDRESS, newInterval);
    }

    LengthArray buildSensorPacket()
    {
        /*
        Returns a txPacket LengthArray containing current sensor values to be transmitted

        Parameters:
        -----------
        None

        Returns:
        --------
        txPacket: LengthArray (array: uint8_t*, length:uint8_t)
        */
        static uint8_t txPacket[25];

        float lat = _gps.location.lat();
        float lon = _gps.location.lng();
        float alt = _gps.altitude.feet();
        int course = _gps.course.deg();
        float speed = _gps.speed.mph();
        float hdop = _gps.hdop.value() / 30.48; //CM to FT conversion 
        int sats = _gps.satellites.value();

        int bufSize = 0;
        unsigned char *tmpBuf;

        txPacket[bufSize++] = GPS_START_BYTE;

        tmpBuf = (unsigned char*)(&lat);
        txPacket[bufSize++] = tmpBuf[0];
        txPacket[bufSize++] = tmpBuf[1];
        txPacket[bufSize++] = tmpBuf[2];
        txPacket[bufSize++] = tmpBuf[3];

        tmpBuf = (unsigned char*)(&lon);
        txPacket[bufSize++] = tmpBuf[0];
        txPacket[bufSize++] = tmpBuf[1];
        txPacket[bufSize++] = tmpBuf[2];
        txPacket[bufSize++] = tmpBuf[3];

        tmpBuf = (unsigned char*)(&alt);
        txPacket[bufSize++] = tmpBuf[0];
        txPacket[bufSize++] = tmpBuf[1];
        txPacket[bufSize++] = tmpBuf[2];
        txPacket[bufSize++] = tmpBuf[3];

        tmpBuf = (unsigned char*)(&hdop);
        txPacket[bufSize++] = tmpBuf[0];
        txPacket[bufSize++] = tmpBuf[1];
        txPacket[bufSize++] = tmpBuf[2];
        txPacket[bufSize++] = tmpBuf[3];

        tmpBuf = (unsigned char*)(&course);
        txPacket[bufSize++] = tmpBuf[0];
        txPacket[bufSize++] = tmpBuf[1];

        tmpBuf = (unsigned char*)(&speed);
        txPacket[bufSize++] = tmpBuf[0];
        txPacket[bufSize++] = tmpBuf[1];
        txPacket[bufSize++] = tmpBuf[2];
        txPacket[bufSize++] = tmpBuf[3];

        tmpBuf = (unsigned char*)(&sats);
        txPacket[bufSize++] = tmpBuf[0];
        txPacket[bufSize++] = tmpBuf[1];

        lastPacketMillis = millis();

        Serial.println("Returning GPS TX packet");

        for (int i=0; i<bufSize; i++){
            Serial.print(txPacket[i]);
            Serial.print(", ");
        }

        LengthArray returnObject;
        returnObject.array = txPacket;
        returnObject.length = bufSize;

        return returnObject;
    }
};