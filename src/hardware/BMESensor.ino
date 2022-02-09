#include "configuration/configuration.h"
#include <BME280I2C.h>

BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
BME280::PresUnit presUnit(BME280::PresUnit_Pa);

class BMESensor {
  protected:
  BME280I2C bmei2c;
  float temp, hum, pres;
  uint32_t lastPacketMillis = 0;
  uint32_t selfSendInterval = BME_DEFAULT_SEND_INTERVAL;

  public:
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
    selfSendInterval = EEPROM.readULong(BME_EEPROM_ADDRESS);
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
    EEPROM.writeULong(BME_EEPROM_ADDRESS, BME_DEFAULT_SEND_INTERVAL);
  }
  
  void update() {
    /*
    Reads current sensor values and writes them to memory

    Parameters:
    -----------
    None

    Returns:
    --------
    None
    */
    bmei2c.read(pres, temp, hum, tempUnit, presUnit);
  }

  float bme_pres() {
    /*
    Returns the current pressure as of the last call of update in kPa

    Parameters:
    -----------
    None

    Returns:
    --------
    pressure: float
    */
    return pres;
  }

  float bme_temp() {
    /*
    Returns the current temperature as of the last call of update in C

    Parameters:
    -----------
    None

    Returns:
    --------
    temperature: float
    */
    return temp;
  }

  float bme_hum() {
    /*
    Returns the current humidity as of the last call of update in %

    Parameters:
    -----------
    None

    Returns:
    --------
    humidity: float
    */
    return hum;
  }

  void loop() {
    /*
    Updates the sensor values in memory by calling update

    Parameters:
    -----------
    None

    Returns:
    --------
    None
    */
    update();
  }

  void init() {
    /*
    Init method for the BME sensor
    Begins communications over I2C and identifies sensor type

    Parameters:
    -----------
    None

    Returns:
    --------
    None
    */
    bmei2c.begin();
    delay(100);

    switch(bmei2c.chipModel()) {
      case BME280::ChipModel_BME280:
        Serial.println("Found BME280 sensor! Success.");
        break;
      case BME280::ChipModel_BMP280:
        Serial.println("Found BMP280 sensor! No Humidity available.");
        break;
      default:
        Serial.println("Found UNKNOWN sensor! Error!");
    }
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
    EEPROM.writeULong(BME_EEPROM_ADDRESS, newInterval);
  }

  void set_send_interval(uint8_t downlinkArray[], uint8_t len) {
    /*
    Called when we receieve a TTN downlink starting with "IB" (1: I for set interval, 2: B for BME).
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
        Serial.print("setting bme send interval to: "); Serial.println((uint32_t)downlinkString.substring(2,len-4).toInt());
        ttn_send(downlinkArray, len, LORAWAN_PORT, false);
        uint32_t tmp_interval = (uint32_t)downlinkString.substring(2,len-4).toInt();
        saveSendInterval(tmp_interval);
        EEPROM.commit();
    }else{
        Serial.print("set_bme_send_internal received string was too short: "); Serial.println(downlinkString);
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

    txPacket[bufSize++] = BME_START_BYTE;

    Serial.println("Returning BME TX packet");

    tmpBuf = (unsigned char*)(&temp);
    txPacket[bufSize++] = tmpBuf[0];
    txPacket[bufSize++] = tmpBuf[1];
    txPacket[bufSize++] = tmpBuf[2];
    txPacket[bufSize++] = tmpBuf[3];

    tmpBuf = (unsigned char*)(&hum);
    txPacket[bufSize++] = tmpBuf[0];
    txPacket[bufSize++] = tmpBuf[1];
    txPacket[bufSize++] = tmpBuf[2];
    txPacket[bufSize++] = tmpBuf[3];

    tmpBuf = (unsigned char*)(&pres);
    txPacket[bufSize++] = tmpBuf[0];
    txPacket[bufSize++] = tmpBuf[1];
    txPacket[bufSize++] = tmpBuf[2];
    txPacket[bufSize++] = tmpBuf[3];

    lastPacketMillis = millis();
    
    Serial.println("Returning GPS TX packet");

    LengthArray returnObject;
    returnObject.array = txPacket;
    returnObject.length = bufSize;

    return returnObject;
  }
};