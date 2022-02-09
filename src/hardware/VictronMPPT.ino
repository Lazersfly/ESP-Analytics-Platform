#include "vehex.h"
#include "vehex.cpp"
#include "configuration/configuration.h"

veHex *ve_hex;
HardwareSerial ve_hex_serial(MPPT_SERIAL_NUM);
veID hex_get_list[] = { VE_PANEL_VOLTAGE, VE_PANEL_POWER, VE_CHARGER_CURRENT, VE_CHARGER_VOLTAGE, VE_YIELD_TODAY, VE_DEVICE_STATE, VE_DEVICE_MODE, VE_REMOTE_CONTROL_USED, VE_CHARGER_INTERNAL_TEMPERATURE, VE_LOAD_CURRENT, VE_LOAD_VOLTAGE, VE_OUTPUT_STATE, VE_NOID };

class VictronMPPT {
    protected:
    float ve_panel_voltage = 0;
    float ve_panel_power = 0;
    float ve_yield_today = 0;

    float ve_charger_current = 0;
    float ve_charger_voltage = 0;
    float ve_charger_internal_temperature = 0;
    uint32_t ve_charging_mode = 0;
    uint32_t ve_charger_mode = 0;

    float ve_load_voltage = 0;
    float ve_load_current = 0;
    uint32_t ve_output_state = 0;

    uint32_t ve_remote = 0;

    uint32_t lastPacketMillis = 0;
    uint32_t selfSendInterval = MPPT_DEFAULT_SEND_INTERVAL;

    void add_parsers() {
        /*
        Adds variable parsers for the ve_hex library to decode 

        Parameters:
        -----------
        None

        Returns:
        --------
        None
        */
        Serial.println("MPPT Adding Parsers");

        ve_hex->add_parser(&ve_panel_voltage, VE_PANEL_VOLTAGE, 4, 0.01);
        ve_hex->add_parser(&ve_panel_power, VE_PANEL_POWER, 8, 0.01);
        ve_hex->add_parser(&ve_yield_today, VE_YIELD_TODAY, 4, 0.01*1000*3600);

        ve_hex->add_parser(&ve_charger_current, VE_CHARGER_CURRENT, 4, 0.1);
        ve_hex->add_parser(&ve_charger_voltage, VE_CHARGER_VOLTAGE, 4, 0.01);
        ve_hex->add_parser(&ve_charger_internal_temperature, VE_CHARGER_INTERNAL_TEMPERATURE, 4, 0.01);
        ve_hex->add_parser_raw(&ve_charging_mode, VE_DEVICE_STATE, 2);
        ve_hex->add_parser_raw(&ve_charger_mode, VE_DEVICE_MODE, 2);

        ve_hex->add_parser(&ve_load_voltage, VE_LOAD_VOLTAGE, 4, 0.01);
        ve_hex->add_parser(&ve_load_current, VE_LOAD_CURRENT, 4, 0.1);
        ve_hex->add_parser_raw(&ve_output_state, VE_OUTPUT_STATE, 2);

        ve_hex->add_parser_raw(&ve_remote, VE_REMOTE_CONTROL_USED, 8);

        ve_hex->get_list(&hex_get_list[0]);
    }

    void restart() {
        /*
        Calls send_hex with a 2 byte Victron reset message 

        Parameters:
        -----------
        None

        Returns:
        --------
        None
        */
        uint8_t restart_array[] = {0x3A, 0x36};
        send_hex(restart_array, 2);
    }

    public:
    void init() {
        /*
        Init method for the Victron MPPT sensor
        Begins communications though the ve_hex library over serial
        Calls add_parsers to assign local vars to the ve_hex library 

        Parameters:
        -----------
        None

        Returns:
        --------
        None
        */
        Serial.println("MPPT Starting");
        ve_hex_serial.begin(MPPT_SERIAL_BAUD, SERIAL_8N1, MPPT_RX, MPPT_TX);
        ve_hex = new veHex(&ve_hex_serial);
        add_parsers();
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
        selfSendInterval = EEPROM.readULong(MPPT_EEPROM_ADDRESS);
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
        EEPROM.writeULong(MPPT_EEPROM_ADDRESS, MPPT_DEFAULT_SEND_INTERVAL);
    }

    void loop() {
        /*
        Loop method for Victron MPPT sensor, calls ve_hex's update
        Calls ve_hex's get every second 

        Parameters:
        -----------
        None

        Returns:
        --------
        None
        */
        int ret;
        static unsigned long last_get = 0;

        if ((millis() - last_get) > 1000) {
        // send a get every second to prevent the controller from going into text mode
        ve_hex->get();
        last_get = millis();
        }

        ret = ve_hex->update();

        if (ret == 0) {
        // update has found a valid message and parsed it
        //Serial.print("MPPT Serial Message, "); Serial.println(ve_hex->last_msg());
        }

        else if ((ret != -1) && (ret != 7)) {
        // update has found an invalid message
        // print return value and the message
        Serial.print("Invalid message ret "); Serial.print(ret);
        Serial.print(" |"); Serial.println(ve_hex->last_msg());
        }
    }

    void send_hex(uint8_t* downlinkArray, uint8_t len) {
        /*
        Sends given array as hex to MPPT over serial
        Appends a calculated checksum to the end of the given message
        Sends the message send over serial to TTN

        Parameters:
        -----------
        downlinkArray: uint8_t*
        length: uin8_t

        Returns:
        --------
        None
        */
        Serial.print("Writing following to MPPT: ");
        for (int i=0; i<len; i++) {
            ve_hex_serial.write(downlinkArray[i]);
            Serial.write(downlinkArray[i]);
        }

        char message[len-1];

        for (int i=1; i<len; i++) {
            message[i-1] = downlinkArray[i];
        }

        byte calc_checksum = 0x55 - checksum(message, len-1);
        
        ve_hex_serial.printf("%02X",calc_checksum);
        Serial.printf("%02X",calc_checksum);

        ve_hex_serial.write('\n');
        ve_hex_serial.write('\0');

        // uint32_t printer = millis();

        // while (millis()-printer < MPPT_COMMAND_WAIT){
        //     if(ve_hex_serial.available()){
        //         Serial.write(ve_hex_serial.read());
        //     }
        // }

        uint8_t * lastMsg = reinterpret_cast<uint8_t *>(ve_hex->last_msg());

        ttn_send(lastMsg, VE_BUFSIZE, LORAWAN_PORT, true);
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
        EEPROM.writeULong(MPPT_EEPROM_ADDRESS, newInterval);
    }

    void restart_mppt(uint8_t downlinkArray[], uint8_t len){
        /*
        Called when we receieve a TTN downlink starting with "RR" (1: R for a reset setting, 2: M for MPPT).
        If the downlink is of a valid length (6), and ends with our RESET_CONFIRM_STRING,
        then we send a reset command over the MPPT's serial interface

        Parameters:
        -----------
        downlinkArray: uint8_t[]
        len: unit8_t

        Returns:
        --------
        None
        */
        String downlinkString = reinterpret_cast<char *>(downlinkArray);
        if(len == 6 && downlinkString.substring(2,len).equals(RESET_CONFIRM_STRING)){
            Serial.println("MPPT Restart Starting");
            ttn_send(downlinkArray, len, LORAWAN_PORT, false);
            restart();
        }else{
            Serial.print("MPPT Restart Authentication Failed: len was "); Serial.print(len); 
            Serial.print(" confirm_string was "); Serial.println(downlinkString.substring(2,len));
        }
    }   

    void set_send_interval(uint8_t downlinkArray[], uint8_t len) {
    /*
    Called when we receieve a TTN downlink starting with "IM" (1: I for set interval, 2: M for MPPT).
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
        Serial.print("setting mppt send interval to: "); Serial.println((uint32_t)downlinkString.substring(2,len-4).toInt());
        ttn_send(downlinkArray, len, LORAWAN_PORT, false);
        uint32_t tmp_interval = (uint32_t)downlinkString.substring(2,len-4).toInt();
        saveSendInterval(tmp_interval);
        EEPROM.commit();
    }else{
        Serial.print("set_mppt_send_internal received string was too short: "); Serial.println(downlinkString);
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
        static uint8_t txPacket[49];

        int bufSize = 0;
        unsigned char *tmpBuf;

        txPacket[bufSize++] = MPPT_START_BYTE;
        
        tmpBuf = (unsigned char*)(&ve_panel_voltage);
        txPacket[bufSize++] = tmpBuf[0];
        txPacket[bufSize++] = tmpBuf[1];
        txPacket[bufSize++] = tmpBuf[2];
        txPacket[bufSize++] = tmpBuf[3];
        
        tmpBuf = (unsigned char*)(&ve_panel_power);
        txPacket[bufSize++] = tmpBuf[0];
        txPacket[bufSize++] = tmpBuf[1];
        txPacket[bufSize++] = tmpBuf[2];
        txPacket[bufSize++] = tmpBuf[3];

        tmpBuf = (unsigned char*)(&ve_yield_today);
        txPacket[bufSize++] = tmpBuf[0];
        txPacket[bufSize++] = tmpBuf[1];
        txPacket[bufSize++] = tmpBuf[2];
        txPacket[bufSize++] = tmpBuf[3];

        tmpBuf = (unsigned char*)(&ve_charger_current);
        txPacket[bufSize++] = tmpBuf[0];
        txPacket[bufSize++] = tmpBuf[1];
        txPacket[bufSize++] = tmpBuf[2];
        txPacket[bufSize++] = tmpBuf[3];

        tmpBuf = (unsigned char*)(&ve_charger_voltage);
        txPacket[bufSize++] = tmpBuf[0];
        txPacket[bufSize++] = tmpBuf[1];
        txPacket[bufSize++] = tmpBuf[2];
        txPacket[bufSize++] = tmpBuf[3];

        tmpBuf = (unsigned char*)(&ve_charger_internal_temperature);
        txPacket[bufSize++] = tmpBuf[0];
        txPacket[bufSize++] = tmpBuf[1];
        txPacket[bufSize++] = tmpBuf[2];
        txPacket[bufSize++] = tmpBuf[3];

        tmpBuf = (unsigned char*)(&ve_charging_mode);
        txPacket[bufSize++] = tmpBuf[0];
        txPacket[bufSize++] = tmpBuf[1];
        txPacket[bufSize++] = tmpBuf[2];
        txPacket[bufSize++] = tmpBuf[3];

        tmpBuf = (unsigned char*)(&ve_charger_mode);
        txPacket[bufSize++] = tmpBuf[0];
        txPacket[bufSize++] = tmpBuf[1];
        txPacket[bufSize++] = tmpBuf[2];
        txPacket[bufSize++] = tmpBuf[3];

        tmpBuf = (unsigned char*)(&ve_load_voltage);
        txPacket[bufSize++] = tmpBuf[0];
        txPacket[bufSize++] = tmpBuf[1];
        txPacket[bufSize++] = tmpBuf[2];
        txPacket[bufSize++] = tmpBuf[3];

        tmpBuf = (unsigned char*)(&ve_load_current);
        txPacket[bufSize++] = tmpBuf[0];
        txPacket[bufSize++] = tmpBuf[1];
        txPacket[bufSize++] = tmpBuf[2];
        txPacket[bufSize++] = tmpBuf[3];

        tmpBuf = (unsigned char*)(&ve_output_state);
        txPacket[bufSize++] = tmpBuf[0];
        txPacket[bufSize++] = tmpBuf[1];
        txPacket[bufSize++] = tmpBuf[2];
        txPacket[bufSize++] = tmpBuf[3];

        tmpBuf = (unsigned char*)(&ve_remote);
        txPacket[bufSize++] = tmpBuf[0];
        txPacket[bufSize++] = tmpBuf[1];
        txPacket[bufSize++] = tmpBuf[2];
        txPacket[bufSize++] = tmpBuf[3];

        lastPacketMillis = millis();

        Serial.println("Returning MPPT TX packet");

        LengthArray returnObject;
        returnObject.array = txPacket;
        returnObject.length = bufSize;

        return returnObject;
    }
};