#include "configuration/configuration.h"
#include "rom/rtc.h"
#include <Wire.h>
#include "EEPROM.h"
#include "esp_task_wdt.h"
#include <array>
#include "types/LengthArray.ino"

#include "axp20x.h"
#include "hardware/ttn.ino"
#include "hardware/GPSSensor.ino"
#include "hardware/BMESensor.ino"
#include "hardware/VictronMPPT.ino"
#include "hardware/i2c.ino"
#include "hardware/AXPSensor.ino"
#include "hardware/MotionSensor.ino"

AXPSensor axp;
GPSSensor gps;
BMESensor bme;
VictronMPPT mppt;
MotionSensor motion;

uint8_t txBuffer[222]; // Max LoRaWAN payload size for DR10
uint8_t txBufferLength = 222; // Send the first this many bytes of the buffer (defaults to all)

uint32_t numberPressed = 0;
uint32_t lastNumberPressed = 0;

bool watchdogResetEnabled = true; // Flag to reset the watchdog timer, this is set to false when we want to cause the watchdog to reset the system

#ifdef USE_MOTION
    void IRAM_ATTR ISR() {
        motion.set_interrupt_triggered(true);
    }
#endif

void eeprom_init() {
    /*
    Init an array in EEPROM to store our sensor send intervals

    Parameters:
    -----------
    None

    Returns:
    --------
    None
    */
    EEPROM.begin(EEPROM_SIZE);
    delay(250);

    #ifdef USE_GPS
        gps.eepromInit();
    #endif
    
    #ifdef USE_AXP
        axp.eepromInit();
    #endif

    #ifdef USE_MPPT
        mppt.eepromInit();
    #endif

    #ifdef USE_BME
        bme.eepromInit();
    #endif
}

void eeprom_reset() {
    /*
    Reset our send intervals in the EEPROM array

    Parameters:
    -----------
    None

    Returns:
    --------
    None
    */
    #ifdef USE_GPS
        gps.eepromReset();
    #endif
    #ifdef USE_AXP
        axp.eepromReset();
    #endif
    #ifdef USE_MPPT
        mppt.eepromReset();
    #endif
    #ifdef USE_BME
        bme.eepromReset();
    #endif

    EEPROM.commit();
}

void system_factory_reset(uint8_t downlinkArray[], uint8_t len) {
    /*
    Called when we receieve a TTN downlink starting with "FS" (1: F for potentially fatal reset, 2: S for system).
    If the downlink is of a valid length (6), and ends with our RESET_CONFIRM_STRING,
    then we trigger the system reset routine 

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
        Serial.println("System Factory Reset Starting");
        eeprom_reset();
        ttn_send(downlinkArray, len, LORAWAN_PORT, false);
        ttn_erase_prefs();
        watchdogResetEnabled = false;
    }else{
        Serial.print("System Factory Reset Authentication Failed: len was "); Serial.print(len); 
        Serial.print(" confirm_string was "); Serial.println(downlinkString.substring(2,len));
    }
}

void checkIntervalMessages() {
    /*
    Checks all sensors to see if they have a message to send, and appends their message buffer to
    txBuffer if so.

    Parameters:
    -----------
    None

    Returns:
    --------
    None
    */

    uint8_t bufSize = 0;

    #ifdef USE_GPS
        // Check if the GPS interval delta is greater than 0, and if so trigger a message to be sent
        // Also check if we are moving faster than 3mph, and send on every loop if so
        // Also check if the motion sensor has sent an interrupt, and send if so
        if(gps.calculateIntervalDelta() >= 0 || gps.gps_speed_mph()>=3 || motion.get_interrupt_triggered()) {
            uint8_t *tmpbuf = gps.buildSensorPacket().array;
            uint8_t tmpbufSize = gps.buildSensorPacket().length;
            Serial.print("GPS sizeof is: "); Serial.println(tmpbufSize);
            if ((tmpbufSize + bufSize) <= txBufferLength){
                for (int i=0; i<tmpbufSize; i++){
                    Serial.print(tmpbuf[i]);
                    Serial.print(", ");
                    txBuffer[bufSize++] = tmpbuf[i];
                }
            }
        }
    #endif
    #ifdef USE_AXP
        // Check if the AXP interval delta is greater than 0, and if so trigger a message to be sent
        if(axp.calculateIntervalDelta() >= 0) {
            uint8_t *tmpbuf = axp.buildSensorPacket().array;
            uint8_t tmpbufSize = axp.buildSensorPacket().length;
            Serial.print("AXP sizeof is: "); Serial.println(tmpbufSize);
            if ((tmpbufSize + bufSize) <= txBufferLength){
                for (int i=0; i<tmpbufSize; i++){
                    txBuffer[bufSize++] = tmpbuf[i];
                }
            }
        }
    #endif
    #ifdef USE_MPPT
        // Check if the MPPT interval delta is greater than 0, and if so trigger a message to be sent
        if(mppt.calculateIntervalDelta() >= 0) {
            uint8_t *tmpbuf = mppt.buildSensorPacket().array;
            uint8_t tmpbufSize = mppt.buildSensorPacket().length;
            Serial.print("MPPT sizeof is: "); Serial.println(tmpbufSize);
            if ((tmpbufSize + bufSize) <= txBufferLength){
                for (int i=0; i<tmpbufSize; i++){
                    txBuffer[bufSize++] = tmpbuf[i];
                }
            }
        }
    #endif
    #ifdef USE_BME
        // Check if the BME interval delta is greater than 0, and if so trigger a message to be sent
        if(bme.calculateIntervalDelta() >= 0) {
            LengthArray tmpbuf = bme.buildSensorPacket();
            Serial.print("BME tmpbuf length is: "); Serial.println(tmpbuf.length);
            if ((tmpbuf.length + bufSize) <= txBufferLength){
                for (int i=0; i<tmpbuf.length; i++){
                    txBuffer[bufSize++] = tmpbuf.array[i];
                }
            }
        }
    #endif
    #ifdef USE_MOTION
        // Check if the Motion interrupt has been triggered, send the event and reset the interrupt flag if so
        if(motion.get_interrupt_triggered()) {
            LengthArray tmpbuf = motion.buildSensorPacket();
            Serial.print("Motion tmpbuf length is: "); Serial.println(tmpbuf.length);
            if ((tmpbuf.length + bufSize) <= txBufferLength){
                for (int i=0; i<tmpbuf.length; i++){
                    txBuffer[bufSize++] = tmpbuf.array[i];
                }
            }
        }
    #endif

    if (bufSize > 0){
        // If bufsize is greater than 0 we have a message queued, we send it here if so
        #if LORAWAN_CONFIRMED_EVERY > 0
        // Check if our new message should be sent as confirmed
        bool confirmed = (ttn_get_count() % LORAWAN_CONFIRMED_EVERY == 0);
        if (confirmed) Serial.println("confirmation enabled");
        #else
            bool confirmed = false;
        #endif

        Serial.print("tx buffer length is:  "); Serial.print(bufSize); Serial.print(" Buffer is: [");

        // Print our txBuffer array out as HEX bytes
        for (int i=0; i<bufSize; i++){
            Serial.printf("%02X",txBuffer[i]);
            Serial.print(",");
        }
        Serial.print("]\n");

        // Start our timer for waiting if a message is currently being sent
        uint32_t timeStartWaiting = millis();

        // Send! Or wait until we can send if a message is currently being transmitted
        while((millis()-timeStartWaiting)<MAX_TTN_PACKET_WAIT){
            if(!(LMIC.opmode & OP_TXRXPEND)){
                ttn_send(txBuffer, bufSize, LORAWAN_PORT, confirmed);   
                break;
            }
            ttn_loop();
        }
    }     
}

void callback(uint8_t message) {
    /*
    Method to run if we receieve a message from TTN

    Parameters:
    -----------
    message: unit8_t

    Returns:
    --------
    None
    */
    bool ttn_joined = false;

    if (EV_JOINED == message) {
        ttn_joined = true;
    }

    if (EV_JOINING == message) {
        if (ttn_joined) {
            Serial.print("TTN joining...\n");
        } else {
            Serial.print("Joined TTN!\n");
        }
    }

    if (EV_JOIN_FAILED == message) Serial.print("TTN join failed\n");
    if (EV_REJOIN_FAILED == message) Serial.print("TTN rejoin failed\n");
    if (EV_RESET == message) Serial.print("Reset TTN connection\n");
    if (EV_LINK_DEAD == message) Serial.print("TTN link dead\n");
    if (EV_ACK == message) Serial.print("ACK received\n");
    if (EV_PENDING == message) Serial.print("Message discarded\n");
    if (EV_QUEUED == message) Serial.print("Message queued\n");

    // We only want to say "Message sent" for our packets (not packets needed for joining)
    if (EV_TXCOMPLETE == message) {
        Serial.print("Message sent\n");
    }

    if (EV_RESPONSE == message) {
        Serial.println("----------Begin TTN Downlink----------");

        Serial.print("[TTN] Response: ");

        // Copy TTN's response to our data and len objects
        size_t len = ttn_response_len();
        uint8_t data[len];
        ttn_response(data, len);

        // Print what we received to Serial
        for (uint8_t i = 0; i < len; i++) {
            Serial.print(char(data[i]));
        }
        Serial.print("\n");
        
        if (len > 0){
            char downlinkAsChar[len];

            Serial.print("downlink as char is: ");
            for (uint8_t i = 0; i < len; i++) {
                downlinkAsChar[i] = (char)data[i];
                Serial.print((char)data[i]);
            }
            Serial.print("\n");
            
            #ifdef USE_MPPT 
                if (downlinkAsChar[0] == ':') mppt.send_hex(data, len);
            #endif
            #ifdef USE_GPS
                if (downlinkAsChar[0] == 'I' && downlinkAsChar[1] == 'G') gps.set_send_interval(data, len);
            #endif
            #ifdef USE_AXP
                if (downlinkAsChar[0] == 'I' && downlinkAsChar[1] == 'A') axp.set_send_interval(data, len);
            #endif
            #ifdef USE_MPPT
                if (downlinkAsChar[0] == 'I' && downlinkAsChar[1] == 'M') mppt.set_send_interval(data, len);
                if (downlinkAsChar[0] == 'R' && downlinkAsChar[1] == 'M') mppt.restart_mppt(data, len);
            #endif
            #ifdef USE_BME
                if (downlinkAsChar[0] == 'I' && downlinkAsChar[1] == 'B') bme.set_send_interval(data, len);
            #endif

            if (downlinkAsChar[0] == 'I' && downlinkAsChar[1] == 'J') set_ttn_rejoin_period(data, len);
            if (downlinkAsChar[0] == 'R' && downlinkAsChar[1] == 'R') watchdogResetEnabled = rejoin_ttn(data, len);
            if (downlinkAsChar[0] == 'F' && downlinkAsChar[1] == 'S') system_factory_reset(data, len);
        }
        Serial.println("----------End TTN Downlink----------");
    }
}

void setup() {
    /*
    Setup method to run once on system startup

    Parameters:
    -----------
    None

    Returns:
    --------
    None
    */
    Serial.begin(SERIAL_BAUD); // Init USB serial

    Wire.begin(I2C_SDA, I2C_SCL); //Begin I2C

    esp_task_wdt_init(WDT_TIMEOUT, true); //Enable system reset in case of WDT timeout
    esp_task_wdt_add(NULL); //Attach the WDT to our task

    eeprom_init(); //Init EEPROM for saving downlinked intervals

    scanI2Cdevice(); //Scan I2C bus for any devices

    // Init all of our sensors
    #ifdef USE_GPS
        gps.init();
    #endif
    #ifdef USE_AXP
        axp.init();
    #endif
    #ifdef USE_MPPT
        mppt.init();
    #endif
    #ifdef USE_BME
        bme.init();
    #endif
    #ifdef USE_MOTION
        motion.init(ISR);
    #endif

    // Init our reset button
    pinMode(BUTTON_PIN, INPUT_PULLUP); //Enable user button, hold for 3sec to clear LoraWan session

    // TTN setup
    if (!ttn_setup()) {
        Serial.print("[ERR] Radio module not found!\n");
    }
    else {
        ttn_register(callback);
        ttn_join();
        ttn_adr(LORAWAN_ADR);
    }
}

void loop() {
    if(numberPressed!=lastNumberPressed){
        lastNumberPressed=numberPressed;
        Serial.print("Interupt triggered total of: "); Serial.println(numberPressed);
    }
    /*
    Loop method to run forever after setup()

    Parameters:
    -----------
    None

    Returns:
    --------
    None
    */
    // Run the loop method for all of our sensors
    #ifdef USE_GPS
        gps.loop();
    #endif
    #ifdef USE_AXP
        axp.loop();
    #endif
    #ifdef USE_MPPT
        mppt.loop();
    #endif
    #ifdef USE_BME
        bme.loop();
    #endif
    #ifdef USE_MOTION
        motion.loop();
    #endif

    // Call ttn_loop to check for any state updates or new messages
    ttn_loop();
    
    // Check the interval deltas for all of our sensors, sends a message if needed
    checkIntervalMessages();

    // if user presses button for more than 3 secs, discard our network prefs and reboot
    static bool wasPressed = false;
    // if the button is released after this timestamp we will trigger a reset and restart
    static uint32_t minPressMs;
    if (!digitalRead(BUTTON_PIN)) {
        if (!wasPressed) {
            // just started a new press
            Serial.println("pressing");
            wasPressed = true;
            minPressMs = millis() + 3000;
        }
    }

    else if (wasPressed) {
        // we just did a release
        wasPressed = false;
        if (millis() > minPressMs) {
            // held long enough
            Serial.println("Held down, resetting perfs and EEPROM");
            ttn_erase_prefs();
            eeprom_reset();
            ESP.restart();
        }
    }

    if (millis() > ttn_rejoin_period){
        // Reset our saved TTN perfs and restart the system
        ttn_erase_prefs();
        ESP.restart();
    }

    if (watchdogResetEnabled){
        // Reset our watchdog timer timeout
        esp_task_wdt_reset();
    }
}
