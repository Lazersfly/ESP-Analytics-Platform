#pragma once

#include <Arduino.h>
#include <lmic.h>
void ttn_register(void (*callback)(uint8_t message));

// -----------------------------------------------------------------------------
// Version
// -----------------------------------------------------------------------------

#define APP_NAME                "Victron MPPT"
#define APP_VERSION             "1.0.0"

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

// Select which T-Beam board is being used. Only uncomment one.
// #define T_BEAM_V07  // AKA Rev0 (first board released)
#define T_BEAM_V10  // AKA Rev1 (second board released)

// If using a single-channel gateway, uncomment this next option and set to your gateway's channel
//#define SINGLE_CHANNEL_GATEWAY  0

//Uncomment to enable discarding network settings by long pressing second button
#define PREFS_DISCARD

// If you are having difficulty sending messages to TTN after the first successful send,
// uncomment the next option and experiment with values (~ 1 - 5)
//#define CLOCK_ERROR             5

#define USE_GPS
#define USE_AXP
#define USE_MPPT
#define USE_BME
#define USE_MOTION

#define SERIAL_BAUD               115200          // Serial debug baud rate
#define LORAWAN_PORT              10              // Port the messages will be sent to
#define LORAWAN_CONFIRMED_EVERY   5              // Send confirmed message every these many messages (0 means never)
#define LORAWAN_SF                DR_SF10         // Spreading factor (recommended DR_SF7 for ttn network map purposes, DR_SF10 works for slow moving trackers)
#define LORAWAN_ADR               0               // Enable ADR
#define DEFAULT_TTN_REJOIN_PERIOD         (48 * 1000 * 3600) // Rejoin TTN as failsafe every this millis
#define EEPROM_SIZE               16
#define RESET_CONFIRM_STRING       "1488"         //Required string to include after R* reset downlink command
#define MAX_TTN_PACKET_WAIT 5000

#define WDT_TIMEOUT 30

#define GPS_START_BYTE 0x0A
#define AXP_START_BYTE 0x0B
#define MPPT_START_BYTE 0x0C
#define BME_START_BYTE 0x0D
#define MOTION_START_BYTE 0x0E

#define GPS_EEPROM_ADDRESS 0
#define AXP_EEPROM_ADDRESS 4
#define MPPT_EEPROM_ADDRESS 8
#define BME_EEPROM_ADDRESS 12

// -----------------------------------------------------------------------------
// Custom messages
// -----------------------------------------------------------------------------

#define EV_QUEUED       100
#define EV_PENDING      101
#define EV_ACK          102
#define EV_RESPONSE     103

// -----------------------------------------------------------------------------
// General
// -----------------------------------------------------------------------------

#define I2C_SDA         21
#define I2C_SCL         22

#if defined(T_BEAM_V07)
#define BUTTON_PIN      39
#elif defined(T_BEAM_V10)
#define BUTTON_PIN      38
#endif

// -----------------------------------------------------------------------------
// GPS
// -----------------------------------------------------------------------------

#define GPS_SERIAL_NUM  1
#define GPS_BAUDRATE    9600
#define GPS_DEFAULT_SEND_INTERVAL (600 * 1000)


#if defined(T_BEAM_V07)
#define GPS_RX_PIN      12
#define GPS_TX_PIN      15
#elif defined(T_BEAM_V10)
#define GPS_RX_PIN      34
#define GPS_TX_PIN      12
#endif

// -----------------------------------------------------------------------------
// LoRa SPI
// -----------------------------------------------------------------------------

#define SCK_GPIO        5
#define MISO_GPIO       19
#define MOSI_GPIO       27
#define NSS_GPIO        18
#if defined(T_BEAM_V10)
#define RESET_GPIO      14
#else
#define RESET_GPIO      23
#endif
#define DIO0_GPIO       26
#define DIO1_GPIO       33 // Note: not really used on this board
#define DIO2_GPIO       32 // Note: not really used on this board

// -----------------------------------------------------------------------------
// AXP192 (Rev1-specific options)
// -----------------------------------------------------------------------------

// #define AXP192_SLAVE_ADDRESS  0x34 // Now defined in axp20x.h
#define GPS_POWER_CTRL_CH     3
#define LORA_POWER_CTRL_CH    2
#define PMU_IRQ               35
#define AXP_DEFAULT_SEND_INTERVAL (240 * 1000)

// -----------------------------------------------------------------------------
// Victron MPPT
// Victron Connect Bluetooth PIN is "001488"
// -----------------------------------------------------------------------------

#define MPPT_SERIAL_NUM       2        // Hardware Serial NUM to init for the MPPT controller's serial
#define MPPT_SERIAL_BAUD      19200    // Baud for Victron MPPT controller
#define MPPT_RX               2        // RX pin for MPPT controller
#define MPPT_TX               13       // TX pin for MPPT controller
#define MPPT_DEFAULT_SEND_INTERVAL (60 * 1000)
// -----------------------------------------------------------------------------
// BME280
// -----------------------------------------------------------------------------
#define BME_DEFAULT_SEND_INTERVAL (60 * 1000)
// -----------------------------------------------------------------------------
// Motion Sensor
// -----------------------------------------------------------------------------
#ifdef USE_MOTION
    #define MOTION_MINIMUM_SEND_INTERVAL (10 * 1000) // Disabled if 0
    #define MOTION_PIN 15
#endif