#ifndef NETWORKCONFIGURATION_H
#define NETWORKCONFIGURATION_H

/*
    file that contains the default network configuration
    Note:
        DEFAULT_WIFI_* are WLAN Pins from Module on Adafruit Feather M0 (see Adafruit ATWINC1500 Feather)
        DEFAULT_MQTT_* are MQTT defaults
*/

#define DEFAULT_WIFI_SSID "DigitalLab"
#define DEFAULT_WIFI_PASSWORD "digital42HSR"
#define DEFAULT_HOSTNAME_SARTBOX "SmartBox"
#define DEFAULT_HOSTNAME_VEHICLE "Vehicle"
#define DEFAULT_WIFI_CS 8 // Pins for Adafruit ATWINC1500 Feather
#define DEFAULT_WIFI_IRQ 7
#define DEFAULT_WIFI_RST 4
#define DEFAULT_WIFI_EN 2
#define DEFAULT_MQTT_BROKER_IP1 192 // broker IP distributed in 4 values
#define DEFAULT_MQTT_BROKER_IP2 168
#define DEFAULT_MQTT_BROKER_IP3 1
#define DEFAULT_MQTT_BROKER_IP4 7
#define DEFAULT_MQTT_PORT 1883
#define MAX_JSON_MESSAGES_SAVED 50 // max num of saved JSON items, must be smaller than num of vehicles!
#define MAX_JSON_PARSE_SIZE 300    // max buffer size to parse JSON objects, size of the pool in bytes, can be calculated in https://arduinojson.org/v5/assistant/

#endif