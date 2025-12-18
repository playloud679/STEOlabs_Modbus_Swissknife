#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>

// RS485 Hardware Pins
#define RS485_RX_PIN 19
#define RS485_TX_PIN 23
#define RS485_DE_RE 18 

// Power Management (Active LOW)
#define MODBUS_SENS_PWR_PIN 33  

// Debug Console
#define SerialMon Serial
#endif
