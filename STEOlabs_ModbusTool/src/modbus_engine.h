#ifndef MODBUS_ENGINE_H
#define MODBUS_ENGINE_H

#include <Arduino.h>

/**
 * STEOlab Modbus swissknife 2.0 - TOTAL DIAGNOSTIC
 * Universal Engine Header
 */
namespace MBEngine {
    // Inizializza i pin RS485 e l'alimentazione del sensore
    void init();

    // Configura il baudrate della Serial2 (4800, 9600, etc.)
    void configureUART(uint32_t baud, uint32_t config);

    // Scansione "Verbose" di tutti gli ID e Baudrate
    void scanNetwork(uint8_t startID, uint8_t endID);

    // Ispezione raw della memoria (Hex, Uint16, Int16)
    void dumpRegisters(uint8_t id, uint32_t baud, uint16_t startReg, uint16_t count);

    // Analisi istantanea con decodifica di TUTTE le 4 combinazioni Float e Int32
    void analyzeRegister(uint8_t id, uint32_t baud, uint16_t reg);

    // Monitoraggio continuo multi-formato (per identificare Endianness e variazioni)
    void watchRegister(uint8_t id, uint32_t baud, uint16_t reg);
}

#endif