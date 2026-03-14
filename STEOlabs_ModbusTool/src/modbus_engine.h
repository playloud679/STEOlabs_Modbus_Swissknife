#ifndef MODBUS_ENGINE_H
#define MODBUS_ENGINE_H

#include <Arduino.h>

/**
 * STEOlab Modbus swissknife 2.9 - Industrial Pro Edition
 * Header basato su specifiche ufficiali Aqualabo e stile minimalista.
 */
namespace MBEngine {
    
    // Inizializza i pin hardware e gestisce il warm-up del sensore.
    void init();

    // Esegue una scansione rapida del bus usando la funzione Modbus 0x11 (Report Slave ID).
    // Testa configurazioni 8N1 e 8N2 per coprire setup standard e factory default.
    void scanNetwork(uint8_t startID, uint8_t endID);

    // Ispeziona un blocco di registri. Include una protezione per non superare i 60 registri,
    // evitando errori di indirizzamento illegale (Exception 0x02) comuni nei sensori industriali.
    void dumpRegisters(uint8_t id, uint32_t baud, uint16_t startReg, uint16_t count);

    // Snapshot diagnostico di un registro specifico con decodifica Float IEEE 754 (Big-Endian/ABCD).
    void analyzeRegister(uint8_t id, uint32_t baud, uint16_t reg);

    // Monitoraggio continuo (loop da 1s) per osservare la stabilizzazione dei parametri chimici.
    void watchRegister(uint8_t id, uint32_t baud, uint16_t reg);

    // Pipeline automatica: scan del bus, dump iniziale, stima formato 32-bit e watch continuo.
    void autoDiscoverAndWatch(uint8_t startID, uint8_t endID);
}

#endif