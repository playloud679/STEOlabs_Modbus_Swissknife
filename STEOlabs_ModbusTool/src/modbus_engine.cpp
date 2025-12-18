#include <Arduino.h>
#include "modbus_engine.h"
#include "config.h"
#include <ModbusMaster.h>

static ModbusMaster node;

void preTx() { digitalWrite(RS485_DE_RE, HIGH); delayMicroseconds(50); }
void postTx() { delayMicroseconds(50); digitalWrite(RS485_DE_RE, LOW); }

float rawToFloat(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
    uint8_t data[4] = {b0, b1, b2, b3};
    float f;
    memcpy(&f, data, 4);
    return f;
}

namespace MBEngine {
    void init() {
        pinMode(RS485_DE_RE, OUTPUT); 
        digitalWrite(RS485_DE_RE, LOW);
        pinMode(MODBUS_SENS_PWR_PIN, OUTPUT);
        digitalWrite(MODBUS_SENS_PWR_PIN, LOW); 
        delay(500);
    }

    void configureUART(uint32_t baud, uint32_t config) {
        Serial2.end();
        delay(100);
        Serial2.begin(baud, config, RS485_RX_PIN, RS485_TX_PIN);
        node.preTransmission(preTx);
        node.postTransmission(postTx);
    }

    // 
    bool probeRaw(uint8_t id) {
        while(Serial2.available()) Serial2.read();
        uint8_t frame[8] = { id, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00 };
        // CRC calcolato internamente o tramite funzione esterna... 
        // Per brevità qui usiamo il probe semplificato della v1.9
        digitalWrite(RS485_DE_RE, HIGH);
        Serial2.write(frame, 8); // Nota: assicurati di avere la funzione CRC se non usi node.begin
        Serial2.flush();
        digitalWrite(RS485_DE_RE, LOW);
        uint32_t start = millis();
        while (millis() - start < 150) {
            if (Serial2.available()) { if (Serial2.read() == id) return true; }
        }
        return false;
    }

    void scanNetwork(uint8_t startID, uint8_t endID) {
        uint32_t bauds[] = {9600, 19200, 115200, 4800};
        int foundCount = 0;
        SerialMon.println("\n[ SCANNING BUS... ]");
        for (int b = 0; b < 4; b++) {
            uint32_t currentBaud = bauds[b];
            configureUART(currentBaud, SERIAL_8N1);
            SerialMon.printf("\n--- Speed: %u baud ---\n", currentBaud);
            for (int id = startID; id <= endID; id++) {
                SerialMon.printf("ID %d: ", id);
                if (probeRaw(id)) { SerialMon.println(">>> [!] FOUND!"); foundCount++; }
                else { SerialMon.println("no"); }
                if (SerialMon.available()) return;
            }
        }
    }

    // 
    void dumpRegisters(uint8_t id, uint32_t baud, uint16_t startReg, uint16_t count) {
        configureUART(baud, SERIAL_8N1);
        node.begin(id, Serial2);
        SerialMon.printf("\n--- DUMP ID:%d (%u baud) ---\n", id, baud);
        uint8_t res = node.readHoldingRegisters(startReg, count);
        if (res == node.ku8MBSuccess) {
            for (int i = 0; i < count; i++) {
                uint16_t val = node.getResponseBuffer(i);
                SerialMon.printf("Reg[%05d]: 0x%04X | Dec: %5u | Signed: %d\n", startReg + i, val, val, (int16_t)val);
            }
        } else SerialMon.printf("Error: 0x%02X\n", res);
    }

    // 
    void analyzeRegister(uint8_t id, uint32_t baud, uint16_t reg) {
        configureUART(baud, SERIAL_8N1);
        node.begin(id, Serial2);
        uint8_t res = node.readHoldingRegisters(reg, 2);
        if (res != node.ku8MBSuccess) { SerialMon.printf("\nError: 0x%02X\n", res); return; }
        uint16_t r1 = node.getResponseBuffer(0), r2 = node.getResponseBuffer(1);
        uint8_t A=(r1>>8)&0xFF, B=r1&0xFF, C=(r2>>8)&0xFF, D=r2&0xFF;
        
        SerialMon.printf("\n--- FULL 32-BIT ANALYSIS (ID:%d Reg:%d) ---\n", id, reg);
        SerialMon.printf("HEX BYTES: [%02X %02X] [%02X %02X]\n", A, B, C, D);
        SerialMon.println("FLOATS (IEEE 754):");
        SerialMon.printf("  > ABCD (Big Endian):    %f\n", rawToFloat(D,C,B,A));
        SerialMon.printf("  > DCBA (Little Endian): %f\n", rawToFloat(A,B,C,D));
        SerialMon.printf("  > CDAB (Mid-Little):    %f\n", rawToFloat(B,A,D,C));
        SerialMon.printf("  > BADC (Mid-Big):       %f\n", rawToFloat(C,D,A,B));
        SerialMon.println("INTEGERS:");
        SerialMon.printf("  > Uint16 (Reg 1):       %u\n", r1);
        SerialMon.printf("  > Int32 ABCD:           %ld\n", (long)((uint32_t)r1 << 16 | r2));
        SerialMon.printf("  > Int32 CDAB:           %ld\n", (long)((uint32_t)r2 << 16 | r1));
    }

    void watchRegister(uint8_t id, uint32_t baud, uint16_t reg) {
        configureUART(baud, SERIAL_8N1);
        node.begin(id, Serial2);
        while(SerialMon.available()) SerialMon.read();
        SerialMon.println("\n[ FULL WATCH ] - Press Enter to stop");
        while (true) {
            uint8_t res = node.readHoldingRegisters(reg, 2);
            if (res == node.ku8MBSuccess) {
                uint16_t r1 = node.getResponseBuffer(0), r2 = node.getResponseBuffer(1);
                uint8_t A=(r1>>8)&0xFF, B=r1&0xFF, C=(r2>>8)&0xFF, D=r2&0xFF;
                SerialMon.printf("\n--------------------------------------------------");
                SerialMon.printf("\nBYTES: [%02X %02X][%02X %02X] | U16: %u", A, B, C, D, r1);
                SerialMon.printf("\nF-ABCD: %f | F-DCBA: %f", rawToFloat(D,C,B,A), rawToFloat(A,B,C,D));
                SerialMon.printf("\nF-CDAB: %f | F-BADC: %f", rawToFloat(B,A,D,C), rawToFloat(C,D,A,B));
            }
            if (SerialMon.available()) break;
            delay(2500);
        }
    }
}