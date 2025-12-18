#include <Arduino.h>
#include "config.h"
#include "modbus_engine.h"

String inputBuffer = "";

// Detailed Help Function - UNIVERSAL VERSION
void printHelp() {
    SerialMon.println("\n--- STEOlab Modbus swissknife 1.9 HELP ---");
    SerialMon.println("COMMANDS & PRACTICAL EXAMPLES:");
    
    SerialMon.println("1. scan <start> <end>");
    SerialMon.println("   Description: Discover active IDs across multiple baud rates.");
    SerialMon.println("   Example: scan 1 15          (Scans IDs 1 to 15)");
    
    SerialMon.println("2. dump <id> <baud> <reg> <n>");
    SerialMon.println("   Description: View raw memory block (Hex/Dec).");
    SerialMon.println("   Example: dump 1 4800 0 20   (Read 20 registers from ID 1)");
    
    SerialMon.println("3. analyze <id> <baud> <reg>");
    SerialMon.println("   Description: Snapshot of 32-bit types (Float/Int32).");
    SerialMon.println("   Example: analyze 2 4800 4   (Check format at Reg 4)");
    
    SerialMon.println("4. watch <id> <baud> <reg>");
    SerialMon.println("   Description: Live monitoring with TX Hex and data variants.");
    SerialMon.println("   Example: watch 1 4800 1     (Monitor data at Reg 1)");
    
    SerialMon.println("5. help");
    SerialMon.println("   Description: Show this detailed guide.");
    
    SerialMon.println("\n--- REVERSE ENGINEERING TIPS ---");
    SerialMon.println("- First use 'scan' to identify the device baud and ID.");
    SerialMon.println("- Use 'dump' to see which registers change when you trigger the sensor.");
    SerialMon.println("- Use 'watch' to capture the exact HEX frames for your documentation.");
    SerialMon.println("------------------------------------------");
}

void setup() {
    SerialMon.begin(115200);
    MBEngine::init();
    SerialMon.println("\n******************************************");
    SerialMon.println("* STEOlab Modbus swissknife 1.9          *");
    SerialMon.println("* Type 'help' for full command list      *");
    SerialMon.println("******************************************");
    SerialMon.print("swissknife> ");
}

void processCommand(String cmd) {
    char buf[64]; 
    cmd.toCharArray(buf, 64);
    char* token = strtok(buf, " "); 
    if (!token) return;

    if (strcmp(token, "help") == 0) {
        printHelp();
    }
    else if (strcmp(token, "scan") == 0) {
        char* a1 = strtok(NULL, " "); 
        char* a2 = strtok(NULL, " ");
        if (a1 && a2) MBEngine::scanNetwork(atoi(a1), atoi(a2));
        else MBEngine::scanNetwork(1, 10);
    } 
    else if (strcmp(token, "dump") == 0) {
        char* id = strtok(NULL, " "); 
        char* baud = strtok(NULL, " ");
        char* reg = strtok(NULL, " "); 
        char* n = strtok(NULL, " ");
        if (id && baud && reg && n) MBEngine::dumpRegisters(atoi(id), atol(baud), atoi(reg), atoi(n));
    } 
    else if (strcmp(token, "analyze") == 0) {
        char* id = strtok(NULL, " "); 
        char* baud = strtok(NULL, " "); 
        char* reg = strtok(NULL, " ");
        if (id && baud && reg) MBEngine::analyzeRegister(atoi(id), atol(baud), atoi(reg));
    } 
    else if (strcmp(token, "watch") == 0) {
        char* id = strtok(NULL, " "); 
        char* baud = strtok(NULL, " "); 
        char* reg = strtok(NULL, " ");
        if (id && baud && reg) MBEngine::watchRegister(atoi(id), atol(baud), atoi(reg));
    } else {
        SerialMon.println("Unknown command. Type 'help'.");
    }
    SerialMon.print("\nswissknife> ");
}

void loop() {
    while (SerialMon.available()) {
        char c = SerialMon.read();
        if (c == '\n' || c == '\r') {
            if (inputBuffer.length() > 0) { 
                processCommand(inputBuffer); 
                inputBuffer = ""; 
            }
        } else { 
            inputBuffer += c; 
        }
    }
}