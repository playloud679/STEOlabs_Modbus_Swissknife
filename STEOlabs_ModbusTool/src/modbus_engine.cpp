#include <Arduino.h>
#include "modbus_engine.h"
#include "config.h"
#include <ModbusMaster.h>
#include <HardwareSerial.h>
#include "driver/uart.h"
#include <math.h>

static ModbusMaster node;

// Use UART1 (Serial1) for RS485 on Waveshare ESP32-S3 RS485-CAN board
#define RS485_SERIAL Serial1

// With UART_MODE_RS485_HALF_DUPLEX the hardware controls TXDEN pin,
// so pre/post transmission hooks can be empty.
void preTx() {}
void postTx() {}

float rawToFloat(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
    uint8_t data[4] = {b0, b1, b2, b3};
    float f;
    memcpy(&f, data, 4);
    return f;
}

enum RegisterKind : uint8_t {
    HoldingRegisters,
    InputRegisters
};

enum FloatOrder : uint8_t {
    OrderABCD,
    OrderBADC,
    OrderCDAB,
    OrderDCBA
};

struct ScanResult {
    bool found = false;
    uint8_t id = 0;
    uint32_t baud = 0;
    uint32_t serialConfig = SERIAL_8N1;
    RegisterKind regType = HoldingRegisters;
    uint16_t probeReg = 0;
};

struct AutoSnapshot {
    static const uint16_t kMaxRegs = 48;
    uint16_t startReg = 0;
    uint16_t count = 0;
    uint16_t values[kMaxRegs] = {0};
    bool valid[kMaxRegs] = {false};
    RegisterKind source[kMaxRegs] = {HoldingRegisters};
};

struct AutoSelection {
    bool valid = false;
    uint16_t reg = 0;
    RegisterKind regType = HoldingRegisters;
    FloatOrder order = OrderABCD;
    uint16_t firstWord = 0;
    uint16_t secondWord = 0;
    float value = 0.0f;
    uint32_t u32 = 0;
    int score = -1000;
    int confidence = 0;
};

struct RegisterActivity {
    bool initialized = false;
    RegisterKind source = HoldingRegisters;
    uint16_t firstValue = 0;
    uint16_t lastValue = 0;
    uint16_t minValue = 0;
    uint16_t maxValue = 0;
    uint16_t changes = 0;
    uint32_t totalDelta = 0;
    uint16_t sampleCount = 0;
    uint16_t nearZeroCount = 0;
    uint16_t nearMaxCount = 0;
};

static const uint8_t kMaxAutoCandidates = 6;

namespace MBEngine {
    void configureUART(uint32_t baud, uint32_t config);
}

const char* serialConfigLabel(uint32_t config) {
    return config == SERIAL_8N2 ? "8N2" : "8N1";
}

const char* registerKindLabel(RegisterKind type) {
    return type == InputRegisters ? "IR" : "HR";
}

const char* floatOrderLabel(FloatOrder order) {
    switch (order) {
        case OrderABCD: return "ABCD";
        case OrderBADC: return "BADC";
        case OrderCDAB: return "CDAB";
        case OrderDCBA: return "DCBA";
        default: return "ABCD";
    }
}

uint8_t readRegisters(RegisterKind type, uint16_t startReg, uint8_t count) {
    if (type == InputRegisters) {
        return node.readInputRegisters(startReg, count);
    }
    return node.readHoldingRegisters(startReg, count);
}

float decodeFloat(uint16_t r1, uint16_t r2, FloatOrder order) {
    uint8_t a = (r1 >> 8) & 0xFF;
    uint8_t b = r1 & 0xFF;
    uint8_t c = (r2 >> 8) & 0xFF;
    uint8_t d = r2 & 0xFF;

    switch (order) {
        case OrderABCD: return rawToFloat(d, c, b, a);
        case OrderBADC: return rawToFloat(c, d, a, b);
        case OrderCDAB: return rawToFloat(b, a, d, c);
        case OrderDCBA: return rawToFloat(a, b, c, d);
        default: return rawToFloat(d, c, b, a);
    }
}

uint32_t decodeUint32(uint16_t r1, uint16_t r2, FloatOrder order) {
    uint8_t a = (r1 >> 8) & 0xFF;
    uint8_t b = r1 & 0xFF;
    uint8_t c = (r2 >> 8) & 0xFF;
    uint8_t d = r2 & 0xFF;
    uint8_t ordered[4];

    switch (order) {
        case OrderABCD:
            ordered[0] = a; ordered[1] = b; ordered[2] = c; ordered[3] = d;
            break;
        case OrderBADC:
            ordered[0] = b; ordered[1] = a; ordered[2] = d; ordered[3] = c;
            break;
        case OrderCDAB:
            ordered[0] = c; ordered[1] = d; ordered[2] = a; ordered[3] = b;
            break;
        case OrderDCBA:
            ordered[0] = d; ordered[1] = c; ordered[2] = b; ordered[3] = a;
            break;
        default:
            ordered[0] = a; ordered[1] = b; ordered[2] = c; ordered[3] = d;
            break;
    }

    return ((uint32_t)ordered[0] << 24) |
           ((uint32_t)ordered[1] << 16) |
           ((uint32_t)ordered[2] << 8) |
           (uint32_t)ordered[3];
}

bool isMostlyEmptyPair(uint16_t r1, uint16_t r2) {
    return (r1 == 0 && r2 == 0) || (r1 == 0xFFFF && r2 == 0xFFFF);
}

bool isSuspiciousConstantValue(float value) {
    if (!isfinite(value)) return true;

    const float suspicious[] = {0.0f, 1.0f, -1.0f, 10.0f, 100.0f, 1000.0f};
    for (uint8_t i = 0; i < 6; i++) {
        if (fabsf(value - suspicious[i]) < 0.0005f) return true;
    }
    return false;
}

int scoreFloatCandidate(float value) {
    if (!isfinite(value)) return -1000;

    float absValue = fabsf(value);
    int score = 0;

    if (absValue == 0.0f) score -= 40;
    if (absValue >= 0.001f && absValue <= 10000.0f) score += 60;
    else if (absValue >= 0.00001f && absValue <= 1000000.0f) score += 35;
    else if (absValue < 1e-20f || absValue > 1e9f) score -= 35;
    else score += 5;

    if (absValue > 0.0f && absValue < 1e-6f) score -= 15;

    float rounded = roundf(value);
    if (fabsf(value - rounded) > 0.0001f) score += 5;

    return score;
}

void clearSerialInput() {
    while (SerialMon.available()) SerialMon.read();
}

void waitForUserTrigger() {
    clearSerialInput();
    SerialMon.println("[AUTO] Press ENTER when ready to stimulate the sensor.");
    while (!SerialMon.available()) {
        delay(50);
    }
    clearSerialInput();
}

bool findFirstDevice(uint8_t startID, uint8_t endID, const char* prefix, ScanResult& result) {
    const uint32_t bauds[] = {9600, 4800, 19200};
    const uint32_t configs[] = {SERIAL_8N1, SERIAL_8N2};
    const RegisterKind regTypes[] = {HoldingRegisters, InputRegisters};
    const uint16_t probeRegs[] = {0, 1};

    while (SerialMon.available()) SerialMon.read();

    for (uint8_t b = 0; b < 3; b++) {
        for (uint8_t c = 0; c < 2; c++) {
            MBEngine::configureUART(bauds[b], configs[c]);
            SerialMon.printf("%s %u %s\n", prefix, bauds[b], serialConfigLabel(configs[c]));

            for (uint8_t id = startID; id <= endID; id++) {
                node.begin(id, RS485_SERIAL);

                for (uint8_t regTypeIndex = 0; regTypeIndex < 2; regTypeIndex++) {
                    for (uint8_t probeIndex = 0; probeIndex < 2; probeIndex++) {
                        uint8_t res = readRegisters(regTypes[regTypeIndex], probeRegs[probeIndex], 1);
                        if (res == node.ku8MBSuccess) {
                            result.found = true;
                            result.id = id;
                            result.baud = bauds[b];
                            result.serialConfig = configs[c];
                            result.regType = regTypes[regTypeIndex];
                            result.probeReg = probeRegs[probeIndex];
                            return true;
                        }
                    }
                }

                if (SerialMon.available()) {
                    int ch = SerialMon.read();
                    if (ch == 'q' || ch == 'Q') {
                        SerialMon.printf("%s aborted by user.\n", prefix);
                        return false;
                    }
                }
            }
        }
    }

    return false;
}

bool captureSnapshot(const ScanResult& device, AutoSnapshot& snapshot, uint16_t startReg, uint16_t count) {
    if (count > AutoSnapshot::kMaxRegs) count = AutoSnapshot::kMaxRegs;

    snapshot.startReg = startReg;
    snapshot.count = count;
    for (uint16_t i = 0; i < AutoSnapshot::kMaxRegs; i++) {
        snapshot.values[i] = 0;
        snapshot.valid[i] = false;
        snapshot.source[i] = device.regType;
    }

    MBEngine::configureUART(device.baud, device.serialConfig);
    node.begin(device.id, RS485_SERIAL);

    const uint8_t step = 4;
    int readable = 0;

    for (uint16_t base = startReg; base < startReg + count; base += step) {
        uint8_t chunk = (uint8_t)min((uint16_t)step, (uint16_t)((startReg + count) - base));
        RegisterKind usedType = device.regType;
        uint8_t res = readRegisters(usedType, base, chunk);

        if (res != node.ku8MBSuccess) {
            usedType = (device.regType == HoldingRegisters) ? InputRegisters : HoldingRegisters;
            res = readRegisters(usedType, base, chunk);
        }

        if (res != node.ku8MBSuccess) continue;

        for (uint8_t i = 0; i < chunk; i++) {
            uint16_t idx = (base - startReg) + i;
            snapshot.values[idx] = node.getResponseBuffer(i);
            snapshot.valid[idx] = true;
            snapshot.source[idx] = usedType;
            readable++;
        }
    }

    return readable > 0;
}

void printSnapshotSummary(const AutoSnapshot& snapshot) {
    int readable = 0;
    int interesting = 0;

    for (uint16_t i = 0; i < snapshot.count; i++) {
        if (snapshot.valid[i]) readable++;
        if (snapshot.valid[i] && snapshot.values[i] != 0 && snapshot.values[i] != 0xFFFF) interesting++;
    }

    SerialMon.printf("[AUTO] Readable regs: %d/%u in window %u..%u\n",
                     readable,
                     snapshot.count,
                     snapshot.startReg,
                     snapshot.startReg + snapshot.count - 1);

    int shown = 0;
    for (uint16_t i = 0; i < snapshot.count && shown < 12; i++) {
        if (!snapshot.valid[i]) continue;
        if (interesting > 0 && (snapshot.values[i] == 0 || snapshot.values[i] == 0xFFFF)) continue;

        SerialMon.printf("[DUMP] R[%05u] %s  0x%04X  %u\n",
                         snapshot.startReg + i,
                         registerKindLabel(snapshot.source[i]),
                         snapshot.values[i],
                         snapshot.values[i]);
        shown++;
    }

    if (shown == 0) {
        for (uint16_t i = 0; i < snapshot.count && shown < 8; i++) {
            if (!snapshot.valid[i]) continue;
            SerialMon.printf("[DUMP] R[%05u] %s  0x%04X  %u\n",
                             snapshot.startReg + i,
                             registerKindLabel(snapshot.source[i]),
                             snapshot.values[i],
                             snapshot.values[i]);
            shown++;
        }
    }
}

void initActivity(const AutoSnapshot& snapshot, RegisterActivity* activity) {
    for (uint16_t i = 0; i < snapshot.count; i++) {
        activity[i].initialized = snapshot.valid[i];
        activity[i].source = snapshot.source[i];
        activity[i].firstValue = snapshot.values[i];
        activity[i].lastValue = snapshot.values[i];
        activity[i].minValue = snapshot.values[i];
        activity[i].maxValue = snapshot.values[i];
        activity[i].changes = 0;
        activity[i].totalDelta = 0;
        activity[i].sampleCount = snapshot.valid[i] ? 1 : 0;
        activity[i].nearZeroCount = (snapshot.valid[i] && snapshot.values[i] <= 255) ? 1 : 0;
        activity[i].nearMaxCount = (snapshot.valid[i] && snapshot.values[i] >= 65000) ? 1 : 0;
    }
}

void updateActivity(RegisterActivity& activity, RegisterKind source, uint16_t value) {
    if (!activity.initialized) {
        activity.initialized = true;
        activity.source = source;
        activity.firstValue = value;
        activity.lastValue = value;
        activity.minValue = value;
        activity.maxValue = value;
        activity.changes = 0;
        activity.totalDelta = 0;
        activity.sampleCount = 1;
        activity.nearZeroCount = value <= 255 ? 1 : 0;
        activity.nearMaxCount = value >= 65000 ? 1 : 0;
        return;
    }

    activity.source = source;
    activity.sampleCount++;
    if (value <= 255) activity.nearZeroCount++;
    if (value >= 65000) activity.nearMaxCount++;
    if (value != activity.lastValue) {
        uint16_t delta = value > activity.lastValue ? value - activity.lastValue : activity.lastValue - value;
        // Treat 16-bit wraparound as a small cyclic jump, not a huge excursion.
        if (delta > 32768U) delta = 65535U - delta;
        activity.changes++;
        activity.totalDelta += delta;
    }

    if (value < activity.minValue) activity.minValue = value;
    if (value > activity.maxValue) activity.maxValue = value;
    activity.lastValue = value;
}

uint32_t activityStrength(const RegisterActivity& activity) {
    if (!activity.initialized) return 0;
    uint32_t span = activity.maxValue - activity.minValue;
    if (span > 4000U) span = 4000U;

    uint32_t totalDelta = activity.totalDelta;
    if (totalDelta > 8000U) totalDelta = 8000U;

    return ((uint32_t)activity.changes * 200U) + totalDelta + span;
}

int32_t registerSignalScore(const RegisterActivity& activity) {
    if (!activity.initialized) return -100000;

    int32_t score = (int32_t)activityStrength(activity);
    uint32_t span = activity.maxValue - activity.minValue;

    // Strongly reward signals that idle near zero and rise positively under stimulus.
    if (activity.firstValue <= 32 && activity.maxValue >= 100) {
        score += 70000;
        if (activity.nearZeroCount > 0 && activity.sampleCount > 0 &&
            activity.nearZeroCount * 2 >= activity.sampleCount) {
            score += 15000;
        }
    }

    // Also reward low-baseline signals that make meaningful excursions.
    if (activity.firstValue <= 255 && span >= 100) {
        score += 15000;
    }

    // Penalize wrapped/signed-style signals that sit near 0xFFFF at rest.
    if (activity.firstValue >= 65000) {
        score -= 55000;
        if (activity.minValue <= 5000) score -= 12000;
    }

    // Penalize noisy zero-heavy registers that mostly collapse to zero.
    if (activity.sampleCount > 0 && activity.firstValue == 0) {
        if (activity.nearZeroCount * 2 < activity.sampleCount) score -= 18000;
        if (activity.lastValue == 0 && span > 10000) score -= 25000;
    }

    if (span < 100 && activity.changes > 0) score -= 5000;

    return score;
}

void printTopActiveRegisters(const AutoSnapshot& snapshot, const RegisterActivity* activity) {
    int topIndex[5] = {-1, -1, -1, -1, -1};
    int32_t topScore[5] = {-100000, -100000, -100000, -100000, -100000};

    for (uint16_t i = 0; i < snapshot.count; i++) {
        int32_t score = registerSignalScore(activity[i]);
        if (score <= -100000) continue;

        for (uint8_t slot = 0; slot < 5; slot++) {
            if (score <= topScore[slot]) continue;

            for (int shift = 4; shift > (int)slot; shift--) {
                topScore[shift] = topScore[shift - 1];
                topIndex[shift] = topIndex[shift - 1];
            }
            topScore[slot] = score;
            topIndex[slot] = i;
            break;
        }
    }

    bool any = false;
    for (uint8_t slot = 0; slot < 5; slot++) {
        if (topIndex[slot] < 0) continue;
        any = true;
        uint16_t idx = (uint16_t)topIndex[slot];
        SerialMon.printf("[AUTO] Moving %u: R[%05u] %s  first:%u  last:%u  span:%u  changes:%u\n",
                         slot + 1,
                         snapshot.startReg + idx,
                         registerKindLabel(activity[idx].source),
                         activity[idx].firstValue,
                         activity[idx].lastValue,
                         activity[idx].maxValue - activity[idx].minValue,
                         activity[idx].changes);
    }

    if (!any) {
        SerialMon.println("[AUTO] No moving registers detected during stimulus window.");
    }
}

void findTopActiveRegisterIndices(const AutoSnapshot& snapshot, const RegisterActivity* activity, int& firstIndex, int& secondIndex) {
    firstIndex = -1;
    secondIndex = -1;
    int32_t firstScore = -100000;
    int32_t secondScore = -100000;

    for (uint16_t i = 0; i < snapshot.count; i++) {
        if (!snapshot.valid[i]) continue;

        int32_t score = registerSignalScore(activity[i]);
        if (score <= -100000) continue;

        if (score > firstScore) {
            secondScore = firstScore;
            secondIndex = firstIndex;
            firstScore = score;
            firstIndex = i;
        } else if (score > secondScore) {
            secondScore = score;
            secondIndex = i;
        }
    }
}

bool runStimulusWindow(const ScanResult& device, AutoSnapshot& snapshot, RegisterActivity* activity, uint32_t durationMs) {
    waitForUserTrigger();
    SerialMon.printf("[AUTO] Stimulate NOW for %lu seconds. Press any key to stop early.\n", durationMs / 1000UL);

    AutoSnapshot previous = snapshot;
    AutoSnapshot current = snapshot;
    uint32_t endAt = millis() + durationMs;
    uint32_t nextStatusAt = millis() + 2000;

    while ((long)(millis() - endAt) < 0) {
        if (!captureSnapshot(device, current, snapshot.startReg, snapshot.count)) {
            delay(250);
            continue;
        }

        bool anyChange = false;
        uint8_t shown = 0;
        for (uint16_t i = 0; i < current.count; i++) {
            if (!current.valid[i]) continue;

            updateActivity(activity[i], current.source[i], current.values[i]);

            if (!previous.valid[i]) continue;
            if (current.values[i] == previous.values[i]) continue;

            if (!anyChange) {
                SerialMon.print("[AUTO] Delta ");
                anyChange = true;
            }

            if (shown < 6) {
                SerialMon.printf("R[%05u]:%u->%u  ",
                                 current.startReg + i,
                                 previous.values[i],
                                 current.values[i]);
                shown++;
            }
        }

        if (anyChange) {
            SerialMon.println();
        } else if ((long)(millis() - nextStatusAt) >= 0) {
            uint32_t remainingMs = endAt > millis() ? endAt - millis() : 0;
            SerialMon.printf("[AUTO] Waiting for register movement... %lus left\n", remainingMs / 1000UL);
            nextStatusAt = millis() + 2000;
        }

        previous = current;

        if (SerialMon.available()) {
            clearSerialInput();
            snapshot = current;
            return true;
        }

        delay(400);
    }

    snapshot = previous;
    return true;
}

void sortCandidatesByScore(AutoSelection* candidates, uint8_t count) {
    for (uint8_t i = 0; i < count; i++) {
        for (uint8_t j = i + 1; j < count; j++) {
            if (candidates[j].score > candidates[i].score) {
                AutoSelection tmp = candidates[i];
                candidates[i] = candidates[j];
                candidates[j] = tmp;
            }
        }
    }
}

void insertCandidate(AutoSelection* candidates, uint8_t maxCount, uint8_t& count, const AutoSelection& candidate) {
    if (count < maxCount) {
        candidates[count++] = candidate;
        sortCandidatesByScore(candidates, count);
        return;
    }

    if (candidate.score <= candidates[count - 1].score) return;

    candidates[count - 1] = candidate;
    sortCandidatesByScore(candidates, count);
}

uint8_t collectTopCandidates(const AutoSnapshot& snapshot, AutoSelection* candidates, uint8_t maxCount) {
    uint8_t count = 0;

    for (uint16_t i = 0; i + 1 < snapshot.count; i++) {
        if (!snapshot.valid[i] || !snapshot.valid[i + 1]) continue;
        if (snapshot.source[i] != snapshot.source[i + 1]) continue;

        uint16_t r1 = snapshot.values[i];
        uint16_t r2 = snapshot.values[i + 1];
        if (isMostlyEmptyPair(r1, r2)) continue;

        int bestOrderScore = -1000;
        int secondOrderScore = -1000;
        FloatOrder bestOrder = OrderABCD;
        float bestValue = 0.0f;

        const FloatOrder orders[] = {OrderABCD, OrderBADC, OrderCDAB, OrderDCBA};
        for (uint8_t orderIndex = 0; orderIndex < 4; orderIndex++) {
            float decoded = decodeFloat(r1, r2, orders[orderIndex]);
            int score = scoreFloatCandidate(decoded);

            if (score > bestOrderScore) {
                secondOrderScore = bestOrderScore;
                bestOrderScore = score;
                bestOrder = orders[orderIndex];
                bestValue = decoded;
            } else if (score > secondOrderScore) {
                secondOrderScore = score;
            }
        }

        int confidence = bestOrderScore - secondOrderScore;
        int totalScore = bestOrderScore + max(confidence, 0);
        AutoSelection candidate;
        candidate.valid = true;
        candidate.reg = snapshot.startReg + i;
        candidate.regType = snapshot.source[i];
        candidate.order = bestOrder;
        candidate.firstWord = r1;
        candidate.secondWord = r2;
        candidate.value = bestValue;
        candidate.u32 = decodeUint32(r1, r2, bestOrder);
        candidate.score = totalScore;
        candidate.confidence = confidence;
        insertCandidate(candidates, maxCount, count, candidate);
    }

    return count;
}

void refineCandidatesWithLiveSamples(const ScanResult& device, AutoSelection* candidates, uint8_t count) {
    MBEngine::configureUART(device.baud, device.serialConfig);
    node.begin(device.id, RS485_SERIAL);

    for (uint8_t i = 0; i < count; i++) {
        AutoSelection& candidate = candidates[i];
        bool hasSample = false;
        bool rawChanged = false;
        uint8_t successReads = 0;
        float minValue = 0.0f;
        float maxValue = 0.0f;
        float lastValue = 0.0f;
        uint16_t lastR1 = candidate.firstWord;
        uint16_t lastR2 = candidate.secondWord;
        uint32_t lastRaw = ((uint32_t)candidate.firstWord << 16) | candidate.secondWord;

        for (uint8_t sample = 0; sample < 4; sample++) {
            uint8_t res = readRegisters(candidate.regType, candidate.reg, 2);
            if (res != node.ku8MBSuccess) {
                candidate.score -= 80;
                delay(250);
                continue;
            }

            uint16_t r1 = node.getResponseBuffer(0);
            uint16_t r2 = node.getResponseBuffer(1);
            float decoded = decodeFloat(r1, r2, candidate.order);
            if (!isfinite(decoded)) {
                candidate.score -= 120;
                delay(250);
                continue;
            }

            uint32_t raw = ((uint32_t)r1 << 16) | r2;
            if (hasSample) {
                if (raw != lastRaw) rawChanged = true;
                if (decoded < minValue) minValue = decoded;
                if (decoded > maxValue) maxValue = decoded;
            } else {
                minValue = decoded;
                maxValue = decoded;
                hasSample = true;
            }

            lastValue = decoded;
            lastR1 = r1;
            lastR2 = r2;
            lastRaw = raw;
            successReads++;

            delay(250);
        }

        if (successReads == 0) {
            candidate.score -= 300;
            continue;
        }

        float span = maxValue - minValue;
        candidate.firstWord = lastR1;
        candidate.secondWord = lastR2;
        candidate.value = lastValue;
        candidate.u32 = decodeUint32(lastR1, lastR2, candidate.order);

        if (rawChanged) {
            candidate.score += 70;
            if (span >= 0.01f && span <= 10000.0f) candidate.score += 25;
        } else {
            candidate.score -= 20;
            if (isSuspiciousConstantValue(lastValue)) candidate.score -= 45;
        }

        if (successReads >= 3) candidate.score += 15;
        if (candidate.confidence <= 0) candidate.score -= 20;
    }

    sortCandidatesByScore(candidates, count);
}

void printCandidateList(const AutoSelection* candidates, uint8_t count) {
    uint8_t shown = count < 3 ? count : 3;
    for (uint8_t i = 0; i < shown; i++) {
        SerialMon.printf("[AUTO] Candidate %u: R[%05u-%05u] %s %s  F:%9.3f  Score:%d\n",
                         i + 1,
                         candidates[i].reg,
                         candidates[i].reg + 1,
                         registerKindLabel(candidates[i].regType),
                         floatOrderLabel(candidates[i].order),
                         candidates[i].value,
                         candidates[i].score);
    }
}

void watchSingleRegister(const ScanResult& device, uint16_t reg, RegisterKind regType) {
    MBEngine::configureUART(device.baud, device.serialConfig);
    node.begin(device.id, RS485_SERIAL);

    clearSerialInput();
    SerialMon.println("[AUTO] Continuous watch on moving register. Press any key to stop.");
    while (!SerialMon.available()) {
        uint8_t res = readRegisters(regType, reg, 1);
        if (res == node.ku8MBSuccess) {
            uint16_t value = node.getResponseBuffer(0);
            SerialMon.printf("[AUTO] R[%05u] %s  HEX:%04X  U16:%5u  I16:%6d\n",
                             reg,
                             registerKindLabel(regType),
                             value,
                             value,
                             (int16_t)value);
        } else {
            SerialMon.printf("[AUTO] R[%05u] read error 0x%02X\n", reg, res);
        }
        delay(500);
    }
}

void watchTwoRegisters(const ScanResult& device,
                       uint16_t regA,
                       RegisterKind typeA,
                       uint16_t regB,
                       RegisterKind typeB) {
    MBEngine::configureUART(device.baud, device.serialConfig);
    node.begin(device.id, RS485_SERIAL);

    clearSerialInput();
    SerialMon.println("[AUTO] Continuous watch on two moving registers. Press any key to stop.");
    while (!SerialMon.available()) {
        uint8_t resA = readRegisters(typeA, regA, 1);
        if (resA == node.ku8MBSuccess) {
            uint16_t valueA = node.getResponseBuffer(0);
            SerialMon.printf("[AUTO] R[%05u] %s  HEX:%04X  U16:%5u  I16:%6d",
                             regA,
                             registerKindLabel(typeA),
                             valueA,
                             valueA,
                             (int16_t)valueA);
        } else {
            SerialMon.printf("[AUTO] R[%05u] read error 0x%02X",
                             regA,
                             resA);
        }

        uint8_t resB = readRegisters(typeB, regB, 1);
        if (resB == node.ku8MBSuccess) {
            uint16_t valueB = node.getResponseBuffer(0);
            SerialMon.printf("  |  R[%05u] %s  HEX:%04X  U16:%5u  I16:%6d\n",
                             regB,
                             registerKindLabel(typeB),
                             valueB,
                             valueB,
                             (int16_t)valueB);
        } else {
            SerialMon.printf("  |  R[%05u] read error 0x%02X\n",
                             regB,
                             resB);
        }

        delay(500);
    }
}

void scoreCandidatesFromActivity(const AutoSnapshot& snapshot, const RegisterActivity* activity, AutoSelection* candidates, uint8_t count) {
    for (uint8_t i = 0; i < count; i++) {
        AutoSelection& candidate = candidates[i];
        uint16_t idx = candidate.reg - snapshot.startReg;
        if (idx + 1 >= snapshot.count) {
            candidate.score -= 200;
            continue;
        }

        uint32_t strengthA = activityStrength(activity[idx]);
        uint32_t strengthB = activityStrength(activity[idx + 1]);
        uint32_t combined = strengthA + strengthB;

        if (combined == 0) {
            candidate.score -= 160;
        } else {
            candidate.score += (int)min((uint32_t)240, combined / 4U);
            if (activity[idx].changes > 0 && activity[idx + 1].changes > 0) {
                candidate.score += 40;
            }
        }

        uint16_t r1 = snapshot.values[idx];
        uint16_t r2 = snapshot.values[idx + 1];
        candidate.firstWord = r1;
        candidate.secondWord = r2;
        candidate.value = decodeFloat(r1, r2, candidate.order);
        candidate.u32 = decodeUint32(r1, r2, candidate.order);

        if (isSuspiciousConstantValue(candidate.value) && combined == 0) {
            candidate.score -= 40;
        }
    }

    sortCandidatesByScore(candidates, count);
}

void watchAutoSelection(const ScanResult& device, const AutoSelection& selection) {
    MBEngine::configureUART(device.baud, device.serialConfig);
    node.begin(device.id, RS485_SERIAL);

    while (SerialMon.available()) SerialMon.read();

    SerialMon.println("[AUTO] Continuous watch. Press any key to stop.");
    while (!SerialMon.available()) {
        uint8_t res = readRegisters(selection.regType, selection.reg, 2);
        if (res == node.ku8MBSuccess) {
            uint16_t r1 = node.getResponseBuffer(0);
            uint16_t r2 = node.getResponseBuffer(1);
            float decoded = decodeFloat(r1, r2, selection.order);
            uint32_t u32 = decodeUint32(r1, r2, selection.order);
            SerialMon.printf("[AUTO] R[%05u] %s %s  RAW:%04X %04X  F:%9.3f  U32:%10lu\n",
                             selection.reg,
                             registerKindLabel(selection.regType),
                             floatOrderLabel(selection.order),
                             r1,
                             r2,
                             decoded,
                             u32);
        } else {
            SerialMon.printf("[AUTO] R[%05u] read error 0x%02X\n", selection.reg, res);
        }
        delay(1000);
    }
}

namespace MBEngine {
    // Internal helper to print a human-readable Modbus error
    void printModbusError(uint8_t res) {
        SerialMon.printf("Error: 0x%02X ", res);
        if (res == 0x01) {
            SerialMon.print("(Illegal Function - try different function or sensor map)");
        } else if (res == 0x02) {
            SerialMon.print("(Invalid Address - Try smaller range or startReg)");
        } else if (res == 0x03) {
            SerialMon.print("(Illegal Data Value)");
        } else if (res == 0xE2) {
            SerialMon.print("(Response Timeout - no reply from slave)");
        }
        SerialMon.println();
    }

    void init() {
        // RS485 direction control pin is managed by UART hardware via TXDEN
        pinMode(RS485_DE_RE, OUTPUT);
        digitalWrite(RS485_DE_RE, LOW);
    }

    void configureUART(uint32_t baud, uint32_t config) {
        RS485_SERIAL.end();
        RS485_SERIAL.begin(baud, config, RS485_RX_PIN, RS485_TX_PIN);

        // Configure Waveshare RS485 driver: TXDEN pin and half-duplex mode
        if (!RS485_SERIAL.setPins(-1, -1, -1, RS485_DE_RE)) {
            SerialMon.println("[RS485] Failed to set TXDEN pin");
        }
        if (!RS485_SERIAL.setMode(UART_MODE_RS485_HALF_DUPLEX)) {
            SerialMon.println("[RS485] Failed to set RS485 half-duplex mode");
        }
    }

    void scanNetwork(uint8_t startID, uint8_t endID) {
        ScanResult result;
        SerialMon.println("\n[SCAN] Searching device...");
        if (!findFirstDevice(startID, endID, "[SCAN]", result) || !result.found) {
            SerialMon.println("[SCAN] No device found.");
            return;
        }

        SerialMon.printf("[FOUND] ID:%d  Baud:%u  Serial:%s  Map:%s  Probe:%u\n",
                         result.id,
                         result.baud,
                         serialConfigLabel(result.serialConfig),
                         registerKindLabel(result.regType),
                         result.probeReg);
        SerialMon.println("[NEXT] Use: dump <id> <baud> <reg> <n> or auto");
    }

    void dumpRegisters(uint8_t id, uint32_t baud, uint16_t startReg, uint16_t count) {
        if (count > 60) count = 60; // Limite protocollo per evitare errori 0x02/0x03
        configureUART(baud, SERIAL_8N1); // BOA default
        node.begin(id, RS485_SERIAL);
        uint8_t res = node.readHoldingRegisters(startReg, count);
        SerialMon.printf("\n--- DUMP ID:%d (%u baud) ---\n", id, baud);
        if (res == node.ku8MBSuccess) {
            for (int i = 0; i < count; i++) {
                uint16_t val = node.getResponseBuffer(i);
                SerialMon.printf("Reg[%05d]: 0x%04X | Dec: %u\n", startReg + i, val, val);
            }
        } else {
            printModbusError(res);
        }
    }

    void analyzeRegister(uint8_t id, uint32_t baud, uint16_t reg) {
        configureUART(baud, SERIAL_8N1);
        node.begin(id, RS485_SERIAL);
        uint8_t res = node.readHoldingRegisters(reg, 2);
        if (res == node.ku8MBSuccess) {
            uint16_t r1 = node.getResponseBuffer(0), r2 = node.getResponseBuffer(1);
            SerialMon.printf("\n--- ANALYSIS (ID:%d Reg:%d) ---", id, reg);
            SerialMon.printf("\nRAW: %04X %04X", r1, r2);
            SerialMon.printf("\nABCD: %f", decodeFloat(r1, r2, OrderABCD));
            SerialMon.printf("\nBADC: %f", decodeFloat(r1, r2, OrderBADC));
            SerialMon.printf("\nCDAB: %f", decodeFloat(r1, r2, OrderCDAB));
            SerialMon.printf("\nDCBA: %f\n", decodeFloat(r1, r2, OrderDCBA));
        } else SerialMon.printf("\nError: 0x%02X\n", res);
    }

    void watchRegister(uint8_t id, uint32_t baud, uint16_t reg) {
        configureUART(baud, SERIAL_8N1);
        node.begin(id, RS485_SERIAL);
        while(SerialMon.available()) SerialMon.read();
        SerialMon.println("\n[WATCH] Press any key to stop.");
        while (!SerialMon.available()) {
            uint8_t res = node.readHoldingRegisters(reg, 2);
            if (res == node.ku8MBSuccess) {
                uint16_t r1 = node.getResponseBuffer(0), r2 = node.getResponseBuffer(1);
                uint32_t i32 = ((uint32_t)r1 << 16) | r2;
                float f = rawToFloat((r2 & 0xFF), (r2 >> 8) & 0xFF, (r1 & 0xFF), (r1 >> 8) & 0xFF);
                SerialMon.printf("R[%05d]  %04X %04X  F:%8.3f  I32:%10lu\n", reg, r1, r2, f, i32);
            } else {
                SerialMon.printf("R[%05d]  read error 0x%02X\n", reg, res);
            }
            delay(1000);
        }
    }

    void autoDiscoverAndWatch(uint8_t startID, uint8_t endID) {
        ScanResult device;
        AutoSnapshot snapshot;
        AutoSelection candidates[kMaxAutoCandidates];
        RegisterActivity activity[AutoSnapshot::kMaxRegs];

        SerialMon.println("\n[AUTO] Guided scan + dump + stimulus detect + watch");
        if (!findFirstDevice(startID, endID, "[AUTO]", device) || !device.found) {
            SerialMon.println("[AUTO] No device found.");
            return;
        }

        SerialMon.printf("[AUTO] Device ID:%u  Baud:%u  Serial:%s  Map:%s  Probe:%u\n",
                         device.id,
                         device.baud,
                         serialConfigLabel(device.serialConfig),
                         registerKindLabel(device.regType),
                         device.probeReg);

        if (!captureSnapshot(device, snapshot, 0, AutoSnapshot::kMaxRegs)) {
            SerialMon.println("[AUTO] Could not read a useful register window.");
            return;
        }

        printSnapshotSummary(snapshot);
        initActivity(snapshot, activity);

        if (!runStimulusWindow(device, snapshot, activity, 20000)) {
            SerialMon.println("[AUTO] Stimulus phase aborted.");
            return;
        }

        SerialMon.println("[AUTO] Stimulus summary:");
        printTopActiveRegisters(snapshot, activity);

        int topActiveIndexA = -1;
        int topActiveIndexB = -1;
        findTopActiveRegisterIndices(snapshot, activity, topActiveIndexA, topActiveIndexB);

        if (topActiveIndexA >= 0 && registerSignalScore(activity[topActiveIndexA]) > -100000) {
            uint16_t regA = snapshot.startReg + (uint16_t)topActiveIndexA;
            if (topActiveIndexB >= 0 && registerSignalScore(activity[topActiveIndexB]) > -100000) {
                uint16_t regB = snapshot.startReg + (uint16_t)topActiveIndexB;
                RegisterKind typeA = activity[topActiveIndexA].source;
                RegisterKind typeB = activity[topActiveIndexB].source;
                uint16_t printFirstA = activity[topActiveIndexA].firstValue;
                uint16_t printLastA = activity[topActiveIndexA].lastValue;
                uint16_t printSpanA = activity[topActiveIndexA].maxValue - activity[topActiveIndexA].minValue;
                uint16_t printChangesA = activity[topActiveIndexA].changes;
                uint16_t printFirstB = activity[topActiveIndexB].firstValue;
                uint16_t printLastB = activity[topActiveIndexB].lastValue;
                uint16_t printSpanB = activity[topActiveIndexB].maxValue - activity[topActiveIndexB].minValue;
                uint16_t printChangesB = activity[topActiveIndexB].changes;

                if (regB < regA) {
                    uint16_t tmpReg = regA;
                    regA = regB;
                    regB = tmpReg;

                    RegisterKind tmpType = typeA;
                    typeA = typeB;
                    typeB = tmpType;

                    uint16_t tmpFirst = printFirstA;
                    uint16_t tmpLast = printLastA;
                    uint16_t tmpSpan = printSpanA;
                    uint16_t tmpChanges = printChangesA;
                    printFirstA = printFirstB;
                    printLastA = printLastB;
                    printSpanA = printSpanB;
                    printChangesA = printChangesB;
                    printFirstB = tmpFirst;
                    printLastB = tmpLast;
                    printSpanB = tmpSpan;
                    printChangesB = tmpChanges;
                }

                SerialMon.printf("[AUTO] Selected moving register A R[%05u] %s  first:%u  last:%u  span:%u  changes:%u\n",
                                 regA,
                                 registerKindLabel(typeA),
                                 printFirstA,
                                 printLastA,
                                 printSpanA,
                                 printChangesA);
                SerialMon.printf("[AUTO] Selected moving register B R[%05u] %s  first:%u  last:%u  span:%u  changes:%u\n",
                                 regB,
                                 registerKindLabel(typeB),
                                 printFirstB,
                                 printLastB,
                                 printSpanB,
                                 printChangesB);
                watchTwoRegisters(device,
                                  regA,
                                  typeA,
                                  regB,
                                  typeB);
            } else {
                SerialMon.printf("[AUTO] Selected moving register A R[%05u] %s  first:%u  last:%u  span:%u  changes:%u\n",
                                 regA,
                                 registerKindLabel(activity[topActiveIndexA].source),
                                 activity[topActiveIndexA].firstValue,
                                 activity[topActiveIndexA].lastValue,
                                 activity[topActiveIndexA].maxValue - activity[topActiveIndexA].minValue,
                                 activity[topActiveIndexA].changes);
                watchSingleRegister(device, regA, activity[topActiveIndexA].source);
            }
            return;
        }

        uint8_t candidateCount = collectTopCandidates(snapshot, candidates, kMaxAutoCandidates);
        if (candidateCount == 0 || !candidates[0].valid) {
            SerialMon.println("[AUTO] No reliable 32-bit candidate found.");
            return;
        }

        SerialMon.println("[AUTO] Ranking candidates from live movement...");
        scoreCandidatesFromActivity(snapshot, activity, candidates, candidateCount);
        printCandidateList(candidates, candidateCount);

        AutoSelection selection = candidates[0];
        SerialMon.printf("[AUTO] Selected R[%05u-%05u] %s %s  RAW:%04X %04X  F:%9.3f  Score:%d\n",
                         selection.reg,
                         selection.reg + 1,
                         registerKindLabel(selection.regType),
                         floatOrderLabel(selection.order),
                         selection.firstWord,
                         selection.secondWord,
                         selection.value,
                         selection.score);

        watchAutoSelection(device, selection);
    }
}