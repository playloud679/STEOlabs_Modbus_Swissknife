# STEOlab Modbus Swissknife

Firmware CLI per ESP32-S3 dedicato al reverse engineering di sensori Modbus RTU su RS485.

Il tool gira su seriale USB a `115200` baud e permette di:
- trovare automaticamente device, baud rate e formato seriale
- fare dump raw dei registri
- analizzare valori a 32 bit con diversi byte order
- osservare in tempo reale i registri che reagiscono a una perturbazione del sensore

## Hardware target

Configurazione attuale:
- board: `esp32-s3-devkitc-1`
- framework: `arduino`
- monitor seriale: `115200`
- upload via `esptool`

Pin RS485 attesi dal firmware:
- RX: `GPIO18`
- TX: `GPIO17`
- DE/RE: `GPIO21`

## Build e flash

Da `STEOlabs_ModbusTool/`:

```bash
pio run -e esp32s3 -t upload
```

Monitor seriale:

```bash
pio device monitor -b 115200
```

## Comandi disponibili

```text
scan <start> <end>
auto <start> <end>
dump <id> <baud> <reg> <n>
analyze <id> <baud> <reg>
watch <id> <baud> <reg>
help
```

## Workflow consigliato

### 1. Scansione rapida

```text
swissknife> scan 1 15
```

Il comando prova le combinazioni seriali supportate e riporta:
- ID del device
- baud rate
- formato seriale
- mappa registri piu` probabile

### 2. Auto-detection guidata

```text
swissknife> auto 1 15
```

Il comando `auto` esegue questa pipeline:
1. scan del bus
2. baseline dump iniziale dei registri leggibili
3. attesa conferma dell'utente
4. finestra di stimolazione del sensore
5. dump continuo delle variazioni osservate
6. ranking dei registri piu` significativi
7. watch finale dei registri selezionati

L'uso corretto e`:
1. lancia `auto`
2. quando compare `Press ENTER when ready to stimulate the sensor.`, premi `ENTER`
3. quando compare `Stimulate NOW for 20 seconds.`, applica una perturbazione reale al sensore

Questo approccio serve a capire quali registri rappresentano davvero la misura utile, invece di fidarsi solo di euristiche statiche.

### 3. Dump manuale

```text
swissknife> dump 1 9600 0 20
```

Utile per ispezionare blocchi di registri in formato raw:
- indice registro
- valore HEX
- valore decimale

### 4. Analisi dei 32 bit

```text
swissknife> analyze 1 9600 22
```

Mostra la stessa coppia di registri interpretata come:
- `ABCD`
- `BADC`
- `CDAB`
- `DCBA`

### 5. Watch manuale

```text
swissknife> watch 1 9600 4
```

Per monitorare manualmente un registro gia` noto.

## Note pratiche

- Alcuni sensori espongono la stessa informazione in forme diverse: raw positiva, signed, wrapped vicino a `0xFFFF` o come word di una coppia 32-bit.
- Il ranking automatico e` stato adattato per privilegiare registri che:
  - stanno vicini a zero a riposo
  - crescono quando il sensore viene perturbato
  - non sembrano solo una versione wrapped del segnale
- Se il comportamento del sensore non e` lineare, usa `dump` e `watch` per validare manualmente il risultato di `auto`.

## Stato attuale

Dalle prove fatte durante lo sviluppo:
- il device risponde su `ID 1`
- il baud rate rilevato e` `9600`
- il formato seriale rilevato e` `8N1`

## Licenza

Codice e documentazione STEOlab.