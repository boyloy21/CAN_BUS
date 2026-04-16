# 📡 CAN BUS Communication with STM32

<p align="center">
  <img src="https://img.shields.io/badge/STM32-HAL_Library-blue?style=for-the-badge&logo=stmicroelectronics&logoColor=white"/>
  <img src="https://img.shields.io/badge/Protocol-CAN_BUS-orange?style=for-the-badge"/>
  <img src="https://img.shields.io/badge/Language-C-lightgrey?style=for-the-badge&logo=c&logoColor=white"/>
  <img src="https://img.shields.io/badge/IDE-STM32CubeIDE-green?style=for-the-badge"/>
  <img src="https://img.shields.io/badge/Baud_Rate-up_to_1Mbps-red?style=for-the-badge"/>
  <img src="https://img.shields.io/badge/License-MIT-yellow?style=for-the-badge"/>
</p>

<p align="center">
  A complete implementation of <strong>CAN Bus (Controller Area Network)</strong> communication using the STM32 microcontroller and HAL library — covering transmit, receive, filtering, interrupts, and multi-node network examples.
</p>

---

## 📋 Table of Contents

- [What is CAN Bus?](#-what-is-can-bus)
- [Repository Structure](#-repository-structure)
- [Hardware Requirements](#-hardware-requirements)
- [CAN Bus Frame Format](#-can-bus-frame-format)
- [STM32 CAN Peripheral Overview](#-stm32-can-peripheral-overview)
- [STM32CubeMX Configuration](#️-stm32cubemx-configuration)
- [Transmitting Messages (TX)](#-transmitting-messages-tx)
- [Receiving Messages (RX)](#-receiving-messages-rx)
- [CAN Filters](#-can-filters)
- [Interrupt-Based Reception](#-interrupt-based-reception)
- [Multi-Node Network Example](#-multi-node-network-example)
- [Bit Timing & Baud Rate Calculator](#-bit-timing--baud-rate-calculator)
- [Wiring Diagram](#-wiring-diagram)
- [Troubleshooting](#-troubleshooting)
- [Getting Started](#-getting-started)
- [Contributing](#-contributing)
- [License](#-license)

---

## 🔍 What is CAN Bus?

**CAN (Controller Area Network)** is a robust, high-speed serial communication protocol originally developed by **Bosch** in 1986 for automotive applications. It allows multiple microcontrollers and devices to communicate with each other over a single two-wire bus — **without a central host computer**.

### Why CAN Bus?

| Feature | Description |
|---------|-------------|
| 🔒 **Noise Immunity** | Differential signaling (CANH/CANL) rejects electrical interference |
| 🕐 **Real-Time** | Prioritized message arbitration ensures critical messages arrive first |
| 🌐 **Multi-Node** | Up to 127 nodes on a single bus |
| 🔁 **Fault Tolerant** | Built-in error detection (CRC, ACK, bit stuffing) |
| ⚡ **Speed** | Up to **1 Mbps** at short distances |
| 🚗 **Industry Standard** | Used in automotive (OBD-II), industrial automation, robotics |

### CAN Bus Applications
- Automotive ECU networks (engine, ABS, airbag)
- Industrial automation & PLCs
- Robotics and multi-motor drive systems
- Medical devices
- Aerospace systems

---

## 📁 Repository Structure

```
CAN_BUS/
│
├── CAN_Transmit/               # Basic CAN message transmission (polling)
│   ├── Core/
│   │   ├── Src/
│   │   │   └── main.c
│   │   └── Inc/
│   └── CAN_Transmit.ioc
│
├── CAN_Receive/                # Basic CAN message reception (polling)
│   ├── Core/
│   │   ├── Src/
│   │   │   └── main.c
│   │   └── Inc/
│   └── CAN_Receive.ioc
│
├── CAN_Interrupt/              # CAN RX via interrupt (FIFO0/FIFO1)
│   ├── Core/
│   │   ├── Src/
│   │   │   └── main.c
│   │   └── Inc/
│   └── CAN_Interrupt.ioc
│
├── CAN_Filter/                 # CAN hardware filter configuration
│   ├── Core/
│   │   ├── Src/
│   │   │   └── main.c
│   │   └── Inc/
│   └── CAN_Filter.ioc
│
├── CAN_MultiNode/              # Multi-node CAN network simulation
│   ├── Node_Master/
│   └── Node_Slave/
│
└── README.md
```

---

## 🔧 Hardware Requirements

### Microcontroller
- **STM32F4xx** series (e.g., STM32F401, STM32F407) — recommended
- **STM32F103xx** (Blue Pill) — also supported
- Any STM32 with built-in **bxCAN** (Basic Extended CAN) peripheral

### Components

| Component | Description | Example Part |
|-----------|-------------|--------------|
| STM32 Development Board | MCU with CAN peripheral | STM32F407 Discovery, Nucleo-F446RE |
| CAN Transceiver | Converts TTL ↔ differential CAN signal | TJA1050, MCP2551, SN65HVD230 |
| Termination Resistors | 120Ω at each end of the bus | 120Ω, 0.25W |
| Twisted Pair Wire | CANH / CANL differential lines | CAT5/CAT6 or standard twisted pair |
| ST-Link V2 | Programmer/Debugger | ST-Link V2 |

### CAN Transceiver Selection

| Transceiver | Logic Level | Notes |
|-------------|-------------|-------|
| TJA1050     | 5V          | Most common, reliable |
| MCP2551     | 5V          | Popular for STM32F103 |
| SN65HVD230  | 3.3V        | Best for 3.3V STM32 (direct connect) |
| TJA1051     | 5V          | Low-power version of TJA1050 |

> ⚠️ **Note:** STM32 GPIO is 3.3V. If using a 5V transceiver, ensure CAN RX is 5V-tolerant or add a voltage divider on the RX line.

---

## 🗂 CAN Bus Frame Format

CAN uses two frame types: **Standard (11-bit ID)** and **Extended (29-bit ID)**.

### Standard Data Frame (11-bit ID)

```
 ┌────┬─────────┬─────┬─────┬────────┬──────────────┬─────┬────┐
 │ SOF│  ID[10:0]│ RTR │ IDE │  DLC   │  Data[0..7]  │ CRC │ ACK│
 │ 1b │  11 bits │ 1b  │ 1b  │ 4 bits │  0–8 bytes   │16b  │ 2b │
 └────┴─────────┴─────┴─────┴────────┴──────────────┴─────┴────┘
```

| Field | Size | Description |
|-------|------|-------------|
| **SOF** | 1 bit | Start of Frame — marks beginning |
| **ID** | 11 bits | Message identifier (priority: lower = higher priority) |
| **RTR** | 1 bit | Remote Transmission Request (0 = data, 1 = request) |
| **IDE** | 1 bit | Identifier Extension (0 = standard, 1 = extended) |
| **DLC** | 4 bits | Data Length Code (0–8 bytes) |
| **Data** | 0–64 bits | Payload (up to 8 bytes) |
| **CRC** | 15 bits | Cyclic Redundancy Check for error detection |
| **ACK** | 2 bits | Acknowledgement slot |

### Extended Data Frame (29-bit ID)
Used when more than 2,048 unique message IDs are needed (e.g., CANopen, J1939).

```
 ┌────┬────────────────┬─────┬─────┬────────┬──────────────┬─────┬────┐
 │ SOF│  ID[28:0]      │ RTR │ IDE │  DLC   │  Data[0..7]  │ CRC │ ACK│
 │ 1b │   29 bits      │ 1b  │ 1b  │ 4 bits │  0–8 bytes   │16b  │ 2b │
 └────┴────────────────┴─────┴─────┴────────┴──────────────┴─────┴────┘
```

---

## 🧩 STM32 CAN Peripheral Overview

STM32 uses **bxCAN (Basic Extended CAN)** controller which supports:
- CAN 2.0A (Standard 11-bit ID)
- CAN 2.0B (Extended 29-bit ID)
- Up to **1 Mbps** baud rate
- 3 **Transmit Mailboxes**
- 2 **Receive FIFOs** (FIFO0 & FIFO1) with 3 levels each
- 28 configurable **Filter Banks**

### CAN Pins (STM32F4xx)

| Function | Alternate Pins | Default |
|----------|----------------|---------|
| CAN1_TX  | PA12, PB9, PD1  | PA12 |
| CAN1_RX  | PA11, PB8, PD0  | PA11 |
| CAN2_TX  | PB6, PB13       | PB13 |
| CAN2_RX  | PB5, PB12       | PB12 |

---

## ⚙️ STM32CubeMX Configuration

### Step-by-Step CAN Setup

1. Open **STM32CubeMX** and select your MCU
2. Navigate to **Connectivity → CAN1**
3. Set **Mode** to `Activated`
4. Configure **Bit Timing Parameters:**

```
For 1 Mbps @ 42 MHz APB1 clock (STM32F407):
  Prescaler        = 3
  Time Quanta in Bit Segment 1 (BS1) = 11
  Time Quanta in Bit Segment 2 (BS2) = 2
  → Baud Rate = 42,000,000 / (3 × (1 + 11 + 2)) = 1,000,000 bps ✅

For 500 kbps @ 42 MHz APB1:
  Prescaler = 6
  BS1 = 11
  BS2 = 2
  → Baud Rate = 42,000,000 / (6 × 14) = 500,000 bps ✅
```

5. Enable **CAN global interrupt** under NVIC settings (for interrupt mode)
6. Generate code

---

## 📤 Transmitting Messages (TX)

### Initialize CAN

```c
/* Start CAN peripheral */
HAL_CAN_Start(&hcan1);
```

### Send a Standard (11-bit ID) Frame

```c
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint32_t TxMailbox;

void CAN_SendMessage(uint16_t id, uint8_t *data, uint8_t length) {
    TxHeader.StdId              = id;          // 11-bit Message ID
    TxHeader.ExtId              = 0x00;
    TxHeader.RTR                = CAN_RTR_DATA;
    TxHeader.IDE                = CAN_ID_STD;  // Standard frame
    TxHeader.DLC                = length;       // 1 to 8 bytes
    TxHeader.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox) != HAL_OK) {
        Error_Handler(); // TX mailbox full or bus error
    }
}
```

### Example: Send Sensor Data

```c
uint8_t txData[4];
float temperature = 36.5f;

// Pack float into 4 bytes
memcpy(txData, &temperature, sizeof(float));

// Send on CAN with ID 0x101
CAN_SendMessage(0x101, txData, 4);
```

### Send an Extended (29-bit ID) Frame

```c
TxHeader.ExtId = 0x1ABCDEF0;   // 29-bit Extended ID
TxHeader.IDE   = CAN_ID_EXT;   // Extended frame
TxHeader.DLC   = 8;
```

---

## 📥 Receiving Messages (RX)

### Polling Mode

```c
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

void CAN_ReceivePolling(void) {
    if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0) {
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
            // Process received message
            uint16_t receivedID  = RxHeader.StdId;
            uint8_t  dataLength  = RxHeader.DLC;

            // Handle data based on ID
            switch (receivedID) {
                case 0x100:
                    // Handle message 0x100
                    break;
                case 0x200:
                    // Handle message 0x200
                    break;
                default:
                    break;
            }
        }
    }
}
```

---

## 🔎 CAN Filters

CAN filters allow the hardware to **automatically discard unwanted messages** before they reach the CPU — reducing software overhead significantly.

### Filter Bank Modes

| Mode | Description |
|------|-------------|
| **Mask Mode** | Accept messages where `(ID & mask) == filter_id` |
| **List Mode** | Accept only exact IDs listed in filter registers |

### Example: Accept Only ID 0x100–0x1FF (Mask Mode)

```c
CAN_FilterTypeDef canfilterconfig;

canfilterconfig.FilterActivation     = CAN_FILTER_ENABLE;
canfilterconfig.FilterBank           = 0;
canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
canfilterconfig.FilterIdHigh         = 0x0100 << 5;  // Filter ID (shifted left 5)
canfilterconfig.FilterIdLow          = 0x0000;
canfilterconfig.FilterMaskIdHigh     = 0x0700 << 5;  // Mask: accept 0x1xx range
canfilterconfig.FilterMaskIdLow      = 0x0000;
canfilterconfig.FilterMode           = CAN_FILTERMODE_IDMASK;
canfilterconfig.FilterScale          = CAN_FILTERSCALE_32BIT;
canfilterconfig.SlaveStartFilterBank = 14;

HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
```

### Example: Accept Only Specific IDs (List Mode)

```c
// Accept ONLY ID 0x101 and ID 0x201
canfilterconfig.FilterIdHigh     = 0x0101 << 5;
canfilterconfig.FilterIdLow      = 0x0201 << 5;
canfilterconfig.FilterMode       = CAN_FILTERMODE_IDLIST;
canfilterconfig.FilterScale      = CAN_FILTERSCALE_32BIT;
```

### Accept All Messages (Pass-Through Filter)

```c
canfilterconfig.FilterIdHigh     = 0x0000;
canfilterconfig.FilterIdLow      = 0x0000;
canfilterconfig.FilterMaskIdHigh = 0x0000;   // 0 = don't care all bits
canfilterconfig.FilterMaskIdLow  = 0x0000;
canfilterconfig.FilterMode       = CAN_FILTERMODE_IDMASK;
```

---

## ⚡ Interrupt-Based Reception

Using interrupts is the **recommended** approach for real-time CAN communication.

### Enable RX Interrupt

```c
/* After HAL_CAN_Start(), activate FIFO0 message pending interrupt */
HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
```

### Callback Function

```c
/* This is called automatically when a message arrives in FIFO0 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {

        // ---- Process message based on ID ----
        if (RxHeader.StdId == 0x100) {
            // Parse 4-byte float from bytes 0–3
            float value;
            memcpy(&value, RxData, sizeof(float));
            // Use value...
        }

        if (RxHeader.StdId == 0x200) {
            uint16_t speed = (RxData[0] << 8) | RxData[1];
            uint8_t  dir   = RxData[2];
            // Motor command received...
        }
    }
}
```

### Error Notification (Optional but Recommended)

```c
/* Enable bus error and bus-off notifications */
HAL_CAN_ActivateNotification(&hcan1,
    CAN_IT_ERROR_WARNING |
    CAN_IT_ERROR_PASSIVE |
    CAN_IT_BUSOFF        |
    CAN_IT_LAST_ERROR_CODE
);

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    uint32_t error = HAL_CAN_GetError(hcan);
    // Log or handle the CAN error
}
```

---

## 🌐 Multi-Node Network Example

This example shows **two STM32 boards communicating** over a shared CAN bus — one as Master (sender) and one as Slave (receiver/responder).

### Network Topology

```
  ┌──────────────┐       CANH ●━━━━━━━━━━━━━━━━━━━● CANH ┌──────────────┐
  │  STM32       │                                        │  STM32       │
  │  Node A      │       CANL ●━━━━━━━━━━━━━━━━━━━● CANL │  Node B      │
  │  (Master)    │                                        │  (Slave)     │
  └──────┬───────┘                                        └──────┬───────┘
         │ TJA1050                                    TJA1050    │
         └───────────── 120Ω ─────────────────── 120Ω ──────────┘
                     termination                termination
```

### Node A — Master (Sender)

```c
// main.c — Node A sends a command every 100ms
while (1) {
    uint8_t cmd[3] = {0x01, 0xFF, 0x01}; // cmd=SET_SPEED, value=255, dir=FWD
    CAN_SendMessage(0x100, cmd, 3);
    HAL_Delay(100);
}
```

### Node B — Slave (Receiver & Responder)

```c
// main.c — Node B receives command and sends ACK
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);

    if (RxHeader.StdId == 0x100) {
        // Execute command
        uint8_t speed     = RxData[1];
        uint8_t direction = RxData[2];
        Motor_SetSpeed(speed, direction);

        // Send ACK back to Master
        uint8_t ack[2] = {0xAC, 0x01}; // ACK OK
        CAN_SendMessage(0x101, ack, 2);
    }
}
```

### Suggested Message ID Map

| ID (Hex) | Direction | Description |
|----------|-----------|-------------|
| `0x100`  | Master → All | Set motor speed/direction |
| `0x101`  | Slave → Master | Motor command ACK |
| `0x200`  | Master → All | Read sensor request |
| `0x201`  | Slave → Master | Sensor data response |
| `0x7FF`  | Master → All | Emergency STOP |

---

## 📐 Bit Timing & Baud Rate Calculator

The CAN baud rate is determined by:

```
Baud Rate = f_APB1 / (Prescaler × (1 + BS1 + BS2))
```

### Common Configurations

| Baud Rate | APB1 Clock | Prescaler | BS1 | BS2 | Sample Point |
|-----------|-----------|-----------|-----|-----|--------------|
| 1 Mbps    | 42 MHz    | 3         | 11  | 2   | 85.7%        |
| 500 kbps  | 42 MHz    | 6         | 11  | 2   | 85.7%        |
| 250 kbps  | 42 MHz    | 12        | 11  | 2   | 85.7%        |
| 125 kbps  | 42 MHz    | 24        | 11  | 2   | 85.7%        |
| 1 Mbps    | 36 MHz    | 2         | 15  | 2   | 88.9%        |
| 500 kbps  | 36 MHz    | 4         | 15  | 2   | 88.9%        |

> 💡 **Tip:** All nodes on the same CAN bus **must have the same baud rate**. Use an online [CAN Bit Timing Calculator](http://www.bittiming.can-wiki.info/) for custom configurations.

---

## 🔌 Wiring Diagram

### Single Node (STM32 + TJA1050)

```
STM32 Board              TJA1050 Transceiver        CAN Bus
───────────              ───────────────────        ───────
PA12 (CAN_TX) ──────────→ TXD
PA11 (CAN_RX) ←────────── RXD
3.3V ─────────────────────→ Vcc (with 5V regulator)
GND  ─────────────────────→ GND
                             CANH ─────────────────→ CANH line ──── 120Ω ────┐
                             CANL ─────────────────→ CANL line               │
                                                                   120Ω ────┘
```

### Two-Node CAN Network

```
Node A (Master)                         Node B (Slave)
─────────────────                       ─────────────────
STM32 → TJA1050                         STM32 → TJA1050
         │  CANH ──────────────────────── CANH │
         │  CANL ──────────────────────── CANL │
120Ω ──┘                                       └── 120Ω
(at bus end)                             (at bus end)
```

> ⚠️ Always place **120Ω termination resistors** at **both physical ends** of the CAN bus. Missing or wrong termination is the most common source of CAN errors.

---

## 🛠 Troubleshooting

### Common Issues & Fixes

| Problem | Possible Cause | Solution |
|---------|---------------|----------|
| No messages received | Missing 120Ω resistors | Add 120Ω at both bus ends |
| TX always returns HAL_ERROR | CAN peripheral not started | Call `HAL_CAN_Start()` before TX |
| Bus-off error | Wiring issue or baud mismatch | Check wiring, verify baud rate matches on all nodes |
| Messages filtered out | Filter too restrictive | Use pass-through filter first to debug |
| Interrupt never fires | Notification not activated | Call `HAL_CAN_ActivateNotification()` after Start |
| Garbage data received | Baud rate mismatch | Verify all nodes use identical baud rate |
| CAN TX stuck in PASSIVE mode | No other node to acknowledge | Add at least 2 nodes — CAN requires ACK |

### CAN Error States

```
           Normal
           ──────
           ↕  (128 errors)
        Warning
           ↕  (128 errors)
         Passive   ← Node listens but sends passive error flags
           ↕  (256 errors)
         Bus-Off   ← Node disconnects from bus
```

To recover from Bus-Off:
```c
HAL_CAN_Stop(&hcan1);
HAL_Delay(100);
HAL_CAN_Start(&hcan1);
```

---

## 🚀 Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/boyloy21/CAN_BUS.git
cd CAN_BUS
```

### 2. Open in STM32CubeIDE

1. Launch **STM32CubeIDE**
2. Go to `File → Open Projects from File System`
3. Browse to the project folder (e.g., `CAN_Transmit/`)
4. Click **Finish**

### 3. Configure for Your Board

- Open the `.ioc` file in **STM32CubeMX**
- Verify CAN pin assignments match your hardware
- Adjust **Prescaler / BS1 / BS2** for your APB1 clock frequency
- Regenerate code if changes are made

### 4. Wire Hardware

- Connect STM32 CAN_TX/RX to CAN transceiver
- Connect transceiver CANH/CANL to the bus
- Add **120Ω resistors** at both ends

### 5. Build and Flash

1. Click **Build** (🔨)
2. Connect ST-Link debugger
3. Click **Run / Debug** to flash

---

## 🤝 Contributing

Contributions are welcome! Here's how:

1. **Fork** the repository
2. Create a branch: `git checkout -b feature/your-feature`
3. Commit your changes: `git commit -m "Add: description"`
4. Push: `git push origin feature/your-feature`
5. Open a **Pull Request**

### Ideas for Contributions
- CANopen protocol implementation
- DBC file parser for message decoding
- FreeRTOS-based CAN task example
- CAN bus logger via UART
- STM32G4 FDCAN (CAN FD) implementation

---

## 📚 References

- [STM32 bxCAN Reference Manual (RM0090)](https://www.st.com/resource/en/reference_manual/rm0090-stm32f405415-stm32f407417-stm32f427437-and-stm32f429439-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [CAN Bus Specification 2.0 — Bosch](https://www.kvaser.com/can-protocol-tutorial/)
- [STM32CubeF4 HAL Driver Documentation](https://www.st.com/en/embedded-software/stm32cubef4.html)
- [CAN Bit Timing Calculator](http://www.bittiming.can-wiki.info/)

---

## 📄 License

This project is licensed under the **MIT License** — free to use, modify, and distribute.

---

## 👤 Author

**Yin Chheanyun**  
GitHub: [@boyloy21](https://github.com/boyloy21)

---

<p align="center">
  ⭐ If this helped your project, please give it a <strong>star</strong> on GitHub!
</p>
