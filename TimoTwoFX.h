/**
 * TimoTwoFX - Arduino Library for the LumenRadio TimoTwo FX Module
 *
 * Full-featured SPI driver for the TimoTwo FX CRMX/W-DMX wireless module.
 * Supports all hardware features:
 *  - DMX transmit (TX mode) with internal frame generation
 *  - DMX receive (RX mode) via callback or direct read
 *  - RDM Responder (RX) and RDM Controller/Master (TX, requires paid option)
 *  - Linking / connection management
 *  - Bluetooth Low Energy (BLE) configuration interface
 *  - RF protocol selection: CRMX, W-DMX G3, W-DMX G4S
 *
 * SPI Specifications:
 *  - Max clock speed : 2 MHz
 *  - Bit order       : MSB first, Big-Endian
 *  - SPI Mode        : 0 (CPOL=0, CPHA=0)
 *  - CS setup time   : 4 µs after CS goes LOW
 *
 * Wiring (Arduino Uno/Nano example):
 *  TimoTwo FX   Arduino
 *  VDD       -> 3.3V  (!) use level shifter for 5V boards
 *  GND       -> GND
 *  CS        -> Pin 10 (or any digital pin)
 *  IRQ       -> Pin 2  (or any digital pin)
 *  SCK       -> Pin 13 (SPI SCK)
 *  MOSI      -> Pin 11 (SPI MOSI)
 *  MISO      -> Pin 12 (SPI MISO)
 *
 * License: MIT
 * Based on the LumenRadio TimoTwo documentation and official SPI example code.
 */

#ifndef TIMOTWOFX_H
#define TIMOTWOFX_H

#include <Arduino.h>
#include <SPI.h>

// ============================================================
// SPI Command Bytes
// ============================================================
#define TIMO_CMD_READ_REG(addr)      ((uint8_t)((addr) & 0x3F))
#define TIMO_CMD_WRITE_REG(addr)     ((uint8_t)(0x40 | ((addr) & 0x3F)))
#define TIMO_CMD_READ_DMX            0x81   ///< Read DMX frame from module
#define TIMO_CMD_READ_ASC            0x82   ///< Read alternate start code frame
#define TIMO_CMD_READ_RDM            0x83   ///< Read RDM packet from module
#define TIMO_CMD_READ_RXTX           0x84   ///< Read RX/TX status data
#define TIMO_CMD_WRITE_DMX           0x91   ///< Write DMX frame to module
#define TIMO_CMD_WRITE_RDM           0x92   ///< Write RDM packet to module
#define TIMO_CMD_WRITE_RXTX          0x93   ///< Write RX/TX control data
#define TIMO_CMD_RADIO_DISCOVERY     0xA0   ///< Start radio (RF) discovery (TX, RDM TX option required)
#define TIMO_CMD_RADIO_DISC_RESULT   0xA1   ///< Read radio discovery result
#define TIMO_CMD_RADIO_MUTE          0xA2   ///< Mute a discovered device
#define TIMO_CMD_RADIO_MUTE_RESULT   0xA3   ///< Read mute result
#define TIMO_CMD_RDM_DISCOVERY       0xA4   ///< Start RDM discovery (TX, RDM TX option required)
#define TIMO_CMD_RDM_DISC_RESULT     0xA5   ///< Read RDM discovery result
#define TIMO_CMD_NODE_QUERY          0xA6   ///< Query a node
#define TIMO_CMD_NODE_QUERY_RESPONSE 0xA7   ///< Read node query response
#define TIMO_CMD_NOP                 0xFF   ///< No-operation: returns IRQ flags instantly (1-phase)

// ============================================================
// Register Addresses
// ============================================================
#define TIMO_REG_CONFIG              0x00   ///< Module configuration (mode, radio on/off, ...)
#define TIMO_REG_STATUS              0x01   ///< Link status, identify, DMX present
#define TIMO_REG_IRQ_MASK            0x02   ///< Enable/disable specific IRQ sources
#define TIMO_REG_IRQ_FLAGS           0x03   ///< Pending interrupt flags (cleared on read)
#define TIMO_REG_DMX_WINDOW          0x04   ///< DMX receive window: start address and length
#define TIMO_REG_ASC_FRAME           0x05   ///< Alternate start code frame configuration
#define TIMO_REG_LINK_QUALITY        0x06   ///< RF signal quality (RX only), 0–255
#define TIMO_REG_DMX_SPEC            0x08   ///< DMX frame spec: channel count and refresh rate
#define TIMO_REG_DMX_CONTROL         0x09   ///< Enable internal DMX generation (TX)
#define TIMO_REG_EXT_IRQ_MASK        0x0A   ///< Enable/disable extended IRQ sources
#define TIMO_REG_EXT_IRQ_FLAGS       0x0B   ///< Extended interrupt flags
#define TIMO_REG_RF_PROTOCOL         0x0C   ///< RF protocol selection (CRMX / W-DMX G3 / G4S)
#define TIMO_REG_DMX_SOURCE          0x0D   ///< Active DMX source (UART / SPI / wireless / BLE)
#define TIMO_REG_LOLLIPOP            0x0E   ///< Reboot detection counter (see getLollipop())
#define TIMO_REG_VERSION             0x10   ///< Hardware + firmware version (8 bytes)
#define TIMO_REG_RF_POWER            0x11   ///< RF transmit power level
#define TIMO_REG_BLOCKED_CHANNELS    0x12   ///< Blocked RF channel bitmap
#define TIMO_REG_BINDING_UID         0x20   ///< RDM binding UID of the host device (6 bytes)
#define TIMO_REG_LINKING_KEY         0x21   ///< Linking key (8 bytes TX, 10 bytes RX)
#define TIMO_REG_BLE_STATUS          0x30   ///< BLE enable/disable and connection status
#define TIMO_REG_BLE_PIN             0x31   ///< BLE pairing PIN code
#define TIMO_REG_BATTERY             0x32   ///< Battery level for BLE display (0–100, 255=N/A)
#define TIMO_REG_UNIVERSE_COLOR      0x33   ///< Universe color tag (RGB, 3 bytes)
#define TIMO_REG_OEM_INFO            0x34   ///< OEM manufacturer + device model IDs
#define TIMO_REG_RXTX_STATUS         0x35   ///< RX/TX status register
#define TIMO_REG_DEVICE_NAME         0x36   ///< Device name string (max 32 bytes, zero padded)
#define TIMO_REG_UNIVERSE_NAME       0x37   ///< Universe name string
#define TIMO_REG_INSTALLED_OPTIONS   0x3D   ///< Installed software options (e.g. RDM TX)
#define TIMO_REG_PRODUCT_ID          0x3F   ///< Product identification

// ============================================================
// CONFIG Register Bits (register 0x00)
// ============================================================
#define TIMO_CONFIG_UART_EN          (1 << 0)  ///< Enable UART DMX output
#define TIMO_CONFIG_TX_MODE          (1 << 1)  ///< 0 = RX mode, 1 = TX mode
#define TIMO_CONFIG_SPI_RDM          (1 << 3)  ///< Route RDM traffic over SPI instead of UART
#define TIMO_CONFIG_RADIO_EN         (1 << 7)  ///< Enable RF radio

// ============================================================
// STATUS Register Bits (register 0x01)
// ============================================================
#define TIMO_STATUS_LINKED           (1 << 0)  ///< Module is linked to a TX/RX partner
#define TIMO_STATUS_RF_LINK          (1 << 1)  ///< Active RF (radio) link established
#define TIMO_STATUS_IDENTIFY         (1 << 2)  ///< Identify mode is active
#define TIMO_STATUS_DMX              (1 << 3)  ///< Valid DMX signal is being received
#define TIMO_STATUS_UPDATE_MODE      (1 << 7)  ///< Module is in firmware update mode

// ============================================================
// IRQ Bits (IRQ_FLAGS / IRQ_MASK, register 0x02 / 0x03)
// ============================================================
#define TIMO_IRQ_RX_DMX              (1 << 0)  ///< New DMX frame received (RX)
#define TIMO_IRQ_LOST_DMX            (1 << 1)  ///< DMX signal lost (RX)
#define TIMO_IRQ_DMX_CHANGED         (1 << 2)  ///< One or more DMX values changed (RX)
#define TIMO_IRQ_RF_LINK             (1 << 3)  ///< RF link status changed
#define TIMO_IRQ_ASC                 (1 << 4)  ///< Alternate start code frame received
#define TIMO_IRQ_IDENTIFY            (1 << 5)  ///< Identify command received/changed
#define TIMO_IRQ_EXTENDED            (1 << 6)  ///< Extended IRQ pending (check EXT_IRQ_FLAGS)
#define TIMO_IRQ_DEVICE_BUSY         (1 << 7)  ///< Module is busy, retry the SPI command

// ============================================================
// Extended IRQ Bits (byte 3 of EXT_IRQ_FLAGS, register 0x0B)
// ============================================================
#define TIMO_EXTIRQ_RDM              (1 << 0)  ///< RDM packet available
#define TIMO_EXTIRQ_RXTX_DA          (1 << 1)  ///< RX/TX data available
#define TIMO_EXTIRQ_RXTX_CTS         (1 << 2)  ///< RX/TX clear to send
#define TIMO_EXTIRQ_RADIO_DISC       (1 << 3)  ///< Radio discovery result available
#define TIMO_EXTIRQ_RADIO_MUTE       (1 << 4)  ///< Radio mute result available
#define TIMO_EXTIRQ_RDM_DISC         (1 << 5)  ///< RDM discovery result available
#define TIMO_EXTIRQ_UNIV_META        (1 << 6)  ///< Universe metadata changed (name/color)
#define TIMO_EXTIRQ_NODE_QUERY       (1 << 7)  ///< Node query response available

// ============================================================
// DMX_SOURCE Values (register 0x0D)
// ============================================================
#define TIMO_SRC_NONE                0   ///< No active DMX source
#define TIMO_SRC_UART                1   ///< DMX sourced from UART input
#define TIMO_SRC_WIRELESS            2   ///< DMX sourced from wireless (RF) link
#define TIMO_SRC_SPI                 3   ///< DMX sourced from SPI (this library)
#define TIMO_SRC_BLE                 4   ///< DMX sourced from BLE connection

// ============================================================
// RF Protocol Constants (register 0x0C)
// ============================================================
#define TIMO_RF_CRMX                 0   ///< CRMX protocol (LumenRadio proprietary, recommended)
#define TIMO_RF_WDMX_G3              1   ///< W-DMX Generation 3 (City Theatrical)
#define TIMO_RF_WDMX_G4S             3   ///< W-DMX Generation 4S (City Theatrical)

// ============================================================
// RF Transmit Power Constants (register 0x11)
// ============================================================
#define TIMO_POWER_20DBM             2   ///< 20 dBm = 100 mW (maximum, default)
#define TIMO_POWER_16DBM             3   ///< 16 dBm =  40 mW
#define TIMO_POWER_11DBM             4   ///< 11 dBm =  13 mW
#define TIMO_POWER_5DBM              5   ///<  5 dBm =   3 mW (minimum)

// ============================================================
// Internal Constants
// ============================================================
#define TIMO_IRQ_TIMEOUT_MS          100      ///< Default IRQ wait timeout in milliseconds
#define TIMO_SPI_CLOCK               2000000UL ///< SPI clock: 2 MHz (hardware maximum)
#define TIMO_RDM_MAX_PACKET_SIZE     257      ///< Maximum RDM packet size in bytes (per E1.20)

// ============================================================
// Enums
// ============================================================

/** Operating mode of the module. Set by beginTX() or beginRX(). */
enum TimoMode {
    TIMO_MODE_RX = 0,   ///< Receive mode: listens for DMX/RDM from a TX transmitter
    TIMO_MODE_TX = 1    ///< Transmit mode: sends DMX/RDM to linked RX receivers
};

/** Return code for library functions. Always check the result of beginTX() / beginRX(). */
enum TimoResult {
    TIMO_OK = 0,                ///< Operation succeeded
    TIMO_ERROR_TIMEOUT,         ///< IRQ wait timed out (wiring/power issue?)
    TIMO_ERROR_BUSY,            ///< Module did not accept the command after retries
    TIMO_ERROR_NO_DATA,         ///< No data available to read (e.g. empty RDM buffer)
    TIMO_ERROR_INVALID_PARAM    ///< A parameter was out of the valid range
};

// ============================================================
// Callback Type Definitions
// ============================================================

/**
 * Called when a new DMX frame has been received (RX mode only).
 * @param data        Pointer to received channel values (data[0] = channel at startAddress)
 * @param length      Number of bytes in data (equals the configured DMX window size)
 * @param startAddress Zero-based DMX start address of data[0]
 */
typedef void (*TimoDmxCallback)(const uint8_t* data, uint16_t length, uint16_t startAddress);

/**
 * Called when the DMX signal is lost (RX mode only).
 * No parameters – simply indicates signal dropout.
 */
typedef void (*TimoDmxLostCallback)();

/**
 * Called when the RF link status changes (both modes).
 * @param linked      true if logically linked to a partner device
 * @param hasRadioLink true if an active radio link is established
 */
typedef void (*TimoLinkCallback)(bool linked, bool hasRadioLink);

/**
 * Called when an RDM Identify command is received or changes state (RX mode).
 * @param active  true = identify is active (device should flash/beep), false = identify ended
 */
typedef void (*TimoIdentifyCallback)(bool active);

/**
 * Called when an RDM packet is available.
 * In RX mode: an RDM request from the TX controller is ready to be processed.
 * In TX mode: an RDM response from a connected device is available.
 * @param packet  Pointer to the raw RDM packet bytes
 * @param length  Packet length in bytes
 */
typedef void (*TimoRdmCallback)(const uint8_t* packet, uint16_t length);


// ============================================================
// TimoTwoFX Main Class
// ============================================================

class TimoTwoFX {
public:

    /**
     * Create a TimoTwoFX instance. Call beginTX() or beginRX() in setup() to initialize.
     *
     * @param csPin   Arduino digital pin connected to the module's CS (chip select) line.
     *                CS is active LOW. Any digital output pin can be used.
     * @param irqPin  Arduino digital pin connected to the module's IRQ line.
     *                IRQ is active LOW, open-drain. Any digital input pin can be used.
     *
     * Example:
     *   TimoTwoFX timo(10, 2);  // CS on pin 10, IRQ on pin 2
     */
    TimoTwoFX(int csPin, int irqPin);

    // ----------------------------------------------------------
    // Initialization
    // ----------------------------------------------------------

    /**
     * Initialize the module as a DMX transmitter (TX mode).
     *
     * Configures the module for wireless DMX transmission. The module generates
     * DMX frames internally at the specified rate and sends the data you write
     * via setChannel() / sendDmx() over the air to linked receivers.
     *
     * Writing to TIMO_REG_CONFIG causes the module to reboot (200 ms delay is applied
     * automatically). Call startLinking() afterwards to connect to receivers.
     *
     * @param channels   Number of DMX channels to transmit (1–512, default: 512).
     *                   Only the channels you actually use; fewer channels = lower latency.
     * @param refreshHz  DMX frame rate in Hz (1–44, default: 40).
     *                   Standard lighting is 40 Hz. Lower values save power.
     * @return TIMO_OK on success, TIMO_ERROR_TIMEOUT if the module does not respond.
     *
     * Example:
     *   timo.beginTX(512, 40);         // Full universe at 40 fps
     *   timo.beginTX(100, 30);         // First 100 channels at 30 fps
     */
    TimoResult beginTX(uint16_t channels = 512, uint8_t refreshHz = 40);

    /**
     * Initialize the module as a DMX receiver (RX mode).
     *
     * Configures the module to listen for wireless DMX from a linked transmitter.
     * Register callbacks (onDmxReceived, onLinkChanged, etc.) before or after calling
     * beginRX(), then call update() repeatedly in loop().
     *
     * @return TIMO_OK on success, TIMO_ERROR_TIMEOUT if the module does not respond.
     *
     * Example:
     *   timo.beginRX();
     *   timo.onDmxReceived(myCallback);
     */
    TimoResult beginRX();

    /**
     * Shut down the module and release the SPI bus.
     * Call this if you need to share the SPI bus with other devices and want
     * to cleanly release it. Re-initialize with beginTX() / beginRX() afterwards.
     */
    void end();

    // ----------------------------------------------------------
    // DMX Transmit (TX mode only)
    // ----------------------------------------------------------

    /**
     * Set a single DMX channel value in the internal TX buffer.
     * The value is not sent until you call sendDmx().
     *
     * @param channel  Channel number, 1-based (1–512).
     * @param value    DMX value 0–255 (0 = off, 255 = full).
     *
     * Example:
     *   timo.setChannel(1, 255);   // Channel 1 full
     *   timo.setChannel(2, 128);   // Channel 2 at 50%
     *   timo.sendDmx();
     */
    void setChannel(uint16_t channel, uint8_t value);

    /**
     * Set multiple consecutive DMX channels from an array.
     * More efficient than calling setChannel() in a loop.
     *
     * @param startChannel  First channel to write, 1-based (1–512).
     * @param data          Array of DMX values.
     * @param length        Number of bytes to copy from data[].
     *
     * Example:
     *   uint8_t rgb[3] = {255, 0, 128};
     *   timo.setChannels(1, rgb, 3);  // Set channels 1, 2, 3
     *   timo.sendDmx();
     */
    void setChannels(uint16_t startChannel, const uint8_t* data, uint16_t length);

    /**
     * Get a direct pointer to the internal 512-byte DMX transmit buffer.
     * Index 0 corresponds to DMX channel 1.
     * You can write values directly and then call sendDmx().
     *
     * Example:
     *   uint8_t* buf = timo.getDmxBuffer();
     *   memset(buf, 0, 512);    // Blackout all channels
     *   buf[0] = 200;           // Channel 1 = 200
     *   timo.sendDmx();
     */
    uint8_t* getDmxBuffer() { return _dmxBuffer; }

    /**
     * Send the entire internal DMX buffer to the module.
     * The module will transmit the data wirelessly in its next DMX frame.
     * Call this once per loop() iteration (or whenever data changes).
     *
     * @return TIMO_OK on success.
     *
     * Example:
     *   timo.setChannel(1, 200);
     *   timo.sendDmx();
     */
    TimoResult sendDmx();

    /**
     * Send only a partial range of the DMX buffer to the module.
     * More efficient than sendDmx() when only a few channels change.
     * The module still transmits the full DMX universe wirelessly —
     * only the SPI transfer is shorter.
     *
     * @param startChannel  First channel to send, 1-based (1–512).
     * @param length        Number of channels to send.
     * @return TIMO_OK on success, TIMO_ERROR_INVALID_PARAM if startChannel is out of range.
     *
     * Example:
     *   // Only update channels 1–3 (RGB fixture)
     *   timo.setChannels(1, rgbData, 3);
     *   timo.sendDmxWindow(1, 3);
     */
    TimoResult sendDmxWindow(uint16_t startChannel, uint16_t length);

    // ----------------------------------------------------------
    // DMX Receive (RX mode)
    // ----------------------------------------------------------

    /**
     * Process pending interrupts from the module. Must be called regularly in loop().
     *
     * Checks the IRQ pin. If an interrupt is pending, reads the IRQ flags register
     * and dispatches events: new DMX frame, signal lost, link changed, identify, RDM.
     * Registered callbacks are invoked from within this function.
     *
     * Call as fast as possible — ideally every loop() iteration with no blocking delays.
     *
     * Example:
     *   void loop() {
     *       timo.update();  // Always call this first
     *       // ... rest of your code
     *   }
     */
    void update();

    /**
     * Read received DMX data directly into a buffer (polling, without callback).
     * Use this if you prefer polling over callbacks, or need to read DMX
     * synchronously (e.g. in a state machine).
     *
     * This only returns useful data when isDmxAvailable() is true, or when called
     * from within the onDmxReceived callback.
     *
     * @param buffer       Destination buffer for DMX values.
     * @param bufferSize   Size of the destination buffer in bytes (max 512).
     * @param startAddress Zero-based start address within the DMX universe (default: 0).
     * @return Number of bytes written to buffer. 0 if no signal or SPI error.
     *
     * Example:
     *   uint8_t dmx[512];
     *   if (timo.isDmxAvailable()) {
     *       uint16_t len = timo.readDmx(dmx, sizeof(dmx));
     *       analogWrite(9, dmx[0]);  // Use channel 1
     *   }
     */
    uint16_t readDmx(uint8_t* buffer, uint16_t bufferSize, uint16_t startAddress = 0);

    /**
     * Check if a valid DMX signal is currently being received.
     * Reads the STATUS register. Does not consume an IRQ event.
     *
     * @return true if a DMX signal is active, false if there is no signal.
     *
     * Example:
     *   if (!timo.isDmxAvailable()) {
     *       Serial.println("No DMX signal!");
     *   }
     */
    bool isDmxAvailable();

    // ----------------------------------------------------------
    // Callback Registration
    // ----------------------------------------------------------

    /**
     * Register a callback for new DMX frames (RX mode).
     * The callback is called from within update() each time a new DMX frame arrives.
     * @param cb  Function pointer of type: void myFunc(const uint8_t* data, uint16_t length, uint16_t startAddr)
     *
     * Example:
     *   void onDmx(const uint8_t* data, uint16_t len, uint16_t start) {
     *       analogWrite(9, data[0]);
     *   }
     *   timo.onDmxReceived(onDmx);
     */
    void onDmxReceived(TimoDmxCallback cb)   { _dmxCallback = cb; }

    /**
     * Register a callback for DMX signal loss (RX mode).
     * Called when the wireless signal is interrupted or the transmitter stops.
     * @param cb  Function pointer of type: void myFunc()
     */
    void onDmxLost(TimoDmxLostCallback cb)   { _dmxLostCallback = cb; }

    /**
     * Register a callback for RF link status changes (both modes).
     * Called when linking succeeds, fails, or the radio link drops.
     * @param cb  Function pointer of type: void myFunc(bool linked, bool hasRadioLink)
     *
     * Example:
     *   void onLink(bool linked, bool rf) {
     *       digitalWrite(LED_BUILTIN, linked ? HIGH : LOW);
     *   }
     *   timo.onLinkChanged(onLink);
     */
    void onLinkChanged(TimoLinkCallback cb)  { _linkCallback = cb; }

    /**
     * Register a callback for RDM Identify commands (RX mode).
     * Called when the TX controller sends an IDENTIFY_DEVICE RDM command.
     * The device should visually indicate its location (e.g. flash an LED).
     * @param cb  Function pointer of type: void myFunc(bool active)
     */
    void onIdentify(TimoIdentifyCallback cb) { _identifyCallback = cb; }

    /**
     * Register a callback for incoming RDM packets.
     * In RX mode: called when an RDM request arrives (you must respond via writeRdm()).
     * In TX mode: called when an RDM response is received from a device.
     * @param cb  Function pointer of type: void myFunc(const uint8_t* packet, uint16_t length)
     */
    void onRdmReceived(TimoRdmCallback cb)   { _rdmCallback = cb; }

    // ----------------------------------------------------------
    // Linking / Connection Management
    // ----------------------------------------------------------

    /**
     * Start the linking procedure.
     *
     * TX mode: The transmitter broadcasts a linking request. Any receiver
     *          in linking mode nearby will automatically link to it.
     * RX mode: The receiver listens for a linking request from a transmitter.
     *
     * Linking status can be monitored via isLinked(), hasRfLink(), or the
     * onLinkChanged() callback.
     *
     * Example:
     *   timo.startLinking();
     *   delay(5000);  // Wait for devices to link
     *   if (timo.isLinked()) Serial.println("Linked!");
     */
    void startLinking();

    /**
     * Check if the module is logically linked to a partner device.
     * A logical link persists even if the radio link temporarily drops.
     * @return true if linked.
     */
    bool isLinked();

    /**
     * Check if an active RF (radio) link is established.
     * Returns false if the partner is out of range or powered off.
     * @return true if the radio link is active right now.
     */
    bool hasRfLink();

    /**
     * Disconnect from the current partner. The module returns to unlinked state.
     * Call startLinking() again to re-link to a new or the same device.
     */
    void unlink();

    /**
     * Set a fixed linking key for TX mode.
     * Only receivers with the same linking key can link to this transmitter.
     * Useful for pairing specific TX/RX pairs in environments with multiple systems.
     *
     * Writing the linking key causes the module to reboot (200 ms delay applied automatically).
     *
     * @param key  8-character numeric string, e.g. "12345678". Use digits 0–9 only.
     *             Pass "" or all 0xFF values to disable the key filter (any receiver can link).
     *
     * Example:
     *   timo.setLinkingKey("87654321");  // Only receivers with the same key can link
     */
    void setLinkingKey(const char* key);

    /**
     * Set a linking key and connect to a specific TX transmitter (RX mode).
     * Causes the module to reboot (200 ms delay applied automatically).
     *
     * @param key     8-character numeric key matching the TX's linking key.
     * @param mode    Linking mode: 0 = CRMX Classic, 1 = CRMX2.
     * @param output  Output number on the TX device to link to (0–7, default: 0).
     *
     * Example:
     *   timo.setLinkingKeyRX("87654321", 0, 0);  // Link to TX key 87654321, output 0
     */
    void setLinkingKeyRX(const char* key, uint8_t mode = 0, uint8_t output = 0);

    // ----------------------------------------------------------
    // Configuration
    // ----------------------------------------------------------

    /**
     * Select the RF protocol.
     * Must be set before calling startLinking(). TX and RX must use the same protocol.
     * Writing this register causes the module to reboot (200 ms delay applied automatically).
     *
     * @param protocol  One of: TIMO_RF_CRMX (default), TIMO_RF_WDMX_G3, TIMO_RF_WDMX_G4S
     *
     * Example:
     *   timo.setRfProtocol(TIMO_RF_CRMX);         // CRMX (recommended, best range)
     *   timo.setRfProtocol(TIMO_RF_WDMX_G3);      // W-DMX G3 (City Theatrical)
     */
    void setRfProtocol(uint8_t protocol);

    /**
     * Set the RF transmit power (TX mode only).
     * Lower power reduces range but extends battery life and reduces interference.
     * Default is TIMO_POWER_20DBM (maximum, 100 mW).
     *
     * @param power  One of: TIMO_POWER_20DBM (100mW), TIMO_POWER_16DBM (40mW),
     *                        TIMO_POWER_11DBM (13mW), TIMO_POWER_5DBM (3mW)
     *
     * Example:
     *   timo.setRfPower(TIMO_POWER_11DBM);  // Medium power, good for most stage setups
     */
    void setRfPower(uint8_t power);

    /**
     * Configure the DMX receive window.
     * The DMX window defines which part of the 512-channel universe is read via SPI
     * when a new frame arrives. Narrowing the window reduces SPI transfer time
     * and speeds up the onDmxReceived callback.
     *
     * Only affects SPI reads — the module still receives the full DMX universe wirelessly.
     *
     * @param startAddr  Zero-based first channel of the window (0–511).
     * @param length     Number of channels in the window (1–512).
     *
     * Example:
     *   // Only read channels 1–3 (RGB fixture at address 1)
     *   timo.setDmxWindow(0, 3);   // startAddr=0 means channel 1
     *
     *   // Read channels 101–200
     *   timo.setDmxWindow(100, 100);
     */
    void setDmxWindow(uint16_t startAddr, uint16_t length);

    /**
     * Enable or disable Bluetooth Low Energy (BLE).
     * When BLE is enabled, the module can be configured via the CRMX Toolbox app
     * on iOS/Android. Useful for live setups where you need to adjust settings wirelessly.
     *
     * WARNING: Changing this setting causes the module to reboot.
     *          A 200 ms delay is applied automatically, but do not call other SPI
     *          functions for at least 200 ms after this call.
     *
     * @param enabled  true = BLE on, false = BLE off (saves power).
     *
     * Example:
     *   timo.setBleEnabled(true);   // Enable BLE for app control
     *   timo.setBleEnabled(false);  // Disable BLE to save power
     */
    void setBleEnabled(bool enabled);

    /**
     * Set the device name shown in the CRMX Toolbox app and other RDM controllers.
     * The name helps identify individual devices in a complex setup.
     *
     * @param name  Name string, max 32 characters. Longer strings are truncated.
     *
     * Example:
     *   timo.setDeviceName("Stage Left Par");
     *   timo.setDeviceName("Controller-1");
     */
    void setDeviceName(const char* name);

    /**
     * Read the current device name from the module.
     * @param buf  Output buffer, must be at least 33 bytes (32 chars + null terminator).
     *
     * Example:
     *   char name[33];
     *   timo.getDeviceName(name);
     *   Serial.println(name);
     */
    void getDeviceName(char* buf);

    /**
     * Set OEM manufacturer and device model identifiers.
     * These IDs appear in the CRMX Toolbox app and are used for product identification.
     * Contact LumenRadio (sales@lumenrad.io) to register official OEM IDs.
     *
     * @param manufacturerId   Your registered manufacturer ID (16-bit).
     * @param deviceModelId    Your device model ID (16-bit).
     *
     * Example:
     *   timo.setOemInfo(0x00FF, 0x0001);  // Manufacturer 0x00FF, model 0x0001
     */
    void setOemInfo(uint16_t manufacturerId, uint16_t deviceModelId);

    /**
     * Set the universe color tag (TX mode).
     * The color is transmitted to all linked receivers and shown in the CRMX Toolbox app.
     * Use it to visually distinguish universes in multi-universe setups (e.g. red for
     * universe 1, blue for universe 2).
     *
     * @param r  Red component   (0–255)
     * @param g  Green component (0–255)
     * @param b  Blue component  (0–255)
     *
     * Example:
     *   timo.setUniverseColor(255, 0, 0);    // Red universe
     *   timo.setUniverseColor(0, 0, 255);    // Blue universe
     *   timo.setUniverseColor(0, 255, 0);    // Green universe
     */
    void setUniverseColor(uint8_t r, uint8_t g, uint8_t b);

    /**
     * Read the current universe color from the module.
     * In RX mode: returns the color set by the linked TX transmitter.
     *
     * @param r  Pointer to receive the red value.
     * @param g  Pointer to receive the green value.
     * @param b  Pointer to receive the blue value.
     *
     * Example:
     *   uint8_t r, g, b;
     *   timo.getUniverseColor(&r, &g, &b);
     *   Serial.printf("Color: #%02X%02X%02X\n", r, g, b);
     */
    void getUniverseColor(uint8_t* r, uint8_t* g, uint8_t* b);

    /**
     * Report the battery level of the host device to the BLE app.
     * This value is displayed in the CRMX Toolbox app when BLE is connected.
     * Update this regularly if your device has a battery.
     *
     * This function only sends the value to the module — it does not read any
     * hardware battery gauge. You must measure the battery voltage yourself
     * and convert it to a percentage before calling this function.
     *
     * @param percent  Battery level: 0–100 (percentage), or 255 if no battery present.
     *                 0   = empty / critical
     *                 50  = half charge
     *                 100 = full charge
     *                 255 = device has no battery (default, hides the indicator in the app)
     *
     * Example:
     *   // Read battery ADC, convert to 0–100%, then report:
     *   int raw = analogRead(A0);
     *   uint8_t pct = map(raw, 614, 818, 0, 100);   // Example: 3.0V–4.0V on 3.3V ADC
     *   pct = constrain(pct, 0, 100);
     *   timo.setBatteryLevel(pct);
     *
     *   // If your device has no battery:
     *   timo.setBatteryLevel(255);
     */
    void setBatteryLevel(uint8_t percent);

    /**
     * Activate or deactivate the Identify mode on this device (TX mode) or
     * read back identify status (RX mode).
     *
     * TX mode: Setting identify to true causes all linked receivers to flash/beep
     *          so you can find them on stage.
     * RX mode: The identify state is set by the TX controller via RDM. Reading
     *          this via isIdentifying() tells you if you should be blinking.
     *
     * @param active  true = activate identify, false = deactivate.
     *
     * Example (TX):
     *   timo.setIdentify(true);   // All linked RX devices flash
     *   delay(3000);
     *   timo.setIdentify(false);  // Stop flashing
     */
    void setIdentify(bool active);

    /**
     * Check if identify mode is currently active.
     * Useful in RX mode to implement visual feedback (e.g. flashing an LED).
     * @return true if identify is active.
     *
     * Example (RX):
     *   void loop() {
     *       timo.update();
     *       digitalWrite(LED_BUILTIN, timo.isIdentifying() ? HIGH : LOW);
     *   }
     */
    bool isIdentifying();

    // ----------------------------------------------------------
    // RDM (Remote Device Management)
    // ----------------------------------------------------------

    /**
     * Send an RDM packet to the module for wireless transmission.
     *
     * TX mode (RDM Controller): Send an RDM request to a device.
     *   Requires the "RDM TX via SPI" option from LumenRadio (see hasRdmTxOption()).
     * RX mode (RDM Responder): Send an RDM response after receiving a request
     *   via the onRdmReceived() callback.
     *
     * The function waits internally for the IRQ to confirm the packet was accepted.
     *
     * @param packet  Pointer to the raw RDM packet bytes (per E1.20 format).
     * @param length  Packet length in bytes (max TIMO_RDM_MAX_PACKET_SIZE = 257).
     * @return TIMO_OK on success, TIMO_ERROR_TIMEOUT if the module is unresponsive.
     */
    TimoResult writeRdm(const uint8_t* packet, uint16_t length);

    /**
     * Read an RDM packet from the module.
     *
     * TX mode: Read an RDM response from a device (after sending a request).
     * RX mode: Read an incoming RDM request (to process and respond to).
     *
     * Normally you don't need to call this directly — the onRdmReceived() callback
     * handles it automatically via update(). Use this for manual/polling RDM handling.
     *
     * @param buffer  Destination buffer. Must be at least TIMO_RDM_MAX_PACKET_SIZE (257) bytes.
     * @param length  Output: number of bytes written to buffer.
     * @return TIMO_OK on success, TIMO_ERROR_NO_DATA if no packet is available.
     */
    TimoResult readRdm(uint8_t* buffer, uint16_t& length);

    /**
     * Start a radio-layer RDM discovery scan (TX mode, RDM TX option required).
     * Sends a DUB (Discovery Unique Branch) packet over the air and collects responses.
     * After calling this, wait for the TIMO_EXTIRQ_RADIO_DISC interrupt (via update()),
     * then call readRadioDiscoveryResult() to get the UID of the discovered device.
     *
     * @param dubData  Pointer to the DUB packet payload.
     * @param length   Length of dubData in bytes.
     */
    TimoResult startRadioDiscovery(const uint8_t* dubData, uint16_t length);

    /**
     * Read the result of a radio discovery scan (TX mode, RDM TX option required).
     * Call after receiving the TIMO_EXTIRQ_RADIO_DISC interrupt.
     *
     * @param buffer  Destination buffer, at least 6 bytes (holds one RDM UID).
     * @param length  Output: number of bytes written (6 if a device was found, 0 otherwise).
     */
    TimoResult readRadioDiscoveryResult(uint8_t* buffer, uint16_t& length);

    /**
     * Start an RDM-layer discovery scan (TX mode, RDM TX option required).
     * @param dubData  DUB packet payload.
     * @param length   Length of dubData.
     */
    TimoResult startRdmDiscovery(const uint8_t* dubData, uint16_t length);

    /**
     * Read the result of an RDM discovery scan (TX mode, RDM TX option required).
     * @param buffer  Destination buffer, at least 24 bytes.
     * @param length  Output: number of bytes written.
     */
    TimoResult readRdmDiscoveryResult(uint8_t* buffer, uint16_t& length);

    // ----------------------------------------------------------
    // Status & Diagnostics
    // ----------------------------------------------------------

    /**
     * Read the hardware and firmware version numbers from the module.
     *
     * @param hwVersion  Output: hardware version as a 32-bit integer (major.minor.patch.build).
     * @param swVersion  Output: firmware (software) version as a 32-bit integer.
     *
     * Example:
     *   uint32_t hw, sw;
     *   timo.getVersion(hw, sw);
     *   Serial.printf("HW: %08X  FW: %08X\n", hw, sw);
     */
    void getVersion(uint32_t& hwVersion, uint32_t& swVersion);

    /**
     * Get the firmware version as a human-readable string.
     *
     * @param buf  Output buffer for the version string. Must be at least 20 bytes.
     *             The string format is "major.minor.patch.build", e.g. "1.0.7.2".
     *
     * Example:
     *   char ver[20];
     *   timo.getVersionString(ver);
     *   Serial.println(ver);   // e.g. "1.0.7.2"
     */
    void getVersionString(char* buf);

    /**
     * Get the RF link quality (RX mode only).
     * Indicates signal strength / quality of the received wireless signal.
     *
     * @return Signal quality: 0 (weakest) to 255 (strongest / 100%).
     *         A value of 0 usually means there is no active RF link.
     *
     * Example:
     *   uint8_t q = timo.getLinkQuality();
     *   Serial.printf("Signal quality: %d%%\n", (q * 100) / 255);
     */
    uint8_t getLinkQuality();

    /**
     * Get the currently active DMX source.
     * Useful for debugging: tells you where the module's DMX data is coming from.
     *
     * @return One of: TIMO_SRC_NONE (0), TIMO_SRC_UART (1), TIMO_SRC_WIRELESS (2),
     *                 TIMO_SRC_SPI (3), TIMO_SRC_BLE (4).
     *
     * Example:
     *   uint8_t src = timo.getDmxSource();
     *   if (src == TIMO_SRC_WIRELESS) Serial.println("Receiving DMX over RF");
     *   if (src == TIMO_SRC_NONE)     Serial.println("No DMX source active");
     */
    uint8_t getDmxSource();

    /**
     * Read the lollipop counter for reboot detection.
     * The module increments this counter on every reboot. Your firmware can compare
     * the value to a stored reference to detect if the module rebooted unexpectedly.
     *
     * A value < 128 typically indicates a fresh reboot since your last check.
     * Values wrap around at 255 (back to 0).
     *
     * Common causes of unexpected reboots: writing CONFIG / BLE / LINKING_KEY registers
     * (these always trigger a reboot — the 200 ms delay in this library accounts for that).
     *
     * @return The current lollipop counter value (0–255).
     *
     * Example:
     *   uint8_t lollipop = timo.getLollipop();
     *   if (lollipop < 128) Serial.println("Module rebooted!");
     */
    uint8_t getLollipop();

    /**
     * Set the RDM binding UID of the host Arduino/controller.
     * The binding UID identifies the Arduino as an RDM device to the CRMX network.
     * Required if you implement a full RDM responder.
     *
     * @param uid  6-byte array containing the RDM UID, MSB first.
     *             Format: [manufacturer MSB, manufacturer LSB, device MSB, device, device, device LSB]
     *
     * Example:
     *   uint8_t uid[6] = {0x00, 0xFF, 0x12, 0x34, 0x56, 0x78};
     *   timo.setBindingUid(uid);
     */
    void setBindingUid(const uint8_t uid[6]);

    /**
     * Check whether the RDM TX via SPI option is installed on this module.
     * This is a paid software option from LumenRadio required for full RDM Controller
     * functionality (sending RDM requests, discovery). Contact: sales@lumenrad.io
     *
     * @return true if the RDM TX option is activated, false otherwise.
     *
     * Example:
     *   if (!timo.hasRdmTxOption()) {
     *       Serial.println("RDM TX not available on this module.");
     *   }
     */
    bool hasRdmTxOption();

    /**
     * Read the current IRQ flags register via a fast NOP command.
     * Does not trigger a full 2-phase SPI transaction — only reads the flags byte
     * that the module returns as the response to the NOP command byte.
     * Useful for low-level debugging without disturbing the IRQ state.
     *
     * @return Bitmask of pending IRQ flags (TIMO_IRQ_* constants).
     */
    uint8_t readIrqFlags();

    /**
     * Read the extended IRQ flags (byte 3 of the EXT_IRQ_FLAGS register).
     * Extended flags cover events like RDM packet received, discovery result ready, etc.
     *
     * @return Bitmask of extended IRQ flags (TIMO_EXTIRQ_* constants).
     */
    uint8_t readExtIrqFlags();

    // ----------------------------------------------------------
    // Low-Level SPI Access (for advanced users / custom extensions)
    // ----------------------------------------------------------

    /**
     * Read one or more bytes from a module register over SPI.
     * Performs a full 2-phase SPI transaction with IRQ handshaking.
     *
     * @param reg     Register address (TIMO_REG_* constant).
     * @param data    Destination buffer.
     * @param length  Number of bytes to read.
     * @return TIMO_OK on success.
     */
    TimoResult readRegister(uint8_t reg, uint8_t* data, uint8_t length);

    /**
     * Write one or more bytes to a module register over SPI.
     * Performs a full 2-phase SPI transaction with IRQ handshaking.
     *
     * WARNING: Writing TIMO_REG_CONFIG, TIMO_REG_BLE_STATUS, TIMO_REG_LINKING_KEY,
     *          and TIMO_REG_RF_PROTOCOL will cause the module to reboot.
     *          Wait at least 200 ms after writing these registers.
     *
     * @param reg     Register address (TIMO_REG_* constant).
     * @param data    Source buffer.
     * @param length  Number of bytes to write.
     * @return TIMO_OK on success.
     */
    TimoResult writeRegister(uint8_t reg, const uint8_t* data, uint8_t length);

    /**
     * Execute a raw SPI command with optional TX and RX payload.
     * Handles the full 2-phase protocol including IRQ handshaking and busy retry.
     *
     * @param cmd      Command byte (TIMO_CMD_* constant).
     * @param txData   Data to send in phase 2 (can be nullptr for read-only commands).
     * @param rxData   Buffer for received data (can be nullptr for write-only commands).
     * @param length   Number of payload bytes to transfer.
     * @return TIMO_OK on success, TIMO_ERROR_TIMEOUT or TIMO_ERROR_BUSY on failure.
     */
    TimoResult spiCommand(uint8_t cmd, const uint8_t* txData, uint8_t* rxData, uint16_t length);

private:
    int      _csPin;
    int      _irqPin;
    TimoMode _mode;
    uint16_t _numChannels;
    uint16_t _dmxWindowStart;
    uint16_t _dmxWindowSize;

    uint8_t  _dmxBuffer[512];
    uint8_t  _rxDmxBuffer[512];

    TimoDmxCallback      _dmxCallback;
    TimoDmxLostCallback  _dmxLostCallback;
    TimoLinkCallback     _linkCallback;
    TimoIdentifyCallback _identifyCallback;
    TimoRdmCallback      _rdmCallback;

    bool _lastLinked;
    bool _lastRfLink;
    bool _lastIdentify;

    bool    _waitForIrqLow(uint32_t timeoutMs = TIMO_IRQ_TIMEOUT_MS);
    bool    _waitForIrqHigh(uint32_t timeoutMs = TIMO_IRQ_TIMEOUT_MS);
    void    _csLow();
    void    _csHigh();
    uint8_t _spiTransfer(uint8_t data);
    uint8_t _readIrqFlagsDirect();
    void    _processIrqs();
    TimoResult _configureDmxSpec(uint16_t channels, uint8_t refreshHz);
};

#endif // TIMOTWOFX_H
