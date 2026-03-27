/**
 * TimoTwoFX.cpp – Implementation of the TimoTwoFX Arduino Library
 *
 * License: MIT
 * See TimoTwoFX.h for full documentation of all public methods.
 */

#include "TimoTwoFX.h"

// ============================================================
// Constructor
// ============================================================

TimoTwoFX::TimoTwoFX(int csPin, int irqPin)
    : _csPin(csPin), _irqPin(irqPin),
      _mode(TIMO_MODE_RX), _numChannels(512),
      _dmxWindowStart(0), _dmxWindowSize(512),
      _dmxCallback(nullptr), _dmxLostCallback(nullptr),
      _linkCallback(nullptr), _identifyCallback(nullptr),
      _rdmCallback(nullptr),
      _lastLinked(false), _lastRfLink(false), _lastIdentify(false)
{
    memset(_dmxBuffer,   0, sizeof(_dmxBuffer));
    memset(_rxDmxBuffer, 0, sizeof(_rxDmxBuffer));
}

// ============================================================
// Initialization
// ============================================================

TimoResult TimoTwoFX::beginTX(uint16_t channels, uint8_t refreshHz) {
    _mode = TIMO_MODE_TX;
    _numChannels = constrain(channels, 1, 512);

    pinMode(_csPin,  OUTPUT);
    pinMode(_irqPin, INPUT);
    _csHigh();

    SPI.begin();

    // Configure module for TX:  RADIO_EN | TX_MODE | SPI_RDM  (no UART output)
    uint8_t config = TIMO_CONFIG_RADIO_EN | TIMO_CONFIG_TX_MODE | TIMO_CONFIG_SPI_RDM;
    TimoResult r = writeRegister(TIMO_REG_CONFIG, &config, 1);
    if (r != TIMO_OK) return r;

    // Writing CONFIG with a mode change triggers a module reboot — wait for it
    delay(200);

    // Configure DMX frame parameters (channel count, refresh rate)
    r = _configureDmxSpec(_numChannels, refreshHz);
    if (r != TIMO_OK) return r;

    // Enable internal DMX frame generation (the module generates DMX frames autonomously)
    uint8_t ctrl = 0x01;
    r = writeRegister(TIMO_REG_DMX_CONTROL, &ctrl, 1);
    if (r != TIMO_OK) return r;

    // Enable extended IRQs for RDM and discovery events
    uint8_t extMask[4] = {0, 0, 0, TIMO_EXTIRQ_RDM | TIMO_EXTIRQ_RADIO_DISC | TIMO_EXTIRQ_RDM_DISC};
    r = writeRegister(TIMO_REG_EXT_IRQ_MASK, extMask, 4);
    if (r != TIMO_OK) return r;

    // Enable IRQs: RF link change, identify change, extended events
    uint8_t irqMask = TIMO_IRQ_RF_LINK | TIMO_IRQ_IDENTIFY | TIMO_IRQ_EXTENDED;
    r = writeRegister(TIMO_REG_IRQ_MASK, &irqMask, 1);

    return r;
}

TimoResult TimoTwoFX::beginRX() {
    _mode = TIMO_MODE_RX;

    pinMode(_csPin,  OUTPUT);
    pinMode(_irqPin, INPUT);
    _csHigh();

    SPI.begin();

    // Configure module for RX: RADIO_EN | SPI_RDM  (no TX_MODE, no UART output)
    uint8_t config = TIMO_CONFIG_RADIO_EN | TIMO_CONFIG_SPI_RDM;
    TimoResult r = writeRegister(TIMO_REG_CONFIG, &config, 1);
    if (r != TIMO_OK) return r;

    delay(200);  // Wait for module reboot after CONFIG write

    // Default: receive the full 512-channel DMX universe
    setDmxWindow(0, 512);

    // Enable extended IRQ for RDM packets
    uint8_t extMask[4] = {0, 0, 0, TIMO_EXTIRQ_RDM};
    r = writeRegister(TIMO_REG_EXT_IRQ_MASK, extMask, 4);
    if (r != TIMO_OK) return r;

    // Enable IRQs: new DMX frame, DMX lost, RF link change, identify, extended
    uint8_t irqMask = TIMO_IRQ_RX_DMX | TIMO_IRQ_LOST_DMX |
                      TIMO_IRQ_RF_LINK | TIMO_IRQ_IDENTIFY | TIMO_IRQ_EXTENDED;
    r = writeRegister(TIMO_REG_IRQ_MASK, &irqMask, 1);

    return r;
}

void TimoTwoFX::end() {
    SPI.end();
}

// ============================================================
// DMX Transmit
// ============================================================

void TimoTwoFX::setChannel(uint16_t channel, uint8_t value) {
    if (channel >= 1 && channel <= 512) {
        _dmxBuffer[channel - 1] = value;
    }
}

void TimoTwoFX::setChannels(uint16_t startChannel, const uint8_t* data, uint16_t length) {
    if (startChannel < 1) return;
    uint16_t idx = startChannel - 1;
    for (uint16_t i = 0; i < length && idx < 512; i++, idx++) {
        _dmxBuffer[idx] = data[i];
    }
}

TimoResult TimoTwoFX::sendDmx() {
    return sendDmxWindow(1, _numChannels);
}

TimoResult TimoTwoFX::sendDmxWindow(uint16_t startChannel, uint16_t length) {
    if (startChannel < 1 || startChannel > 512) return TIMO_ERROR_INVALID_PARAM;
    uint16_t idx       = startChannel - 1;
    uint16_t actualLen = min(length, (uint16_t)(512 - idx));
    return spiCommand(TIMO_CMD_WRITE_DMX, &_dmxBuffer[idx], nullptr, actualLen);
}

// ============================================================
// DMX Receive
// ============================================================

void TimoTwoFX::update() {
    // Fast check: if IRQ pin is HIGH there is nothing to process
    if (digitalRead(_irqPin) == HIGH) return;

    _processIrqs();
}

void TimoTwoFX::_processIrqs() {
    uint8_t flags = _readIrqFlagsDirect();

    // Module is busy — back off and wait
    if (flags & TIMO_IRQ_DEVICE_BUSY) {
        delay(1);
        return;
    }

    // ---- RF link status changed ----
    if (flags & TIMO_IRQ_RF_LINK) {
        uint8_t status;
        readRegister(TIMO_REG_STATUS, &status, 1);
        bool linked   = (status & TIMO_STATUS_LINKED)   != 0;
        bool rfLink   = (status & TIMO_STATUS_RF_LINK)  != 0;
        bool identify = (status & TIMO_STATUS_IDENTIFY) != 0;

        if ((linked != _lastLinked || rfLink != _lastRfLink) && _linkCallback) {
            _linkCallback(linked, rfLink);
        }
        _lastLinked = linked;
        _lastRfLink = rfLink;

        if (identify != _lastIdentify && _identifyCallback) {
            _identifyCallback(identify);
        }
        _lastIdentify = identify;
    }

    // ---- New DMX frame received ----
    if (flags & TIMO_IRQ_RX_DMX) {
        uint16_t len = readDmx(_rxDmxBuffer, sizeof(_rxDmxBuffer), _dmxWindowStart);
        if (len > 0 && _dmxCallback) {
            _dmxCallback(_rxDmxBuffer, len, _dmxWindowStart);
        }
    }

    // ---- DMX signal lost ----
    if (flags & TIMO_IRQ_LOST_DMX) {
        uint8_t status;
        readRegister(TIMO_REG_STATUS, &status, 1);  // Reading STATUS clears this IRQ
        if (_dmxLostCallback) {
            _dmxLostCallback();
        }
    }

    // ---- Identify state changed ----
    if (flags & TIMO_IRQ_IDENTIFY) {
        uint8_t status;
        readRegister(TIMO_REG_STATUS, &status, 1);
        bool identify = (status & TIMO_STATUS_IDENTIFY) != 0;
        if (identify != _lastIdentify && _identifyCallback) {
            _identifyCallback(identify);
        }
        _lastIdentify = identify;
    }

    // ---- Extended IRQ (RDM, discovery, ...) ----
    if (flags & TIMO_IRQ_EXTENDED) {
        uint8_t extFlags[4];
        readRegister(TIMO_REG_EXT_IRQ_FLAGS, extFlags, 4);
        uint8_t ext = extFlags[3];

        if (ext & TIMO_EXTIRQ_RDM) {
            if (_rdmCallback) {
                uint8_t rdmBuf[TIMO_RDM_MAX_PACKET_SIZE];
                uint16_t rdmLen = 0;
                if (readRdm(rdmBuf, rdmLen) == TIMO_OK) {
                    _rdmCallback(rdmBuf, rdmLen);
                }
            }
        }
    }
}

uint16_t TimoTwoFX::readDmx(uint8_t* buffer, uint16_t bufferSize, uint16_t startAddress) {
    uint16_t len = min(bufferSize, _dmxWindowSize);
    TimoResult r = spiCommand(TIMO_CMD_READ_DMX, nullptr, buffer, len);
    if (r != TIMO_OK) return 0;
    return len;
}

bool TimoTwoFX::isDmxAvailable() {
    uint8_t status;
    if (readRegister(TIMO_REG_STATUS, &status, 1) != TIMO_OK) return false;
    return (status & TIMO_STATUS_DMX) != 0;
}

// ============================================================
// Linking / Connection Management
// ============================================================

void TimoTwoFX::startLinking() {
    uint8_t status;
    readRegister(TIMO_REG_STATUS, &status, 1);
    status |= TIMO_STATUS_RF_LINK;
    writeRegister(TIMO_REG_STATUS, &status, 1);
}

bool TimoTwoFX::isLinked() {
    uint8_t status;
    if (readRegister(TIMO_REG_STATUS, &status, 1) != TIMO_OK) return false;
    return (status & TIMO_STATUS_LINKED) != 0;
}

bool TimoTwoFX::hasRfLink() {
    uint8_t status;
    if (readRegister(TIMO_REG_STATUS, &status, 1) != TIMO_OK) return false;
    return (status & TIMO_STATUS_RF_LINK) != 0;
}

void TimoTwoFX::unlink() {
    uint8_t status;
    readRegister(TIMO_REG_STATUS, &status, 1);
    status |= TIMO_STATUS_LINKED;  // Writing 1 to LINKED bit triggers unlink
    writeRegister(TIMO_REG_STATUS, &status, 1);
}

void TimoTwoFX::setLinkingKey(const char* key) {
    // TX format: 8 bytes, each byte = one digit (0–9). 0xFF = unused digit slot.
    uint8_t buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    for (int i = 0; i < 8 && key[i] != '\0'; i++) {
        buf[i] = (uint8_t)(key[i] - '0');
    }
    writeRegister(TIMO_REG_LINKING_KEY, buf, 8);
    delay(200);  // Writing LINKING_KEY causes a module reboot
}

void TimoTwoFX::setLinkingKeyRX(const char* key, uint8_t mode, uint8_t output) {
    // RX format: 8 bytes key + 1 byte mode + 1 byte output
    uint8_t buf[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00};
    for (int i = 0; i < 8 && key[i] != '\0'; i++) {
        buf[i] = (uint8_t)(key[i] - '0');
    }
    buf[8] = mode;
    buf[9] = output;
    writeRegister(TIMO_REG_LINKING_KEY, buf, 10);
    delay(200);  // Writing LINKING_KEY causes a module reboot
}

// ============================================================
// Configuration
// ============================================================

void TimoTwoFX::setRfProtocol(uint8_t protocol) {
    writeRegister(TIMO_REG_RF_PROTOCOL, &protocol, 1);
    delay(200);  // Protocol change causes a module reboot
}

void TimoTwoFX::setRfPower(uint8_t power) {
    writeRegister(TIMO_REG_RF_POWER, &power, 1);
}

void TimoTwoFX::setDmxWindow(uint16_t startAddr, uint16_t length) {
    _dmxWindowStart = startAddr;
    _dmxWindowSize  = length;
    uint8_t buf[4];
    buf[0] = (length    >> 8) & 0xFF;
    buf[1] =  length          & 0xFF;
    buf[2] = (startAddr >> 8) & 0xFF;
    buf[3] =  startAddr       & 0xFF;
    writeRegister(TIMO_REG_DMX_WINDOW, buf, 4);
}

void TimoTwoFX::setBleEnabled(bool enabled) {
    uint8_t val = enabled ? 0x01 : 0x00;
    writeRegister(TIMO_REG_BLE_STATUS, &val, 1);
    delay(200);  // BLE setting change causes a module reboot
}

void TimoTwoFX::setDeviceName(const char* name) {
    uint8_t buf[32];
    memset(buf, 0, 32);
    strncpy((char*)buf, name, 32);
    writeRegister(TIMO_REG_DEVICE_NAME, buf, 32);
}

void TimoTwoFX::getDeviceName(char* buf) {
    uint8_t data[32];
    memset(data, 0, 32);
    readRegister(TIMO_REG_DEVICE_NAME, data, 32);
    memcpy(buf, data, 32);
    buf[32] = '\0';
}

void TimoTwoFX::setOemInfo(uint16_t manufacturerId, uint16_t deviceModelId) {
    uint8_t buf[4];
    buf[0] = (deviceModelId   >> 8) & 0xFF;
    buf[1] =  deviceModelId         & 0xFF;
    buf[2] = (manufacturerId  >> 8) & 0xFF;
    buf[3] =  manufacturerId        & 0xFF;
    writeRegister(TIMO_REG_OEM_INFO, buf, 4);
}

void TimoTwoFX::setUniverseColor(uint8_t r, uint8_t g, uint8_t b) {
    uint8_t buf[3] = {r, g, b};
    writeRegister(TIMO_REG_UNIVERSE_COLOR, buf, 3);
}

void TimoTwoFX::getUniverseColor(uint8_t* r, uint8_t* g, uint8_t* b) {
    uint8_t buf[3];
    readRegister(TIMO_REG_UNIVERSE_COLOR, buf, 3);
    *r = buf[0];
    *g = buf[1];
    *b = buf[2];
}

void TimoTwoFX::setBatteryLevel(uint8_t percent) {
    writeRegister(TIMO_REG_BATTERY, &percent, 1);
}

void TimoTwoFX::setIdentify(bool active) {
    uint8_t status;
    readRegister(TIMO_REG_STATUS, &status, 1);
    if (active) status |=  TIMO_STATUS_IDENTIFY;
    else        status &= ~TIMO_STATUS_IDENTIFY;
    writeRegister(TIMO_REG_STATUS, &status, 1);
}

bool TimoTwoFX::isIdentifying() {
    uint8_t status;
    if (readRegister(TIMO_REG_STATUS, &status, 1) != TIMO_OK) return false;
    return (status & TIMO_STATUS_IDENTIFY) != 0;
}

// ============================================================
// RDM
// ============================================================

TimoResult TimoTwoFX::writeRdm(const uint8_t* packet, uint16_t length) {
    return spiCommand(TIMO_CMD_WRITE_RDM, packet, nullptr, length);
}

TimoResult TimoTwoFX::readRdm(uint8_t* buffer, uint16_t& length) {
    // Phase 1: read the first 2 bytes to determine packet length
    uint8_t header[2];
    TimoResult r = spiCommand(TIMO_CMD_READ_RDM, nullptr, header, 2);
    if (r != TIMO_OK) { length = 0; return r; }

    // RDM packet length is encoded in byte 2 (Message Length field per E1.20).
    // Full packet size = Message Length + 2 bytes (start code + sub-start code).
    uint16_t rdmLen = (header[1] > 0) ? (uint16_t)(header[1] + 2) : 0;
    if (rdmLen == 0 || rdmLen > TIMO_RDM_MAX_PACKET_SIZE) {
        length = 0;
        return TIMO_ERROR_NO_DATA;
    }

    // Phase 2: read the full packet
    r = spiCommand(TIMO_CMD_READ_RDM, nullptr, buffer, rdmLen);
    length = rdmLen;
    return r;
}

TimoResult TimoTwoFX::startRadioDiscovery(const uint8_t* dubData, uint16_t length) {
    return spiCommand(TIMO_CMD_RADIO_DISCOVERY, dubData, nullptr, length);
}

TimoResult TimoTwoFX::readRadioDiscoveryResult(uint8_t* buffer, uint16_t& length) {
    TimoResult r = spiCommand(TIMO_CMD_RADIO_DISC_RESULT, nullptr, buffer, 6);
    length = (r == TIMO_OK) ? 6 : 0;
    return r;
}

TimoResult TimoTwoFX::startRdmDiscovery(const uint8_t* dubData, uint16_t length) {
    return spiCommand(TIMO_CMD_RDM_DISCOVERY, dubData, nullptr, length);
}

TimoResult TimoTwoFX::readRdmDiscoveryResult(uint8_t* buffer, uint16_t& length) {
    TimoResult r = spiCommand(TIMO_CMD_RDM_DISC_RESULT, nullptr, buffer, 24);
    length = (r == TIMO_OK) ? 24 : 0;
    return r;
}

// ============================================================
// Status & Diagnostics
// ============================================================

void TimoTwoFX::getVersion(uint32_t& hwVersion, uint32_t& swVersion) {
    uint8_t buf[8];
    readRegister(TIMO_REG_VERSION, buf, 8);
    hwVersion = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                ((uint32_t)buf[2] <<  8) |  (uint32_t)buf[3];
    swVersion = ((uint32_t)buf[4] << 24) | ((uint32_t)buf[5] << 16) |
                ((uint32_t)buf[6] <<  8) |  (uint32_t)buf[7];
}

void TimoTwoFX::getVersionString(char* buf) {
    uint8_t raw[8];
    readRegister(TIMO_REG_VERSION, raw, 8);
    snprintf(buf, 20, "%d.%d.%d.%d", raw[4], raw[5], raw[6], raw[7]);
}

uint8_t TimoTwoFX::getLinkQuality() {
    uint8_t val = 0;
    readRegister(TIMO_REG_LINK_QUALITY, &val, 1);
    return val;
}

uint8_t TimoTwoFX::getDmxSource() {
    uint8_t val = 0;
    readRegister(TIMO_REG_DMX_SOURCE, &val, 1);
    return val;
}

uint8_t TimoTwoFX::getLollipop() {
    uint8_t val = 0;
    readRegister(TIMO_REG_LOLLIPOP, &val, 1);
    return val;
}

void TimoTwoFX::setBindingUid(const uint8_t uid[6]) {
    writeRegister(TIMO_REG_BINDING_UID, uid, 6);
}

bool TimoTwoFX::hasRdmTxOption() {
    uint8_t buf[13];
    if (readRegister(TIMO_REG_INSTALLED_OPTIONS, buf, 13) != TIMO_OK) return false;
    uint8_t nOpts = buf[0];
    for (uint8_t i = 0; i < nOpts && i < 6; i++) {
        uint16_t optId = ((uint16_t)buf[1 + i * 2] << 8) | buf[2 + i * 2];
        if (optId == 0x2001) return true;  // 0x2001 = RDM TX via SPI option ID
    }
    return false;
}

uint8_t TimoTwoFX::readIrqFlags() {
    return _readIrqFlagsDirect();
}

uint8_t TimoTwoFX::readExtIrqFlags() {
    uint8_t buf[4];
    readRegister(TIMO_REG_EXT_IRQ_FLAGS, buf, 4);
    return buf[3];
}

// ============================================================
// Low-Level SPI
// ============================================================

TimoResult TimoTwoFX::readRegister(uint8_t reg, uint8_t* data, uint8_t length) {
    return spiCommand(TIMO_CMD_READ_REG(reg), nullptr, data, length);
}

TimoResult TimoTwoFX::writeRegister(uint8_t reg, const uint8_t* data, uint8_t length) {
    return spiCommand(TIMO_CMD_WRITE_REG(reg), data, nullptr, length);
}

TimoResult TimoTwoFX::spiCommand(uint8_t cmd, const uint8_t* txData, uint8_t* rxData, uint16_t length) {
    // NOP is a single-phase transaction: send command byte, receive IRQ flags immediately.
    // No IRQ handshake required.
    if (cmd == TIMO_CMD_NOP) {
        SPI.beginTransaction(SPISettings(TIMO_SPI_CLOCK, MSBFIRST, SPI_MODE0));
        _csLow();
        delayMicroseconds(4);           // CS setup time (hardware requirement: 4 µs)
        uint8_t flags = _spiTransfer(cmd);
        _csHigh();
        SPI.endTransaction();
        if (rxData && length > 0) rxData[0] = flags;
        return TIMO_OK;
    }

    // All other commands use a 2-phase protocol with IRQ handshaking.
    uint8_t retries = 5;
    while (retries--) {
        SPI.beginTransaction(SPISettings(TIMO_SPI_CLOCK, MSBFIRST, SPI_MODE0));

        // --- Phase 1: Send command byte ---
        _csLow();
        delayMicroseconds(4);
        uint8_t irqFlags = _spiTransfer(cmd);
        _csHigh();
        SPI.endTransaction();

        // Module busy? Back off and retry the command
        if (irqFlags & TIMO_IRQ_DEVICE_BUSY) {
            delayMicroseconds(100);
            continue;
        }

        // Wait for module to pull IRQ LOW (signals "ready for phase 2")
        if (!_waitForIrqLow()) return TIMO_ERROR_TIMEOUT;

        // --- Phase 2: Transfer payload ---
        SPI.beginTransaction(SPISettings(TIMO_SPI_CLOCK, MSBFIRST, SPI_MODE0));
        _csLow();
        delayMicroseconds(4);

        // Per hardware spec: first byte of phase 2 is a dummy byte (ignored by module)
        _spiTransfer(0xFF);

        for (uint16_t i = 0; i < length; i++) {
            uint8_t out = (txData  != nullptr) ? txData[i] : 0x00;
            uint8_t in  = _spiTransfer(out);
            if (rxData != nullptr) rxData[i] = in;
        }

        _csHigh();
        SPI.endTransaction();

        // Wait for IRQ to go HIGH (signals "processing complete")
        if (!_waitForIrqHigh()) return TIMO_ERROR_TIMEOUT;

        return TIMO_OK;
    }

    return TIMO_ERROR_BUSY;
}

// ============================================================
// Private Helper Methods
// ============================================================

bool TimoTwoFX::_waitForIrqLow(uint32_t timeoutMs) {
    uint32_t start = millis();
    while (digitalRead(_irqPin) == HIGH) {
        if (millis() - start > timeoutMs) return false;
    }
    return true;
}

bool TimoTwoFX::_waitForIrqHigh(uint32_t timeoutMs) {
    uint32_t start = millis();
    while (digitalRead(_irqPin) == LOW) {
        if (millis() - start > timeoutMs) return false;
    }
    return true;
}

void TimoTwoFX::_csLow() {
    digitalWrite(_csPin, LOW);
}

void TimoTwoFX::_csHigh() {
    digitalWrite(_csPin, HIGH);
}

uint8_t TimoTwoFX::_spiTransfer(uint8_t data) {
    return SPI.transfer(data);
}

uint8_t TimoTwoFX::_readIrqFlagsDirect() {
    // Fast NOP read: sends 0xFF and returns the IRQ flags byte from the module.
    // This is a 1-phase transaction with no waiting.
    SPI.beginTransaction(SPISettings(TIMO_SPI_CLOCK, MSBFIRST, SPI_MODE0));
    _csLow();
    delayMicroseconds(4);
    uint8_t flags = _spiTransfer(TIMO_CMD_NOP);
    _csHigh();
    SPI.endTransaction();
    return flags;
}

TimoResult TimoTwoFX::_configureDmxSpec(uint16_t channels, uint8_t refreshHz) {
    // Refresh period in microseconds: 1,000,000 / Hz
    // Example: 40 Hz → 25,000 µs
    uint32_t refreshUs = (refreshHz > 0) ? (1000000UL / refreshHz) : 25000UL;

    uint8_t buf[8];
    // Bytes 0–1: N_CHANNELS (16-bit big-endian)
    buf[0] = (channels >> 8) & 0xFF;
    buf[1] =  channels       & 0xFF;
    // Bytes 2–3: INTERSLOT_TIME (16-bit) — set to 0 (hardware default)
    buf[2] = 0;
    buf[3] = 0;
    // Bytes 4–7: REFRESH_PERIOD (32-bit big-endian, in microseconds)
    buf[4] = (refreshUs >> 24) & 0xFF;
    buf[5] = (refreshUs >> 16) & 0xFF;
    buf[6] = (refreshUs >>  8) & 0xFF;
    buf[7] =  refreshUs        & 0xFF;

    return writeRegister(TIMO_REG_DMX_SPEC, buf, 8);
}
