// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <lora.h>
#include <spi.h>
#include <gpio.h>
#include <main.h>

// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_PKT_SNR_VALUE        0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define MAX_PKT_LENGTH           255

int _frequency;
int _packetIndex;
int _implicitHeaderMode;
void (*_onReceive)(int);

void lora_init() {
    _frequency = 0;
    _packetIndex = 0;
    _implicitHeaderMode = 0;
}

void reset_low() {
    HAL_GPIO_WritePin(CE_GPIO_Port,CE_Pin,GPIO_PIN_RESET);
}

void reset_high() {
    HAL_GPIO_WritePin(CE_GPIO_Port,CE_Pin,GPIO_PIN_SET);
}

void ss_high() {
    HAL_GPIO_WritePin(NSS_GPIO_Port,NSS_Pin,GPIO_PIN_SET);
}

void ss_low() {
    HAL_GPIO_WritePin(NSS_GPIO_Port,NSS_Pin,GPIO_PIN_RESET);
}

void delay(uint32_t ms) {
    HAL_Delay(ms);
}

int lora_begin(long frequency)
{
    // perform reset
    reset_low();
    delay(10);
    reset_high();
    delay(10);
    
    // set SS high
    ss_high();
    
    // check version
    uint8_t version = lora_readRegister(REG_VERSION);
    if (version != 0x12) {
        return 0;
    }
    
    // put in sleep mode
    lora_sleep();
    
    // set frequency
    lora_setFrequency(frequency);
    
    // set base addresses
    lora_writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
    lora_writeRegister(REG_FIFO_RX_BASE_ADDR, 0);
    
    // set LNA boost
    lora_writeRegister(REG_LNA, lora_readRegister(REG_LNA) | 0x03);
    
    // set auto AGC
    lora_writeRegister(REG_MODEM_CONFIG_3, 0x04);
    
    // set output power to 17 dBm
    lora_setTxPower(17,PA_OUTPUT_PA_BOOST_PIN);
    
    // put in standby mode
    lora_idle();
    
    return 1;
}

void lora_end()
{
    // put in sleep mode
    lora_sleep();
}

int lora_beginPacket(int implicitHeader)
{
    // put in standby mode
    lora_idle();
    
    if (implicitHeader) {
        lora_implicitHeaderMode();
    } else {
        lora_explicitHeaderMode();
    }
    
    // reset FIFO address and paload length
    lora_writeRegister(REG_FIFO_ADDR_PTR, 0);
    lora_writeRegister(REG_PAYLOAD_LENGTH, 0);
    
    return 1;
}

int lora_endPacket()
{
    // put in TX mode
    lora_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
    
    // wait for TX done
    while((lora_readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0);
    
    // clear IRQ's
    lora_writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    
    return 1;
}

int lora_parsePacket(int size)
{
    int packetLength = 0;
    int irqFlags = lora_readRegister(REG_IRQ_FLAGS);
    
    
    if (size > 0) {
        lora_implicitHeaderMode();
        
        lora_writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
    } else {
        lora_explicitHeaderMode();
    }
    
    // clear IRQ's
    lora_writeRegister(REG_IRQ_FLAGS, irqFlags);
    
    if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
        // received a packet
        _packetIndex = 0;
        
        // read packet length
        if (_implicitHeaderMode) {
            packetLength = lora_readRegister(REG_PAYLOAD_LENGTH);
        } else {
            packetLength = lora_readRegister(REG_RX_NB_BYTES);
        }
        
        // set FIFO address to current RX address
        lora_writeRegister(REG_FIFO_ADDR_PTR, lora_readRegister(REG_FIFO_RX_CURRENT_ADDR));
        
        // put in standby mode
        lora_idle();
    } else if (lora_readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
        // not currently in RX mode
        
        // reset FIFO address
        lora_writeRegister(REG_FIFO_ADDR_PTR, 0);
        
        // put in single RX mode
        lora_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
    }
    
    return packetLength;
}

int lora_packetRssi()
{
    return (lora_readRegister(REG_PKT_RSSI_VALUE) - (_frequency < 868E6 ? 164 : 157));
}

float lora_packetSnr()
{
  return ((int8_t)lora_readRegister(REG_PKT_SNR_VALUE)) * 0.25;
}


size_t lora_write(const uint8_t *buffer, size_t size)
{
    int currentLength = lora_readRegister(REG_PAYLOAD_LENGTH);
    
    // check size
    if ((currentLength + size) > MAX_PKT_LENGTH) {
        size = MAX_PKT_LENGTH - currentLength;
    }
    
    // write data
    for (size_t i = 0; i < size; i++) {
        lora_writeRegister(REG_FIFO, buffer[i]);
    }
    
    // update length
    lora_writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);
    
    return size;
}

int lora_available()
{
    return (lora_readRegister(REG_RX_NB_BYTES) - _packetIndex);
}

int lora_read()
{
    if (!lora_available()) {
        return -1;
    }
    
    _packetIndex++;
    
    return lora_readRegister(REG_FIFO);
}

int lora_peek()
{
    if (!lora_available()) {
        return -1;
    }
    
    // store current FIFO address
    int currentAddress = lora_readRegister(REG_FIFO_ADDR_PTR);
    
    // read
    uint8_t b = lora_readRegister(REG_FIFO);
    
    // restore FIFO address
    lora_writeRegister(REG_FIFO_ADDR_PTR, currentAddress);
    
    return b;
}

void lora_flush()
{
}

void lora_onReceive(void(*callback)(int))
{
    _onReceive = callback;
    
    /* TODO: attachInterrupt
    if (callback) {
        lora_writeRegister(REG_DIO_MAPPING_1, 0x00);
        
        attachInterrupt(digitalPinToInterrupt(_dio0), LoRaClass::onDio0Rise, RISING);
    } else {
        detachInterrupt(digitalPinToInterrupt(_dio0));
    }
    */
}

void lora_receive(int size)
{
    if (size > 0) {
        lora_implicitHeaderMode();
        
        lora_writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
    } else {
        lora_explicitHeaderMode();
    }
    
    lora_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void lora_idle()
{
    lora_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void lora_sleep()
{
    lora_writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void lora_setTxPower(int level, int outputPin)
{
    if (PA_OUTPUT_RFO_PIN == outputPin) {
        // RFO
        if (level < 0) {
            level = 0;
        } else if (level > 14) {
            level = 14;
        }
        
        lora_writeRegister(REG_PA_CONFIG, 0x70 | level);
    } else {
        // PA BOOST
        if (level < 2) {
            level = 2;
        } else if (level > 17) {
            level = 17;
        }
        
        lora_writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
    }
}

void lora_setFrequency(long frequency)
{
    _frequency = frequency;
    
    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
    
    lora_writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
    lora_writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
    lora_writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void lora_setSpreadingFactor(int sf)
{
    if (sf < 6) {
        sf = 6;
    } else if (sf > 12) {
        sf = 12;
    }
    
    if (sf == 6) {
        lora_writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
        lora_writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
    } else {
        lora_writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
        lora_writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
    }
    
    lora_writeRegister(REG_MODEM_CONFIG_2, (lora_readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
}

void lora_setSignalBandwidth(long sbw)
{
    int bw;
    
    if (sbw <= 7.8E3) {
        bw = 0;
    } else if (sbw <= 10.4E3) {
        bw = 1;
    } else if (sbw <= 15.6E3) {
        bw = 2;
    } else if (sbw <= 20.8E3) {
        bw = 3;
    } else if (sbw <= 31.25E3) {
        bw = 4;
    } else if (sbw <= 41.7E3) {
        bw = 5;
    } else if (sbw <= 62.5E3) {
        bw = 6;
    } else if (sbw <= 125E3) {
        bw = 7;
    } else if (sbw <= 250E3) {
        bw = 8;
    } else /*if (sbw <= 250E3)*/ {
        bw = 9;
    }
    
    lora_writeRegister(REG_MODEM_CONFIG_1, (lora_readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}

void lora_setCodingRate4(int denominator)
{
    if (denominator < 5) {
        denominator = 5;
    } else if (denominator > 8) {
        denominator = 8;
    }
    
    int cr = denominator - 4;
    
    lora_writeRegister(REG_MODEM_CONFIG_1, (lora_readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void lora_setPreambleLength(long length)
{
    lora_writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
    lora_writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void lora_setSyncWord(int sw)
{
    lora_writeRegister(REG_SYNC_WORD, sw);
}

void lora_enableCrc()
{
    lora_writeRegister(REG_MODEM_CONFIG_2, lora_readRegister(REG_MODEM_CONFIG_2) | 0x04);
}

void lora_disableCrc()
{
    lora_writeRegister(REG_MODEM_CONFIG_2, lora_readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

uint8_t lora_random()
{
    return lora_readRegister(REG_RSSI_WIDEBAND);
}

void lora_explicitHeaderMode()
{
    _implicitHeaderMode = 0;
    
    lora_writeRegister(REG_MODEM_CONFIG_1, lora_readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

void lora_implicitHeaderMode()
{
    _implicitHeaderMode = 1;
    
    lora_writeRegister(REG_MODEM_CONFIG_1, lora_readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

void lora_handleDio0Rise()
{
    int irqFlags = lora_readRegister(REG_IRQ_FLAGS);
    
    // clear IRQ's
    lora_writeRegister(REG_IRQ_FLAGS, irqFlags);
    
    if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
        // received a packet
        _packetIndex = 0;
        
        // read packet length
        int packetLength = _implicitHeaderMode ? lora_readRegister(REG_PAYLOAD_LENGTH) : lora_readRegister(REG_RX_NB_BYTES);
        
        // set FIFO address to current RX address
        lora_writeRegister(REG_FIFO_ADDR_PTR, lora_readRegister(REG_FIFO_RX_CURRENT_ADDR));
        
        if (_onReceive) {
            _onReceive(packetLength);
        }
        
        // reset FIFO address
        lora_writeRegister(REG_FIFO_ADDR_PTR, 0);
    }
}

uint8_t lora_readRegister(uint8_t address)
{
    return lora_singleTransfer(address & 0x7f, 0x00);
}

void lora_writeRegister(uint8_t address, uint8_t value)
{
    lora_singleTransfer(address | 0x80, value);
}

uint8_t lora_singleTransfer(uint8_t address, uint8_t value)
{
    uint8_t response;
    
    ss_low();
    
    HAL_SPI_Transmit(&hspi1,&address,1,1000);
    HAL_SPI_TransmitReceive(&hspi1,&value,&response,1,1000);
    
    ss_high();
    
    return response;
}

void lora_onDio0Rise()
{
    lora_handleDio0Rise();
}
