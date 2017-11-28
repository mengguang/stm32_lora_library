// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef LORA_H
#define LORA_H

#include <stdint.h>

#define PA_OUTPUT_RFO_PIN      0
#define PA_OUTPUT_PA_BOOST_PIN 1

void lora_init();

int lora_begin(long frequency);
void lora_end();

int lora_beginPacket(int implicitHeader);
int lora_endPacket();

int lora_parsePacket(int size);
int lora_packetRssi();
float lora_packetSnr();

uint32_t lora_write(const uint8_t *buffer, uint32_t size);

int lora_available();
int lora_read();
int lora_peek();
void lora_flush();

void lora_onReceive(void(*callback)(int));

void lora_receive(int size);
void lora_idle();
void lora_sleep();

void lora_setTxPower(int level, int outputPin);
void lora_setFrequency(long frequency);
void lora_setSpreadingFactor(int sf);
void lora_setSignalBandwidth(long sbw);
void lora_setCodingRate4(int denominator);
void lora_setPreambleLength(long length);
void lora_setSyncWord(int sw);
void lora_enableCrc();
void lora_disableCrc();

uint8_t lora_random();

void lora_explicitHeaderMode();
void lora_implicitHeaderMode();

void lora_handleDio0Rise();

uint8_t lora_readRegister(uint8_t address);
void lora_writeRegister(uint8_t address, uint8_t value);
uint8_t lora_singleTransfer(uint8_t address, uint8_t value);

static void onDio0Rise();

#endif
