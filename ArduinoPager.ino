/**
 * 
 * Arduino   GND          CC1101 GND / 9
 * Arduino   VCC (+3.3v)  CC1101 VCC / 1
 * Arduino   13           CC1101 SCK / 4
 * Arduino   12           CC1101 SO (MISO) / 5
 * Arduino   11           CC1101 SI (MOSI) / 3
 * Arduino   10           CC1101 CSN (SS)  / 7
 * Arduino   02           CC1101 GD0 / 8
 * Arduino   03           Keypad interrupt
 * Arduino   09           PCD8544 D/C
 * Arduino   08           PCD8544 CS
 * Arduino   07           PCD8544 RST
 * Arduino   06           Button 3
 * Arduino   05           Button 2
 * Arduino   04           Button 1
 * 
 */

#ifndef _ARDUINO_PAGER_
#define _ARDUINO_PAGER_

#define USE_UART

#include "EEPROM.h"
#include "cc1101.h"
#include "Pocsag.h"
#include "watchdog.h"
#include "Arduino.h"
#include "avr/interrupt.h"
#include "avr/sleep.h"
#include "avr/power.h"
#include "avr/wdt.h"
#include <Adafruit_GFX.h>
#include <gfxfont.h>
#include <Adafruit_PCD8544.h>
#include "pins.h"

// The LED is wired to the Arduino Output 4 (physical panStamp pin 19)
#define LEDOUTPUT 4

// Define Button pins
#define BUTTON_3 6
#define BUTTON_2 5
#define BUTTON_1 4

// The connection to the hardware chip CC1101 the RF Chip
static CC1101 cc1101;

// the display connection
static Adafruit_PCD8544 lcd = Adafruit_PCD8544(SPI_PCD8544_DC, SPI_PCD8544_CS, SPI_PCD8544_RST);

// byte b;
// byte i;
static byte syncWord = 199;
// long counter = 0;
// byte chan = 0;

// a flag that a wireless packet has been received
volatile boolean packetAvailable = false;
volatile boolean keyboardInt = false;

void blinker() {
    digitalWrite(LEDOUTPUT, HIGH);
    delay(100);
    digitalWrite(LEDOUTPUT, LOW);
    delay(100);
}

/* Handle interrupt from CC1101 (INT0) gdo0 on pin2 */
void cc1101signalsInterrupt(void) {
    // disable sleep otherwise mcu could crash
    sleep_disable();
    // Disable wireless reception interrupt
    detachInterrupt(0);
    // set the flag that a package is available
    packetAvailable = true;
}

/* Handle interrupt from keys (INT1) on pin3 */
void keyboardInterrupt(void) {
    // disable sleep otherwise mcu could crash
    sleep_disable();
    // Disable wireless reception interrupt
    detachInterrupt(1);
    // set the flag that a package is available
    keyboardInt = true;
}

/**
 * Set CC1101 Register as calculated by SmartRF 7
 * Configuration:
 *
 * Deviation = 4.364014 
 * Base frequency = 439.987396
 * Carrier frequency = 439.987396
 * Channel number = 0 
 * Modulated = true 
 * Modulation format = 2-FSK 
 * Manchester enable = false
 * Data whitening = off
 * Sync word qualifier mode = No preamble/sync
 * Channel spacing = 25.390625 
 * Data rate = 1.19948 Kbps
 * RX filter BW = 58.035714
 * Data format = Normal mode 
 * Length config = Infinite packet length mode
 * CRC enable = false 
 * Packet length = 255 
 * Device address = 0 
 * Address config = No address check
 * Append status = Append two status bytes to the payload of the packet. The status bytes contain RSSI and LQI values, as well as CRC OK
 * CRC autoflush = false 
 * PA ramping = false 
 * TX power = 0
 * GDO0 mode = Asserts when sync word has been sent / received, and de-asserts at the end of the packet.
 * In RX, the pin will also de-assert when a packet is discarded due to address or maximum length filtering
 * or when the radio enters RXFIFO_OVERFLOW state. In TX the pin will de-assert if the TX FIFO underflows
 * Settings optimized for low current consumption 
 */
void setCC1101Regs(void) {
    cc1101.writeReg(CC1101_IOCFG2, 0x29);
    cc1101.writeReg(CC1101_IOCFG1, 0x2E);
    cc1101.writeReg(CC1101_IOCFG0, 0x06);
    cc1101.writeReg(CC1101_FIFOTHR, 0x47);
    cc1101.writeReg(CC1101_PKTLEN, 0xFF);
    cc1101.writeReg(CC1101_PKTCTRL1, 0x04);
    cc1101.writeReg(CC1101_PKTCTRL0, 0x06);
    cc1101.writeReg(CC1101_FSCTRL1, 0x06);
    cc1101.writeReg(CC1101_FSCTRL0, 0x00);
    cc1101.writeReg(CC1101_MDMCFG4, 0xF5);
    cc1101.writeReg(CC1101_MDMCFG3, 0x83);
    cc1101.writeReg(CC1101_MDMCFG2, 0x80);
    cc1101.writeReg(CC1101_MDMCFG1, 0x20);
    cc1101.writeReg(CC1101_MDMCFG0, 0x00);
    cc1101.writeReg(CC1101_DEVIATN, 0x13);
    cc1101.writeReg(CC1101_MCSM2, 0x07);
    cc1101.writeReg(CC1101_MCSM1, 0x30);
    cc1101.writeReg(CC1101_MCSM0, 0x18);
    cc1101.writeReg(CC1101_FOCCFG, 0x16);
    cc1101.writeReg(CC1101_BSCFG, 0x6C);
    cc1101.writeReg(CC1101_AGCCTRL2, 0x03);
    cc1101.writeReg(CC1101_AGCCTRL1, 0x40);
    cc1101.writeReg(CC1101_AGCCTRL0, 0x91);
    cc1101.writeReg(CC1101_WOREVT1, 0x87);
    cc1101.writeReg(CC1101_WOREVT0, 0x6B);
    cc1101.writeReg(CC1101_WORCTRL, 0xFB);
    cc1101.writeReg(CC1101_FREND1, 0x56);
    cc1101.writeReg(CC1101_FREND0, 0x10);
    cc1101.writeReg(CC1101_FSCAL3, 0xE9);
    cc1101.writeReg(CC1101_FSCAL2, 0x2A);
    cc1101.writeReg(CC1101_FSCAL1, 0x00);
    cc1101.writeReg(CC1101_FSCAL0, 0x1F);
    cc1101.writeReg(CC1101_RCCTRL1, 0x41);
    cc1101.writeReg(CC1101_RCCTRL0, 0x00);
    cc1101.writeReg(CC1101_FSTEST, 0x59);
    cc1101.writeReg(CC1101_PTEST, 0x7F);
    cc1101.writeReg(CC1101_AGCTEST, 0x3F);
    cc1101.writeReg(CC1101_TEST2, 0x81);
    cc1101.writeReg(CC1101_TEST1, 0x35);
    cc1101.writeReg(CC1101_TEST0, 0x09);
}

ISR(WDT_vect)
{
  lcd.powerSaving(true);
  wdt_disable();
  sleep_enable();
  sleep_cpu();
}

void setup() {
  wdt_disable();
#ifdef USE_UART
    Serial.begin(9600);
    Serial.println("start");
#endif

    // setup the blinker output
    pinMode(LEDOUTPUT, OUTPUT);
    digitalWrite(LEDOUTPUT, LOW);

    // define button pins as input
    pinMode(BUTTON_3, INPUT)
    pinMode(BUTTON_2, INPUT)
    pinMode(BUTTON_1, INPUT)

    // blink once to signal the setup
    blinker();
    // initialize the RF Chip
    cc1101.init();

    cc1101.setSyncWord(&syncWord, false);
    cc1101.setCarrierFreq(CFREQ_433);
    cc1101.disableAddressCheck(); //if not specified, will only display "packet received"
    cc1101.setTxPowerAmp(PA_LowPower);

    setCC1101Regs();

    lcd.begin(84, 48);

#ifdef USE_UART
    Serial.print("CC1101_PARTNUM "); //cc1101=0
    Serial.println(cc1101.readReg(CC1101_PARTNUM, CC1101_STATUS_REGISTER));
    Serial.print("CC1101_VERSION "); //cc1101=4
    Serial.println(cc1101.readReg(CC1101_VERSION, CC1101_STATUS_REGISTER));
    Serial.print("CC1101_MARCSTATE ");
    Serial.println(cc1101.readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & 0x1f);

    Serial.println("device initialized");
#endif
}

void reduce_power() {
    sleep_enable();
    attachInterrupt(0, cc1101signalsInterrupt, FALLING);
    attachInterrupt(1, keyboardInterrupt, FALLING);

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    //    sleep_bod_disable();
    
    // power down adc
    power_adc_disable();
    
    // power down i2c
    power_twi_disable();
    
    // power down analog comparator
    // power_aca_disable();
    ACSR = B10000000;
#ifndef USE_UART
    // if not in use, power down usart
    power_usart_disable();
#endif
}

/* 
 * Use this functions to turn on display, it will activate the watchdog 
 * to turn off the display after 4s.
 */
void displayOn() {
  lcd.powerSaving(false);
  setup_watchdog(WDTO_4S);
}

void ReadLQI() {
    byte lqi = 0;
    byte value = 0;
    lqi = (cc1101.readReg(CC1101_LQI, CC1101_STATUS_REGISTER));
    value = 0x3F - (lqi & 0x3F);
#ifdef USE_UART
    Serial.print("CC1101_LQI ");
    Serial.println(value);
#endif
}

/**
 * Read the radio signal strength indicator
 */
void ReadRSSI() {
    byte rssi = 0;
    byte value = 0;

    rssi = (cc1101.readReg(CC1101_RSSI, CC1101_STATUS_REGISTER));

    if (rssi >= 128) {
        value = 255 - rssi;
        value /= 2;
        value += 74;
    } else {
        value = rssi / 2;
        value += 74;
    }
#ifdef USE_UART
    Serial.print("CC1101_RSSI ");
    Serial.println(value);
#endif
}

void loop() {
    if (packetAvailable) {
        cli();
#ifdef USE_UART        
        Serial.println("packet received");
#endif

        ReadRSSI();
        ReadLQI();
        // clear the flag
        packetAvailable = false;

        CCPACKET packet;

        if (cc1101.receiveData(&packet) > 0) {
            if (!packet.crc_ok) {
#ifdef USE_UART
                Serial.println("crc not ok");
#endif
            } else if (packet.length > 0) {
#ifdef USE_UART
                Serial.print("packet: len ");
                Serial.print(packet.length);
                Serial.print(" data: ");
                
                for (int j = 0; j < packet.length; j++) {
                    Serial.print(packet.data[j], HEX);
                    Serial.print(" ");
                }

                Serial.println(".");
#endif
            }
        }
        // reactivate sleep again, get's deactivated during interrupt
        sleep_enable();
        // Enable wireless reception interrupt
        attachInterrupt(0, cc1101signalsInterrupt, FALLING);
    }

    if (keyboardInt) {
        cli();

        
        
        // reactivate sleep again, get's deactivated during interrupt
        sleep_enable();
        // Enable keyboard again
        attachInterrupt(1, keyboardInterrupt, FALLING);

    }

    // send mcu to sleep and activate interrupts
    wdt_reset();
    sei();
    sleep_cpu();
}

#endif
