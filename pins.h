/**
 * IO Mapping file. You can alter the pin numbers here.
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

#ifndef _PINS_H
#define _PINS_H
/**
 * SPI pins
 * CC1101
 */
#define SPI_SS   10     // PB2 = SPI_SS
#define SPI_MOSI 11     // PB3 = MOSI
#define SPI_MISO 12     // PB4 = MISO
#define SPI_SCK  13     // PB5 = SCK
#define GDO0   2        // PD2 = INT0

#define PORT_SPI_MISO  PINB
#define BIT_SPI_MISO  4

#define PORT_SPI_SS  PORTB
#define BIT_SPI_SS   2

#define PORT_GDO0  PIND
#define BIT_GDO0  2

/**
 * SPI pins
 * PCD8544
 */
#define SPI_PCD8544_DC   9
#define SPI_PCD8544_CS   8
#define SPI_PCD8544_RST  7

// The LED is wired to the Arduino Output 4 (physical panStamp pin 19)
#define LEDOUTPUT 4

// Define Button pins
#define BUTTON_3 6
#define BUTTON_2 5
#define BUTTON_1 4

#endif

