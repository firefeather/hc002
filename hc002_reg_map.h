#ifndef HC002_REG_MAP_H__
#define HC002_REG_MAP_H__

/**
 ** UART
 **/
#define HC_SERIAL_PINS UART1_PORTB

/**
 ** nRF24l01+ RF transceiver
 **/

// SPI: SCK = PB_4, MOSI = PB_7, MISO = PB_6
#define HC_RF_CE  PE_1
#define HC_RF_CSN PE_2
#define HC_RF_IRQ PE_3

/**
 ** DHT22 Humidity sensor
 **/

#define HC_DHT PE_4

/**
 ** Servo
 **/

#define HC_SERVO PF_1

/**
 ** Fan control
 **/

#define HC_FAN PF_5

#endif // HC002_REG_MAP_H__
