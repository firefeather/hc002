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
#define HC_RF_CE_PIN        PE_1
#define HC_RF_CSN_PIN       PE_2
#define HC_RF_IRQ_PIN       PE_3

#define HC_TX_ACT_PIN       PC_4
#define HC_RX_ACT_PIN       PC_5
#define HC_RF_STATE_ERR_PIN PC_6

/**
 ** DHT22 Humidity sensor
 **/

#define HC_DHT_PIN PE_4

/**
 ** Servo
 **/

#define HC_SERVO_PIN PF_1

/**
 ** Fan control
 **/

#define HC_FAN_PIN PA_6

/**
 ** Speed PIN
 **/
#define HC_SPEED_PIN PB_3

#endif // HC002_REG_MAP_H__
