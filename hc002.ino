// Includes

#include <SPI.h>
#include <Enrf24.h>
#include <nRF24L01.h>
#include <DHT22_430.h>
#include <Servo.h>
#include <SimpleTimer.h>
#include <EEPROM.h>

#include "hc002_reg_map.h"

#include "wiring_analog.c"

// Definitions

#define SerialDbg Serial1
#define STR_SZ 33

// Constants and global objects

Enrf24 radio (HC_RF_CE_PIN, HC_RF_CSN_PIN, HC_RF_IRQ_PIN);
Servo servo;
DHT22 dht (HC_DHT_PIN);
SimpleTimer stimer;

boolean dht_flag;

const uint16_t servoPeriodSeconds = 3;
const uint16_t sensorsPeriodSeconds = 3;

const uint8_t txaddr[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x01 };
const uint8_t rxaddr[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x02 };
const uint16_t speedaddr = 0;

char rxBuf[STR_SZ];
char txBuf[STR_SZ];

uint8_t speed = 0;
boolean speed_by_rx = 0;

// Functions prototypes

void setupSerial ();
void setupSPI ();
void setupRadio ();
void setupServo ();
void setupDHT ();
void setupTimer ();
void setupFAN ();
void setupSpeedControl ();

void dumpRadioStatus (uint8_t);

void readRFCommand ();
void rotateServo ();
void readSensors ();
void readSpeedControl ();
void changeSpeed (uint16_t);

// Functions implementation

void setup()
{
	setupSerial ();
	setupSPI ();
	setupRadio ();
	setupServo ();
	setupDHT ();
	setupTimer ();
	setupFAN ();
	setupSpeedControl ();
}

void setupSerial ()
{
	SerialDbg.setPins (HC_SERIAL_PINS);
	SerialDbg.begin (9600);

	SerialDbg.println ("Serial setup completed");
}

void setupSPI ()
{
	SPI.setModule (3);
	SPI.begin ();
	SPI.setClockDivider (SPI_CLOCK_DIV8);
	SPI.setDataMode (SPI_MODE0);
	SPI.setBitOrder (MSBFIRST);

	SerialDbg.println ("SPI setup completed");
}

void setupRadio ()
{
	radio.begin ();
	dumpRadioStatus (radio.radioState ());

	radio.setTXaddress ( (void*) txaddr);
	radio.setRXaddress ( (void*) rxaddr);
	dumpRadioStatus (radio.radioState ());

	radio.enableRX ();
	dumpRadioStatus (radio.radioState ());

	pinMode (HC_RX_ACT_PIN, OUTPUT);
	pinMode (HC_TX_ACT_PIN, OUTPUT);
	pinMode (HC_RF_STATE_ERR_PIN, OUTPUT);

	SerialDbg.println ("Radio setup completed");
}

void setupServo ()
{
	servo.attach (HC_SERVO_PIN);
	servo.write (90);

	SerialDbg.println ("Servo setup completed");
}

void setupDHT ()
{
	dht.begin ();

	SerialDbg.println ("DHT setup completed");
}

void setupTimer ()
{
	stimer.setInterval (servoPeriodSeconds * 1000, rotateServo);
	stimer.setInterval (sensorsPeriodSeconds * 1000, readSensors);

	SerialDbg.println ("Timer setup completed");
}

void setupFAN ()
{
	changeSpeed (EEPROM.read (speedaddr));

	SerialDbg.println ("FAN setup completed");
}

void setupSpeedControl ()
{
	pinMode (HC_SPEED_PIN, INPUT);
}

void loop()
{
	stimer.run ();

	readRFCommand ();
	readSpeedControl ();
}

void rotateServo ()
{
	int pos;

	SerialDbg.println ("Servo rotation started");

	for (pos = 0; pos < 180; pos += 30) {
		servo.write (pos);
		delay (30);
	}
	for (pos = 180; pos >= 1; pos -= 30) {
		servo.write (pos);
		delay (30);
	}

	SerialDbg.println ("Servo rotation finished");
}

void readSensors ()
{
	SerialDbg.println ("Sensors read started");

	digitalWrite (HC_TX_ACT_PIN, HIGH);

	//TODO(DZhon): Debug purpose!
	dht_flag = 0;//dht.get ();
	int16_t h = dht.humidityX10 ();
	int16_t t = dht.temperatureX10 ();

	SerialDbg.println ("DHT stage finished");

	if (!dht_flag) {
		snprintf (txBuf, sizeof (txBuf), "Failed to read from DHT22");
	} else {
		snprintf (txBuf, sizeof (txBuf),
				  "H:%d.%d, T:%d.%d, S:%d",
				  h / 10, h % 10,
				  t / 10, t % 10,
				  speed);
	}

	SerialDbg.println (txBuf);
	SerialDbg.flush ();
	radio.println (txBuf);
	radio.flush ();

	dumpRadioStatus (radio.radioState ());

	digitalWrite (HC_TX_ACT_PIN, LOW);

	SerialDbg.println ("Sensors read finished");
}

void readSpeedControl ()
{
	if (speed_by_rx) {
		changeSpeed ( (100 * analogRead (HC_SPEED_PIN)) / 4095);
	}
}

void readRFCommand ()
{
	if (radio.available (true)) {
		if (radio.read (rxBuf)) {
			digitalWrite (HC_RX_ACT_PIN, HIGH);

			SerialDbg.print ("Received packet: ");
			SerialDbg.println (rxBuf);

			char command_type = rxBuf[0];
			//char command_sep = rxBuf[1];
			char command_val = rxBuf[2];
			//char zero_byte = rxBuf[3];

			switch (command_type) {
				case 'S': changeSpeed (command_val); break;
				case 'F': rotateServo (); break;
				case 'R': readSensors (); break;

				default: SerialDbg.println ("Unknown command");
			}

			digitalWrite (HC_RX_ACT_PIN, LOW);
		}
	}
}

void dumpRadioStatus (uint8_t status)
{
	SerialDbg.print ("Enrf24 radio transceiver status: ");

	switch (status) {
		case ENRF24_STATE_NOTPRESENT:
			digitalWrite (HC_RF_STATE_ERR_PIN, HIGH);
			SerialDbg.println ("NO TRANSCEIVER PRESENT");
			break;

		case ENRF24_STATE_DEEPSLEEP:
			SerialDbg.println ("DEEP SLEEP <1uA power consumption");
			break;

		case ENRF24_STATE_IDLE:
			SerialDbg.println ("IDLE module powered up w/ oscillators running");
			break;

		case ENRF24_STATE_PTX:
			SerialDbg.println ("Actively Transmitting");
			break;

		case ENRF24_STATE_PRX:
			SerialDbg.println ("Receive Mode");
			break;

		default:
			digitalWrite (HC_RF_STATE_ERR_PIN, HIGH);
			SerialDbg.println ("UNKNOWN STATUS CODE");
	}
}

void changeSpeed (uint16_t new_speed)
{
	speed = new_speed;
	PWMWrite (HC_FAN_PIN, 255, (speed * 255) / 100, 25000);
}
