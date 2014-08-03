// Includes

#include <SPI.h>
#include <Enrf24.h>
#include <nRF24L01.h>
#include <DHT22_430.h>
#include <Servo.h>
#include <SimpleTimer.h>
#include <EEPROM.h>

#include "hc002_reg_map.h"

// Definitions

#define SerialDbg Serial1

// Constants and global objects

Enrf24 radio (HC_RF_CE, HC_RF_CSN, HC_RF_IRQ);
Servo servo;
DHT22 dht (HC_DHT);
SimpleTimer stimer;

boolean dht_flag;

const uint16_t servoPeriodSeconds = 3;
const uint16_t sensorsPeriodSeconds = 3;

const uint8_t txaddr[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x01 };
const uint8_t rxaddr[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x02 };

char rxBuf[33];

// Functions prototypes

void dump_radio_status_to_serialport (uint8_t);

void readRFCommand ();
void rotateServo ();
void readSensors ();
void readSpeedControl ();

// Functions implementation

void setup()
{
        // Init debugging UART capabilities.

	SerialDbg.setPins (HC_SERIAL_PINS);
	SerialDbg.begin (9600);

        // Init SPI for nRF24l01+.
        // Using PD port.

	SPI.setModule (3);
	SPI.begin ();
	SPI.setClockDivider (SPI_CLOCK_DIV8);
	SPI.setDataMode (SPI_MODE0);
	SPI.setBitOrder (MSBFIRST);

        // Bring up radio module.

	radio.begin ();
	dump_radio_status_to_serialport (radio.radioState ());
	radio.setTXaddress ( (void*) txaddr);
	radio.setRXaddress ( (void*) rxaddr);
	dump_radio_status_to_serialport (radio.radioState ());
	radio.enableRX ();
	dump_radio_status_to_serialport (radio.radioState ());

        // Init servo.

	servo.attach (HC_SERVO);
	servo.write (90);

        // Init DHT22

	dht.begin ();

        // Prepare periodic tasks

        stimer.setInterval (servoPeriodSeconds * 1000, rotateServo);
	stimer.setInterval (sensorsPeriodSeconds * 1000, readSensors);
}

void loop()
{
        // Periodic tasks scheduler
	stimer.run ();

        // Immediate reaction for incoming requests
        readRFCommand ();
        readSpeedControl ();
}

void rotateServo ()
{
	int pos;

        SerialDbg.println ("Servo rotation started");

	for (pos = 0; pos < 180; pos += 1) {
		servo.write (pos);
		delay (15);
	}
	for (pos = 180; pos >= 1; pos -= 1) {
		servo.write (pos);
		delay (15);
	}

        SerialDbg.println ("Servo rotation finished");
}

void readSensors ()
{
	dht_flag = dht.get ();
	int32_t h = dht.humidityX10 ();
	int32_t t = dht.temperatureX10 ();

	if (!dht_flag) {
		SerialDbg.println ("Failed to read from DHT22");
                radio.println ("DHTFail");
	} else {
		SerialDbg.print ("RH% \t");
		SerialDbg.print (h / 10);
		SerialDbg.print (".");
		SerialDbg.print (h % 10);
		SerialDbg.println (" %\t");

		SerialDbg.print ("oC \t");
		SerialDbg.print (t / 10);
		SerialDbg.print (".");
		SerialDbg.print (t % 10);
		SerialDbg.println (" *C");

                dump_radio_status_to_serialport (radio.radioState ());

                radio.print ("H:");
                radio.print (h/10);
                radio.print (".");
                radio.print (h%10);

                radio.print (", T:");
                radio.print (t/10);
                radio.print (".");
                radio.println (t % 10);

                radio.flush ();

                dump_radio_status_to_serialport (radio.radioState ());
	}
}

void readSpeedControl ()
{
    //TODO(DZhon): Implement me.
}

void readRFCommand ()
{
    if (radio.available (true)) {
	if (radio.read (rxBuf)) {
	    Serial.print ("Received packet: ");
            Serial.println (rxBuf);

            //TODO(DZhon): Some parsing and reaction needed.
	}
    }
}

void dump_radio_status_to_serialport (uint8_t status)
{
	SerialDbg.print ("Enrf24 radio transceiver status: ");
	switch (status) {
		case ENRF24_STATE_NOTPRESENT:
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
			SerialDbg.println ("UNKNOWN STATUS CODE");
	}
}
