// Includes

#include <SPI.h>
#include <Enrf24.h>
#include <nRF24L01.h>
#include <DHT22_430.h>
#include <Servo.h>
#include <EEPROM.h>

// PE_1 = CE
// PE_2 = CSN
// PE_3 = IRQ

// SPI: SCK = PB_4, MOSI = PB_7, MISO = PB_6
Enrf24 radio (PE_1, PE_2, PE_3);
Servo servo;

DHT22 dht (PE_4);
boolean dht_flag;

const uint8_t txaddr[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x01 };
const uint8_t rxaddr[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x02 };

const char* str_on = "ON";
const char* str_off = "OFF";

void dump_radio_status_to_serialport (uint8_t);

void setup()
{
	Serial.begin (9600);

	SPI.setModule (3);
	SPI.begin ();
	SPI.setClockDivider (SPI_CLOCK_DIV8);
	SPI.setDataMode (SPI_MODE0);
	SPI.setBitOrder (MSBFIRST);

	radio.begin ();
	dump_radio_status_to_serialport (radio.radioState ());

	radio.setTXaddress ( (void*) txaddr);
	radio.setRXaddress ( (void*) rxaddr);

	dump_radio_status_to_serialport (radio.radioState ());

	radio.enableRX ();

	dump_radio_status_to_serialport (radio.radioState ());

	servo.attach (PE_5);

	dht.begin ();
}

void loop()
{
	char inbuf[33];

	if (Serial.available () > 0) {
		Serial.print ("Sending packet ... ");

		int8_t byte = Serial.read ();
		radio.print (str_on);
		radio.flush ();

		dump_radio_status_to_serialport (radio.radioState()); // Should report IDLE
	}

	if (radio.available (true)) {
		if (radio.read (inbuf)) {
			Serial.print ("Received packet: ");
			Serial.println (inbuf);
		}
	}

	dht_flag = dht.get ();
	int32_t h = dht.humidityX10 ();
	int32_t t = dht.temperatureX10 ();

	if (!dht_flag) {
		Serial.println ("Failed to read from DHT22");
	} else {
		Serial.print ("RH% \t");
		Serial.print (h / 10);
		Serial.print (".");
		Serial.print (h % 10);
		Serial.println (" %\t");

		Serial.print ("oC \t");
		Serial.print (t / 10);
		Serial.print (".");
		Serial.print (t % 10);
		Serial.println (" *C");
	}
}

void dump_radio_status_to_serialport (uint8_t status)
{
	Serial.print ("Enrf24 radio transceiver status: ");
	switch (status) {
		case ENRF24_STATE_NOTPRESENT:
			Serial.println ("NO TRANSCEIVER PRESENT");
			break;

		case ENRF24_STATE_DEEPSLEEP:
			Serial.println ("DEEP SLEEP <1uA power consumption");
			break;

		case ENRF24_STATE_IDLE:
			Serial.println ("IDLE module powered up w/ oscillators running");
			break;

		case ENRF24_STATE_PTX:
			Serial.println ("Actively Transmitting");
			break;

		case ENRF24_STATE_PRX:
			Serial.println ("Receive Mode");
			break;

		default:
			Serial.println ("UNKNOWN STATUS CODE");
	}
}
