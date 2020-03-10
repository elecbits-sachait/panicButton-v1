/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */

#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

//Beginning of Auto generated function prototypes by Atmel Studio
//End of Auto generated function prototypes by Atmel Studio

#define ACK_LED			9
#define BUTTON_PIN		2

uint32_t ref_debounce;
bool debounced = true;
bool triggered = false;
uint8_t retry;

bool time_out;

uint32_t ref_send_data;
uint8_t packet[4] = {'T', '\0'};
uint8_t addresses[][6] = {"D000B"};
RF24 radio(8, 10);


void enterSleepMode()
{
	PRR = (1<<PRTWI) | (1<<PRTIM2) | (1<<PRTIM0) | (1<<PRTIM1) | (1<<PRSPI) | (1<<PRUSART0) | (1<<PRADC);
	MCUCR = (1<<BODS) | (1<<BODSE);		//brown out detection disable
	MCUCR = (1<<BODS);
	sleep_cpu();	//enter sleep mode
	PRR = 0;
}

bool send_data()
{
	radio.powerUp();
	digitalWrite(ACK_LED, HIGH);
	delay(50);
	digitalWrite(ACK_LED, LOW);
	
	radio.stopListening();
	if (!radio.write(packet, sizeof(packet)))
	{
		radio.powerDown();
		Serial.println("failed");
		return false;
	}
	radio.stopListening();
	radio.powerDown();
	delay(200);
	digitalWrite(ACK_LED, HIGH);
	delay(50);
	digitalWrite(ACK_LED, LOW);
	
	Serial.println("sent");
	return true;
	
}

void read_battery_voltage()
{
	
}

unsigned char debug;

void setup() {
  Serial.begin(115200);
  
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  WDTCSR = (1<<WDIE) | (0b111<<WDP0);
  
  Serial.println(WDTCSR, BIN);
  
  set_sleep_mode(0b010<<SM0); //power down sleep mode
  sleep_enable();	//enable sleep
	ADCSRA &= ~(1<<ADEN);
	pinMode(ACK_LED,OUTPUT);
	pinMode(BUTTON_PIN, INPUT_PULLUP);
	
	cli();
	EICRA |= (1<<ISC00);	// falling edge interrupt
	EIFR = 3;
	EIMSK |= 1<<INT0;		//external interrupt0 enable
	sei();
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_2MBPS);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.enableDynamicAck();
  radio.setChannel(50);
  
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1,addresses[0]);
  pinMode(A3,OUTPUT);
  
  Serial.print("ChipConnected = ");
  Serial.println(radio.isChipConnected() ? ("true"):("false"));
  digitalWrite(A3,LOW);
  delay(100);
  radio.powerDown();
}



void loop() {
	
	if (!debounced)
	{
		if (millis() - ref_debounce >= 10)
		{
			debounced = true;
			if (!digitalRead(BUTTON_PIN))
			{
				if (!triggered)
				{
					retry = 0;
					ref_send_data = millis();
					triggered = !send_data();
				}
			}
		}
	}
	
	if (triggered)
	{
		if (millis() - ref_send_data >= 2000)
		{
			ref_send_data = millis();
			if (send_data() || ++retry >= 50)
				triggered = false;
		}
	}
	else if (debounced)
	{
		Serial.println("entering sleep");
		delay(100);
		enterSleepMode();
	}
	
	if (time_out)
	{
		ADCSRA |= (1<<ADEN);
		time_out = false;
		digitalWrite(ACK_LED, HIGH);
		delay(50);
		digitalWrite(ACK_LED, LOW);
		Serial.println("WDT time out");
		Serial.print("band gap = ");
		Serial.println(analogRead(0b1110));
		ADCSRA &= ~(1<<ADEN);
	}
}




ISR (INT0_vect)
{
	ref_debounce = millis();
	debounced = false;
}

ISR(WDT_vect)
{
	time_out = true;
}