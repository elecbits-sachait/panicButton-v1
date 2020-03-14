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

#define ACK_LED			A2
#define BUTTON_PIN		3

uint32_t ref_debounce;		//reference time for denounce
bool debounced = true;		//debounced flag
bool triggered = false;		//triggered flag
uint8_t retry;				//retry count

bool time_out = true;				//time out flag
uint32_t time_wdt;			//time counter for watch dog timer

bool resleep;				//sleep immediate flag

uint32_t ref_send_data;		//reference time retry
uint8_t packet[4] = {'T', '\0'};	//data packet buffer
uint8_t addresses[][6] = {"D001B"};	//nrf address buffer
	
RF24 radio(10, 9);			



inline void enterSleepMode()
{
	start:
	PRR = (1<<PRTWI) | (1<<PRTIM2) | (1<<PRTIM0) | (1<<PRTIM1) | (1<<PRSPI) | (1<<PRUSART0) | (1<<PRADC);	//disable all paripharal
	MCUCR = (1<<BODS) | (1<<BODSE);		//brown out detection disable
	MCUCR = (1<<BODS);		//brown out detection disable
	sleep_cpu();	//enter sleep mode
	if (resleep)	//if resleep is true sleep again
	{
		resleep = false;
		goto start;
	}
	PRR = 0;
}

bool send_data()
{
	radio.powerUp();		//NRF power up
	digitalWrite(ACK_LED, HIGH); 
	delay(50);
	digitalWrite(ACK_LED, LOW);
	
	packet[0] = 'B';		//indicates device
	packet[1] = 'T';		//indicate trigger signal
	packet[2] = '\0';		//NULL
	
	radio.stopListening();	//stop resiving
	if (!radio.write(packet, sizeof(packet)))	//write data
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
	radio.powerUp();
	float bandgap, battery_voltage;
	uint16_t temp;
	ADCSRA |= (1<<ADEN);
	delay(50);
	Serial.print("battery = ");
	bandgap = analogRead(0b1110);
	battery_voltage = (1/bandgap)*1023.0;
	temp = battery_voltage * 100;
	packet[0] = 'B';
	packet[1] = 'B';
	*((uint16_t *)&packet[2]) = temp;
	Serial.println(*((uint16_t *)&packet[2]));
	radio.stopListening();
	radio.write(packet, sizeof(packet));
	ADCSRA &= ~(1<<ADEN);
}

unsigned char debug;

void setup() {
  Serial.begin(115200);
  
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  WDTCSR = (1<<WDIE) | (0b000111<<WDP0);
  
  Serial.println(WDTCSR, BIN);
  
  set_sleep_mode(0b010<<SM0); //power down sleep mode
  sleep_enable();	//enable sleep
	ADCSRA &= ~(1<<ADEN);
	pinMode(ACK_LED,OUTPUT);
	pinMode(BUTTON_PIN, INPUT_PULLUP);
	
	cli();
	EICRA |= (1<<ISC10);	// falling edge interrupt
	EIFR = 3;
	EIMSK |= 1<<INT1;		//external interrupt0 enable
	sei();
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_2MBPS);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.enableDynamicAck();
  
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1,addresses[0]);
  
  Serial.print("ChipConnected = ");
  Serial.println(radio.isChipConnected() ? ("true"):("false"));
  delay(100);
  radio.powerDown();
}



void loop() {
	
	
	if (time_out)
	{
		time_out = false;
		digitalWrite(ACK_LED, HIGH);
		delay(50);
		read_battery_voltage();
		digitalWrite(ACK_LED, LOW);
	}
	
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
	
	
}




ISR (INT1_vect)
{
	ref_debounce = millis();
	debounced = false;
}

ISR(WDT_vect)
{
	time_wdt++;
	if (time_wdt < (60*60/8)) resleep = true;
	else time_out = true, time_wdt = 0;
}