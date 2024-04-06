#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include <avr/interrupt.h>
#include "usart.h"
#include "bit.h"

#define MAX 255

#define BUZZER_PWM_OUT PD6
#define LED_PWM_OUT PD5

#define UST PB5 // Ultrasonic sensor trigger pin
#define USE PB4 // Ultrasonic sensor echo pin
#define BTN1 PD3 // Button to turn system on and off
#define BTN2 PD2 // Button to adjust volume of system

int setPrescaler_tc0(char option);
void set_tc0_mode(char mode);
void buzzBuzzer(double volume);
void blinkLED(double photores);
void pwmController(int pulseTime, double buzzer_strength, double led_strength);

void init_adc();
void init_btns();

uint16_t read_adc();
double getDistance();

volatile bool btn1_status_old = 1;
volatile bool btn1_status;

volatile bool btn2_status_old = 1;
volatile bool btn2_status;

volatile char num_btn2_presses = 0;

ISR(INT1_vect) {
  btn1_status = bitRead(PIND, BTN1);

  if(btn1_status != btn1_status_old) {
    _delay_ms(10);
    btn1_status = bitRead(PIND, BTN1);
    if(btn1_status != btn1_status_old) {
      btn1_status_old = btn1_status;
      if(btn1_status == 0) {
        bitInverse(PORTB, PB3);
        btn1_status_old = 1;
      }
    }
  }
}

ISR(INT0_vect) {
  btn2_status = bitRead(PIND, BTN2);

  if(btn2_status != btn2_status_old) {
    _delay_ms(10);
    btn2_status = bitRead(PIND, BTN2);
    if(btn2_status != btn2_status_old) {
      btn2_status_old = btn2_status;
      if(btn2_status == 0) {
        if(num_btn2_presses == 2) {
          bitInverse(PORTB, PB3);
          btn2_status_old = 1;
          num_btn2_presses = 0;
        } else {
          btn2_status_old = 1;
          num_btn2_presses++;
        }
      }
    }
  }
}

int main()
{
  init_adc();
  init_btns();

  //test LED
  bitSet(DDRB, PB3);
  bitSet(PORTB, PB3);

  //set sensor
  bitSet(DDRB, PB5);
  bitClear(DDRB, PB4);

  sei();

  usart_init(103);
  while(1)
  {
    double distance = getDistance();

    int pt = 2000 * distance; // Pulse time in ms. pt is proportional to distance
    double buzzer_strength = 0.01; // strength is proportional to volume
    double led_strength = (read_adc())/1000.0; // strength is proportional to photoresistor value
    pwmController(pt, buzzer_strength, led_strength);
  }
}

double getDistance() {

  long duration = 0;
  bitClear(PORTB, UST);
  _delay_us(5);
  bitSet(PORTB, UST);
  _delay_us(10);
  bitClear(PORTB, UST);

  while(!(PINB & (1 << USE))) {
  }
  while((PINB & (1 << USE))) {
    duration = duration + 1;
  }

  double distance = ((duration/2) / 29.1) / 400;
  return distance;
}

/*
 * Alternates between controlling LED and buzzer using TC0's Fast PWM max mode. Alternation is necessary to allow for independent control over the duty cycle of the LED as well as buzzer
 *
 * pulseTime: the duration of each pulse for either led or buzzer
 * buzzer_strength: percentage of duty cycle for the buzzer -- 1.0: 50% duty, 0.0: 0% duty (off)
 * led_strength: percentage of duty cycle for the LED -- 1.0: 50% duty, 0.0: 0% duty (off)
 * */
void pwmController(int pulseTime, double buzzer_strength, double led_strength) {
  set_tc0_mode(3); // Fast PWM MAX
  setPrescaler_tc0(3); // 64 -> 976.56Hz wave

  bitSet(TCCR0A,COM0A1); // Clear OC0A on compare match
  bitSet(TCCR0A,COM0B1); // Clear OC0B on compare match

  // Toggle output ON/OFF for certain pulseTime
  buzzBuzzer(buzzer_strength);
  blinkLED(led_strength);
  int j = 0;
  while(j < pulseTime) {
    _delay_ms(1);
    j++;
  }

  bitClear(DDRD,LED_PWM_OUT);
  bitClear(DDRD,BUZZER_PWM_OUT);
  int i = 0;
  while(i < pulseTime) {
    _delay_ms(1);
    i++;
  }
}

// Buzzes the Buzzer for a certain pulseTime, at a certain strength determined by volume.
void buzzBuzzer(double volume)
{
  bitSet(DDRD,BUZZER_PWM_OUT);
  OCR0A = volume*(MAX/2); // Set OCR0A accordingly to volume(duty cycle)
}

void blinkLED(double photores) {
  bitSet(DDRD,LED_PWM_OUT);
  OCR0B = photores*(MAX/2); // Set OCR0A accordingly to volume(duty cycle)
}

void init_adc() {
  // reference voltage
  bitSet(ADMUX, REFS0);
  bitClear(ADMUX, REFS1);

  // ADC prescaler
  bitSet(ADCSRA, ADPS2);
  bitSet(ADCSRA, ADPS1);
  bitSet(ADCSRA, ADPS0);

  // ADC input hcannel
  ADMUX &= 0xF0; // A0
}

uint16_t read_adc() {
  bitSet(ADCSRA, ADEN);
  bitSet(ADCSRA, ADSC);

  while (ADCSRA & (1 << ADSC));
  uint16_t result = ADCL;
  result |= (ADCH << 8);


  return result;
}

int setPrescaler_tc0(char option)
{
  /* clear all CS0n */
  TCCR0B &= 0b11111000; // Clear bits 2:0

  /* set CS0n accordingly */
  TCCR0B |= (option - '0');

  /* return prescaler value */
  int prescalers[] = {0,1,8,64,256,1024};
  return prescalers[option - '0'];
}

void set_tc0_mode(char mode)
{
  /* clear all WGM0n */
  TCCR0A &= 0b11111100; // Clear bits 1:0
  TCCR0B &= 0b11110111; // Clear bit 3

  /* set WGM0n accordingly */
  TCCR0A |= (0b00000011 & (mode - '0'));
  TCCR0B |= (0b00001000 & ((mode - '0') << 1));
}

void init_btns() {
  bitSet(DDRD, UST);
  bitClear(DDRD, USE);

  bitClear(DDRD, BTN1);
  bitSet(PORTD, BTN1);

  bitClear(DDRD, BTN2);
  bitSet(PORTD, BTN2);

  //interrupt for INT1
  bitSet(SREG, SREG_I);
  bitSet(EICRA, ISC11);
  bitClear(EICRA, ISC10);
  bitSet(EIMSK, INT1);
  //Interrupt for INT0
  bitSet(EICRA, ISC01);
  bitClear(EICRA, ISC00);
  bitSet(EIMSK, INT0);
}
