#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include <avr/interrupt.h>
#include "usart.h"
#include "bit.h"

#define MAX 255
#define PWMOUT PD6
#define UST PD2 // Ultrasonic sensor trigger pin
#define USE PD3 // Ultrasonic sensor echo pin
#define BTN1 PB5 // Button to turn system on and off
#define BTN2 PB4 // Button to adjust volume of system

int setPrescaler_tc0(char option);
void set_tc0_mode(char mode);
void buzzBuzzer(double pulseTime, double volume);
double getDistance();

volatile unsigned long numOV = 0;

int main()
{
  bitSet(DDRD, UST);
  bitClear(DDRD, USE);

  bitClear(DDRB, BTN1);
  bitSet(PORTB, BTN1);
  bitClear(DDRB, BTN2);
  bitSet(PORTB, BTN2);

  //test LED
  bitSet(DDRB, PB3);
  bitSet(PORTB, PB3);

  int debounceCounter = 0;
  int secondDebounceCounter = 0;

  while(1)
  {
    // double tp = 500; // Pulse time in ms. pt is proportional to distance
    // double strength = 0.01; // strength is proportional to volume
    // buzzBuzzer(tp,strength);

    if(!(bitRead(PINB, BTN1))) {
      _delay_ms(1);
      if(!(bitRead(PINB, BTN1))) {
        debounceCounter++;
        if(debounceCounter >= 20) {
          _delay_ms(50);
          bitInverse(PORTB, PB3);
          debounceCounter = 0;
        }
      } else {
        debounceCounter = 0;
      }
    }

    if(!(bitRead(PINB, BTN2))) {
      _delay_ms(1);
      if(!(bitRead(PINB, BTN2))) {
        secondDebounceCounter++;
        if(secondDebounceCounter >= 20) {
          // turn on / off the system.
          secondDebounceCounter = 0;
        }
      } else {
        secondDebounceCounter = 0;
      }
    }
  }
}

double getDistance() {

  long duration = 0;
  bitClear(PORTD, PORTD2);
  _delay_us(5);
  bitSet(PORTD, PORTD2);
  _delay_us(10);
  bitClear(PORTD, PORTD2);

  while(!(PIND & (1 << PIND3))) {
  }
  while((PIND & (1 << PIND3))) {
    duration = duration + 1;
  }

  double distance = (duration / 2);
  /*
    Currently in CMs but maybe we just change it so that it set OCR0A here?
                  Echo reading
      OCR0A =  ------------------- x MAX
                 Echo reading max

  */
  return distance;
}

// Buzzes the Buzzer for a certain pulseTime, at a certain strength determined by volume.
void buzzBuzzer(double pulseTime, double volume)
{
  set_tc0_mode(3); // Fast PWM MAX
  bitSet(TCCR0A,COM0A1); // Clear OC0A on compare match
  OCR0A = volume*(MAX/2); // Set OCR0A accordingly to volume(duty cycle)
  setPrescaler_tc0(3); // 64 -> 976.56Hz wave

  // Toggle output ON/OFF for certain pulseTime
  bitSet(DDRD,PWMOUT);
  _delay_ms(pulseTime);
  bitClear(DDRD,PWMOUT);
  _delay_ms(pulseTime);

  set_tc0_mode(0);
  setPrescaler_tc0(0);
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
