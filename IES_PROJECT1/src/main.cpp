#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include <avr/interrupt.h>
#include "usart.h"
#include "bit.h"

#define MAX 255
#define PWMOUT PD6

int setPrescaler_tc0(char option);
void set_tc0_mode(char mode);
void buzzBuzzer(int pulseTime, double volume); 

int main()
{
  while(1)
  {
    int tp = 500; // Pulse time in ms. pt is proportional to distance
    double strength = 0.05; // strength is proportional to volume
    buzzBuzzer(tp,strength);
  }
}

// Buzzes the Buzzer for a certain pulseTime, at a certain strength determined by volume.
void buzzBuzzer(int pulseTime, double volume)
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
