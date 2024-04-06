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

int setPrescaler_tc0(char option);
void set_tc0_mode(char mode);
void buzzBuzzer(double volume); 
void blinkLED(double photores);
void pwmController(int pulseTime, double buzzer_strength, double led_strength);

int main()
{
  while(1)
  {
    int tp = 500; // Pulse time in ms. pt is proportional to distance
    double buzzer_strength = 1; // strength is proportional to volume
    double led_strength = 1; // strength is proportional to photoresistor value
    
    pwmController(tp, buzzer_strength, led_strength);
  }
}

/*
 * 
 * */
void pwmController(int pulseTime, double buzzer_strength, double led_strength) {
  set_tc0_mode(3); // Fast PWM MAX
  setPrescaler_tc0(3); // 64 -> 976.56Hz wave
  
  // Toggle output ON/OFF for certain pulseTime
  buzzBuzzer(1.0);
  _delay_ms(pulseTime);


  blinkLED(1.0);
  _delay_ms(pulseTime);
}

// Buzzes the Buzzer for a certain pulseTime, at a certain strength determined by volume.
void buzzBuzzer(double volume)
{
  bitClear(DDRD,LED_PWM_OUT);
  bitSet(DDRD,BUZZER_PWM_OUT);
  bitSet(TCCR0A,COM0A1); // Clear OC0A on compare match
  OCR0A = volume*(MAX/2); // Set OCR0A accordingly to volume(duty cycle)
}

void blinkLED(double photores) {
  bitClear(DDRD,BUZZER_PWM_OUT);
  bitSet(DDRD,LED_PWM_OUT);
  bitSet(TCCR0A,COM0B1); // Clear OC0B on compare match
  bitClear(TCCR0A,COM0B0); // Clear OC0B on compare match
  OCR0A = photores*(MAX/2); // Set OCR0A accordingly to volume(duty cycle)
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
