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
void init_adc();
uint16_t read_adc();

int main()
{
  init_adc();
  usart_init(8);

  while(1)
  {
    int tp = 500; // Pulse time in ms. pt is proportional to distance
    double buzzer_strength = 0.01; // strength is proportional to volume
    double led_strength = (read_adc())/1000.0; // strength is proportional to photoresistor value
    
    pwmController(tp, buzzer_strength, led_strength);
  }
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
  _delay_ms(pulseTime);


  blinkLED(led_strength);
  _delay_ms(pulseTime);
}

// Buzzes the Buzzer for a certain pulseTime, at a certain strength determined by volume.
void buzzBuzzer(double volume)
{
  bitClear(DDRD,LED_PWM_OUT);
  bitSet(DDRD,BUZZER_PWM_OUT);
  OCR0A = volume*(MAX/2); // Set OCR0A accordingly to volume(duty cycle)
}

void blinkLED(double photores) {
  bitClear(DDRD,BUZZER_PWM_OUT);
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
