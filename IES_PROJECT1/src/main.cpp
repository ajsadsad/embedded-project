#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include <avr/interrupt.h>
#include "usart.h"
#include "bit.h"

#define MAX 255

#define BUZZER_PWM_OUT PD6 // PWM output for Buzzer  (~6)
#define LED_PWM_OUT PD5 // PWM output for LED        (~5) 
#define ONOFF_LIGHT PD7 // LED indicator for ON/OFF   (0)

#define UST PB5 // Ultrasonic sensor trigger pin     (13)
#define USE PB4 // Ultrasonic sensor echo pin        (12)
#define BTN1 PD3 // Button to turn system on and off (~3)
#define BTN2 PD2 // Button to adjust volume of system (2)
#define BTN1_PCINT PC5 //                         PCINT13 (No external connection required - internal Pin Change Interrupt)
#define BTN2_PCINT PD7 //                         PCINT23 (No external connection required - internal Pin Change Interrupt)

int setPrescaler_tc0(char option);
void set_tc0_mode(char mode);
void buzzBuzzer(double volume);
void blinkLED(double photoresInput);
void pwmController(int pulseTime, double buzzer_strength, double led_strength);
void buttonRead1();
void buttonRead2();
void OnOffStatusLight();

void init_adc();
void init_btns();
void init_ISRs();

uint16_t read_adc();
double getDistance();

/* for button reading */
bool btn1_status_old = 1; // Initialize as 1 as button is connected to a pull up pin
bool btn1_status;
bool btn2_status_old = 1; // Initialize to 1 as button is connected to a pull up pin
bool btn2_status;

/* for ISRs */
int led_on = 1;
int timer_on = 1;
int volume = 0;
volatile unsigned long numOV = 0;

/* ISRs */
ISR(TIMER0_OVF_vect) {
  numOV++;
}
ISR(PCINT1_vect){
  led_on = !led_on;
  timer_on = !timer_on;
}
ISR(PCINT2_vect){
  // led_on = !led_on; // Uncomment for debounce test, keep commented out for correct ON/OFF status.
  volume++;
  if(volume==3)
  {
    volume=0;
  }
}

int main()
{
  init_adc();
  init_btns();
  init_ISRs();
  sei();
  bitSet(DDRD,ONOFF_LIGHT); // Set LED ON/OFF light to OUTPUT
  bitSet(DDRB,UST);         // Set Sensor TRIG to OUTPUT
  bitClear(DDRB,USE);       // Set Sensor ECHO to INPUT

  while(1)
  {
    OnOffStatusLight(); // On/Off status light for testing button debouncing

    // Read buttons for press, and action if pressed (debounced)
    buttonRead1();
    buttonRead2();

    // Array of buzzer volumes button2 switches through
    double buzzer_strength[] = {0.01, 0.04, 0.3};             // strength is proportional to volume

    // Calculate distance and pulse time using Ultrasonic Sensor
    int pt = 2000 * getDistance();                            // Pulse time in ms. pt is proportional to distance
    double led_strength = (read_adc())/1023.0;                // strength is proportional to photoresistor value
    pwmController(pt, buzzer_strength[volume], led_strength); // volume according to button presses

    _delay_ms(5);
  }
}

void OnOffStatusLight()
{
  if (led_on)
  {
    bitSet(PORTD, ONOFF_LIGHT);
  }
  else if (!led_on)
  {
    bitClear(PORTD, ONOFF_LIGHT);
  }
}

/* Initializes registers for Interrupt use */
void init_ISRs()
{
  bitSet(TIMSK0, TOIE0);    // set timer 0 overflow
  
  // Initializes buttons and registers such that a button press triggers an Interrupt
  bitSet(DDRC, BTN1_PCINT); // Set PC5 as INPUT
  bitSet(PCICR, PCIE1);     // ENABLE Pin change interrupt
  bitSet(PCMSK1, PCINT13);  // ENABLE individual Pin Change Interrupt 13 - PC5 (A5)

  bitSet(DDRD, BTN2_PCINT); // Set PD7 as INPUT
  bitSet(PCICR, PCIE2);     // ENABLE Pin change interrupt
  bitSet(PCMSK2, PCINT23);  // ENABLE individual Pin Change Interrupt 23 - PD7 (7)
}

/* Reads button 1 and triggers respective interrupt if pressed */
void buttonRead1()
{
  btn1_status = bitRead(PIND, BTN1);    // Read button
  if (btn1_status != btn1_status_old)   // If change in button status
  {
    _delay_ms(10);                      // wait for 10ms
    btn1_status = bitRead(PIND, BTN1);  // read button again
    if (btn1_status != btn1_status_old) // if button has truly changed
    {
      btn1_status_old = btn1_status;
      if (btn1_status == 0)             // if button has grounded the pin
      {
        bitInverse(PORTC, BTN1_PCINT);  // Trigger Button 1 Pin Change Interrupt
      }
    }
  }
}

/* Reads button 2 and triggers respective interrupt if pressed */
void buttonRead2()
{
  btn2_status = bitRead(PIND, BTN2);    // Read button
  if (btn2_status != btn2_status_old)   // If change in button status
  {
    _delay_ms(10);                      // wait for 10ms
    btn2_status = bitRead(PIND, BTN2);  // read button again
    if (btn2_status != btn2_status_old) // if button has truly changed
    {
      btn2_status_old = btn2_status;
      if (btn2_status == 0)             // if button has grounded the pin
      {
        bitInverse(PORTD, BTN2_PCINT);  // Trigger Button 2 Pin Change Interrupt
      }
    }
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
  if(!timer_on)
  {
    set_tc0_mode(0);
    return;
  }

  set_tc0_mode(3); // Fast PWM MAX
  setPrescaler_tc0(3); // 64 -> 976.56Hz wave

  bitSet(TCCR0A,COM0A1); // Clear OC0A on compare match
  bitSet(TCCR0A,COM0B1); // Clear OC0B on compare match

  // with prescaler 64, an overflow would take (16e3/64)/256 = 0.97 ms
  // therefore, we can assume that each overflow is roughly 1 ms in this scenario
  if (numOV < pulseTime) {
    return;
  }
  // Toggle output ON/OFF for certain pulseTime
  buzzBuzzer(buzzer_strength);
  blinkLED(led_strength);

  if (numOV < pulseTime * 2) {
    return;
  }
  bitInverse(DDRD,LED_PWM_OUT);
  bitInverse(DDRD,BUZZER_PWM_OUT);

  numOV = 0;
}

// Buzzes the Buzzer for a certain pulseTime, at a certain strength determined by volume.
void buzzBuzzer(double volume)
{
  bitSet(DDRD,BUZZER_PWM_OUT);
  OCR0A = volume*(MAX/2); // Set OCR0A accordingly to volume(duty cycle)
}

void blinkLED(double photoresInput) {
  bitSet(DDRD,LED_PWM_OUT);
  double photores = 1000*photoresInput;

  // double scalar;
  // double low = 0.01; // when photores is 
  // double med = 0.1;
  // double high = 0.9;

  // photores = amount of light hitting photocell
  // photores ~0.99 when BRIGHT LIGHT
  // photores ~0.01 when LOW LIGHT
  
  if(photores > 500){
    OCR0B = 0.01*MAX;
  }
  // If med lighting - MEDIUM
  if((photores > 200) & (photores < 500)){
    OCR0B = 0.1*MAX;
  }
  // If bright lighting - DIM
  if(photores < 200){
    OCR0B = 0.9*MAX;
  }
  }

void init_adc() {
  // reference voltage
  bitSet(ADMUX, REFS0);
  bitClear(ADMUX, REFS1);

  // ADC prescaler: 1024
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
  bitSet(DDRD, UST);   // Set Ultrasonic Sensor TRIG pin as OUTPUT
  bitClear(DDRD, USE); // Set Ultrasonic Sensor ECHO pin as INPUT

  bitClear(DDRD, BTN1); // Set BTN1 pin as INPUT
  bitSet(PORTD, BTN1);  // Enable pull up for BTN1

  bitClear(DDRD, BTN2); // Set BTN2 pin as INPUT
  bitSet(PORTD, BTN2);  // Enable pull up for BTN2

  //interrupt for INT1
  bitSet(SREG, SREG_I);

}



/*

WIP

*/

/* need to figure out how to pass a register to a function, if possible */
// void buttonRead(register btn, bool btnStat, bool btnStatOld, register PCINTport, register PCINTreg)
// {
//   btnStat = bitRead(PIND,btn); // read button
//   if(btnStat != btnStatOld) // if change in button status
//   {
//     _delay_ms(10); // wait for 10ms
//     btnStat = bitRead(PIND,btn); // read button again
//     if(btnStat != btnStatOld) // if button has truly changed (emotional)
//     {
//       btnStat=btnStatOld;
//       if(btnStat==0) // if button is grounding the pin
//       {
//         bitInverse(PCINTport,PCINTreg);
//       }
//     }
//   }
// }