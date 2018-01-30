/*
 * ServoTester_bd - provide servo PWM output controlled by a knob
 * - typical motor controllers [UPDATE TO LIST] use 1-2 msec pulse width
 * - knob controls from 900 usec to 2100 usec
 * - red and green leds mimic motor controllers:
 * -- flash ~ 2Hz for just starting, ~10 Hz almost full,
 *    continuous for <=1000 usec and >=2000 usec respectively
 * - yellow led indicates centered (+/- 20 usec for now)
 *
 * created 1/17/16 BD from ServoTester by trandi (trandi 15 June 2013)
 * - he put in heavy filtering due to noisey signal from pot
 * updated 1/18/16
 * - change limits to 900, 2100, from 600, 2400
 * - add code for leds
 * updated 1/19/16
 * - initialize potValue with call to getCleanReading() so that it starts
 * with the correct output
 * - remove IIR filter on potValue
 * - delay on startup to let A/D input settle to correct value
 * updated 1/21/16
 * - add sleep code
 * - measuring ~300uA, not incl pot (500uA with breadboard pot)
 * - build pot is 10k, 500uA, so about 800uA total
 * - sleep_bod_disable() won't compile, would save 20uA
 * - power_aca_disable() won't compile, would save 35uA
 * updated 1/29/18
 * - invert outputs:
 * for pcb's, LEDs connected to VCC, with pin pulling down
 */

#include <SoftwareServo.h>

// added for sleep mode and to manipulate port pins
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/io.h>

#define PIN_SERVO 0
#define PIN_POT 2 // ADC#, not PB#
#define GREEN_LED 1
#define RED_LED 2
#define YELLOW_LED 3
#define PULSE_MIN 900 // in microsecs
#define PULSE_MAX 2100 // in microsecs
#define SOFT_MIN 1000
#define SOFT_MAX 2000
#define DEAD_MIN 1480
#define DEAD_MAX 1520
#define READINGS_COUNT 10
// time until sleep in usec
// test - 30 seconds
// #define SLEEP_COUNT 30000000
// real - 10 minutes (600 seconds)
#define SLEEP_COUNT 600000000

SoftwareServo servo;
long potValue = 0;
int readings[READINGS_COUNT] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte readingsPos = 0;
unsigned long nextReadingDue = 0;
unsigned long ledFlashChange = 0;
int servoPulse = 0; // pulsewidth in usec
unsigned long nextSleep = 0;

void setup(){
  servo.attach(PIN_SERVO);
  servo.setMinimumPulse(PULSE_MIN);
  servo.setMaximumPulse(PULSE_MAX);

  pinMode(GREEN_LED, OUTPUT);        
  pinMode(RED_LED, OUTPUT);        
  pinMode(YELLOW_LED, OUTPUT);        

  digitalWrite(GREEN_LED, 0);
  digitalWrite(RED_LED, 0);
  digitalWrite(YELLOW_LED, 0);
  
  delay(200);
  
  digitalWrite(GREEN_LED, 1);
  digitalWrite(RED_LED, 1);
  digitalWrite(YELLOW_LED, 1);
  
  // Initialize potValue so that output will start at the correct value
  potValue = getCleanReading();
  
  nextSleep = micros() + SLEEP_COUNT;
}

void loop(){
  // read the analog pin once every millisec, and store the data until we have enough to filter
  unsigned long currentTime = micros();
  if(currentTime > nextSleep){
    sleepNow();
  }

  if(currentTime > nextReadingDue){
    readings[readingsPos] = analogRead(PIN_POT);
    readingsPos ++;

    nextReadingDue = currentTime + 1000;
  }

  // once every READINGS_COUNT readings, clean the data and update the servo
  // also filter using weighted moving average to avoid excessive jittering due to poor potentiometer
  if(readingsPos > READINGS_COUNT){
    // potValue = (9 * potValue + getCleanReading())/ 10;
    potValue = getCleanReading();

    servo.write(map(potValue, 0, 1023, 0, 180));
    servoPulse = map(potValue, 0, 1023, PULSE_MIN, PULSE_MAX);  

    readingsPos = 0;
  }

  if(currentTime > ledFlashChange){
    // Time to change led
    // use ledFlashChange to hold delta time, then update at the end of the if's 
    if(servoPulse <= SOFT_MIN) {
      // Solid RED
      digitalWrite(GREEN_LED, 1);
      digitalWrite(RED_LED, 0);
      digitalWrite(YELLOW_LED, 1);
      ledFlashChange = 50000;
    } 
    else if(servoPulse < DEAD_MIN) {
      // Flashing RED
      digitalWrite(GREEN_LED, 1);
      digitalWrite(RED_LED, !digitalRead(RED_LED));
      digitalWrite(YELLOW_LED, 1);
      ledFlashChange = map(servoPulse, SOFT_MIN, DEAD_MIN, 50000, 250000);
    } 
    else if(servoPulse <= DEAD_MAX) {
      // Solid Yellow
      digitalWrite(GREEN_LED, 1);
      digitalWrite(RED_LED, 1);
      digitalWrite(YELLOW_LED, 0);
      ledFlashChange = 50000;
    } 
    else if(servoPulse < SOFT_MAX) {
      // Flashing Green
      digitalWrite(GREEN_LED, !digitalRead(GREEN_LED));
      digitalWrite(RED_LED, 1);
      digitalWrite(YELLOW_LED, 1);
      ledFlashChange = map(servoPulse, DEAD_MAX, SOFT_MAX, 250000, 50000);
    } 
    else {
      // Solid Green
      digitalWrite(GREEN_LED, 0);
      digitalWrite(RED_LED, 1);
      digitalWrite(YELLOW_LED, 1);
      ledFlashChange = 50000;
    }
    // update ledFlashChange to actual time
    ledFlashChange += currentTime;
  }

  // good to call this as often as we can, so that we don't miss the moment when the signal has to go up or down
  SoftwareServo::refresh();
}

// reads several analog values and tries to do some smart filtering
int getCleanReading(){
  int reading, minReading=9999, maxReading=0;
  long result = 0;

  for(byte i=0; i<READINGS_COUNT; i++){
    readings[i] = analogRead(PIN_POT);
    result += readings[i];

    if(readings[i] < minReading){
      minReading = readings[i];
    }
    if(readings[i] > maxReading){
      maxReading = readings[i];
    }
  }

  // reaturn the average after eliminating min and max readings
  return (result - minReading - maxReading) / (READINGS_COUNT - 2);
}

void sleepNow()
{
  // Choose our preferred sleep mode:
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
 
  // Set sleep enable (SE) bit:
  sleep_enable();
 
  // indicate to user
  digitalWrite(GREEN_LED, 0);
  digitalWrite(RED_LED, 0);
  digitalWrite(YELLOW_LED, 0);
  delay(500);
  digitalWrite(GREEN_LED, 1);
  digitalWrite(RED_LED, 1);
  digitalWrite(YELLOW_LED, 1);

  // start shutting down peripherals
  servo.detach();
  power_adc_disable();
  power_timer0_disable();
  power_timer1_disable();
  power_usi_disable();
  // hmmm.  Following line does not compile
  // sleep_bod_disable();
  
  // Put the device to sleep:
  sleep_mode();
 
  // Upon waking up, sketch continues from this point.
  sleep_disable();
}


