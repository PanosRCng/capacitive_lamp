#include <avr/io.h>
#include "CapacitiveSensor.h"


#define LED_PIN 3
#define CS_SEND_PIN 1
#define CS_RECEIVE_PIN 2

#define CS_THRESHOLD_CALIBRATION_DELAY 5000
#define CS_THRESHOLD_CALIBRATION 10
#define CS_SMOOTHING 3

#define CHANGE_STATE_DELAY 10

#define OFF 0
#define ON 1


CapacitiveSensor cSensor = CapacitiveSensor(CS_SEND_PIN, CS_RECEIVE_PIN);

long cs_threshold;
bool cs_enabled;

int state;
int change_state_counter;

volatile int watchdog_counter;




void setup_watchdog(int ii)
{
  byte bb;
  int ww;

  if(ii > 9)
  {
    ii = 9;
  }

  bb = ii & 7;

  if(ii > 7)
  {
    bb |= (1<<5);
  }

  bb |= (1<<WDCE);

  ww == bb;

  MCUSR &= ~(1<<WDRF);

  // start timed sequence
  WDTCR |= (1<<WDCE) | (1<<WDE);

  // set new watchdog timeout value
  WDTCR = bb;

  WDTCR |= _BV(WDIE);
}


void cs_init()
{
  cSensor.set_CS_AutocaL_Millis(0xFFFFFFFF);

  cs_threshold = 0;
  cs_enabled = false;

  cs_setup();
}


void cs_setup()
{
  delay(CS_THRESHOLD_CALIBRATION_DELAY);

  for(int i=0; i<CS_THRESHOLD_CALIBRATION; i++)
  { 
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    cs_threshold = (cs_threshold + cs_read()) / 2;
    digitalWrite(LED_PIN, LOW);
    delay(1000);
    cs_threshold = (cs_threshold + cs_read()) / 2;
  }

  cs_threshold = (cs_threshold * 2);
}


void cs_check_enabled()
{
  cs_enabled = false;

  if(cs_read() > cs_threshold)
  {
    cs_enabled = true;
  }
}


long cs_read()
{ 
  long cs_avg = 0;

  for(int i=0; i<=CS_SMOOTHING; i++)
  {
    cs_avg = (cs_avg + cSensor.capacitiveSensor(30)) / 2; 
  }
  
  return cs_avg;
}


bool change_state_disabled()
{
  if(change_state_counter < CHANGE_STATE_DELAY)
  {
    change_state_counter++;
    return true;
  }

  return false;
}


void on()
{ 
  digitalWrite(LED_PIN, HIGH);

  if(change_state_disabled())
  {
    return;
  }
                    // ~30min
  if(cs_enabled || (watchdog_counter > 180))
  {
    state = OFF;
    change_state_counter = 0;
    return;
  }
}


void off()
{
  digitalWrite(LED_PIN, LOW);

  if(change_state_disabled())
  {
    return;
  }
  
  if(cs_enabled)
  {
    state = ON;
    change_state_counter = 0;
    watchdog_counter = 0;
    return;
  }
}


void setup()                    
{     
  pinMode(LED_PIN, OUTPUT);

  // ~ 10s
  setup_watchdog(9);

  cs_init();

  state = ON;
  change_state_counter = CHANGE_STATE_DELAY+1;

  sei();
}


void loop()                    
{ 
  cs_check_enabled();
  
  switch(state)
  {
    case OFF:
      off();
      break;

    case ON:
      on();
      break;
  }
}


ISR(WDT_vect)
{  
  watchdog_counter++;
}