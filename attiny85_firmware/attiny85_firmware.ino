#include <avr/io.h>
#include "CapacitiveSensor.h"


#define LED_PIN 3
#define CS_SEND_PIN 1
#define CS_RECEIVE_PIN 2

#define CS_SMOOTHING 10
#define CS_THRESHOLD 2500
#define CS_ENABLED_DELAY 255

#define ON_DELAY_MINUTES 1

#define CHANGE_STATE_DELAY 100

#define OFF 0
#define ON 1


CapacitiveSensor cSensor = CapacitiveSensor(CS_SEND_PIN, CS_RECEIVE_PIN);

int cs_counter;
long cs_sum;
bool cs_enabled;

int state;

int change_state_counter;

volatile int watchdog_counter;



void cs_init()
{
  cSensor.set_CS_AutocaL_Millis(0xFFFFFFFF);

  cs_sum = 0;
  cs_counter = 0;
  cs_enabled = false;
}


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


void cs_read()
{  
  cs_enabled = false;
  
  cs_sum += cSensor.capacitiveSensor(30);
  cs_counter++;

  if(cs_counter < CS_SMOOTHING)
  {
    return;
  }

  if((cs_sum / CS_SMOOTHING) > CS_THRESHOLD)
  {
    cs_enabled = true;
  }

  cs_sum = 0;
  cs_counter = 0;
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

  cs_init();

  // ~ 10s
  setup_watchdog(9);

  state = ON;
  change_state_counter = CHANGE_STATE_DELAY+1;

  sei();
}


void loop()                    
{  
  cs_read();
  
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
