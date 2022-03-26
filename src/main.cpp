#include <Arduino.h>

#define _DEBUG 0

#define PRINT(x) if (_DEBUG) Serial.println(x);

#define TOGGLE_THRESHOLD 5
#define BUFFER_SIZE 16



volatile uint16_t t_curr = 0;
uint16_t buffer[BUFFER_SIZE];
uint8_t index = 0;
uint16_t calib_avg;

void touch_timer_init()
{
  TCCR4A=0;
  TCCR4A = _BV(WGM40);
  TCCR4B=0;
  TCCR4B = _BV(CS40)|_BV(CS42)|_BV(WGM42);
  TCNT4 = 0;
  OCR4A = 192;
  TIMSK4 = _BV(OCIE4A)|_BV(TOIE4);
}
void led_pwm_timer_init()
{
  TCCR5A=0;
  TCCR5A = _BV(WGM50);
  TCCR5B=0;
  TCCR5B = _BV(CS50)|_BV(CS52)|_BV(WGM52);
  TCNT5 = 0;
  OCR5A = 192;
  TIMSK5 = _BV(OCIE5A)|_BV(TOIE5);
}
void comparator_init()
{
  ACSR = (0 << ACD) | (1 << ACBG) | (0 << ACO) | (1 << ACI) | (1 << ACIE) | (0 << ACIC) | (1 << ACIS1) | (1 << ACIS0);
}

// void ext_int_init()
// {
//   DDRE &= ~(1<<DDE5); //(PORTE-n) van soros kommunikáció és problémát okoz
//   PORTE &= ~(1<<PB5);
//   EICRB &= ~(_BV(ISC51)|_BV(ISC50));
//   EIMSK |= _BV(INT5);
// }

void init_buffer()
{
  for (int i = 0; i < BUFFER_SIZE; i++)
  {
    buffer[i] = 0;
  }
}

void update_buffer()
{
  buffer[index] = t_curr;
  index++;
  index = index % BUFFER_SIZE;
}

uint16_t get_avg()
{
  uint16_t avg = 0;
  for (uint8_t i = 0; i < BUFFER_SIZE; i++)
  {
    avg += buffer[i];
  }
  PRINT(avg / BUFFER_SIZE);
  return avg / BUFFER_SIZE;
}

int main(void)
{
  DDRB = _BV(DDB6)|(0<<DDB5);
  PORTB = _BV(PB6);
  DDRH = _BV(DDH6);
  PORTH = (0<<PH6);

  uint8_t counter = 0;
  bool pressed_before = false;

  sei(); //enable interrupts
  
  touch_timer_init();
  comparator_init();

  init_buffer();

  if (_DEBUG == 1) Serial.begin(9600); 
  
  while(true)
  {
    //Calibrtion polling
    if(PINB & _BV(PINB5))
    {
      while(counter != 16)
      {
        if(~ACSR & _BV(ACI))
        {
          calib_avg += t_curr;
          counter++;
        }
        if(counter == 16)
        {
          calib_avg = (calib_avg>>4);
        } 
      }
      _delay_ms(1000);
    }

   // get_avg();

    // Control and debouncing
    if(get_avg() > calib_avg + TOGGLE_THRESHOLD)
    {
      if(!pressed_before)
      {
        PORTB ^= _BV(PB6);
        pressed_before = true;
      }
    }
    else
    {
      pressed_before = false;
    }
  }
  return 0;
}

ISR(TIMER4_COMPA_vect)
{
  DDRH = _BV(DDH6);
  PORTH = _BV(PH6);
}

ISR(TIMER4_OVF_vect)
{
  DDRH &= ~_BV(DDH6);
  PORTH &= ~_BV(PH6);
}

ISR(ANALOG_COMP_vect)
{
  t_curr = TCNT4;
  update_buffer();
  //Serial.println(t_curr);
}