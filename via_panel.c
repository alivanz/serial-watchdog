#include <avr/io.h>

/* Functions */
void release_all(void){
  PORTC = 0;
}
void power_push(void){
  PORTC |= 0x01;
}

/* TIMER */
volatile int t;
void reset_timer(void){
  t = 0;
  TCNT1 = 0;
}

/* STATEs */
volatile int state;
struct TState{
  int timeout;
  int strobe_INT0;
  int on_timeout;
  int on_INT0;
  void (*cb_init)(void);
  void (*cb_timeout)(void);
  void (*cb_INT0)(void);
} states[] = {
  {0, 2,    1, 1,     release_all, NULL, NULL},
  {3, 0,    0, 0,     power_push, NULL, NULL}
};
void toState(const int i){
  state = i;
  t = 0;
  if(states[state].cb_init != NULL) states[state].cb_init();
}

/* INTERRUPTs */
#define INT0_value (PIND & 4)
ISR(TIMER1_COMPA_vect){
  t += 1;
  if(states[state].timeout != 0)
  if(states[state].timeout <= t){
    if(states[state].cb_timeout != NULL) states[state].cb_timeout();
    if(states[state].on_timeout != -1  ) toState(states[state].on_timeout);
  }
  /* Blink */
  PORTB |= 0x10;
  _delay_ms(100);
  PORTB &= 0xdf;
}
ISR(INT0_vect){
  if((states[state].strobe_INT0 == 1) && (INT0_value == 1)){
    if(states[state].cb_INT0 != NULL) states[state].cb_INT0();
    if(states[state].on_INT0 != -1  ) toState(states[state].on_INT0);
  }else if((states[state].strobe_INT0 == 2) && (INT0_value == 0)){
    if(states[state].cb_INT0 != NULL) states[state].cb_INT0();
    if(states[state].on_INT0 != -1  ) toState(states[state].on_INT0);
  }
  TCNT1 = 0;
}

int main(){
  /* PINs */
  DDRC = 0x03;
  DDRD = 0x00;
  PORTD= 0x04; /* pull up INT0 */
  /* Blink */
  DDRB = 0x10;
  PORTB = 0x00;
  /* Timer */
  TCCR1B  = (1<<WGM12);           /* CTC */
  TCCR1B |= (1<<CS12)|(1<<CS10);  /* prescaler 1024 */
  OCR1A   = 15625;                /* 1 sec */
  TIMSK1 |= 1<<OCIE1A;            /* Enable timer interrupt */
  /* INTERRUPT */
  EICRA = (1<<ISC00);  /* Any change */
  EIMSK = (1<<INT0);   /* Enable interrupt */
  /* Init state */
  toState(0);
  /* Set Enable Interrupt */
  sei();
  while(1);
}
