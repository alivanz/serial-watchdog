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
  int on_timeout;
  void (*cb_init)(void);
  void (*cb_timeout)(void);
} states[] = {
  {3, 1, release_all, NULL},
  {3, 0, power_push, NULL}
};
void toState(const int i){
  state = i;
  t = 0;
  if(states[state].cb_init != NULL) states[state].cb_init();
}

/* INTERRUPTs */
ISR(TIMER1_COMPA_vect){
  t += 1;
  if(states[state].timeout == 0) return;
  if(states[state].timeout <= t){
    if(states[state].cb_timeout != NULL) states[state].cb_timeout();
    if(states[state].on_timeout != -1  ) toState(states[state].on_timeout);
  }
}

int main(){
  /* PINs */
  DDRC = 0x03;
  /* Timer */
  TCCR1B  = (1<<WGM12);           /* CTC */
  TCCR1B |= (1<<CS12)|(1<<CS10);  /* prescaler 1024 */
  OCR1A   = 15625;                /* 1 sec */
  TIMSK1 |= 1<<OCIE1A;            /* Enable timer interrupt */
  /* Init state */
  toState(0);
  /* Set Enable Interrupt */
  sei();
  while(1);
}
