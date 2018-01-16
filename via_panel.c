#include <avr/io.h>

void serial_write(const char *);
int SerialTXQueue();

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
  const char *name;
  int timeout;
  int strobe_INT0;
  int strobe_INT1;
  int on_timeout;
  int on_INT0;
  int on_INT1;
  void (*cb_init)(void);
  void (*cb_timeout)(void);
  void (*cb_INT0)(void);
  void (*cb_INT1)(void);
} states[] = {
  {"init",            5, 1, 2,    2, 3, 0,    release_all, NULL, NULL, NULL},
  {"off",             0, 1, 2,    0, 3, 0,    release_all, NULL, NULL, NULL},
  {"power_up",        0, 1, 0,    0, 3, 0,    power_push, NULL, release_all, NULL},
  {"on",              0, 2, 0,    0, 1, 0,    NULL, NULL, NULL, NULL},

  {"on_active",       10, 2, 0,   6, 5, 0,    NULL, NULL, NULL, NULL},
  {"force_shutdown",  0, 2, 0,    0, 1, 0,    power_push, NULL, release_all, NULL},
  {"hard_reset",      0, 2, 0,    0, 0, 0,    power_push, NULL, release_all, NULL}
};
void toState(const int i){
  state = i;
  t = 0;
  if(states[state].cb_init != NULL) states[state].cb_init();
  serial_write(states[state].name);
}

/* INTERRUPTs */
#define INT0_value (PIND & 4)
#define INT1_value (PIND & 8)
ISR(TIMER1_COMPA_vect){
  t += 1;
  if(states[state].timeout != 0)
  if(states[state].timeout <= t){
    if(states[state].cb_timeout != NULL) states[state].cb_timeout();
    if(states[state].on_timeout != -1  ) toState(states[state].on_timeout);
  }
  /* Blink */
  PORTB |= 0x20;
  _delay_ms(100);
  PORTB &= 0xdf;
}
ISR(INT0_vect){
  if((states[state].strobe_INT0 == 1) && (INT0_value)){
    if(states[state].cb_INT0 != NULL) states[state].cb_INT0();
    if(states[state].on_INT0 != -1  ) toState(states[state].on_INT0);
  }else if((states[state].strobe_INT0 == 2) && (!INT0_value)){
    if(states[state].cb_INT0 != NULL) states[state].cb_INT0();
    if(states[state].on_INT0 != -1  ) toState(states[state].on_INT0);
  }
  TCNT1 = 0;
}
ISR(INT1_vect){
  if((states[state].strobe_INT1 & 1) && (INT1_value)){
    if(states[state].cb_INT1 != NULL) states[state].cb_INT1();
    if(states[state].on_INT1 != -1  ) toState(states[state].on_INT1);
  }else if((states[state].strobe_INT1 & 2) && (!INT1_value)){
    if(states[state].cb_INT1 != NULL) states[state].cb_INT1();
    if(states[state].on_INT1 != -1  ) toState(states[state].on_INT1);
  }
  TCNT1 = 0;
}

/* COMMANDS */
char * get_state(void){
  return (char*)states[state].name;
}
char * touch(void){
  if(state<3 | state>4) return (char*)"not on state";
  toState(4);
  return (char*)"touched!";
}
char * force_shutdown(void){
  if(state<3 | state>4) return (char*)"not on state";
  toState(5);
  return NULL;
}
char * hard_reset(void){
  if(state<3 | state>4) return (char*)"not on state";
  toState(6);
  return NULL;
}
char * power_up(void){
  if(state>=3) return (char*)"not off state";
  toState(2);
  return NULL;
}
struct TCommand{
  char *name;
  char *(*fx)(void);
} commands[] = {
  {(char*)"get_state", get_state},
  {(char*)"touch", touch},
  {(char*)"force_shutdown", force_shutdown},
  {(char*)"hard_reset", hard_reset},
  {(char*)"power_up", power_up},
  {NULL,NULL}
};

#define BAUD 9600
#define UBBR (F_CPU/16/BAUD-1)
volatile char* tx_active;
volatile char* tx_buffer[256];
volatile char tx_buffer_i;
volatile char tx_buffer_j;
char rx_buffer[64];
volatile int rx_buffer_i;
int SerialTXQueue(){
  return (UCSR0B & (1<<UDRIE0));
}
/* Non blocking serial write */
void serial_write(const char *s){
  if(tx_active==NULL){
    tx_active = (char*)s;
    UCSR0B |= 1<<UDRIE0; // Enable UDR Empty Interrupt
  }else{
    tx_buffer[tx_buffer_j++] = (char*)s;
  }
}
/* Serial interrupt for send char */
ISR(USART_UDRE_vect){
  char c = *(tx_active++);
  if(c == '\0'){
    UDR0 = '\n';
    if(tx_buffer_i != tx_buffer_j){
      tx_active = tx_buffer[tx_buffer_i++];
    }else{
      tx_active = NULL;
      UCSR0B &= 0xff-(1<<UDRIE0); // Disable UDR Empty Interrupt
    }
    return;
  }
  UDR0 = c;
}
ISR(USART_RX_vect){
  volatile char c = UDR0;
  if(c=='\n'){
    rx_buffer[rx_buffer_i] = '\0';
    struct TCommand *cmd = commands;
    while(cmd->name!=NULL){
      if(strcmp(cmd->name,rx_buffer)==0){
        char *msg = (char*)cmd->fx();
        if(msg!=NULL) serial_write(msg);
        break;
      }
      cmd++;
    }
    if(cmd->name==NULL){
      serial_write("unknown command");
    }
    rx_buffer_i = 0;
  }else{
    rx_buffer[rx_buffer_i++] = c;
  }
}

int main(){
  /* PINs */
  DDRC = 0x03;
  DDRD = 0x00;
  PORTD= 0x0c; /* pull up INT0, INT1 */
  /* Blink */
  DDRB = 0x20;
  PORTB = 0x00;
  /* Timer */
  TCCR1B  = (1<<WGM12);           /* CTC */
  TCCR1B |= (1<<CS12)|(1<<CS10);  /* prescaler 1024 */
  OCR1A   = 15625;                /* 1 sec */
  TIMSK1 |= 1<<OCIE1A;            /* Enable timer interrupt */
  /* INTERRUPT */
  EICRA = (1<<ISC00)|(1<<ISC10);  /* Any change */
  EIMSK = (1<<INT0)|(1<<INT1);    /* Enable interrupt */
  /* USART */
  UBRR0H = UBBR>>8;                   /* UBRR */
  UBRR0L = UBBR;                      /* UBRR */
  UCSR0B = (1<<TXEN0)|(1<<RXEN0);     /* Enable Transmit & Receive */
  UCSR0B|= (1<<RXCIE0);               /* On receive byte interrupt */
  UCSR0C = (1<<UCSZ00)|(1<<UCSZ01);   /* 8 bit data */
  rx_buffer_i = 0;
  /* Init state */
  toState(0);
  /* Set Enable Interrupt */
  sei();
  while(1);
}
