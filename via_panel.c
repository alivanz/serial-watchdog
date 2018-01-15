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
  int strobe_INT1;
  int on_timeout;
  int on_INT0;
  int on_INT1;
  void (*cb_init)(void);
  void (*cb_timeout)(void);
  void (*cb_INT0)(void);
  void (*cb_INT1)(void);
} states[] = {
  {0, 2, 0,   1, 1, 0,    release_all, NULL, NULL, NULL},
  {3, 0, 0,   0, 0, 0,    power_push, NULL, NULL, NULL}
};
void toState(const int i){
  state = i;
  t = 0;
  if(states[state].cb_init != NULL) states[state].cb_init();
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
  if((states[state].strobe_INT0 == 1) && (INT0_value == 1)){
    if(states[state].cb_INT0 != NULL) states[state].cb_INT0();
    if(states[state].on_INT0 != -1  ) toState(states[state].on_INT0);
  }else if((states[state].strobe_INT0 == 2) && (INT0_value == 0)){
    if(states[state].cb_INT0 != NULL) states[state].cb_INT0();
    if(states[state].on_INT0 != -1  ) toState(states[state].on_INT0);
  }
  TCNT1 = 0;
}
ISR(INT1_vect){
  if((states[state].strobe_INT1 & 1) && (INT1_value == 1)){
    if(states[state].cb_INT1 != NULL) states[state].cb_INT1();
    if(states[state].on_INT1 != -1  ) toState(states[state].on_INT1);
  }else if((states[state].strobe_INT1 & 2) && (INT1_value == 0)){
    if(states[state].cb_INT1 != NULL) states[state].cb_INT1();
    if(states[state].on_INT1 != -1  ) toState(states[state].on_INT1);
  }
  TCNT1 = 0;
}

/* COMMANDS */
char * get_state(void){
  switch(state){
    case 0: return "A\n";
    case 1: return "B\n";
  }
  return "unknown\n";
}
char * touch(void){
  toState(1);
  return "touched!\n";
}
struct TCommand{
  char *name;
  char *(*fx)(void);
} commands[] = {
  {"get_state", get_state},
  {"touch", touch},
  {NULL,NULL}
};

#define BAUD 9600
#define UBBR (F_CPU/16/BAUD-1)
volatile char *tx_buffer;
volatile int tx_buffer_i;
char rx_buffer[64];
volatile int rx_buffer_i;
int SerialTXQueue(){
  return (UCSR0B & (1<<UDRIE0));
}
/* Non blocking serial write */
void serial_write(char *s){
  while(SerialTXQueue());
  tx_buffer = s;
  tx_buffer_i = 0;
  UCSR0B |= 1<<UDRIE0; // Enable UDR Empty Interrupt
}
/* Serial interrupt for send char */
ISR(USART_UDRE_vect){
  char c = tx_buffer[tx_buffer_i++];
  if(c == '\0'){
    UCSR0B &= 0xff-(1<<UDRIE0); // Disable UDR Empty Interrupt
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
        serial_write(cmd->fx());
        break;
      }
      cmd++;
    }
    if(cmd->name==NULL){
      serial_write("unknown command\n");
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
