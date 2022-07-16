#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdlib.h>

//#define F_CPU 1000000L
#define F_CPU 8000000L
#define FOSC F_CPU
#define UART_BAUD 1200
#define BAUDRATE ((F_CPU)/(UART_BAUD*16UL)-1)
//#define RTC_ADDR 0x68
#define RTC_ADDR 0xD0
#define PORTB_SEG_MASK 0x3E
#define PORTB_DIG_MASK 0x81
#define PORTB_SEG 0
#define PORTB_DIG 0
#define PORTC_SEG_MASK 0x07
//#define PORTC_DIG_MASK 0x00
#define PORTC_SEG 1
//#define PORTD_SEG_MASK 0x00
#define PORTD_DIG_MASK 0xE0
#define PORTD_DIG 1
#define PORTC_COL 0x04
#define PORTD_COL 0x40

// 7seg
unsigned int segmat[10][2] =
{
  {0x3E,0x01},
  {0x0C,0x00},
  {0x36,0x02},
  {0x1E,0x02},
  {0x0C,0x03},
  {0x1A,0x03},
  {0x3A,0x03},
  {0x0E,0x00},
  {0x3E,0x03},
  {0x1E,0x03}
};
//Y 0,1,2,3,4,5,6,7,8,9
//X a,b,c,d,e,f,g,:(A-F PB[0-5], G: PC[01])
//: is always 0
//Z PB,PC. use PORTB_SEG and PORTC_SEG_MASK

unsigned int digmat[4][2] =
{
  {0x01,0x00},
  {0x00,0x80},
  {0x00,0x20},
  {0x80,0x00}
//  {0x00,0x40}
};
//
//PB,PD. use PORTB_DIG and PORTD_DIG

//port assign
/*
PB0 - 1
PB1 - A
PB2 - B
PB3 - C
PB4 - D
PB5 - E
PB6 - MD1
PB7 - 5
PC0 - F
PC1 - G
PC2 - :
PC3 - 
PD2 - SET1
PD3 - SET2
PD4 - MD2
PD5 - 4
PD6 - 3(:)
PD7 - 2
*/

// TIMER0 for filament drive

unsigned char md=0;

//time date

volatile struct rtc_data
{
  unsigned int dig[5];
} rtc_data_ins;

void timer_init(void){
  TIMSK |= (1 << TOIE0); 
  TCCR0 |= ((1 << CS01)|(1 << CS00));
}

ISR(TIMER0_OVF_vect){
	if (!md)
  {
    PORTB |= 0x40;
    PORTD &= ~0x10;
    md = 1;
  }else{
    PORTB &= ~0x40;
    PORTD |= 0x10;
    md =0;
  }
}

// port modules

void port_init(void){
  DDRB |= 0xff;
  DDRC |= 0x07;
  DDRD |= 0xe0;
  PORTD |= 0x0c; //pull-up
}

void vfd_atom_drive(int dig[],int col){
  for (size_t i = 0; i < 4; i++){
    PORTB &= ~(PORTB_SEG_MASK | PORTB_DIG_MASK);
    PORTC &= ~PORTC_SEG_MASK;
    PORTD &= ~PORTD_DIG_MASK;
    _delay_us(1);
    PORTB |= (segmat[dig[i]][PORTB_SEG] | digmat[i][PORTB_DIG]);
    if (col)
    {
      PORTC |= (segmat[dig[i]][PORTC_SEG] | PORTC_COL);
      PORTD |= (digmat[i][PORTD_DIG] | PORTD_COL);
    } else {
      PORTC |= (segmat[dig[i]][PORTC_SEG] & ~PORTC_COL);
      PORTD |= (digmat[i][PORTD_DIG] & ~PORTD_COL);
    }
    
    //PORTC |= (col)?segmat[dig[i]][PORTC_SEG] | PORTC_COL : segmat[dig[i]][PORTC_SEG] & ~PORTC_COL;
    //PORTD |= (col)?digmat[i][PORTD_DIG] | PORTD_COL : digmat[i][PORTD_DIG] & ~PORTD_COL;
    _delay_us(49);
  }
}

void vfd_time_drive(int col){
  for (size_t i = 0; i < 4; i++){
    //int tmp = (rtc_data_ins.dig[i]!=0)?rtc_data_ins.dig[i]-1:0;
    int tmp = rtc_data_ins.dig[i];
    PORTB &= ~(PORTB_SEG_MASK | PORTB_DIG_MASK);
    PORTC &= ~PORTC_SEG_MASK;
    PORTD &= ~PORTD_DIG_MASK;
    _delay_us(1);
    PORTB |= (segmat[tmp][PORTB_SEG] | digmat[i][PORTB_DIG]);
    if (col)
    {
      PORTC |= (segmat[tmp][PORTC_SEG] | PORTC_COL);
      PORTD |= (digmat[i][PORTD_DIG] | PORTD_COL);
    } else {
      PORTC |= (segmat[tmp][PORTC_SEG] & ~PORTC_COL);
      PORTD |= (digmat[i][PORTD_DIG] & ~PORTD_COL);
    }
    _delay_us(99);
  }
}

// UART modules

void uart_init (void){
  UBRRH = (BAUDRATE>>8);
  UBRRL = BAUDRATE;
  UCSRB = (1<<TXEN);
  UCSRC|= (1<<URSEL)|(1<<UCSZ0)|(1<<UCSZ1);
}

void uart_transmit(unsigned char data){
  while (!( UCSRA & (1<<UDRE)));
  UDR = data;
}

void uart_sendmsg(unsigned char *str, unsigned char flg){
    if (flg == 'r') uart_transmit(13);
    while (*str != '\0')
    {
        uart_transmit(*str++);
    }
    if (flg == 'n')
    {
        uart_transmit(10);
        uart_transmit(13);
    }
}

// i2c modules

void i2cinit(void) {
  TWSR = 0x00;
  TWBR = 0x0C;
  TWCR = (1<<TWEN);
}

void i2cstart(void){
  TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
  while (!(TWCR & (1<<TWINT)));
}

void i2cstop(void) {
  TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN); 
}

void i2cwrite(uint8_t data) { 
  TWDR = data;
  TWCR = (1<<TWINT)|(1<<TWEN);
  while (!(TWCR & (1<<TWINT)));
}

uint8_t i2creadack(void) {
  TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
  while (!(TWCR & (1<<TWINT)));
  return TWDR;
}

uint8_t i2creadnak(void) {
  TWCR = (1<<TWINT)|(1<<TWEN);
  while (!(TWCR & (1<<TWINT)));
  return TWDR;
}

uint8_t i2cgetstatus(void) {
  uint8_t status;
  status = TWSR & 0xF8;
  return status;
}

void i2cerror(void) {
  uart_sendmsg("i2c error ",'n');  
}

//rtc modules

void rtc_osc(void){
  i2cstart();
	if (i2cgetstatus() != 0x08) i2cerror();
	i2cwrite((uint8_t)(RTC_ADDR));
	if (i2cgetstatus() != 0x18) i2cerror();
	i2cwrite((uint8_t)(0x00));
	if (i2cgetstatus() != 0x28) i2cerror();
	i2cwrite((uint8_t)(0x00));
	if (i2cgetstatus() != 0x28) i2cerror();
  i2cstop();
}

uint8_t rtc_read(uint8_t mem) {
	uint8_t u8data;
	i2cstart();
	if (i2cgetstatus() != 0x08) i2cerror();
	i2cwrite((uint8_t)(RTC_ADDR));
	if (i2cgetstatus() != 0x18) i2cerror();
	i2cwrite((uint8_t)(mem));
	if (i2cgetstatus() != 0x28) i2cerror();
	i2cstart();
	if (i2cgetstatus() != 0x10) i2cerror();
	i2cwrite((uint8_t)(RTC_ADDR + 1));
	if (i2cgetstatus() != 0x40) i2cerror();
	u8data = i2creadnak();
	if (i2cgetstatus() != 0x58) i2cerror();
	i2cstop();
	return u8data;
}

void rtcupdate(void) {
	uint8_t u8data;
  u8data = rtc_read(0x02);
	u8data &= 0x3F;
  rtc_data_ins.dig[0] = ((u8data & 0xF0) >> 4);
  rtc_data_ins.dig[1] = (u8data & 0x0F);
	u8data = rtc_read(0x01);
	u8data &= 0x7F;
  rtc_data_ins.dig[2] = ((u8data & 0xF0) >> 4);
  rtc_data_ins.dig[3] = (u8data & 0x0F);
	u8data = rtc_read(0x00);
	u8data &= 0x7F;
  rtc_data_ins.dig[4] = ((u8data & 0xF0) >> 4);
  rtc_data_ins.dig[5] = (u8data & 0x0F);
}

//main

int main(void){
  port_init();
  _delay_ms(300);
  uart_init();
  uart_sendmsg("uart init done",'n');
  i2cinit();
  rtc_osc();
  uart_sendmsg("rtc init done",'n');
  timer_init();
  uart_sendmsg("timer init done",'n');
  sei();
  unsigned char buf[10];

  while (1)
  {
    rtcupdate();
    int coltmp = rtc_data_ins.dig[5]%2;

    for (size_t i = 0; i < 100; i++)
    {
      vfd_time_drive(coltmp);
    }
    // uart_transmit(rtc_data_ins.dig[0]+0x30);
    // uart_transmit(rtc_data_ins.dig[1]+0x30);
    // uart_transmit(':');
    // uart_transmit(rtc_data_ins.dig[2]+0x30);
    // uart_transmit(rtc_data_ins.dig[3]+0x30);
    // uart_transmit(':');
    // uart_transmit(rtc_data_ins.dig[4]+0x30);
    // uart_transmit(rtc_data_ins.dig[5]+0x30);
    // uart_transmit(13);
  }

  return 0;
}
