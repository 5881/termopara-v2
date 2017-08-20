//Термометр на термопаре К типа с контроллером max31855 версия 2.0
#define F_CPU 11059200UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdio.h>
#include <util/delay.h>
#define ST PC2
#define DS PC0
#define SH PC1
#define PORT PORTC
int room_t;//комнатная температура
unsigned char symbol[11]={0x03,0x9f,0x25,0x0d,0x99,\
			    0x49,0x41,0x1f,0x01,0x09,0xfd};


void usart_init(void){
	//скорость обмена 115200бод для тактовой частоты 11.059мгц
	UBRRH=0x00;//старший байт 0 тк значение ubbr=0x5<255
	UBRRL=0x5;
	//настройка конфигурации
	UCSRC|=(1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);//8N1
	UCSRB|=(1<<RXEN)|(1<<TXEN);//включаем usart
	};

void put_char(unsigned char data,FILE *stream){
	while(!(UCSRA&(1<<5)));//ждём готовности
	UDR=data;//отправляем
	//return 0;
	};
	
static unsigned char get_char(FILE *stream){
	while(!(UCSRA&(1<<7)));
	return UDR;
	};

static FILE mystdout=\
	    FDEV_SETUP_STREAM(put_char,get_char,_FDEV_SETUP_RW);
//использованы фрагменты кода 
//https://www.seanet.com/~karllunt/max31855.html
/*
 *  ThermoInit      set up hardware for using the MAX31855
 *
 *  This routine configures the SPI as a master for exchanging
 *  data with the MAX31855 thermocouple converter.  All pins
 *  and registers for accessing the various port lines are
 *  defined at the top of this code as named literals.
 */
//функция сна, как оказалось неэфективна
/*
 ISR( INT1_vect )
{
	PORT|=(1<<DS);
	for(unsigned char i=0;i<24;i++){
		PORT|=(1<<SH);
		PORT&=~(1<<SH);
		}
	PORT|=(1<<ST);
	PORT&=~(1<<ST);
	DDRB=0;	
asm("sleep");
}
*/
static void  ThermoInit(void)
{
    DDRB|=(1<<PB2)|(1<<PB5);
    DDRB&=~(1<<PB4);
    PORTB|=(1<<PB2);
    SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0) | (1<<SPR1) | (1<<CPHA);
    // enable SPI as master, slowest clock,
    // data active on trailing edge of SCK
}


/*
 *  ThermoReadRaw      return 32-bit raw value from MAX31855
 *
 *  This routine uses a four-byte SPI exchange to collect a
 *  raw reading from the MAX31855 thermocouple converter.  That
 *  value is returned unprocessed to the calling routine.
 *
 *  Note that this routine does NO processing.  It does not
 *  check for error flags or reasonable data ranges.
 */
static int32_t  ThermoReadRaw(void)
{
    int32_t                      d;
    unsigned char                n;

    PORTB&=~(1<<PB2);    // pull thermo CS low
    d = 0;               // start with nothing
    for (n=0; n<4; n++)
    {
        SPDR = 0;                         // send a null byte
        while ((SPSR & (1<<SPIF)) == 0)  ;// wait until transfer ends
        d = (d<<8) + SPDR;          // add next byte, starting with MSB
    }
    PORTB|=(1<<PB2);     // done, pull CS high

/*
 *                             Test cases
 *
 *  Uncomment one of the following lines of code to return known values
 *  for later processing.
 *
 *  Test values are derived from information in Maxim's MAX31855
 *  data sheet, page 10 (19-5793 Rev 2, 2/12).
 */
//  d = 0x01900000; // thermocouple = +25C, reference = 0C, no faults
//  d = 0xfff00000; // thermocouple = -1C, reference = 0C, no faults
//  d = 0xf0600000; // thermocouple = -250C, reference = 0C, no faults
//  d = 0x00010001; // thermocouple = N/A, reference = N/A, open fault
//  d = 0x00010002; // thermocouple = N/A, reference = N/A, short to GND
//  d = 0x00010004; // thermocouple = N/A, refernece = N/A, short to VCC

    return  d;
}


/*
 *  ThermoReadC      return thermocouple temperature in degrees C
 *
 *  This routine takes a raw reading from the thermocouple converter
 *  and translates that value into a temperature in degrees C.  That
 *  value is returned to the calling routine as an integer value,
 *  rounded.
 *
 *  The thermocouple value is stored in bits 31-18 as a signed 14-bit
 *  value, where the LSB represents 0.25 degC.  To convert to an
 *  integer value with no intermediate float operations, this code
 *  shifts the value 20 places right, rather than 18, effectively
 *  dividing the raw value by 4 and scaling it to unit degrees.
 *
 *  Note that this routine does NOT check the error flags in the
 *  raw value.  This would be a nice thing to add later, when I've
 *  figured out how I want to propagate the error conditions...
 */
static int correction(float temp){
    if (temp>25) return (int)temp;
    float t;
    t=-1.04578e-10*pow(temp,6)-3.36487e-8*pow(temp,5)\
    -3.50252e-6*pow(temp,4)-9.11223e-5*pow(temp,3)\
    +0.00212556*pow(temp,2)+1.14494984*temp-0.34729796;
    return (int)roundf(t);
    }

static int  mesure(){
    int32_t d;
    int temp;
    d = ThermoReadRaw();        // get a raw value
    temp=d&0xffff;
    room_t=(int)roundf((float)(temp>>4)/16);
    return correction((float)((d>>=18)/4));
    }




void indicate(int data){
	//ST-ind,DS-data,SH-sync
	unsigned char a,b,c=0;
	if(data<0){c=symbol[10]; data*=-1;}
	a=symbol[data%10];
	data/=10;
	b=symbol[data%10];
	data/=10;
	if(!c)c=symbol[data%10];
	if(c==0xfd||c==0x03){//убираем раздражающие нули.
		if(c==0x03) c=0xff;
		if(b==0x03) b=0xff;
	}
	for(unsigned char i=0;i<8;i++){
		if(a&1) PORT|=(1<<DS); else PORT&=~(1<<DS);
		PORT|=(1<<SH);
		a>>=1;
		PORT&=~(1<<SH);
	}
	for(unsigned char i=0;i<8;i++){
		if(b&1) PORT|=(1<<DS); else PORT&=~(1<<DS);
		PORT|=(1<<SH);
		b>>=1;
		PORT&=~(1<<SH);
	}
	for(unsigned char i=0;i<8;i++){
		if(c&1) PORT|=(1<<DS); else PORT&=~(1<<DS);
		PORT|=(1<<SH);
		c>>=1;
		PORT&=~(1<<SH);
	}
	PORT|=(1<<ST);
	PORT&=~(1<<ST);
}

int main(void){
	DDRC=0b111;
	PORTC=0;
	DDRD&=~(1<<PD3);
	PORTD|=(1<<PD3);
	//настройка прерывания int0
	//MCUCR|=(1<<ISC11)|(1<<ISC10);//передний фронт
	//разрешаем внешнее прерывание INT1 
	//GICR |= (1<<INT1);
	//выставляем флаг глобального разрешения прерываний
	//power down
	//MCUCR|=(1<<SE)|(1<<SM1);
	//sei();
	ThermoInit();
	usart_init();
	stdin=stdout=&mystdout;
	printf("Hello world!\r\n");

	while(1){
	    if(!(PIND&(1<<PD3))) indicate(room_t);
		else indicate(mesure());
	    _delay_ms(500);
	    //indicate(room_t);
	    //_delay_ms(500);
	    };
	return 0;
	}
