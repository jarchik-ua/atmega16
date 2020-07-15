#define F_CPU 16000000L
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

volatile unsigned char Napravlenie;
unsigned char R1=0, R2=0, R3=0, R4=0;
unsigned char n_count=0;
int encoder = 0;
int schet = 0;
int flag_stop = 0;
int flag_start = 0;
int ActiveTimer = 0;
int flag_light = 0;
int button_state = 0;
int button_state_prev = 0;

void segchar (unsigned char seg);
void segchars (unsigned char seg);
void init_potrb(void);  
void Enkoder(void);  
void timer_ini(void); 
void ledprint(unsigned int number); 
void init_pwm_work (void);			   
void init_pwm_stop (void);
void heater_and_other_start(void);
void heater_and_other_stop(void);			   
void process_timeoff(void);
void init_data(void);
void button_pwm(void);
void start(void);
void init_port_pwm(void);
void init_limit_switch(void);
void init_func(void);
void process_reset(void);
void process_pause(void);
void backlight_works(void);
void heater_and_motor_stop(void);
uint8_t input_state (uint8_t pin);

ISR (TIMER2_OVF_vect)
{
	if(n_count==0) {PORTC&=~(1<<PORTC7);PORTC|=(1<<PORTC6) | (1<<PORTC5) | (1<<PORTC4);segchar(R1);}
	if(n_count==1) {PORTC&=~(1<<PORTC6);PORTC|=(1<<PORTC7) | (1<<PORTC5) | (1<<PORTC4);segchar(R2);}
	if(n_count==2 && encoder > 59) {PORTC&=~(1<<PORTC5);PORTC|=(1<<PORTC7) | (1<<PORTC6) | (1<<PORTC4);if (R3 > 0 || R4 > 0) segchars (R3); else segchar(R3);}
	if(n_count==3 && encoder > 59) {PORTC&=~(1<<PORTC4);PORTC|=(1<<PORTC7) | (1<<PORTC6) | (1<<PORTC5);segchar(R4);}
	n_count++;
	if (n_count>3) n_count=0;
}

ISR (TIMER1_COMPA_vect)
{
	if(ActiveTimer == 1 && encoder > 0)
	{
		encoder--;
		if (encoder != 0)
			heater_and_other_start();     
		else
			heater_and_other_stop();
		if (encoder == 0) process_timeoff();
	}      
}

enum states{
	switched_off,
	switched_on,
	pause
   };

enum states state = switched_off;

int main(void)
{
	init_func();
  	
	while (1)
	{ 
		ledprint (encoder);
		switch (state)
		{	    
			case switched_off:
				heater_and_other_stop();
				if (!(PINB & (1 << 0)))
					state = switched_on;
			break;
			case switched_on:
				if (~PINB & (1 << 0))
				{
					while (~PINB & (1 << 0));
					if (encoder == 0 )
					{
						encoder = 30;
						flag_start = 0;
						start();
					}
					else if (encoder != 0 && flag_start == 0 )
					{
						start();
					}
					else if (encoder != 0 && flag_start == 1 )
					{
						encoder += 30;
						flag_start = 0;
						start();
					}
				}
				 
				if (!(PINB & (1 << 1))) 
				{
					state = pause;                        
				} 
			break;
			case pause:
				if (!(PINB & (1 << 1))) 
				{
					while(!(PINB & (1 << 1)));    	  
					if (encoder != 0 && flag_start == 1 )
					{
						process_pause();
					}
					else
					{
						process_reset();
						flag_start = 0;
					}	   
				}
				if (!(PINB & (1 << 0)))
					state = switched_on;
			break;
			default:
				state = switched_off;
			break;
		}  
	}
	return 0;
}

void
start(void)
{
	TCCR1B &= ~(1 << CS11);
	TCCR1B |= (1 << CS10) | (1 << CS12); 
   
	TIMSK |= (1 << OCIE1A);
   
	OCR1AH = 0b00111101;
	OCR1AL = 0b00001001;
   
	TCNT1 = 0;
   
	TCCR1B |= (1 << WGM12);
   
	ActiveTimer = 1;
	flag_stop = 0;
	flag_start = 1;
}

void
process_pause(void)
{
	ActiveTimer = 0;
	flag_start = 0;
	backlight_works();
	heater_and_motor_stop();
	init_pwm_stop();
	button_pwm();
}

void
heater_and_motor_stop(void)
{
	DDRB |= (1 << 3);
	PORTB &= ~(1 << 3);
	button_pwm();
}

void
process_reset(void)
{
	
	init_data();
	state = switched_off;
}

void
backlight_works(void)
{
	PORTC |= (1 << 0) ;
}

uint8_t
input_state (uint8_t pin)
{
	uint8_t button_state;
	button_state = PINB & (1 << pin);
	if (button_state != 0)
		button_state = 1;
	else
		button_state = 0;
	return button_state;
}

void
button_pwm(void)
{	
	button_state = input_state(2);
	if ((button_state == 1) && (button_state_prev == 0))
	{
			if (schet > 2) schet = 0;
			switch(schet)
			{
				case 0:
					PORTC |= (1 << 1);
					PORTC &= ~((1 << 2) | (1 << 3));
					OCR0 = 255;
				break;
				case 1:
					PORTC |= (1 << 2);
					PORTC &= ~((1 << 1) | (1 << 3));
					OCR0 = 170;
				break;
				case 2:
					PORTC |= (1 << 3);
					PORTC &= ~((1 << 1) | (1 << 2));
					OCR0 = 134;
				break;
			}
			schet++;
	}
	button_state_prev = button_state;
}

void
heater_and_other_start(void)
{
	PORTB |= (1 << 3);
	PORTC |= (1 << 0); 
	init_pwm_work();
}

void
heater_and_other_stop(void)
{
	PORTB &= ~(1 << 3);
	PORTC &= ~(1 << 0); 
	init_pwm_stop();
	button_pwm();
	Enkoder();
}

void
process_timeoff(void)
{
	init_data();
	flag_start = 0;
}

void
init_limit_switch(void)
{
	DDRB &= ~(1 << 3);
	PORTB |= (1 << 3);
}

void
init_data(void)
{
	heater_and_other_stop();
	encoder = 0;
	ActiveTimer = 0;
}

void
init_port_pwm(void)
{
	PORTB |= (1 << 3);
	PORTC |= (1 << 1);
	PORTC &= ~( (1 << 2) | (1 << 3) );
	OCR0 = 255;
}

void
init_func(void)
{
	timer_ini();
	init_data();
	init_limit_switch();
	init_port_pwm();
	init_potrb();
}

void
init_pwm_work (void)
{
	TCCR0 |= (1 << COM01);
	TCCR0 &= ~(1 << COM00);
	
	TCCR0 |= (1 << WGM00);
	TCCR0 |= (1 << WGM01);
	
	TCCR0 |= (1 << CS01);
	TCCR0 &= ~((1 << CS00) | (1 << CS02));
}

void
init_pwm_stop (void)
{
	TCCR0 |= (1 << COM01);
	TCCR0 &= ~(1 << COM00);

	TCCR0 &= ~(1 << WGM00);
	TCCR0 &= ~(1 << WGM01);

	TCCR0 |= (1 << CS01);
	TCCR0 &= ~((1 << CS00) | (1 << CS02));
}

void 
segchar (unsigned char seg)
{
	switch(seg)
	{
		case 1: PORTA = 0b11111001; break;
		case 2: PORTA = 0b10100100; break;
		case 3: PORTA = 0b10110000; break;
		case 4: PORTA = 0b10011001; break;
		case 5: PORTA = 0b10010010; break;
		case 6: PORTA = 0b10000010; break;
		case 7: PORTA = 0b11111000; break;
		case 8: PORTA = 0b10000000; break;
		case 9: PORTA = 0b10010000; break;
		case 0: PORTA = 0b11000000; break;
	}
}

void 
segchars (unsigned char seg)
{
	switch(seg)
	{
		case 1: PORTA = 0b01111001; break;
		case 2: PORTA = 0b00100100; break;
		case 3: PORTA = 0b00110000; break;
		case 4: PORTA = 0b00011001; break;
		case 5: PORTA = 0b00010010; break;
		case 6: PORTA = 0b00000010; break;
		case 7: PORTA = 0b01111000; break;
		case 8: PORTA = 0b00000000; break;
		case 9: PORTA = 0b00010000; break;
		case 0: PORTA = 0b01000000; break;
	}
}

void 
timer_ini(void)
{
	DDRA = 0xFF;
	PORTA = 0b11111111;
	DDRC = 0b11111111;
   
	TCCR2 |= ((1 << 1) | (1 << 2));
	TCCR2 &= ~(1 << 0);
	TIMSK |= (1 << TOIE2);
	TCNT2 = 0;
	sei();
}

void 
ledprint(unsigned int number)
{
	R1 = number % 60 % 10;
	R2 = number % 60 / 10;
	R3 = number / 60 % 10;
	R4 = number / 60 / 10;  
}

void
Enkoder(void)
{
	if ( Napravlenie & (1<<0))
	{
		if (PIND & (1<<3)){}
		else
		{
			if (PIND & (1<<2)){}
			else
				Napravlenie = 0;
		}
	}
	else
	{
		if (PIND & (1<<3)) {}
		else
		{
			if (PIND & (1<<2))
			{
				Napravlenie |=(1<<0);
				encoder += 5;
				flag_stop = 0;
				if (encoder > 3600) encoder = 0;
			}
		}
		if (PIND & (1<<2)) {}
		else
		{
			if (PIND & (1<<3))
			{
				Napravlenie |=(1<<0);
				encoder -= 5;
				flag_stop = 0;
				if (encoder < 0) encoder = 0;
			}
		}
	}
}

void
init_potrb(void)
{
	DDRB |= (1 << 3); //pwm
	DDRB &= ~((1 << 0) | (1 << 1) | (1 << 2)); //button
	PORTB |= (1 << 0) | (1 << 1) | (1 << 2);
}
