/*
Copyright (c) 2013, Augusto Damasceno.
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#define F_CPU 16000000UL
#define FOSC 16000000
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD- 1

#include <avr/io.h>
#include <util/delay.h>

/* These get_buttons functions save in a unsigned int variable the states of buttons
   of Snes Controller.
    PINOUT:
   -----------------------------------------------------
   | |o|     |o|    |o|     |o| - |o|       |o|     |o| |
   -----------------------------------------------------
     +5v     Clk    OUTD    Data   N         N     GND
     
    serial:
    [N|N|N|N|R|L|X|A|Right|Left|Down|Up|Start|Select|Y|B]
    MSB                                               LSB
*/


void get_buttons1(unsigned int * serial)
{
    *serial = 0x0; //Reset serial variable.
       
    PORTB |= 0b00000001; //Pulse OUTD.
    _delay_ms(0.012);
       
    PORTB &= 0b11111110; //Wait 6 us.
    _delay_ms(0.006);
       
    PORTB &= 0b11111101; //Clock down for bit0.
    if((PIND & 0b00000100) == 0b00000000) //If bit 0 is low, button B is pressed.
    {
        *serial |= 0b0000000000000001;
    }
    _delay_ms(0.006);
    
    PORTB |= 0b00000010; //Clock up.
    _delay_ms(0.006);
     
    PORTB &= 0b11111101; //Clock down for bit1.
    if((PIND & 0b00000100) == 0b00000000) //If bit 1 is low, button Y is pressed.
    {
       *serial |= 0b0000000000000010;
    }
    _delay_ms(0.006);
 
    PORTB |= 0b00000010; //Clock up.
    _delay_ms(0.006);
 
    PORTB &= 0b11111101; //Clock down for bit2.
    if((PIND & 0b00000100) == 0b00000000) //If bit 2 is low, button Select is pressed.
    {
       *serial |= 0b0000000000000100;
    }
    _delay_ms(0.006);
     
    PORTB |= 0b00000010; //Clock up.
    _delay_ms(0.006);
     
    PORTB &= 0b11111101; //Clock down for bit3.
    if((PIND & 0b00000100) == 0b00000000) //If bit 3 is low, button Start is pressed.
    {
       *serial |= 0b0000000000001000;
    }
    _delay_ms(0.006);
     
    PORTB |= 0b00000010; //Clock up.
    _delay_ms(0.006);
     
    PORTB &= 0b11111101; //Clock down for bit4.
    if((PIND & 0b00000100) == 0b00000000) //If bit 4 is low, button Up is pressed.
    {
        *serial |= 0b0000000000010000;
    }
    _delay_ms(0.006);

    PORTB |= 0b00000010; //Clock up.
    _delay_ms(0.006);

    PORTB &= 0b11111101; //Clock down for bit5.
    if((PIND & 0b00000100) == 0b00000000) //If bit 5 is low, button Down is pressed.
    {
       *serial |= 0b0000000000100000;
    }
    _delay_ms(0.006);

    PORTB |= 0b00000010; //Clock up.
    _delay_ms(0.006);

    PORTB &= 0b11111101; //Clock down for bit6.
    if((PIND & 0b00000100) == 0b00000000) //If bit 6 is low, button Left is pressed.
     {
       *serial |= 0b0000000001000000;
     }
    _delay_ms(0.006);

    PORTB |= 0b00000010; //Clock up.
    _delay_ms(0.006);

    PORTB &= 0b11111101; //Clock down for bit7.
    if((PIND & 0b00000100) == 0b00000000) //If bit 7 is low, button Right is pressed.
    {
       *serial |= 0b0000000010000000;
    }
    _delay_ms(0.006);

    PORTB |= 0b00000010; //Clock up.
    _delay_ms(0.006);

    PORTB &= 0b11111101; //Clock down for bit8.
    if((PIND & 0b00000100) == 0b00000000) //If bit 8 is low, button A is pressed.
    {
       *serial |= 0b0000000100000000;
    }
    _delay_ms(0.006);

    PORTB |= 0b00000010; //Clock up.
    _delay_ms(0.006);

    PORTB &= 0b11111101; //Clock down for bit9.
    if((PIND & 0b00000100) == 0b00000000) //If bit 9 is low, button X is pressed.
    {
       *serial |= 0b0000001000000000;
    }
    _delay_ms(0.006);

    PORTB |= 0b00000010; //Clock up.
    _delay_ms(0.006);
    
    PORTB &= 0b11111101; //Clock down for bit10.
    if((PIND & 0b00000100) == 0b00000000) //If bit 10 is low, button L is pressed.//
     {
        *serial |= 0b0000010000000000;
     }
     _delay_ms(0.006);
     
     PORTB |= 0b00000010; //Clock up.
     _delay_ms(0.006);
     
     PORTB &= 0b11111101; //Clock down for bit11.
     if((PIND & 0b00000100) == 0b00000000) //If bit 11 is low, button R is pressed.
     {
        *serial |= 0b0000100000000000;
     }
     _delay_ms(0.006);
     
     PORTB |= 0b00000010; //Clock up.
     _delay_ms(0.006);
     
     PORTB &= 0b11111101; //Clock down. Bit 12 means nothing.
     _delay_ms(0.006);
     
     PORTB |= 0b00000010; //Clock up.
     _delay_ms(0.006);
     
     PORTB &= 0b11111101; //Clock down. Bit 13 means nothing.
     _delay_ms(0.006);
     
     PORTB |= 0b00000010; //Clock up.
     _delay_ms(0.006);
     
     PORTB &= 0b11111101; //Clock down. Bit 14 means nothing.
     _delay_ms(0.006);
     
     PORTB |= 0b00000010; //Clock up.
     _delay_ms(0.006);
     
     PORTB &= 0b11111101; //Clock down. Bit 15 means nothing.
     _delay_ms(0.006);
     
     PORTB |= 0b00000010; //Clock up.
     _delay_ms(0.006);
}

void get_buttons2(unsigned int * serial)
{
    *serial = 0x0; //Reset serial variable.
       
    PORTD |= 0b00010000; //Pulse OUTD.
    _delay_ms(0.012);
       
    PORTD &= 0b11101111; //Wait 6 us.
    _delay_ms(0.006);
       
    PORTD &= 0b11110111; //Clock down for bit0.
    if((PIND & 0b00100000) == 0b00000000) //If bit 0 is low, button B is pressed.
    {
        *serial |= 0b0000000000000001;
    }
    _delay_ms(0.006);
    
    PORTD |= 0b00001000; //Clock up.
    _delay_ms(0.006);
     
    PORTD &= 0b11110111; //Clock down for bit1.
    if((PIND & 0b00100000) == 0b00000000) //If bit 1 is low, button Y is pressed.
    {
       *serial |= 0b0000000000000010;
    }
    _delay_ms(0.006);
 
    PORTD |= 0b00001000; //Clock up.
    _delay_ms(0.006);
 
    PORTD &= 0b11110111; //Clock down for bit2.
    if((PIND & 0b00100000) == 0b00000000) //If bit 2 is low, button Select is pressed.
    {
       *serial |= 0b0000000000000100;
    }
    _delay_ms(0.006);
     
    PORTD |= 0b00001000; //Clock up.
    _delay_ms(0.006);
     
    PORTD &= 0b11110111; //Clock down for bit3.
    if((PIND & 0b00100000) == 0b00000000) //If bit 3 is low, button Start is pressed.
    {
       *serial |= 0b0000000000001000;
    }
    _delay_ms(0.006);
     
    PORTD |= 0b00001000; //Clock up.
    _delay_ms(0.006);
     
    PORTD &= 0b11110111;; //Clock down for bit4.
    if((PIND & 0b00100000) == 0b00000000) //If bit 4 is low, button Up is pressed.
    {
        *serial |= 0b0000000000010000;
    }
    _delay_ms(0.006);

    PORTD |= 0b00001000; //Clock up.
    _delay_ms(0.006);

    PORTD &= 0b11110111; //Clock down for bit5.
    if((PIND & 0b00100000) == 0b00000000) //If bit 5 is low, button Down is pressed.
    {
       *serial |= 0b0000000000100000;
    }
    _delay_ms(0.006);

    PORTD |= 0b00001000; //Clock up.
    _delay_ms(0.006);

    PORTD &= 0b11110111; //Clock down for bit6.
    if((PIND & 0b00100000) == 0b00000000) //If bit 6 is low, button Left is pressed.
     {
       *serial |= 0b0000000001000000;
     }
    _delay_ms(0.006);

    PORTD |= 0b00001000; //Clock up.
    _delay_ms(0.006);

    PORTD &= 0b11110111; //Clock down for bit7.
    if((PIND & 0b00100000) == 0b00000000) //If bit 7 is low, button Right is pressed.
    {
       *serial |= 0b0000000010000000;
    }
    _delay_ms(0.006);

    PORTD |= 0b00001000; //Clock up.
    _delay_ms(0.006);

    PORTD &= 0b11110111; //Clock down for bit8.
    if((PIND & 0b00100000) == 0b00000000) //If bit 8 is low, button A is pressed.
    {
       *serial |= 0b0000000100000000;
    }
    _delay_ms(0.006);

    PORTD |= 0b00001000; //Clock up.
    _delay_ms(0.006);

    PORTD &= 0b11110111; //Clock down for bit9.
    if((PIND & 0b00100000) == 0b00000000) //If bit 9 is low, button X is pressed.
    {
       *serial |= 0b0000001000000000;
    }
    _delay_ms(0.006);

    PORTD |= 0b00001000; //Clock up.
    _delay_ms(0.006);
    
    PORTD &= 0b11110111; //Clock down for bit10.
    if((PIND & 0b00100000) == 0b00000000) //If bit 10 is low, button L is pressed.
     {
        *serial |= 0b0000010000000000;
     }
     _delay_ms(0.006);
     
     PORTD |= 0b00001000; //Clock up.
     _delay_ms(0.006);
     
     PORTD &= 0b11110111; //Clock down for bit11.
     if((PIND & 0b00100000) == 0b00000000) //If bit 11 is low, button R is pressed.
     {
        *serial |= 0b0000100000000000;
     }
     _delay_ms(0.006);
     
     PORTD |= 0b00001000; //Clock up.
     _delay_ms(0.006);
     
     PORTD &= 0b11110111; //Clock down. Bit 12 means nothing.
     _delay_ms(0.006);
     
     PORTD |= 0b00001000; //Clock up.
     _delay_ms(0.006);
     
     PORTD &= 0b11110111; //Clock down. Bit 13 means nothing.
     _delay_ms(0.006);
     
     PORTD |= 0b00001000; //Clock up.
     _delay_ms(0.006);
     
     PORTD &= 0b11110111; //Clock down. Bit 14 means nothing.
     _delay_ms(0.006);
     
     PORTD |= 0b00001000; //Clock up.
     _delay_ms(0.006);
     
     PORTD &= 0b11110111; //Clock down. Bit 15 means nothing.
     _delay_ms(0.006);
     
     PORTD |= 0b00001000; //Clock up.
     _delay_ms(0.006);
}

int main()
{
   DDRB |= 0b00101111;
   //SPI: Bit 2 is SS. Bit 3 is MOSI. Bit 4 is MISO. Bit 5 is SCK.
   //Controler 1: Bit 0 is pulse OUTD to controller. Bit 1 is Clock to controller.
   PORTB |= 0b00100100; //Ports levels and pull-up.
   
   DDRD |= 0b00011010;
   //USART: Bit 0 and 1 are RX-TX.
   //Controller 1: Bit 2 is serial data from buttons states.
   //Controller 2: Bit 3 is clock to controller. Bit 4 is pulse OUTD to controller. Bit 5 is serial data from buttons states. 
   PORTD |= 0b00000000; //Ports levels and pull-up.
   
   DDRC |= 0b00111000;
   //Bit 0 is ADC input (select mode). Bit 2 is ON-OFF switch.
   //Bit 3 is led indicator controller 1. Bit 4 is led indicator controller 2. Bit 5 is led indicator SPI.
   PORTC |= 0b00000100; //Ports levels and pull-up.
   
   unsigned int buffer1 = 0b0000000000000000; //Save last serial data from controller 1 (16 bits).
   unsigned int buffer2 = 0b0000000000000000; //Save last serial data from controller 2 (16 bits).
   unsigned int mode = 0x0; //Save result of ADC conversion.
   
   SPCR |= (1 << SPE) | (1 << MSTR) | (1 << CPOL) | (1 << SPR1) | (1 << SPR0);
   //SPI configuration: Mode master. SS is high when idle and data send in clock falling edge. Clock Prescale = 128 (125KHz) and high when idle.
   
   
   UCSR0B |= (1 << RXEN0) | (1 << TXEN0) ;
   UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00) | (1 << USBS0) ;
   //USART configuration: Asynchronous. Baud Rate = 9600 bps. Start Bit = 1 and Stop Bit = 1. Character Size = 8 bits. Disabled Parity. 

   unsigned int ubrr = MYUBRR;
   UBRR0H = (unsigned char ) ( ubrr>>8) ;
   UBRR0L = (unsigned char ) ubrr ;
   
   ADMUX |= (1<<REFS0);
   ADCSRA |= (1<<ADEN) | (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2);
   //ADC configuration: Voltage Reference = AVCC. Input Channel = ADC0. ADC Enable. ADC Prescaler = 128 (125 KHz).
   
   while(1)
   {
     if( (PINC & 0b00000100) == 0b00000000 )
     {       
       ADCSRA |= 0b01000000; //Start conversion
       while ( !(ADCSRA & (1 << ADIF)) ){} // Wait conversion.
       mode = ADC;
       
       if(mode <= 169) //Mode 0 = Player 1 On, Player 2 On, SPI on.
       {
         PORTC |= 0b00111000; 
         
         get_buttons1(&buffer1);
         _delay_ms(8);
         get_buttons2(&buffer2);
         
         SPDR = buffer1;
         while(!( SPSR & (1<<SPIF) )) ;
         SPDR = buffer1>>8;
         while(!( SPSR & (1<<SPIF) )) ;
         SPDR = buffer2;
         while(!( SPSR & (1<<SPIF) )) ;
         SPDR = buffer2>>8;
         while(!( SPSR & (1<<SPIF) )) ;
       }
       else if (mode > 169 && mode <= 340) //Mode 1 = Player 1 On, Player 2 Off, SPI on.
       {
         PORTC &= 0b11101111;
         PORTC |= 0b00101000;
         
         get_buttons1(&buffer1);
         _delay_ms(8);
         buffer2 = 0b0000000000000000;
         
         SPDR = buffer1;
         while(!( SPSR & (1<<SPIF) )) ;
         SPDR = buffer1>>8;
         while(!( SPSR & (1<<SPIF) )) ;
         SPDR = buffer2;
         while(!( SPSR & (1<<SPIF) )) ;
         SPDR = buffer2>>8;
         while(!( SPSR & (1<<SPIF) )) ;
       }
       else if (mode > 340 && mode <= 511) //Mode 2 = Player 1 Off, Player 2 On, SPI on.
       {
         PORTC &= 0b11110111;
         PORTC |= 0b00110000;
         
         buffer1 = 0b0000000000000000;
         _delay_ms(8);
         get_buttons2(&buffer2);
         
         SPDR = buffer1;
         while(!( SPSR & (1<<SPIF) )) ;
         SPDR = buffer1>>8;
         while(!( SPSR & (1<<SPIF) )) ;
         SPDR = buffer2;
         while(!( SPSR & (1<<SPIF) )) ;
         SPDR = buffer2>>8;
         while(!( SPSR & (1<<SPIF) )) ;
       }
       else if (mode > 511 && mode <= 681) //Mode 3 = Player 1 On, Player 2 On, SPI off.
       {
         PORTC &= 0b11011111;
         PORTC |= 0b00011000;
         
         get_buttons1(&buffer1);
         _delay_ms(8);
         get_buttons2(&buffer2);
       }
       else if (mode > 681 && mode <= 852) //Mode 4 = Player 1 On, Player 2 Off, SPI off.
       {
         PORTC &= 0b11001111;
         PORTC |= 0b00001000;
         
         get_buttons1(&buffer1);
         _delay_ms(8);
         buffer2 = 0b0000000000000000;
       }
       else if (mode > 852 && mode <= 1023) //Mode 5 = Player 1 Off, Player 2 On, SPI off.
       {
         PORTC &= 0b11010111;
         PORTC |= 0b00010000;
         
         buffer1 = 0b0000000000000000;
         _delay_ms(8);
         get_buttons2(&buffer2);
       }
       
       while (!( UCSR0A & (1<<UDRE0) )) ;
       UDR0 = 0b11111111;
       while (!( UCSR0A & (1<<UDRE0) )) ;
       UDR0 = 0b11111111;
       while (!( UCSR0A & (1<<UDRE0) )) ;
       UDR0 = buffer1;
       while (!( UCSR0A & (1<<UDRE0) )) ;
       UDR0 = buffer1>>8;
       while (!( UCSR0A & (1<<UDRE0) )) ;
       UDR0 = buffer2;
       while (!( UCSR0A & (1<<UDRE0) )) ;
       UDR0 = buffer2>>8;
     
       _delay_ms(8);
     
       }
       else
       {
         PORTC &= 0b11000111;
       }
     
   }
   
   return 0;
}
