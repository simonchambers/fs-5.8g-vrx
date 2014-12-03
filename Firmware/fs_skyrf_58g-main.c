/*
 * fs_skyrf_58g-main.c
 * 
 * Sends the channel tuning data over SPI to a 5.8G SkyRF module.
 *
 * Written by Simon Chambers 
 *
 * 16/12/13 Rev 1 - Initial Release
 * 25/01/14 Rev 2 - Board revision 2
 * 01/04/14 Rev 2 - On startup, tune the 5.8GHz module before the band+channel beeps
 *
 * Compile using avr-gcc and the following commands:
 *    avr-gcc -g -Os -c -mmcu=attiny24 fs_skyrf_58g-main.c
 *    avr-gcc -mmcu=attiny24 fs_skyrf_58g-main.o -o fs_skyrf_58g-main.elf
 *    avr-objcopy -O ihex -R .eeprom fs_skyrf_58g-main.elf fs_skyrf_58g-main.hex
 *
 * Assumes the following Pin usage:
 *    PA0 = Fatshark-CH0
 *    PA1 = Fatshark-CH1
 *    PA2 = Fatshark-CH2
 *    PA3 = Status LED
 *    PA4 = [N/C]
 *    PA5 = Buzzer+
 *    PA6 = Buzzer-
 *    PA7 = Channel Change Button
 * 
 *    PB0 = Module-CH0 (DataIO)
 *    PB1 = Module-CH1 (Enable)
 *    PB2 = Module-CH2 (Clock)
 *    PB3 = [N/C - Reset]
 * 
*/

/*
The MIT License (MIT)

Copyright (c) 2014 Simon Chambers

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <avr/eeprom.h>
#include <avr/io.h>
#define F_CPU 800000UL
#include <util/delay.h>
#include <avr/pgmspace.h>

#define LED_ON()            PORTA |= 0x08
#define LED_OFF()           PORTA &= ~0x08

#define BUZZER_ON()         PORTA |= 0x20; PORTA &=~0x40;
#define BUZZER_TOGGLE()     PORTA ^= 0x60;
//#define BUZZER_TOGGLE()     PORTA ^= 0x20;
#define BUZZER_OFF()        PORTA &= ~0x60;

/*
#define MODULE_CLK_LOW()    DDRB |= 0x04;
#define MODULE_CLK_HIGH()   DDRB &= ~0x04;

#define MODULE_DATA_LOW()   DDRB |= 0x01;
#define MODULE_DATA_HIGH()  DDRB &= ~0x01;

#define MODULE_EN_LOW()     DDRB |= 0x02;
#define MODULE_EN_HIGH()    DDRB &= ~0x02;
*/

#define MODULE_CLK_LOW()    PORTB &= ~0x04;
#define MODULE_CLK_HIGH()   PORTB |= 0x04;

#define MODULE_DATA_LOW()   PORTB &= ~0x01;
#define MODULE_DATA_HIGH()  PORTB |= 0x01;

#define MODULE_EN_LOW()     PORTB &= ~0x02;
#define MODULE_EN_HIGH()    PORTB |= 0x02;

#define TOTAL_BANDS         5

//PROGMEM prog_uint16_t channelTable[] = {
uint16_t channelTable[] = {
  0x2A05,	0x299B,	0x2991,	0x2987,	0x291D,	0x2913,	0x2909,	0x289F,	
  0x2903,	0x290C,	0x2916,	0x291F,	0x2989,	0x2992,	0x299C,	0x2A05,	
  0x2895,	0x288B,	0x2881,	0x2817,	0x2A0F,	0x2A19,	0x2A83,	0x2A8D,	
  0x2906,	0x2910,	0x291A,	0x2984,	0x298E,	0x2998,	0x2A02,	0x2A0C,
  0x289F, 0x2A05, 0x2A8D, 0x2A0C, 0x289F, 0x2A05, 0x2A8D, 0x2A0C,
};


void SERIAL_SENDBIT1()
{
  MODULE_CLK_LOW();
  _delay_us(300);
  
  MODULE_DATA_HIGH();
  _delay_us(300);
  MODULE_CLK_HIGH();
  _delay_us(300);
  
  MODULE_CLK_LOW();
  _delay_us(300);
}

void SERIAL_SENDBIT0()
{
  MODULE_CLK_LOW();
  _delay_us(300);
  
  MODULE_DATA_LOW();
  _delay_us(300);
  MODULE_CLK_HIGH();
  _delay_us(300);
  
  MODULE_CLK_LOW();
  _delay_us(300);
}

void SERIAL_ENABLE_LOW()
{
  _delay_us(100);
  MODULE_EN_LOW();
  _delay_us(100);
}

void SERIAL_ENABLE_HIGH()
{
  _delay_us(100); 
  MODULE_EN_HIGH();
  _delay_us(100);
}

void setChannelModule(uint8_t channel)
{
  uint8_t i;
  uint16_t channelData;
  
  //channelData = pgm_read_word(&channelTable[channel]);
  channelData = channelTable[channel];
  
  // bit bash out 25 bits of data
  // Order: A0-3, !R/W, D0-D19
  // A0=0, A1=0, A2=0, A3=1, RW=0, D0-19=0
  SERIAL_ENABLE_HIGH();
  _delay_ms(2);
  SERIAL_ENABLE_LOW();

  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT1();
  
  SERIAL_SENDBIT0();
  
  // remaining zeros
  for (i=20;i>0;i--)
    SERIAL_SENDBIT0();
  
  // Clock the data in
  SERIAL_ENABLE_HIGH();
  _delay_ms(2);
  SERIAL_ENABLE_LOW();

  // Second is the channel data from the lookup table
  // 20 bytes of register data are sent, but the MSB 4 bits are zeros
  // register address = 0x1, write, data0-15=channelData data15-19=0x0
  SERIAL_ENABLE_HIGH();
  SERIAL_ENABLE_LOW();
  
  // Register 0x1
  SERIAL_SENDBIT1();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  
  // Write to register
  SERIAL_SENDBIT1();
  
  // D0-D15
  //   note: loop runs backwards as more efficent on AVR
  for (i=16;i>0;i--)
  {
    // Is bit high or low?
    if (channelData & 0x1)
    {
      SERIAL_SENDBIT1();
    }
    else
    {
      SERIAL_SENDBIT0();
    }
    
    // Shift bits along to check the next one
    channelData >>= 1;
  }
  
  // Remaining D16-D19
  for (i=4;i>0;i--)
    SERIAL_SENDBIT0();
  
  // Finished clocking data in
  SERIAL_ENABLE_HIGH();
  _delay_ms(2);
  
  MODULE_EN_HIGH();
  MODULE_CLK_LOW();
  MODULE_DATA_LOW();
}

int8_t readChannelFatshark()
{
  return PINA & 0x07;
}

uint8_t readChannelButton()
{
  static uint8_t LastButtonState=0;
  
  // Is the button pushed? i.e. Low
  if ((LastButtonState == 0) && ((PINA & 0x80) == 0))
  {
    // delay for a moment and check again for debounce
    _delay_ms(1500);
    if ((PINB & 0x20) == 0)
    {
      LastButtonState = 1;
    }
  }
  else if ((LastButtonState == 1) && ((PINA & 0x80) != 0))
  {
    LastButtonState = 0;
  }
  
  return LastButtonState;
}

void buzzLow()
{
uint8_t i;
  
  LED_ON();
  BUZZER_ON();
  
  for (i=255;i>0;i--)
  {
    BUZZER_TOGGLE();
    _delay_us(300);
  }
  
  LED_OFF();
  BUZZER_OFF();
}

// Buzz for 125ms
void buzzHigh()
{
  uint8_t i;
  
  LED_ON();
  BUZZER_ON();
  
  for (i=255;i>0;i--)
  {
    BUZZER_TOGGLE();
    _delay_us(150);
  }
  
  LED_OFF();
  BUZZER_OFF();
}

// Buzz for 125mS
void buzzHigh_Short()
{
  buzzHigh();
}

// Buzz for 500ms
void buzzHigh_Long()
{
  buzzHigh();
  buzzHigh();
  buzzHigh();
  buzzHigh();
}


// Buzz for 125mS
void buzzLow_Short()
{
  buzzLow();
}

// Buzz for 500ms
void buzzLow_Long()
{
  buzzLow();
  buzzLow();
  buzzLow();
  buzzLow();
}

// Don't buzz for a 750ms (used when band changing normally)
inline void buzz_Long_Pause()
{
  _delay_ms(750);
}

// Don't buzz for 250ms (used for startup)
inline void buzz_Short_Pause()
{
  _delay_ms(250);
}

int main(void)
{
  uint8_t currentBand=0;
  uint8_t currentChannel=0;
  uint8_t i=0;
  uint8_t channelChangeFlag=0;
  
  // Setup I/O Ports 
  
  // PORTA
  //  DDR-bit: high=output, bit low=input
  DDRA = 0x68;
  // Ensure that the Fatshark inputs are pulled up and the button is too
  PORTA = 0x97;
  
  // PORTB
  //  DDR-bit: high=output, bit low=input
  //DDRB = 0x00;
  DDRB = 0x07;
  // Ensure the I/O ports are all off
  PORTB = 0x00;
  
  // Read in the current band from EEPROM 
  currentBand = eeprom_read_byte(0x00);
  
  // Set the channel and band when first powered on - before the beeps.
  // Also give a moment for the goggles to initalise and set the relevant channel on the IO pins
  _delay_ms(500);
  currentChannel = readChannelFatshark();
  setChannelModule((currentBand * 8)+currentChannel);
  
  // Make sure the current band is a valid number
  if (currentBand >= TOTAL_BANDS)
    currentBand = 0;
  
  
  // Beep out the current band
  for (i=0;i<=currentBand;i++)
  {
    buzzLow_Long();
    buzz_Long_Pause();
  }
  
  // force the current band to be set on the first loop run
  //  using the currentChannel code
  currentChannel = 0xFF;
  
  // main loop
  while(1)
  {
    // See if we should change frequency band
    if (readChannelButton())
    {
      if (channelChangeFlag == 0)
      {
        currentBand++;
        if (currentBand >= TOTAL_BANDS)
        {
          currentBand = 0;
        }
        
        eeprom_write_byte(0x00, currentBand);
        
        // Set new band and send it to the module
        setChannelModule((currentBand * 8)+currentChannel);
        
        // Beep out the new band
        for (i=0;i<=currentBand;i++)
        {
          buzzLow_Long();
          buzz_Long_Pause();
        }
        channelChangeFlag = 1;
      }
    }
    else
    {
      channelChangeFlag = 0;
    }
    
    if (readChannelFatshark() != currentChannel)
    {
      currentChannel = readChannelFatshark();
      
      // Calculate new index and set the module to it
      setChannelModule((currentBand * 8)+currentChannel);
      
      for (i=0;i<=currentChannel;i++)
      {
        buzzHigh_Short();
        buzz_Short_Pause();
      }
    }
  }
}
