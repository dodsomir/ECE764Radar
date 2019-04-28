//This sketch is for the combination of all radar functions in the ECE764 project

#include <U8x8lib.h>
#include "SPI.h"

#define lcd_CS 10 //chip select pin for LCD
#define lcd_RST 14 //active LOW reset pin for LCD
#define lcd_REG 15 //register select pin for LCD, 0: instruction, 1: data
#define pll_CS 17 //chip select pin for PLL
#define ADC_pin 18 //waveform input sampling pin
#define sw_MOM_pin 22 //momentary switch input pin
#define sw_TOG_pin 23 //toggle switch input pin

const uint32_t f_default = 5802000; //default target frequency in KHz
const uint32_t f_lim_upper = 5875000; //upper limit of frequency in KHz
const uint32_t f_lim_lower = 5725000; //lower limit of frequency in KHz
const uint16_t f_osc = 10000; //reference oscillator frequency in KHz
const uint16_t pll_channel_spacing = 1000; //channel spacing in KHz
const uint8_t pll_P = 32; //default prescaler

uint32_t f_current = f_default;
uint32_t f_new = 0;

enum state {DOPPLER, FMCW} state = DOPPLER;

//MAKE LCD INSTANCE
//U8X8_ST7565_NHD_C12864_4W_SW_SPI u8x8(/* clock=*/ 13, /* data=*/ 11, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8); //for software spi
U8X8_ST7565_NHD_C12864_4W_HW_SPI u8x8(/* cs=*/ lcd_CS, /* dc=*/ lcd_REG, /* reset=*/ lcd_RST);

//MAKE SPISettings INSTANCE for pll
SPISettings pll_spi(10000000, MSBFIRST, SPI_MODE0);

void setup() {
  // put your setup code here, to run once:
  pin_init();
  pll_init();
  Serial.begin(9600); //Teensy USB Serial accepts any speed
  u8x8.begin();
  SPI.begin();
  while (!Serial);
}

void loop() {
  // put your main code here, to run repeatedly:

}

void switch_poll(enum state current_state) //returns new frequency
{
  uint32_t f_new = f_current;
  uint16_t momentary_sw_in = 0;
  if (current_state != digitalRead(sw_TOG_pin))
    //if the current state isn't equal to TOGGLE switch (0 or 1, in both cases)
  {
    delay(10); //debounce
    if (digitalRead(sw_TOG_pin))
      //if still not equal
    {
      state != state; //toggle state
    }
  }
  momentary_sw_in = analogRead(sw_MOM_pin); //measure MOMENTARY switch
  if (momentary_sw_in < 205)
  {
    while (analogRead(sw_MOM_pin) < 205)
    {
      f_new -= (pll_channel_spacing * 3);
      delay(25);
      if (f_new < f_lim_lower) f_new = f_lim_upper - (f_lim_upper % (pll_channel_spacing * 3)); //wraparound
    }
  }
  else if (momentary_sw_in > 819)
  {
    while (analogRead(sw_MOM_pin) > 819)
    {
      f_new += (pll_channel_spacing * 3);
      delay(25);
      if (f_new > f_lim_upper) f_new = f_lim_lower + pll_channel_spacing - (f_lim_lower % (pll_channel_spacing * 3)); //wraparound
    }
  }

  if (f_new != f_current)
  {
    Serial.print("New Frequency Set (KHz) : ");
    Serial.println(f_new);
    freq_set(f_new);
    f_current = f_new;
  }
}

void pin_init(void)
{
  //  pinMode(lcd_CS, OUTPUT);
  //  pinMode(lcd_RST, OUTPUT);
  //  pinMode(lcd_REG, OUTPUT);
  pinMode(pll_CS, OUTPUT);
  pinMode(ADC_pin, INPUT);
  pinMode(sw_MOM_pin, INPUT);
  pinMode(sw_TOG_pin, INPUT);
}

void pll_24b_transfer(uint32_t data) //writes contents of 'data' to pll shift register, ignoring MSByte
{
  uint8_t i = 0;
  byte data_buffer[3];
  //pll uses 21-bit shift register
  for (i = 0; i < 3; i++)
  {
    data_buffer[2 - i] = data >> (i * 8); //bitshift 32 bit int into 3 byte array. Drop the MSByte
  }
  SPI.beginTransaction(pll_spi);
  digitalWrite(pll_CS, LOW);
  SPI.transfer(&data_buffer, 3);
  digitalWrite(pll_CS, HIGH);
  SPI.endTransaction();
}

void pll_init(void)
//frequency of reference oscillator in KHz
{
  //initialize pll by following startup sequence from datasheet (p15)
  //WILL ONLY WORK IF SHIFT REGISTER PUSHES OUT EXTRA VALUES

  //SET FUNCTION/INITIALIZATION LATCH
  Serial.print("Setting Function Latch, transfer input = ");
  Serial.println(0x000000C3, BIN);
  pll_24b_transfer(0x000000C3);

  //SET R-DIVIDER LATCH
  uint32_t R = f_osc / pll_channel_spacing; //find R divider
  R = R << 2; // //shift R parameter to correct location
  //R requires 0b00 as control bits
  Serial.print("Setting R Value, transfer input = ");
  Serial.println(R, BIN);
  pll_24b_transfer(R);

  //SET N-DIVIDER LATCH
  freq_set(f_default);
}

void freq_set(uint32_t freq)
//sets OUTPUT frequency in KHz, rounded down to nearest channel KHz.
//ENSURE VALUE IS WITHIN VCO RANGE*3 (after multiplier)
{
  //calculations
  freq = (freq - freq % (pll_channel_spacing * 3)) / 3; //round down input, divide by 3
  uint32_t N = freq / pll_channel_spacing; //find N divider
  uint32_t B = N / pll_P; //find B parameter
  uint16_t A = N - (B * pll_P); //find A parameter

  //N divider
  A = A << 2; //shift A parameter to correct location
  B = B << 7; //shift B parameter to correct location
  uint32_t data_temp = 0; //allocate temp variable
  data_temp |= A; //place A
  data_temp |= B; //place B
  data_temp |= 1; //set N control bit
  Serial.print("Setting N Value, transfer input = ");
  Serial.println(data_temp, BIN);
  pll_24b_transfer(data_temp);
}
