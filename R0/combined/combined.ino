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

//PLL CONSTANTS
const uint32_t f_default = 5802000; //default target frequency in KHz
const uint32_t f_lim_upper = 5875000; //upper limit of frequency in KHz
const uint32_t f_lim_lower = 5725000; //lower limit of frequency in KHz
const uint16_t f_osc = 10000; //reference oscillator frequency in KHz
const uint16_t pll_channel_spacing = 1000; //channel spacing in KHz
const uint8_t pll_P = 32; //default prescaler

//PLL VARIABLES
uint32_t f_current = f_default;
uint32_t f_new = 0;

//ADC CONSTANTS
const uint16_t sample_T = 100; //ms, sample period
const uint16_t sample_N = 2048; //samples per period
const uint16_t sample_delta = sample_T * 1000 / sample_N; //us, time between samples
const uint32_t sample_T_actual = sample_delta * sample_N; //us, actual sampling period
const uint8_t speed_bin_N = 10; //number of speed bins (same as number of LEDs)
const uint8_t light_offset = 0; //offset number of pins to first light pin
const uint8_t max_speed = 20; //m/s, max bin speed
const double speed_per_bin = max_speed / speed_bin_N; //(m/s)/index
const double speed_per_frequency = 0.02584; //(m/s)/Hz, calculated at 5.8GHz, technically changes at different channels
const double indexdiff_to_freq = sample_N * 1000000 / (2 * sample_T_actual); //Hz*index, divide by index difference to find frequency
const double indexdiff_to_speed = indexdiff_to_freq * speed_per_frequency; //(m/s)*index, divide by index difference to find speed
const uint8_t rms_percentage = 15; //the required deviation from 0 to record new zero crossing, in percentage of rms value

//ADC VARIABLES
int16_t adc_buffer[sample_N]; // initialize ADC buffer
uint16_t speed_bin[speed_bin_N]; //initialize doppler speed bins
uint16_t current_rms_value = 0; //rms value of input signal, digital scale
uint16_t buffer_index = 0; //index for use in adc_buffer
uint16_t index_delta = 0; //difference between adc_buffer indices
uint8_t bin_index = 0; //index for use in speed_bin
uint8_t active_light = 0; //the pin number for the light which is currently "on" during normal use
uint8_t sample_flag = 1; //if true, allows start of next sampling period
uint8_t calc_flag = 0; //if true, starts calculation loop
uint32_t adc_timer = 0; //us, adc_timer
double freq_sum = 0; //Hz, sum of recorded frequencies, then divided to find average
uint32_t actual_time_0 = 0; //ms, sample-start time
uint32_t actual_time_1 = 0; //ms, sample-end time, for purpose of result verification

//MEASUREMENT STATES
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

  if (adc_timer <= micros()) //*****************FIX FOR OVERFLOW
  {
    if (buffer_index == 0)
    {
      actual_time_0 = millis();
    }
    adc_timer = adc_timer + sample_delta; //every sample_delta microseconds...
    adc_buffer[buffer_index++] = analogRead(ADC_pin); //record adc values
  }

  //if all samples are taken, start calculations
  if (buffer_index == sample_N)
  {
    actual_time_1 = millis() - actual_time_0;
    Serial.print("Time Elapsed (ms): ");
    Serial.println(actual_time_1);
    buffer_index = 0; //reset index
    current_rms_value = zero_average_and_rms(adc_buffer); //make samples bipolar, return rms value
    Serial.print("RMS value = ");
    Serial.println(current_rms_value); //print bipolar ADC RMS value

    switch(state)
    {
      case DOPPLER:
        stitch_sandwich_stacker(adc_buffer); //stack frequency data from zero-crossings into bins and update output
        break;
      case FMCW: //**************NOT YET WRITTEN************************************************//
        for (active_light = light_offset; active_light <= light_offset + 9; active_light++)
        {
          digitalWrite(active_light, LOW);
          delay(20);
          digitalWrite(active_light, HIGH);
        }
        delay(100);
        break;
    }
    switch_poll(state);
  }

  //timer overflow reset -- note that overflow will cause 1-2 false readings
  if (adc_timer - sample_delta > micros()) adc_timer = micros() + sample_delta;
}

uint16_t zero_average_and_rms(int16_t sample[]) //offsets input array to zero-average, calculates and returns ADC RMS value
{
  uint16_t rms_value;
  int32_t sum = 0;
  int32_t squaresum = 0;
  int16_t diff = 0;
  for (uint16_t j = 0; j < sample_N; j++)
  {
    sum = sum + sample[j];
  }
  diff = sum / sample_N;
  for (int j = 0; j < sample_N; j++)
  {
    adc_buffer[j] = sample[j] - diff;
  }
  for (int j = 0; j < sample_N; j++)
  {
    squaresum = squaresum + (adc_buffer[j] * adc_buffer[j]);
  }

  rms_value = sqrt(squaresum / sample_N); //calculate rms value
  return rms_value;
}

//takes bipolar sample vector
//steps through samples and finds zero crossing indices
//calls bin_increment function to find frequency content
//calculates and prints average frequency and speed
//updates LED indicator
void stitch_sandwich_stacker(int16_t sample[])
{
  uint8_t sign = 0;
  uint8_t old_sign = 0;
  uint8_t old_light = active_light;
  uint16_t new_zero_index = 0;
  uint16_t old_zero_index = 0;
  uint16_t crossing_count = 0;
  double freq_float;
  double speed_float;
  for (bin_index = 0; bin_index < speed_bin_N; bin_index++) //reset bins
  {
    speed_bin[bin_index] = 0;
  }
  for (buffer_index = 0; buffer_index < sample_N; buffer_index++)
  {
    //Serial.println(adc_buffer[buffer_index]); //***TEST CODE***
    //step through samples
    //check if absolute value greater than or equal to XX% rms value
    if ((abs(sample[buffer_index]) * 100) >= (current_rms_value * rms_percentage))
    {
      if (sample[buffer_index] > 0)
      {
        sign = 1;
      } else {
        sign = 0;
      }
      if (sign != old_sign)
      {
        old_sign = sign;
        //new_zero_index = (buffer_index + old_zero_index) / 2;
        new_zero_index = buffer_index;
        index_delta = new_zero_index - old_zero_index;
        if (old_zero_index != 0)
        {
          crossing_count++;
          bin_increment(index_delta);
        }
        old_zero_index = new_zero_index; //update
      }
    }
    //if so, check if positive/negative
    //record sign, check if different than last sign
    //if so, record a zero crossing index halfway between indices
    //record difference between current and previous crossing index
  }
  //  Serial.print("freq_sum = ");
  //  Serial.println(freq_sum);
  //  Serial.print("crossing_count = ");
  //  Serial.println(crossing_count);
  freq_float = freq_sum / crossing_count; //find average frequency
  freq_sum = 0; //reset sum
  speed_float = freq_float * speed_per_frequency; //find average speed
  Serial.print("Frequency (Hz): ");
  Serial.println(freq_float, 1); //print average frequency
  Serial.print("Speed (m/s): ");
  Serial.println(speed_float, 1); //print average speed
  active_light = bin_peak_search(speed_bin) + light_offset;
  if (old_light != active_light && active_light >= light_offset && active_light <= light_offset + 9)
  {
    digitalWrite((old_light), HIGH);
    digitalWrite((active_light), LOW);
  }
  buffer_index = 0; //reset adc_buffer index
  adc_timer = micros(); //update adc_timer
}

//takes zero-crossing index delta
//using defined constants, calculates frequency for the given delta
//increments value inside of speed_bin vector
void bin_increment(uint16_t index_difference)
{
  uint8_t bin;
  double freq_float;
  double speed_float;
  freq_float = indexdiff_to_freq / index_difference;// N/T
  freq_sum = freq_sum + freq_float;
  speed_float = indexdiff_to_speed / index_difference; //calculate single-sample speed
  bin = speed_float / speed_per_bin + 0.5; //record rounded bin
  if (bin <= speed_bin_N) //don't write outside of available bins
  {
    speed_bin[bin]++;
  }
}

uint16_t bin_peak_search(uint16_t vector[]) //take vector with unsigned 16-bit values and return the position of the max value
{
  uint16_t search_index;
  uint16_t peak_index = 0;
  uint16_t current_max = 0;
  for (search_index = 0; search_index < speed_bin_N; search_index++) //for the length of the input vector
  {
    if (vector[search_index] > current_max) //check if value is larger than recorded max
    {
      current_max = vector[search_index]; //update max
      peak_index = search_index; //update index
    }
  }
  return peak_index;
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
      delay(50);
      if (f_new < f_lim_lower) f_new = f_lim_upper - (f_lim_upper % (pll_channel_spacing * 3)); //wraparound
    }
  }
  else if (momentary_sw_in > 819)
  {
    while (analogRead(sw_MOM_pin) > 819)
    {
      f_new += (pll_channel_spacing * 3);
      delay(50);
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
  for (active_light = light_offset; active_light <= light_offset + 9; active_light++)
  {
    pinMode(active_light, OUTPUT);
    digitalWrite(active_light, LOW);
    delay(20);
    digitalWrite(active_light, HIGH);
  }
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
