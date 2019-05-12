//This sketch is for the combination of all radar functions in the ECE764 project

#include <U8x8lib.h>
#include <SPI.h>
#include "fix_fft.h"

#define lcd_CS 10 //chip select pin for LCD
#define lcd_RST 14 //active LOW reset pin for LCD
#define lcd_REG 15 //register select pin for LCD, 0: instruction, 1: data
#define pot_CS 16 //chip select pin for potentiometer
#define pll_CS 17 //chip select pin for PLL
#define ADC_pin 18 //waveform input sampling pin
#define SYNC_pin 21 //signal generator pulse synchronization pin
#define sw_MOM_pin 22 //momentary switch input pin
#define sw_TOG_pin 23 //toggle switch input pin

//PLL CONSTANTS and VARIABLES
const uint32_t f_default = 5802000; //default target frequency in KHz
const uint32_t f_lim_upper = 5875000; //upper limit of frequency in KHz
const uint32_t f_lim_lower = 5725000; //lower limit of frequency in KHz
const uint16_t f_osc = 10000; //reference oscillator frequency in KHz
const uint16_t pll_channel_spacing = 1000; //channel spacing in KHz
const uint8_t pll_P = 32; //default prescaler
uint32_t f_current = f_default;
uint32_t f_new = 0;

//POT CONSTANTS and VARIABLES
const uint16_t rms_target = 275; //358 is max for full-range operation
//rms_hysteresis * loop_gain > 100
const uint8_t rms_hysteresis = 15; //acceptable input power difference range
const uint8_t loop_gain = 8; //this number divided by 100 is the loop gain
int16_t pot_pos = 127; //start at midpoint gain, NOTE LOWER NUMBER IS HIGHER GAIN
uint8_t old_pos = 0;

//ADC CONSTANTS and VARIABLES
const uint16_t sample_T = 100; //ms, sample period
const uint16_t sample_N = 1024; // = 2^m, samples per period
const uint16_t sample_delta = sample_T * 1000 / sample_N; //us, time between samples
const uint32_t sample_T_actual = sample_delta * sample_N; //us, actual sampling period
const uint8_t speed_bin_N = 10; //number of speed bins (same as number of LEDs)
const uint8_t light_offset = 9; //offset number of pins to first light pin
const uint8_t max_speed = 20; //m/s, max bin speed
const uint8_t max_distance = 50; //m, max expected distance
const double speed_per_bin = max_speed / speed_bin_N; //(m/s)/index
const double speed_per_frequency = 0.02584; //(m/s)/Hz, calculated at 5.8GHz, technically changes at different channels
const double indexdiff_to_freq = sample_N * 1000000 / (2 * sample_T_actual); //Hz*index, divide by index difference to find frequency
const double indexdiff_to_speed = indexdiff_to_freq * speed_per_frequency; //(m/s)*index, divide by index difference to find speed
const float range_res = 299792458 / (2000 * (f_lim_upper - f_lim_lower)); //m, FMCW range resolution
const uint8_t rms_percentage = 20; //the required deviation from 0 to record new zero crossing, in percentage of rms value
int16_t adc_buffer_new[sample_N]; // allocate ADC buffer
int16_t adc_buffer_diff[sample_N];
int16_t adc_buffer_old[sample_N]; // allocate MTI ADC buffer
uint16_t speed_bin[speed_bin_N]; //allocate doppler speed bins
uint16_t current_rms_value = 0; //rms value of input signal, digital scale
uint16_t buffer_index = 0; //index for use in adc_buffer_new
uint16_t index_delta = 0; //difference between adc_buffer_new indices
uint8_t bin_index = 0; //index for use in speed_bin
uint8_t active_light = 9; //the pin number for the light which is currently "on" during normal use
uint8_t sample_flag = 1; //if true, allows start of next sampling period
uint8_t calc_flag = 0; //if true, starts calculation loop
uint32_t adc_timer = 0; //us, adc_timer
double freq_sum = 0; //Hz, sum of recorded frequencies, then divided to find average
uint32_t actual_time_0 = 0; //ms, sample-start time
uint32_t actual_time_1 = 0; //ms, sample-end time, for purpose of result verification

//FFT CONSTANTS and VARIABLES
const double delta_f = 1000 / sample_T; //Hz, FFT frequency spacing
//const uint8_t plot_bins = (double)max_distance  delta_f  range_res
const uint8_t m = 10; //fft parameter
uint8_t tile[4][128]; //2D array of 'tile' objects
int32_t x_bin[128]; //128-wide vector for display pixels
int16_t fft_mag[sample_N];
const uint8_t target_N = 1; // 1-3 target capable
uint8_t MTI_flag = 0; //default MTI off, turn on in FMCW mode with momentary switch

//PLOT CONSTANTS AND VARIABLES
const uint8_t plot_wait = 5; //for time-domain plots, reduces update rate
uint8_t plot_wait_count = 0;
uint8_t vert_bar_x2[8] = {0, 255, 0, 0, 0, 0, 0, 0};
uint8_t vert_bar_x4[8] = {0, 0, 0, 255, 0, 0, 0, 0};

typedef union //data structure for efficient plotting
{
  uint32_t values;
  uint8_t val[4];
} column_tile;

//MEASUREMENT STATES
enum state {DOPPLER, FMCW} state = DOPPLER;

//MAKE LCD INSTANCE
U8X8_ST7565_NHD_C12864_4W_HW_SPI u8x8(/* cs=*/ lcd_CS, /* dc=*/ lcd_REG, /* reset=*/ lcd_RST);

//MAKE SPISettings INSTANCE for pll
SPISettings pll_spi(1000000, MSBFIRST, SPI_MODE0);
SPISettings pot_spi(500000, MSBFIRST, SPI_MODE0);

//TEST VARIABLES
uint8_t test_flag = 1;

void setup() {
  // put your setup code here, to run once:
  pin_init();
  Serial.begin(9600); //Teensy USB Serial accepts any speed
  SPI.begin();
  u8x8.begin();
  pll_init();
  //while (!Serial);
  delay(1000);
  Serial.println("Serial Port Initialized");
  pre();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (state == FMCW && MTI_flag)
  {
    while (digitalRead(SYNC_pin)); //wait for new cycle
    while (!digitalRead(SYNC_pin)); //wait for positive transition
  }
  actual_time_0 = millis();
  adc_timer = micros();
  for (buffer_index = 0; buffer_index < sample_N;)
  {
    while (adc_timer > micros());
    adc_timer = adc_timer + sample_delta; //every sample_delta microseconds...
    adc_buffer_new[buffer_index++] = analogRead(ADC_pin); //record adc values
  }
  //if all samples are taken, start calculations

  actual_time_1 = millis() - actual_time_0;
  //  Serial.print("Time Elapsed (ms): ");
  //  Serial.println(actual_time_1);
  buffer_index = 0; //reset index
  current_rms_value = zero_average_and_rms(adc_buffer_new); //make samples bipolar, return rms value
  pot_update(current_rms_value);
  switch (state)
  {
    case DOPPLER:
      stitch_sandwich_stacker(adc_buffer_new); //stack frequency data from zero-crossings into bins and update output
      if (plot_wait_count++ == plot_wait)
      {
        plot_samples(adc_buffer_new, sample_N, 1);
        plot_wait_count = 0;
      }
      break;
    case FMCW:
      //***************UNCOMMENT FOR MTI
      if (MTI_flag)
      {
        moving_target_indication(); //update difference buffer
        update_fft(adc_buffer_diff, sample_N); //perform fft from adc_buffer_diff
      } else {
        update_fft(adc_buffer_new, sample_N); //perform fft from adc_buffer_new
      }

      if (plot_wait_count++ == plot_wait)
      {
        plot_samples(fft_mag, 128, 0); //each FFT div is ~1m in range
        plot_wait_count = 0;
      }
      break;
  }
  switch_poll(state);

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
  for (uint16_t j = 0; j < sample_N; j++)
  {
    adc_buffer_new[j] = sample[j] - diff;
  }
  for (uint16_t j = 0; j < sample_N; j++)
  {
    squaresum = squaresum + (adc_buffer_new[j] * adc_buffer_new[j]);
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
    //Serial.println(adc_buffer_new[buffer_index]); //***TEST CODE***
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
  //  Serial.print("Frequency (Hz): ");
  //  Serial.println(freq_float, 1); //print average frequency
  //  Serial.print("Speed (m/s): ");
  //  Serial.println(speed_float, 1); //print average speed
  u8x8.clearLine(2);
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.noInverse();
  u8x8.setCursor(0, 2);
  u8x8.print("Speed=");
  u8x8.print(speed_float, 1);
  u8x8.print(" m/s");
  active_light = light_offset - bin_peak_search(speed_bin);
  if (old_light != active_light && active_light <= light_offset && active_light >= light_offset - 9)
  {
    digitalWrite((old_light), HIGH);
    digitalWrite((active_light), LOW);
  }
  buffer_index = 0; //reset adc_buffer_new index
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

void switch_poll(uint8_t current_state)
{
  uint32_t f_new = f_current;
  uint16_t momentary_sw_in = 0;
  uint8_t toggle_sw_in = digitalRead(sw_TOG_pin);
  if (current_state != toggle_sw_in)
    //if the current state isn't equal to TOGGLE switch (0 or 1, in both cases)
  {
    delay(10); //debounce
    if (current_state != digitalRead(sw_TOG_pin))
      //if still not equal
    {
      if (toggle_sw_in)
      {
        state = FMCW;
        pll_init();
        f_current = f_default;
      }
      else if (!toggle_sw_in)
      {
        state = DOPPLER;
        pll_init();
        f_current = f_default;
      }
      Serial.println("toggled");
      pre();
    }
  }
  momentary_sw_in = analogRead(sw_MOM_pin); //measure MOMENTARY switch
  if (momentary_sw_in > 819)
  {
    while (analogRead(sw_MOM_pin) > 819)
    {
      switch (state)
      {
        case DOPPLER:
          f_new -= (pll_channel_spacing * 3);
          delay(150);
          if (f_new < f_lim_lower) f_new = f_lim_upper - (f_lim_upper % (pll_channel_spacing * 3)); //wraparound
          break;
        case FMCW:
          MTI_flag = 0;
          u8x8.setCursor(1, 3);
          u8x8.print("MTI disabled");
          delay(250);
          break;
      }
    }
  }
  else if (momentary_sw_in < 205)
  {
    while (analogRead(sw_MOM_pin) < 205)
    {
      switch (state)
      {
        case DOPPLER:
          f_new += (pll_channel_spacing * 3);
          delay(50);
          if (f_new > f_lim_upper) f_new = f_lim_lower + pll_channel_spacing - (f_lim_lower % (pll_channel_spacing * 3)); //wraparound
          break;
        case FMCW:
          MTI_flag = 1;
          u8x8.setCursor(1, 3);
          u8x8.print("MTI enabled");
          delay(250);
          break;
      }
    }
  }

  if (f_new != f_current)
  {
    Serial.print("New Frequency Set (KHz) : ");
    Serial.println(f_new);
    freq_set(f_new);
    f_current = f_new;
  }
  pre();
}

void pin_init(void)
{
  //  pinMode(lcd_CS, OUTPUT);
  //  pinMode(lcd_RST, OUTPUT);
  //  pinMode(lcd_REG, OUTPUT);
  pinMode(pll_CS, OUTPUT);
  pinMode(pot_CS, OUTPUT);
  pinMode(ADC_pin, INPUT);
  pinMode(SYNC_pin, INPUT);
  pinMode(sw_MOM_pin, INPUT);
  pinMode(sw_TOG_pin, INPUT);
  for (active_light = light_offset - 9; active_light <= light_offset; active_light++)
  {

    pinMode(active_light, OUTPUT);
    digitalWrite(active_light, LOW);
    delay(20);
    digitalWrite(active_light, HIGH);
  }
  Serial.println("Pin Directions initialized");
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
  Serial.print("Setting INIT Latch, transfer input = ");
  Serial.println(0x00000093, BIN);
  pll_24b_transfer(0x00000093); //supposed to be C3?
  Serial.print("Setting INIT Latch, transfer input = ");
  Serial.println(0x00000092, BIN);
  pll_24b_transfer(0x00000092); //supposed to be C3?

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
  f_current = freq * 3;
}

void pre(void)
{
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
  u8x8.clear();

  u8x8.inverse();
  u8x8.print(" KISSintel ");
  if (!state)
  {
    u8x8.print("DPLR ");
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.noInverse();
    u8x8.setCursor(0, 1);
    u8x8.print("Ch: ");
    u8x8.print(f_current);
    u8x8.print(" KHz");
    u8x8.setCursor(0, 2);
  }
  else if (state)
  {
    u8x8.print("FMCW ");
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.noInverse();
    u8x8.setCursor(4, 3);
    u8x8.print("50 m");
    u8x8.drawTile(6, 3, 1, vert_bar_x2);
    u8x8.setCursor(9, 3);
    u8x8.print("100 m");
    u8x8.drawTile(12, 3, 1, vert_bar_x4);
  }
}

void update_fft(int16_t *samples, uint16_t sample_length)
{
  for (buffer_index = 0; buffer_index < sample_length; buffer_index++)
  {
    fft_mag[buffer_index] = samples[buffer_index];
  }
  fix_fftr(fft_mag, m, 0);
}

void MCP41010Write(uint8_t value)
{
  digitalWrite(pot_CS, LOW);
  SPI.beginTransaction(pot_spi);
  SPI.transfer(0b00000000); // This tells the chip to set the pot
  SPI.transfer(value);     // This tells it the pot position
  digitalWrite(pot_CS, HIGH);
}

void pot_update(uint16_t rms_in)
{
  //  Serial.print("INPUT RMS: ");
  //  Serial.println(rms_in);
  int16_t rms_diff = rms_target - rms_in;
  if (abs(rms_diff) > rms_hysteresis)
  {
    pot_pos = pot_pos - loop_gain * rms_diff / 100; //recalculate potentiometer position
    if (pot_pos > 255) pot_pos = 255; //protect from 8-bit overflow
    if (pot_pos < 1) pot_pos = 1; //protect from lockup
    if (old_pos != pot_pos)
    {
      MCP41010Write((byte)pot_pos); //cast to byte, set position, update gain
      Serial.print("NEW POT POSITION: ");
      Serial.println(pot_pos);
      old_pos = pot_pos;
    }
  }
}

void plot_samples(int16_t *samples, uint16_t samples_len, uint8_t avg_or_peak)
//plots input across the bottom 4 rows of display, INPUT MUST BE LONGER THAN 128 VALUES
//samples_len MUST STAY INSIDE OF samples VECTOR
//avg_or_peak is 1 for averaging, 0 for peak measurement. PEAK MEASUREMENT ONLY VALID FOR POSITIVE-VALUED INPUTS
{
  column_tile column;
  buffer_index = 0;
  uint8_t value_N = 0;
  int16_t bin_min = 0;
  int16_t bin_max = 0;
  uint8_t x_bin_index = 0;
  uint8_t row_index = 0;
  uint16_t range = 0;
  for (x_bin_index = 0; x_bin_index < 128; x_bin_index++) //step through bins
  {
    x_bin[x_bin_index] = 0;
    while (buffer_index < ((x_bin_index + 1) * samples_len / 128)) //bin input samples
    {
      if (avg_or_peak) //if input is 1, perform averaging
      {
        x_bin[x_bin_index] += samples[buffer_index++]; //add to bin the sample, increment sample index
        value_N++;
      } else { //if input is 0, perform peak measurement
        if (x_bin[x_bin_index] < samples[buffer_index]) x_bin[x_bin_index] = samples[buffer_index];
        buffer_index++;
      }
    }
    x_bin[x_bin_index] = -x_bin[x_bin_index]; //invert for proper plotting on display
    if (avg_or_peak)
    {
      if (!value_N) x_bin[x_bin_index] /= value_N; //avoid divide by 0
      value_N = 0;
    }
    if (x_bin[x_bin_index] > bin_max) bin_max = x_bin[x_bin_index]; //record max
    else if (x_bin[x_bin_index] < bin_min) bin_min = x_bin[x_bin_index]; //record min
  }
  range = bin_max - bin_min;
  for (x_bin_index = 0; x_bin_index < 128; x_bin_index++) //scale to y axis
  { //goal is to map all values to 0-31 values
    x_bin[x_bin_index] = (x_bin[x_bin_index] - bin_min) * 32 / range - 0.5;
    column.values = 1 << x_bin[x_bin_index];
    if (!avg_or_peak) column.values = ~(column.values - 1); //create solid bars for peak measurements
    for (row_index = 0; row_index < 4; row_index++)
    {
      tile[row_index][x_bin_index] = column.val[row_index];
    }
  }
  //bin samples in x axis (x128)
  //bin lower limit = bin# * samples_len / 128
  //average or peak measure time bins
  //bin samples in y axis (x32)
  for (row_index = 0; row_index < 4; row_index++)
  {
    u8x8.drawTile(/* tile x */ 0,/* tile y */ 4 + row_index,/* nTiles */ 16,/* tile ptr */ tile[row_index]);
  }
}

void moving_target_indication(void)
{
  for (buffer_index = 0; buffer_index < sample_N; buffer_index++)
  {
    adc_buffer_diff[buffer_index] = adc_buffer_new[buffer_index] - adc_buffer_old[buffer_index];
    adc_buffer_old[buffer_index] = adc_buffer_new[buffer_index];
  }
}
