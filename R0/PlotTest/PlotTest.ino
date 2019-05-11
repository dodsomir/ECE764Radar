#include <U8x8lib.h>
#include "fix_fft.h"

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

typedef union
{
  uint32_t values;
  uint8_t val[4];
} column_tile;

uint16_t buffer_index;
const uint8_t m = 10;
const uint16_t function_length = 1024; // = 2^m
const uint16_t min_f = 100;
double function_value[function_length];
const uint16_t function_scale = 20;
int16_t function_ints[function_length];
//int32_t char_temp[function_length];
//int16_t char_out[function_length];
//int16_t imag_out[function_length];
int16_t fft_real[function_length];
//int16_t fft_imag[function_length];

uint8_t tile[4][128];
int32_t x_bin[128];

U8X8_ST7565_NHD_C12864_4W_HW_SPI u8x8(/* cs=*/ 10, /* dc=*/ 15, /* reset=*/ 14);

void setup(void)
{

  Serial.begin(9600);
  delay(2000);
  make_curve();
  //  int16_to_char(function_ints, function_length);
  for (buffer_index = 0; buffer_index < function_length; buffer_index++)
  {
    fft_real[buffer_index] = function_ints[buffer_index];
  }
  fix_fftr(fft_real, m, 0);
  for (buffer_index = 0; buffer_index < function_length; buffer_index++)
  {
    Serial.println(fft_real[buffer_index]);
  }
  //  char_to_int16();
  //  fft_mag();
  SPI.begin();
  delay(200);
  u8x8.begin();
  delay(200);
  pre();
}

void loop(void)
{
  u8x8.clearLine(2);
  u8x8.setCursor(0, 2);
  u8x8.print("Time Domain");
  plot_samples(function_ints, function_length / min_f * 16, 1);
  delay(5000);
  u8x8.clearLine(2);
  u8x8.setCursor(0, 2);
  u8x8.print("Freq Domain");
  plot_samples(fft_real, function_length / 2, 0);
  delay(5000);
}

void make_curve(void)
{
  for (buffer_index = 0; buffer_index < function_length; buffer_index++)
  {
    function_ints[buffer_index] = (int16_t)function_scale * cos(min_f * 3.14159 * ((double)buffer_index) / function_length) +
                                  (int16_t)function_scale * cos(6 * min_f * 3.14159 * ((double)buffer_index) / function_length);
  }
}

void pre(void)
{
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
  u8x8.clear();
  u8x8.inverse();
  u8x8.print("   Plot  Test   ");
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.noInverse();
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
