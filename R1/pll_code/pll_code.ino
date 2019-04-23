// RESOURCES:
// teensy 3.2 ADC: https://forum.pjrc.com/threads/25532-ADC-library-update-now-with-support-for-Teensy-3-1

// INCLUDES (fft, pll library, etc)
#include "SPI.h"

//GLOBAL VARIABLES
const uint8_t pll_CS = 10; //chip select pin for pll
const uint8_t ADC_pin = A9; //waveform input sampling pin
const uint8_t pll_P = 32; //default prescaler
const uint16_t f_osc = 10000; //reference oscillator frequency in KHz
const uint32_t f_default = 5800000; //default target frequency in KHz
const uint16_t sample_N = 800; //number of ADC samples to take at a time
const uint16_t sample_T = 100;
const uint16_t sample_delta = 1000 * sample_T / sample_N; //change in time between ADC samples in microseconds
const uint16_t sample_wait = 200; //time between sample periods in milliseconds
const uint8_t max_velocity = 20; //maximum target velocity in m/s
const uint32_t c = 299792458; //speed of light in m/s

SPISettings pll_spi(10000000, MSBFIRST, SPI_MODE0);

int16_t adc_buffer[sample_N]; // initialize ADC buffer
uint16_t buffer_index = 0;
uint8_t general_index = 0;
uint32_t adc_timer = 0; //microseconds adc_timer
int sample_flag = 1; //if true, allows start of next sampling period (provided calc flag)
int calc_flag = 0;
int numeric_flag = 1;
uint32_t f_new = 0;
uint32_t sample_flag_timer = 0;

enum state {DOPPLER, FMCW} state = DOPPLER; //will eventually be controlled by on-board switch

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(ADC_pin, INPUT);
  pinMode(pll_CS, OUTPUT);
  pll_init();
  freq_set(f_default);
  //  ADC *adc = wave_in ADC();
  //  adc->enableInterrupts(ADC_0);
  sample_flag_timer = millis();
  while (!Serial) ;
  Serial.print("Setup Finished. Reference frequency expected: ");
  Serial.print(f_osc);
  Serial.println(" KHz.");
  Serial.print("PLL frequency set to ");
  Serial.print(f_default);
  Serial.println(" KHz.");
  Serial.println("To change PLL frequency, enter Target Frequency in KHz");
}

void loop()
{
  // put your main code here, to run repeatedly:
  //Serial.println(sample_flag);
  if (sample_flag) //if not done sampling
  {
    if (adc_timer <= micros())
    {
      adc_timer = adc_timer + sample_delta; //every sample_delta microseconds...
      adc_buffer[buffer_index++] = analogRead(ADC_pin); //record adc values
    }
    if (buffer_index == sample_N)
    {
      buffer_index = 0; //reset index
      sample_flag = 0; //stop samples
      calc_flag = 1; //start calculations
    }

  }
  if (!sample_flag && calc_flag) //if not taking samples and ready to calculate
  {
    zero_average(adc_buffer); //make samples bipolar
    switch (state)
    {
      case DOPPLER: //purpose is to control led bar graph to indicate measured speed

        break;
      case FMCW: //purpose is to output distance information about target(s)

        break;
    }
    calc_flag = 0; //stop calcs
  }

  if (sample_flag_timer <= millis() && !calc_flag)
    //if (sample_flag_timer <= millis())
  {
    if (Serial.available()) //check for serial commands
    {
      serial_in(Serial.readString());
    }
  }
  sample_flag_timer += sample_wait;
  sample_flag = 1; //start samples
  adc_timer = micros(); //set current time**************
}

//FUNCTIONS

void pll_init(void)
//frequency of reference oscillator in KHz
{
  //initialize pll by following startup sequence from datasheet (p15)
  SPI.begin();
  SPI.beginTransaction(pll_spi);
  digitalWrite(pll_CS, LOW);
  SPI.transfer16(0); //WILL ONLY WORK IF SHIFT REGISTER PUSHES OUT EXTRA VALUES
  SPI.transfer(0x03); //set control bits to 0b11 to reset
  digitalWrite(pll_CS, HIGH);
  SPI.endTransaction();
}

void freq_set(uint32_t freq)
//sets OUTPUT frequency in KHz, rounded down to nearest 300 KHz.
//ENSURE VALUE IS WITHIN VCO RANGE*3 (after multiplier)
{
  uint8_t i = 0;
  byte data_buffer[3];
  //calculations
  freq = freq - freq % 300; //round down input
  uint16_t R = f_osc / 50; //find R divider
  uint32_t N = freq / 300; //find N divider
  uint32_t B = N / pll_P; //find B parameter
  uint8_t  A = N - (B * pll_P); //find A parameter

  //N divider
  A = A << 2; //shift A parameter to correct location
  B = B << 7; //shift B parameter to correct location
  uint32_t data_temp = 0; //init temp variable
  data_temp |= A; //place A
  data_temp |= B; //place B
  data_temp |= 1; //set N control bit
  //pll uses 21-bit shift register
  for (i = 0; i < 3; i++)
  {
    data_buffer[2 - i] = data_temp >> (i * 8); //bitshift 32 bit int into 3 byte array. Drop the MSByte
  }
  digitalWrite(pll_CS, LOW);
  SPI.beginTransaction(pll_spi);
  SPI.transfer(&data_buffer, 3);
  digitalWrite(pll_CS, HIGH);

  //R divider
  data_temp = 0; //reset temp variable
  R = R << 2; // //shift R parameter to correct location
  data_temp |= R; //place R
  //R requires 0b00 as control bits
  for (i = 0; i < 3; i++)
  {
    data_buffer[2 - i] = data_temp >> (i * 8); //bitshift 32 bit int into 3 byte array. Drop the MSByte
  }
  digitalWrite(pll_CS, LOW);
  SPI.transfer(&data_buffer, 3);
  digitalWrite(pll_CS, HIGH);
  SPI.endTransaction();
}

void zero_average(int16_t sample[]) //offsets input array to zero-average
{
  int32_t sum = 0;
  int16_t diff = 0;
  for (uint16_t j = 0; j < sample_N; j++)
  {
    sum = sum + sample[j];
  }
  diff = sum / sample_N;
  for (int j = 0; j < sample_N; j++)
  {
    sample[j] = sample[j] - diff;
  }
}

void serial_in(String input_string)
{
  input_string.remove(input_string.length() - 1);
  Serial.print("Input string = ");
  Serial.println(input_string);
  for (general_index = 0; general_index < input_string.length(); general_index++)
  {
    if (!isDigit(input_string.charAt(general_index)))
    {
      numeric_flag = 0; //is not valid--not a number
    }
  }
  if (numeric_flag)
  {
    f_new = input_string.toInt();
    if (5725000 <= f_new && f_new <= 5875000)
    {
      freq_set(f_new);
      Serial.println("Frequency Updated");
    } else {
      Serial.println("Input frequency is out of range. Must be between 5725000 and 5875000 (KHz)");
    }
  } else {
    Serial.println("Input string is not a number. Must be number between 5725000 and 5875000 (KHz)");
    numeric_flag = 1;
  }
}
