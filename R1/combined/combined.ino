//This sketch is for the combination of all radar functions in the ECE764 project

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

const uint16_t pll_channel_spacing = 1000; //channel spacing in KHz

uint32_t f_current = f_default;

enum state {DOPPLER, FMCW} state = DOPPLER; //will eventually be controlled by on-board switch

void setup() {
  // put your setup code here, to run once:
  pin_init();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

}

uint32_t pin_poll(enum state current_state) //returns new frequency
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
  if (current_state == DOPPLER)
  {
    momentary_sw_in = analogRead(sw_MOM_pin);
    if (momentary_sw_in < 205)
    {
      while (analogRead(sw_MOM_pin) < 205)
      {
        f_new -= (pll_channel_spacing * 3);
        delay(20);
        if (f_new < f_lim_lower) f_new = f_lim_upper % (pll_channel_spacing * 3);
      }
    }
    else if (momentary_sw_in > 819)
    {
      while (analogRead(sw_MOM_pin) > 819)
      {
        f_new += (pll_channel_spacing * 3);
        delay(20);
        if (f_new > f_lim_upper) f_new = (f_lim_lower + pll_channel_spacing / 2) % (pll_channel_spacing * 3);
      }
    }

  }
  if (f_new != f_current)
  {
    Serial.print("New Frequency Set (KHz) : ");
    Serial.println(f_new);
  }
  return f_new;
}

void pin_init(void)
{
  pinMode(lcd_CS, OUTPUT);
  pinMode(pll_CS, OUTPUT);
  pinMode(ADC_pin, INPUT);
  pinMode(sw_MOM_pin, INPUT);
  pinMode(sw_TOG_pin, INPUT);
}
