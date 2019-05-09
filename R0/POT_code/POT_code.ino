#include <SPI.h>
#include <U8x8lib.h>

#define lcd_CS 10 //chip select pin for LCD
#define lcd_RST 14 //active LOW reset pin for LCD
#define lcd_REG 15 //register select pin for LCD, 0: instruction, 1: data
#define pot_CS 16
#define ADC_pin 18 //waveform input sampling pin

U8X8_ST7565_NHD_C12864_4W_HW_SPI u8x8(/* cs=*/ lcd_CS, /* dc=*/ lcd_REG, /* reset=*/ lcd_RST);

SPISettings pot_spi(500000, MSBFIRST, SPI_MODE0);

void setup() {
  // put your setup code here, to run once:
  pinMode(pot_CS, OUTPUT);
  pinMode(ADC_pin, INPUT);
  SPI.begin();
  u8x8.begin();
  delay(100);
  pre();
  Serial.begin(9600);
  //while (!Serial);
  delay(100);
  Serial.println("initialized.");
}

void loop() {
  // put your main code here, to run repeatedly:
  MCP41010Write(50);
  delay(1000);
  update_display();
  MCP41010Write(100);
  delay(1000);
  update_display();
  MCP41010Write(150);
  delay(1000);
  update_display();
  MCP41010Write(200);
  delay(1000);
  update_display();
  MCP41010Write(250);
  delay(1000);
  update_display();
  Serial.println("cycled");
}

void MCP41010Write(uint8_t value)
{
  // Note that the integer value passed to this subroutine
  // is cast to a byte
  digitalWrite(pot_CS, LOW);
  SPI.beginTransaction(pot_spi);
  SPI.transfer(0b00000000); // This tells the chip to set the pot
  SPI.transfer(value);     // This tells it the pot position
  digitalWrite(pot_CS, HIGH);
}

void pre(void)
{
  u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
  u8x8.clear();
  u8x8.inverse();
  u8x8.print(" KISSintel TEST");
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.noInverse();
  u8x8.setCursor(0, 1);
}

void update_display(void)
{
  u8x8.clearLine(1);
  u8x8.setCursor(0, 1);
  u8x8.print("adc: ");
  u8x8.print(analogRead(ADC_pin), 1);
}
