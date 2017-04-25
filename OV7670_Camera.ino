
#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_GFX.h>    // Core graphics library
//#include "Adafruit_ILI9340.h" // Hardware-specific library
#include "ILI9341_due.h" // Hardware-specific library
#include <SPI.h>
#include <SD.h>
#include "Parallel.h"


#define ENABLE_AGC_STUFF
#define ENABLE_GAMMA_STUFF
#define ENABLE_MAGIC_STUFF


#define ADDRESS         0x21 //Define i2c address of OV7670
#define REGISTERS       0xC9 //Define total numbers of registers on OV7076

#define OV_CLKRC 0x11

#define OV_HSTART 0x17
#define OV_HEND 0x18
#define OV_HREF 0x32
#define OV_VSTART 0x19
#define OV_VEND 0x1a
#define OV_VREF  0x03
#define OV_TSLB 0x3a
#define OV_SCALING_PCLK_DELAY 0xa2

#define OV_COM1 0x04
#define OV_COM2 0x09
#define OV_COM3 0x0c
#define OV_COM4 0x0d
#define OV_COM5 0x0e
#define OV_COM6 0x0f
#define OV_COM7 0x12
#define OV_COM8 0x13
#define OV_COM9 0x14
#define OV_COM10 0x15
#define OV_COM14 0x3E  // scaling flag and pclk divider
#define OV_COM15 0x40  //output data format


#define XCLK_FREQ 10 * 1000000

//3v3					3v3
//gnd                   gnd
//reset                 3v3
//pwdn                  gnd
//SIOD					SDA1 + 1k pullup
//SIOC					SCL1 + 1k pullup
#define VSYNC_PIN	10
#define VSYNC_BIT       (REG_PIOC_PDSR & (1 << 29))

#define HREF_PIN 	9
#define HREF_BIT       (REG_PIOC_PDSR & (1 << 21))

#define PCLK_PIN	8  //86kHz
#define PCLK_BIT       (REG_PIOC_PDSR & (1 << 22))

#define XCLK_PIN	7  //10.5MHz
#define D0_PIN		25
#define D1_PIN		26
#define D2_PIN		27
#define D3_PIN		28
#define D4_PIN		14
#define D5_PIN		15
#define D6_PIN		29
#define D7_PIN		11

#define QVGA

#define LANDSCAPE

#ifdef VGA
#define WIDTH           640
#define HEIGHT          480
#endif

#ifdef QVGA
#ifdef LANDSCAPE
#define WIDTH           320
#define HEIGHT          240
#else
#define WIDTH           240
#define HEIGHT          320
#endif
#endif


#define TFT_CS 53
#define _sclk 52
#define _mosi 51
#define _miso 50
#define TFT_DC 49
#define TFT_RST 48
#define _bl 6
#define SD_CS 47

// Using software SPI is really not suggested, its incredibly slow
//Adafruit_ILI9340 tft = Adafruit_ILI9340(_cs, _dc, _mosi, _sclk, _rst, _miso);
// Use hardware SPI
ILI9341_due tft = ILI9341_due(TFT_CS, TFT_DC, TFT_RST);




volatile boolean onFrame = false;
volatile boolean onPixel = false;

void wrReg(byte reg, byte dat)
{
  delay(5);
  Wire1.beginTransmission(ADDRESS);  //Start communication
  Wire1.write(reg);				   //Set the register
  Wire1.write(dat);				   //Set the value
  Wire1.endTransmission();		   //Send data and close communication
}

byte rdReg(byte reg)
{
  delay(1);
  Wire1.beginTransmission(ADDRESS);	//Start communication
  Wire1.write(reg);					//Set the register to read
  Wire1.endTransmission();			//Send data and close communication
  Wire1.requestFrom(ADDRESS, 1);		//Set the channel to read
  while (Wire1.available() < 1);		//Wait for all data to arrive
  return Wire1.read();                //Read the data and return them
}

void printRegister(byte reg, int mode)
{
  char tmpStr[80];
  byte highByte = rdReg(reg); //Read the byte as an integer

  if (mode == 0)
  {
    Serial.print(F("0x")); if (reg < 0x10) Serial.print(0, HEX); Serial.print(reg, HEX);
    Serial.print(F(" : "));
    Serial.print(F("0x")); if (highByte < 0x10) Serial.print(0, HEX); Serial.print(highByte, HEX);
  }
  if (mode == 1)
  {
    Serial.print("Register ");
    sprintf(tmpStr, "%03d", reg); Serial.print(tmpStr);
    Serial.print(" ");
    itoa(reg, tmpStr, 16); sprintf(tmpStr, "0x%02d", atoi(tmpStr)); Serial.print(tmpStr);
    Serial.print(" ");
    itoa(reg, tmpStr, 2); sprintf(tmpStr, "0b%08d", atoi(tmpStr)); Serial.print(tmpStr);
    Serial.print(": ");
    sprintf(tmpStr, "%03d", highByte); Serial.print(tmpStr);
    Serial.print(" ");
    itoa(highByte, tmpStr, 16); sprintf(tmpStr, "0x%02d", atoi(tmpStr)); Serial.print(tmpStr);
    Serial.print(" ");
    itoa(highByte, tmpStr, 2); sprintf(tmpStr, "0b%08d", atoi(tmpStr)); Serial.print(tmpStr);
  }
  Serial.print("\r\n");
}

void printAllRegisters(int mode)
{
  for (byte reg = 0x00; reg <= REGISTERS; reg++)
  {
    printRegister(reg, mode);
  }
}

void setupXCLK()
{
  pmc_enable_periph_clk(PWM_INTERFACE_ID);
  PWMC_ConfigureClocks(XCLK_FREQ * 2, 0, VARIANT_MCK); //freq * period
  PIO_Configure(
    g_APinDescription[XCLK_PIN].pPort,
    g_APinDescription[XCLK_PIN].ulPinType,
    g_APinDescription[XCLK_PIN].ulPin,
    g_APinDescription[XCLK_PIN].ulPinConfiguration);
  uint32_t channel = g_APinDescription[XCLK_PIN].ulPWMChannel;
  PWMC_ConfigureChannel(PWM_INTERFACE, channel, PWM_CMR_CPRE_CLKA, 0, 0);
  PWMC_SetPeriod(PWM_INTERFACE, channel, 2);
  PWMC_SetDutyCycle(PWM_INTERFACE, channel, 1);
  PWMC_EnableChannel(PWM_INTERFACE, channel);
  //pmc_mck_set_prescaler(2);
}

void vsync_end() //frame start
{
  onFrame = true;
}

void pclk_rising() //pixel start
{
  onPixel = true;
}

void setup()
{
  delay(500);
  //	Serial.begin(250000); //can't go any faster otherwise DUE would output garbage
  Serial.begin(115200); //can't go any faster otherwise DUE would output garbage

  pinMode(_bl, OUTPUT);
  digitalWrite(_bl, HIGH);

  tft.begin();
  tft.fillScreen(ILI9341_BLUE);

#ifdef LANDSCAPE
  tft.setRotation((iliRotation)1);  // landscape
#else
  tft.setRotation((iliRotation)2);  //portarait
#endif

  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(2);
  tft.println("OV7670 Test");

  //  output_image();
  //  while (1);


  setupXCLK();
  Wire1.begin();
  //Wire1.setClock(400000); //should work but needs some delay after every read/write. buggy?

  //  setup_registers1();
  //  setup_registers2();
  setup_registers3();

#ifdef ENABLE_AGC_STUFF
  AGC_set();
#endif

#ifdef ENABLE_GAMMA_STUFF
  gamma_set();
#endif

#ifdef ENABLE_MAGIC_STUFF
  magic_set();
#endif

  //	//code to read registers and check if they were written ok
  //	printRegister(0x01, 1);
  //	printRegister(0x12, 1);
  //	printRegister(0x15, 1);
  //	printRegister(0x11, 1);
  //	Serial.print(F("\n"));
  //	printAllRegisters(1);
  //
  //        while(1);

  pinMode(D0_PIN, INPUT);
  pinMode(D1_PIN, INPUT);
  pinMode(D2_PIN, INPUT);
  pinMode(D3_PIN, INPUT);
  pinMode(D4_PIN, INPUT);
  pinMode(D5_PIN, INPUT);
  pinMode(D6_PIN, INPUT);
  pinMode(D7_PIN, INPUT);

  attachInterrupt(VSYNC_PIN, vsync_end, FALLING);
  //attachInterrupt(PCLK_PIN, pclk_rising, RISING); // code hang here. Occur too fast?
}


void output_image(void)
{
  byte r, g, b;
  int pixel;
  int w, h;

  r = 0;  //max
  g = 0;  //max
  b = 0;  //max

#ifdef LANDSCAPE
  tft.setAddrWindow(0, 0, 319, 239);

  for (w = 0; w < 240; w++)
  {
    for (h = 0; h < 320; h++)
    {
      r = h / 2;
      b = w;
#else
  tft.setAddrWindow(0, 0, 239, 319);

  for (w = 0; w < 320; w++)
  {
    for (h = 0; h < 240; h++)
    {
      r = h;
      b = w / 2;
#endif

      //      tft.pushColor(tft.Color565(r, g, b));

      pixel = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
      tft.pushColor(pixel);
      //
      //      Serial.print(w);
      //      Serial.print(" ");
      //      Serial.print(h);
      //      Serial.print(" ");
      //      Serial.print(r);
      //      Serial.print(" ");
      //      Serial.print(g);
      //      Serial.print(" ");
      //      Serial.print(b);
      //      Serial.print(" ");
      //      Serial.println(pixel, HEX);
      //     delay (10);
    }
  }
}

bool singleFrame = true;

void loop()
{
  //We take only one frame for testing purposes.
  //You're free to comment next line to take multiple frames.

  //  if (!singleFrame) return; singleFrame = false;

  tft.setAddrWindow(0, 0, 319, 239);

  onFrame = false;
  //  interrupts();
  while (!onFrame);
  //  noInterrupts();

  // tft.setCursor(0, 0);
  //
  //long pixc = 0;
  //  while (VSYNC_BIT==0) //2349 high
  //      {
  //    while (PCLK_BIT); //wait for low
  //    while (PCLK_BIT == 0); //wait for high
  //    pixc++;
  //    }
  //
  //  Serial.print("Pix count:");
  //  Serial.println(pixc);
  //

  int pixel;

  int rowlength = 0;
  int rowactive = 0;

  int cols = 0;
  long pixc = 0;
  boolean href;

  int store[250];
#define ROWSTART 0

  long start = micros();

  //  for (int i = 0; i < WIDTH * HEIGHT; i++)
  while (VSYNC_BIT == 0)
  {
    if (HREF_BIT && (href == 0))
    {
      if (cols < 250) store[cols] = rowlength;
      cols++;
      rowlength = 0;
      rowactive = 0;
    }
    else
    {
      if (HREF_BIT) rowactive++;
      rowlength++;
    }

    if (HREF_BIT) href = 1;
    else href = 0;

    while (PCLK_BIT); //wait for low
    pixel = (REG_PIOD_PDSR & 0xFF) << 8;
    while (PCLK_BIT == 0); //wait for high
    while (PCLK_BIT); //wait for low
    pixel |= (REG_PIOD_PDSR & 0xFF);
    while (PCLK_BIT == 0); //wait for high

    if ((rowlength >= ROWSTART) && (rowlength < (ROWSTART + 320))) tft.pushColor(pixel);
    pixc++;

  }

  Serial.print("Time: ");
  Serial.println(micros() - start);
  Serial.print("found cols ");
  Serial.println(cols);

  Serial.print("found pix ");
  Serial.println(pixc);

  for (int i = 0; i < cols; i++)
  {
    if (i % 16 == 0) Serial.println();
    else Serial.print(" ");
    Serial.print(store[i]);
  }

  //  for (int i = 0; i < WIDTH * HEIGHT; i++)
  //    {
  //    //pin8 is port C22 as stated here -> http://www.arduino.cc/en/Hacking/PinMappingSAM3X
  //    while (REG_PIOC_PDSR & (1 << 22)); //wait for low
  //    Serial.write(REG_PIOD_PDSR & 0xFF);
  //    while (!(REG_PIOC_PDSR & (1 << 22))); //wait for high
  //  }
  Serial.println(F(" *FRAME_STOP*"));
}





void setup_registers1( void)
{

  //some registers to debug and test
  wrReg(OV_COM7, 0x80); //Reset all the values
  delay(100);

  wrReg(OV_COM3, 0x04);  // enable scaling
  //     wrReg(OV_COM3, 0x00);  // no scaling

  //  wrReg(OV_COM7, 0x10);  //QVGA YUV format,no colour bar
  //  wrReg(OV_COM7, 0x12);  //QVGA YUV format,colour bar
  wrReg(OV_COM7, 0x14);  //QVGA RGB format,no colour bar
  //   wrReg(OV_COM7, 0x16);  //QVGA RGB format,colour bar

  //  wrReg (OV_COM14, 0x00);  // manual scaling disabled
  //    wrReg (OV_COM14, 0x08);  // manual scaling allowed

  wrReg(OV_COM15, 0xd0); // RGB565 range 00-ff needs com7:2 = 1, com7:0 = 0
  //  wrReg(0x40,0x10);  // RGB565 range 10-f0 needs com7:2 = 1, com7:0 = 0



  //	wrReg(0x12, 0x02); //ColorBar
  //wrReg(0x42, 0x08); //ColorBar

  //wrReg(0x15, 0x02); //VSYNC inverted

  // default value gives 5.25MHz pixclock
  //  wrReg(0x11, 10); //slow divider because of slow serial limit 238kHz
  wrReg(0x11, 15); //slow divider because of slow serial limit 120kHz
  //  wrReg(0x11, 30); //slow divider because of slow serial limit 84kHz
  //  wrReg(0x11, 60); //slow divider because of slow serial limit 43kHz
  // wrReg(0x11, 0x82); //Prescaler x3 (10 fps)  1.75MHz


  //wrReg(OV_COM7, 0x80);

  // QVGA values taken from https://github.com/ComputerNerd/ov7670-no-ram-arduino-uno/blob/master/ov7670.c

  wrReg(OV_TSLB,  0x04);
  //wrReg(OV_COM7, 0);

  wrReg(OV_COM14, 0x19);    //0b00011001
  wrReg(0x72, 0x11);  //downscaling V:2/H:2
  //  wrReg(0x72, 0x10);  //downscaling V:2/H:0

  wrReg(0x73, 0xf1);    //PCLK scaling divide by 2
  //  wrReg(0x73, 0xf2);    //PCLK scaling divide by 4
  //  wrReg(0x73, 0xf0);    //PCLK scaling divide by 1 (zero columns)


  //H start:180
  //H end:36
  //V start:10
  //V end:490
  //Manu H: 0x7F
  //Manu L: 0xA2
  //found rows 5005
  //found cols 240
  //found pix 198744
  // *FRAME_STOP*
  //
  //
  wrReg(OV_HSTART, 0x16);   // (becomes 176d + 4 from HREF = 180)
  wrReg(OV_HEND, 0x04);     // (becomes 32d + 4 from HREF = 36) .. no idea
  wrReg(OV_HREF, 0x24);

  wrReg(OV_VSTART, 0x02);
  wrReg(OV_VEND, 0x7a);
  wrReg(OV_VREF, 0x0a);

  //  wrReg(OV_HSTART, 0x16);   // (becomes 176d + 4 from HREF = 180)
  //  wrReg(OV_HEND, 0x3e);     // (becomes 496d + 4 from HREF = 500)
  //  wrReg(OV_HREF, 0x24);
  //
  //  wrReg(OV_VSTART, 0x02);
  //  wrReg(OV_VEND, 0x7a);
  //  wrReg(OV_VREF, 0x0a);
  //
  //display resolution
  int hstart = rdReg(OV_HSTART) << 3; //high 8 bits
  hstart |= rdReg(OV_HREF) & 0x07;
  int hend = rdReg(OV_HEND) << 3; //high 8 bits
  hend |= (rdReg(OV_HREF) & 0x38) >> 3;    //7654 3210
  int vstart = rdReg(OV_VSTART) << 2; //high 8 bits
  vstart |= rdReg(OV_VREF) & 0x03;
  int vend = rdReg(OV_VEND) << 2; //high 8 bits
  vend |= (rdReg(OV_VREF) & 0x0c) >> 2;  //7654 3210

  //  Serial.print("H start:");
  //  Serial.println(hstart);
  //  Serial.print("H end:");
  //  Serial.println(hend);
  //  Serial.print("V start:");
  //  Serial.println(vstart);
  //  Serial.print("V end:");
  //  Serial.println(vend);
  //  Serial.print("Manu H: 0x");
  //  Serial.println(rdReg(0x1c), HEX);
  //  Serial.print("Manu L: 0x");
  //  Serial.println(rdReg(0x1d), HEX);

}
















void setup_registers2( void)
{

  wrReg(OV_COM7, 0x80); //Reset all the values
  delay(5);    // specs say 1mS

  wrReg(OV_COM7, 0x14);  //QVGA RGB format,no colour bar
  //  wrReg(OV_COM7, 0x24);  //CIF RGB format,no colour bar
  //  wrReg(OV_COM7, 0x0C);  //QCIF RGB format,no colour bar

  wrReg(OV_COM15, 0xd0); // RGB565 range 00-ff needs com7:2 = 1, com7:0 = 0

  // default value gives 5.25MHz pixclock
  //  wrReg(0x11, 10); //slow divider because of slow serial limit 238kHz
  wrReg(0x11, 30); //slow divider because of slow serial limit 84kHz
  //  wrReg(0x11, 60); //slow divider because of slow serial limit 43kHz
  // wrReg(0x11, 0x82); //Prescaler x3 (10 fps)  1.75MHz

  //  // QVGA values taken from https://github.com/ComputerNerd/ov7670-no-ram-arduino-uno/blob/master/ov7670.c
  //  wrReg(OV_TSLB,  0x04);
  //  //wrReg(OV_COM7, 0);
  //  wrReg(OV_COM14, 0x19);    //0b00011001
  //  wrReg(0x72, 0x11);  //downscaling V:2/H:2
  ////  wrReg(0x72, 0x10);  //downscaling V:2/H:0
  //
  //  wrReg(0x73, 0xf1);    //PCLK scaling divide by 2
  ////  wrReg(0x73, 0xf2);    //PCLK scaling divide by 4
  ////  wrReg(0x73, 0xf0);    //PCLK scaling divide by 1 (zero columns)
  //

  //H start:180
  //H end:36
  //V start:10
  //V end:490
  //Manu H: 0x7F
  //Manu L: 0xA2
  //found rows 5005
  //found cols 240
  //found pix 198744
  // *FRAME_STOP*
  //
  //

  //  wrReg(OV_HSTART, 0x16);   // (becomes 176d + 4 from HREF = 180)
  //  wrReg(OV_HEND, 0x04);     // (becomes 32d + 4 from HREF = 36) .. no idea
  //  wrReg(OV_HREF, 0x24);
  //
  //  wrReg(OV_VSTART, 0x02);
  //  wrReg(OV_VEND, 0x7a);
  //  wrReg(OV_VREF, 0x0a);

  //  wrReg(OV_HSTART, 0x16);   // (becomes 176d + 4 from HREF = 180)
  //  wrReg(OV_HEND, 0x3e);     // (becomes 496d + 4 from HREF = 500)
  //  wrReg(OV_HREF, 0x24);
  //
  //  wrReg(OV_VSTART, 0x02);
  //  wrReg(OV_VEND, 0x7a);
  //  wrReg(OV_VREF, 0x0a);
  //
  //display resolution
  int hstart = rdReg(OV_HSTART) << 3; //high 8 bits
  hstart |= rdReg(OV_HREF) & 0x07;
  int hend = rdReg(OV_HEND) << 3; //high 8 bits
  hend |= (rdReg(OV_HREF) & 0x38) >> 3;    //7654 3210
  int vstart = rdReg(OV_VSTART) << 2; //high 8 bits
  vstart |= rdReg(OV_VREF) & 0x03;
  int vend = rdReg(OV_VEND) << 2; //high 8 bits
  vend |= (rdReg(OV_VREF) & 0x0c) >> 2;  //7654 3210

  Serial.print("H start:");
  Serial.println(hstart);
  Serial.print("H end:");
  Serial.println(hend);
  Serial.print("V start:");
  Serial.println(vstart);
  Serial.print("V end:");
  Serial.println(vend);



  Serial.print("Com11: 0x");
  Serial.println(rdReg(0x3b), HEX);

  Serial.print("EXHCH: 0x");
  Serial.println(rdReg(0x2a), HEX);

  Serial.print("EXHCL: 0x");
  Serial.println(rdReg(0x2b), HEX);

}



void setup_registers3( void)
{
  byte tmp;

#define MAIN_CLOCK_PLL  0  // 0 off, 1 X4, 2 X6, 3 X8
#define MAIN_CLOCK_DIVIDER 15

#define QVGA_SELECT 0x10
#define CIF_SELECT 0x20
#define QCIF_SELECT 0x08
#define VGA_SELECT 0x00  // not sure if so

#define YUV_MODE 0
#define RGB_MODE 4
#define RAW_BAYER_MODE  1
#define PROCESS_BAYER_MODE  5

#define USE_OUTPUT_SCALE   QVGA_SELECT
#define USE_OUTPUT_FORMAT  RGB_MODE

#define SCALE_ENABLE 1
#define DCW_ENABLE 0

#define AUTO_WINDOW_ENABLE 1

 #define V_START 10
 #define V_END 250
 #define H_START 120
 #define H_END 640
 
#define HORIZ_DOWN_SAMPLE 1  // 0..3  2 doubles image horizontally on qvga
#define VERT_DOWN_SAMPLE 1   //0 .. 3
#define PIX_CLK_DIV 2        //0 .. 3

#define SCALING_MYSTERY_BIT 0
#define SCALING_PCLK_DELAY 2

//test patterns
#define SCALING_XSC_ON0 0    // 0 = none, 8bar  1 = shift1,grey fade
#define SCALING_XSC_ON1 0    // 0 = none, shifting1  1 = 8bar,grey fade
#define SCALING_XSC_SCALE 0x3a  // scale 0..127
#define SCALING_YSC_SCALE 0x35  // scale 0..127

#define AEC_AVERAGE_WINDOW 0  // 0 normal, 1 1/2, 2 1/4, 3 1/8?
#define COLOUR_BAR 0  // 1 enable
#define FLIP_H 0    // 1 = mirror
#define FLIP_V 0    // 1 = invert

  wrReg(OV_COM7, 0x80); //Reset all the values
  delay(5);    // specs say 1mS


  wrReg(OV_CLKRC, MAIN_CLOCK_DIVIDER);
  wrReg(0x6b, 0x0a | (MAIN_CLOCK_PLL << 6)); //PLL controller and reserved


  wrReg(OV_COM7, USE_OUTPUT_SCALE | USE_OUTPUT_FORMAT);  //QVGA RGB format,no colour bar
  wrReg(0x8c, 0x00);     // < -- NEW, disable RGB444 ? required ?
  wrReg(OV_COM1, 0x00);     // < -- NEW, disable RGB656 ? required ?
  
  wrReg(OV_COM15, 0x90); // RGB565 range 01-fe needs com7:2 = 1, com7:0 = 0
  
  wrReg(OV_COM9, 0x18);     // AGC ceiling
  

  // default value gives 5.25MHz pixclock


  tmp = rdReg(OV_COM3);
  tmp &= 0xf3;
  wrReg(OV_COM3, tmp | SCALE_ENABLE<<3 | DCW_ENABLE<<2);   // UNLOCK !

#if AUTO_WINDOW_ENABLE 
  tmp = rdReg(OV_TSLB);
  tmp != 0x01;
  wrReg(OV_TSLB, tmp | AUTO_WINDOW_ENABLE);
#else
  tmp = rdReg(OV_TSLB);
  tmp &= 0x7e;
  wrReg(OV_TSLB, tmp | AUTO_WINDOW_ENABLE);

  wrReg(OV_HSTART, H_START >>3);
  wrReg(OV_HEND, H_END >>3);
  tmp = rdReg(OV_HREF);
  tmp &= 0xc0;
  wrReg(OV_HREF, tmp | (H_END & 0x07)<<3 | (H_START & 0x07));

  wrReg(OV_VSTART, V_START>>2);
  wrReg(OV_VEND,V_END>>2);
  tmp = rdReg(OV_VREF);
  tmp &= 0xf0;
  wrReg(OV_VREF,tmp | (V_END & 0x03)<<2 | (V_START & 0x03));
#endif

  wrReg(OV_COM3, 0x0c);   // down sample enable
  
  wrReg(0x72, (VERT_DOWN_SAMPLE << 4)  | HORIZ_DOWN_SAMPLE);  // down sample rate 	DVE: was 0x11 240 lines

  wrReg(0x73, 0xf0 | PIX_CLK_DIV);      //SCALING PCLK:  for DPSP 1  Scaling Pixel Clock Devider	DVE: was 0xf0

tmp = rdReg (OV_COM14);
wrReg(OV_COM14, tmp | 0x10 | 1);  // NOTE bit 3 does odd things, not sure whats going on, odd alternate length lines
//  wrReg(OV_COM14, 0x18 | PIX_CLK_DIV); // allow scaling and PCLK divider 1:783 pix

  wrReg(OV_SCALING_PCLK_DELAY, SCALING_MYSTERY_BIT << 7 | SCALING_PCLK_DELAY); //19/ SCALING PCLK: Scaling Pixel Delay


  wrReg(0x42, AEC_AVERAGE_WINDOW << 6 | COLOUR_BAR << 3); //ColorBar enabled
  tmp = rdReg(OV_COM4);
  wrReg(OV_COM4, tmp | AEC_AVERAGE_WINDOW << 4);

  wrReg(0x70, (SCALING_XSC_ON0 << 7) | SCALING_XSC_SCALE);  // SCALING XSC : Horizontal Scale Factor	DVE: was 0x3a
  wrReg(0x71, (SCALING_XSC_ON1 << 7) | SCALING_YSC_SCALE);  // SCALING YSC : Vertical Scale Factor DVE: was 0x35

  wrReg(OV_COM10, 0x00);//20/ COM10 : VSYNC , HREF , PCLK Settings

  tmp = rdReg(0x1e);
  wrReg(0x1e, tmp | FLIP_H << 5 | FLIP_V << 4);


  Serial.print("H start:");
  Serial.println((rdReg(OV_HSTART) << 3) | (rdReg(OV_HREF) & 0x07));
  Serial.print("H end:");
  Serial.println((rdReg(OV_HEND) << 3) | ((rdReg(OV_HREF) & 0x38) >> 3));
  Serial.print("V start:");
  Serial.println((rdReg(OV_VSTART) << 2) | (rdReg(OV_VREF) & 0x03));
  Serial.print("V end:");
  Serial.println((rdReg(OV_VEND) << 2) | ((rdReg(OV_VREF) & 0x0c) >> 2));
  Serial.print("dummy pixels: 0x");
  Serial.println(rdReg(0x2a) << 8 | rdReg(0x2b), HEX);

  //
  //  Serial.print("EXHCH: 0x");
  //  Serial.println(rdReg(0x2a), HEX);
  //
  //  Serial.print("EXHCL: 0x");
  //  Serial.println(rdReg(0x2b), HEX);

}


void gamma_set(void)
{
  wrReg(0x7a, 0x20);//21/ SLOP : Gamma Curve Highest Segment Slope
  wrReg(0x7b, 0x10);//22/ GAM1 : Gamme Curve 1st Segment
  wrReg(0x7c, 0x1e);//23/ GAM2 : Gamme Curve 2st Segment
  wrReg(0x7d, 0x35);//24/ GAM3 : Gamme Curve 3st Segment
  wrReg(0x7e, 0x5a);//25/ GAM4 : Gamme Curve 4st Segment
  wrReg(0x7f, 0x69);//26/ GAM5 : Gamme Curve 5st Segment
  wrReg(0x80, 0x76);//27/ GAM6 : Gamme Curve 6st Segment
  wrReg(0x81, 0x80);//28/ GAM7 : Gamme Curve 7st Segment
  wrReg(0x82, 0x88);//29/ GAM8 : Gamme Curve 8st Segment
  wrReg(0x83, 0x8f);//30/ GAM9 : Gamme Curve 9st Segment
  wrReg(0x84, 0x96);//31/ GAM10: Gamme Curve 10st Segment
  wrReg(0x85, 0xa3);//32/ GAM11: Gamme Curve 11st Segment
  wrReg(0x86, 0xaf);//33/ GAM12: Gamme Curve 12st Segment
  wrReg(0x87, 0xc4);//34/ GAM13: Gamme Curve 13st Segment
  wrReg(0x88, 0xd7);//35/ GAM14: Gamme Curve 14st Segment
  wrReg(0x89, 0xe8);//36/ GAM15: Gamme Curve 15st Segment
}



void AGC_set (void)
{
  wrReg(0x13, 0x00);//37/ COM8 : Fast AGC/AEC Algorithm
  wrReg(0x00, 0x00);//38/ GAIN
  wrReg(0x10, 0x00);//39/ AECH
  wrReg(0x0d, 0x00);//40/ COM4 :
  wrReg(0x14, 0x18);//41/ COM9 : Automatic Gain Ceiling : 8x
  wrReg(0xa5, 0x05);//42/ BD50MAX: 50 Hz Banding Step Limit
  wrReg(0xab, 0x07);//43/ BD60MAX: 60 Hz Banding Step Limit
  wrReg(0x24, 0x95);//44/ AGC - Stable Operating Region Upper Limit
  wrReg(0x25, 0x33);//45/ AGC - Stable Operating Region Lower Limit
  wrReg(0x26, 0xe3);//46/ AGC - Fast Mode Operating Region
  wrReg(0x9f, 0x78);//47/ HAECC1 : Histogram based AEC Control 1
  wrReg(0xa0, 0x68);//48/ HAECC2 : Histogram based AEC Control 2
  wrReg(0xa1, 0x03);//49/ Reserved
  wrReg(0xa6, 0xd8);//50/ HAECC3 : Histogram based AEC Control 3
  wrReg(0xa7, 0xd8);//51/ HAECC4 : Histogram based AEC Control 4
  wrReg(0xa8, 0xf0);//52/ HAECC5 : Histogram based AEC Control 5
  wrReg(0xa9, 0x90);//53/ HAECC6 : Histogram based AEC Control 6
  wrReg(0xaa, 0x94);//54/ HAECC7 : AEC Algorithm Selection
  wrReg(0x13, 0xe5);//55/ COM8 : Fast AGC Algorithm, Unlimited Step Size , Banding Filter ON, AGC and AEC enable.
}


void magic_set (void)

{
  wrReg(0x0e, 0x61);//56/ COM5 : Reserved
  wrReg(0x0f, 0x4b);//57/ COM6 : Reserved
  wrReg(0x16, 0x02);//58/ Reserved
  //  wrReg(0x1e, 0x07);//59/ MVFP : Mirror/Vflip disabled ( 0x37 enabled ) //DVE: was 0x07 is now 0x2f so mirror ENABLED. Was required for my TFT to show pictures without mirroring (probably my TFT setting is also mirrored)
  wrReg(0x21, 0x02);//60/ Reserved
  wrReg(0x22, 0x91);//61/ Reserved
  wrReg(0x29, 0x07);//62/ Reserved
  wrReg(0x33, 0x0b);//63/ Reserved
  wrReg(0x35, 0x0b);//64/ Reserved
  wrReg(0x37, 0x1d);//65/ Reserved
  wrReg(0x38, 0x71);//66/ Reserved
  wrReg(0x39, 0x2a);//67/ Reserved
  wrReg(0x3c, 0x78);//68/ COM12 : Reserved
  wrReg(0x4d, 0x40);//69/ Reserved
  wrReg(0x4e, 0x20);//70/ Reserved
  //  wrReg(0x69, 0x00);//71/ GFIX : Fix Gain for RGB Values
  //  wrReg(0x6b, 0x80);//DEBUG DBLV: aantal frames per seconde dmv PCLK hoger of lager, staat nu op INPUT CLK x6 en INT regulator enabled
  wrReg(0x74, 0x10);//73/ Reserved
  wrReg(0x8d, 0x4f);//74/ Reserved
  wrReg(0x8e, 0x00);//75/ Reserved
  wrReg(0x8f, 0x00);//76/ Reserved
  wrReg(0x90, 0x00);//77/ Reserved
  wrReg(0x91, 0x00);//78/ Reserved
  wrReg(0x92, 0x00);//79/ Reserved
  wrReg(0x96, 0x00);//80/ Reserved
  wrReg(0x9a, 0x00);//81/ Reserved
  wrReg(0xb0, 0x84);//82/ Reserved
  wrReg(0xb1, 0x0c);//83/ Reserved
  wrReg(0xb2, 0x0e);//84/ Reserved
  wrReg(0xb3, 0x82);//85/ Reserved
  wrReg(0xb8, 0x0a);//86/ Reserved
  //
  // Reserved Values without function specification
  //
  wrReg(0x43, 0x0a);//87/ Reserved
  wrReg(0x44, 0xf0);//88/ Reserved
  wrReg(0x45, 0x34);//89/ Reserved
  wrReg(0x46, 0x58);//90/ Reserved
  wrReg(0x47, 0x28);//91/ Reserved
  wrReg(0x48, 0x3a);//92/ Reserved
  wrReg(0x59, 0x88);//93/ Reserved
  wrReg(0x5a, 0x88);//94/ Reserved
  wrReg(0x5b, 0x44);//95/ Reserved
  wrReg(0x5c, 0x67);//96/ Reserved
  wrReg(0x5d, 0x49);//97/ Reserved
  wrReg(0x5e, 0x0e);//98/ Reserved
  wrReg(0x64, 0x04);//99/ Reserved
  wrReg(0x65, 0x20);//100/ Reserved
  wrReg(0x66, 0x05);//101/ Reserved
  wrReg(0x94, 0x04);//102/ Reserved
  wrReg(0x95, 0x08);//103/ Reserved
  wrReg(0x6c, 0x0a);//104/ Reserved
  wrReg(0x6d, 0x55);//105/ Reserved
  wrReg(0x6e, 0x11);//106/ Reserved
  wrReg(0x6f, 0x9f);//107/ Reserved
  wrReg(0x6a, 0x40);//108/ Reserved
  wrReg(0x01, 0x40);//109/ REG BLUE : Reserved
  wrReg(0x02, 0x40);//110/ REG RED : Reserved
  wrReg(0x13, 0xe7);//111/ COM8 : FAST AEC, AEC unlimited STEP, Band Filter, AGC , ARC , AWB enable.
  //
  // Matrix Coefficients
  //
  wrReg(0x4f, 0x80);//112/ MTX 1 : Matrix Coefficient 1
  wrReg(0x50, 0x80);//113/ MTX 2 : Matrix Coefficient 2
  wrReg(0x51, 0x00);//114/ MTX 3 : Matrix Coefficient 3
  wrReg(0x52, 0x22);//115/ MTX 4 : Matrix Coefficient 4
  wrReg(0x53, 0x5e);//116/ MTX 5 : Matrix Coefficient 5
  wrReg(0x54, 0x80);//117/ MTX 6 : Matrix Coefficient 6
  wrReg(0x58, 0x9e);//118/ MTXS : Matrix Coefficient Sign for Coefficient 5 to 0
  wrReg(0x41, 0x08);//119/ COM16 : AWB Gain enable
  wrReg(0x3f, 0x00);//120/ EDGE : Edge Enhancement Adjustment
  wrReg(0x75, 0x05);//121/ Reserved
  wrReg(0x76, 0xe1);//122/ Reserved
  wrReg(0x4c, 0x00);//123/ Reserved
  wrReg(0x77, 0x01);//124/ Reserved
  wrReg(0x3d, 0xc0);//125/ COM13
  wrReg(0x4b, 0x09);//126/ Reserved
  wrReg(0xc9, 0x60);//127/ Reserved
  wrReg(0x41, 0x38);//128/ COM16
  wrReg(0x56, 0x40);//129/ Reserved
  wrReg(0x34, 0x11);//130/ Reserved
  wrReg(0x3b, 0x12);//131/ COM11 : Exposure and Hz Auto detect enabled.
  wrReg(0xa4, 0x88);//132/ Reserved
  wrReg(0x96, 0x00);//133/ Reserved
  wrReg(0x97, 0x30);//134/ Reserved
  wrReg(0x98, 0x20);//135/ Reserved
  wrReg(0x99, 0x30);//136/ Reserved
  wrReg(0x9a, 0x84);//137/ Reserved
  wrReg(0x9b, 0x29);//138/ Reserved
  wrReg(0x9c, 0x03);//139/ Reserved
  wrReg(0x9d, 0x4c);//140/ Reserved
  wrReg(0x9e, 0x3f);//141/ Reserved
  wrReg(0x78, 0x04);//142/ Reserved
  //
  // Mutliplexing Registers
  //
  wrReg(0x79, 0x01);//143/ Reserved
  wrReg(0xc8, 0xf0);//144/ Reserved
  wrReg(0x79, 0x0f);//145/ Reserved
  wrReg(0xc8, 0x00);//146/ Reserved
  wrReg(0x79, 0x10);//147/ Reserved
  wrReg(0xc8, 0x7e);//148/ Reserved
  wrReg(0x79, 0x0a);//149/ Reserved
  wrReg(0xc8, 0x80);//150/ Reserved
  wrReg(0x79, 0x0b);//151/ Reserved
  wrReg(0xc8, 0x01);//152/ Reserved
  wrReg(0x79, 0x0c);//153/ Reserved
  wrReg(0xc8, 0x0f);//154/ Reserved
  wrReg(0x79, 0x0d);//155/ Reserved
  wrReg(0xc8, 0x20);//156/ Reserved
  wrReg(0x79, 0x09);//157/ Reserved
  wrReg(0xc8, 0x80);//158/ Reserved
  wrReg(0x79, 0x02);//159/ Reserved
  wrReg(0xc8, 0xc0);//160/ Reserved
  wrReg(0x79, 0x03);//161/ Reserved
  wrReg(0xc8, 0x40);//162/ Reserved
  wrReg(0x79, 0x05);//163/ Reserved
  wrReg(0xc8, 0x30);//164/ Reserved
  wrReg(0x79, 0x26);//165/ Reserved
  //
  // Additional Settings
  //
  wrReg(0x09, 0x00);//166/ COM2 : Output Drive Capability
  wrReg(0x55, 0x00);//167/ Brightness Control

}

