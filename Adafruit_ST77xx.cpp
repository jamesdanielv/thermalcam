/***************************************************
  This is a library for the Adafruit 1.8" SPI display.

This library works with the Adafruit 1.8" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/358
The 1.8" TFT shield
  ----> https://www.adafruit.com/product/802
The 1.44" TFT breakout
  ----> https://www.adafruit.com/product/2088
as well as Adafruit raw 1.8" TFT display
  ----> http://www.adafruit.com/products/618

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
#define TFT_CS     10 //chip select pin for the TFT screen
#define TFT_RST    9  // you can also connect this to the Arduino reset
                      // in which case, set this #define pin to 0!
#define TFT_DC     8
#include "digitalfastwrite.h"
#include "Adafruit_ST77xx.h"
#include <limits.h>
#ifndef ARDUINO_STM32_FEATHER
  #include "pins_arduino.h"
  #include "wiring_private.h"
#endif
#include "SPIxx.h"

inline uint16_t swapcolor(uint16_t x) { 
  return (x << 11) | (x & 0x07E0) | (x >> 11);
}

#if defined (SPI_HAS_TRANSACTION)
  static SPISettings mySPISettings;
#elif defined (__AVR__) || defined(CORE_TEENSY)
  static uint8_t SPCRbackup;
  static uint8_t mySPCR;
#endif


#if defined (SPI_HAS_TRANSACTION)
#define SPI_BEGIN_TRANSACTION()    if (_hwSPI)    SPI.beginTransaction(mySPISettings)
#define SPI_END_TRANSACTION()      if (_hwSPI)    SPI.endTransaction()
#else
#define SPI_BEGIN_TRANSACTION()   // (void)
#define SPI_END_TRANSACTION()     // (void)
#endif

// Constructor when using software SPI.  All output pins are configurable.
Adafruit_ST77xx::Adafruit_ST77xx(int8_t cs, int8_t dc, int8_t sid, int8_t sclk, int8_t rst) 
  : Adafruit_GFX(ST7735_TFTWIDTH_128, ST7735_TFTHEIGHT_160)
{
  _cs   = cs;
  _dc   = dc;
  _sid  = sid;
  _sclk = sclk;
  _rst  = rst;
  _hwSPI = false;
}

// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
Adafruit_ST77xx::Adafruit_ST77xx(int8_t cs, int8_t dc, int8_t rst) 
  : Adafruit_GFX(ST7735_TFTWIDTH_128, ST7735_TFTHEIGHT_160) {
  _cs   = cs;
  _dc   = dc;
  _rst  = rst;
  _hwSPI = true;
  _sid  = _sclk = -1;
}

inline void Adafruit_ST77xx::spiwrite(uint8_t c) {

  //Serial.println(c, HEX);

  if (_hwSPI) {
#if defined (SPI_HAS_TRANSACTION)
      SPI.transfer(c);
#elif defined (__AVR__) || defined(CORE_TEENSY)
      //this is where transfer happens. try to move as much as possible here
      SPCRbackup = SPCR;
      SPCR = mySPCR;
      SPI.transfer(c);
      __asm__("nop\n\t"); //delayed for fast spi not to need another loop
      __asm__("nop\n\t"); 
      __asm__("nop\n\t"); 
      __asm__("nop\n\t"); 
      __asm__("nop\n\t"); 
      __asm__("nop\n\t"); 
      __asm__("nop\n\t"); 
      __asm__("nop\n\t"); 
      __asm__("nop\n\t"); 
      __asm__("nop\n\t"); 
      __asm__("nop\n\t"); 
      __asm__("nop\n\t"); 
      __asm__("nop\n\t"); 
      __asm__("nop\n\t"); 
      __asm__("nop\n\t"); 
      //while (!(SPSR & _BV(SPIF)));
      SPCR = SPCRbackup;
#elif defined (__arm__)
      SPI.setClockDivider(21); //4MHz
      SPI.setDataMode(SPI_MODE0);
      SPI.transfer(c);
#endif
  } else {

    // Fast SPI bitbang swiped from LPD8806 library
    for(uint8_t bit = 0x80; bit; bit >>= 1) {
#if defined(USE_FAST_IO)
      if(c & bit) *dataport |=  datapinmask;
      else        *dataport &= ~datapinmask;
      *clkport |=  clkpinmask;
      *clkport &= ~clkpinmask;
#else
      if(c & bit) digitalWrite(_sid, HIGH);
      else        digitalWrite(_sid, LOW);
      digitalWrite(_sclk, HIGH);
      digitalWrite(_sclk, LOW);
#endif
    }
  }
}


void Adafruit_ST77xx::writecommand(uint8_t c) {
digitalWriteFast(TFT_DC,LOW); //DC_LOW();
 digitalWriteFast(TFT_DC,LOW); //DC_LOW();
 digitalWriteFast(TFT_CS,LOW);// CS_LOW();
  digitalWriteFast(TFT_CS,LOW);// CS_LOW();
  SPI_BEGIN_TRANSACTION();

  spiwrite(c);
 digitalWriteFast(TFT_CS,HIGH); //CS_HIGH();
 digitalWriteFast(TFT_CS,HIGH); //CS_HIGH();
  SPI_END_TRANSACTION();
}


void Adafruit_ST77xx::writedata(uint8_t c) {
  SPI_BEGIN_TRANSACTION();

   digitalWriteFast(TFT_DC,HIGH); //DC_LOW();
   digitalWriteFast(TFT_DC,HIGH); //DC_LOW();
 digitalWriteFast(TFT_CS,LOW);// CS_LOW();
  digitalWriteFast(TFT_CS,LOW);// CS_LOW();
//  DC_HIGH();
 // CS_LOW();
    
  spiwrite(c);
 digitalWriteFast(TFT_CS,HIGH);
  digitalWriteFast(TFT_CS,HIGH);
 // CS_HIGH();
  SPI_END_TRANSACTION();
}


// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void Adafruit_ST77xx::displayInit(const uint8_t *addr) {

  uint8_t  numCommands, numArgs;
  uint16_t ms;

  numCommands = pgm_read_byte(addr++);   // Number of commands to follow
  while(numCommands--) {                 // For each command...
    writecommand(pgm_read_byte(addr++)); //   Read, issue command
    numArgs  = pgm_read_byte(addr++);    //   Number of args to follow
    ms       = numArgs & ST_CMD_DELAY;   //   If hibit set, delay follows args
    numArgs &= ~ST_CMD_DELAY;            //   Mask out delay bit
    while(numArgs--) {                   //   For each argument...
      writedata(pgm_read_byte(addr++));  //     Read, issue argument
    }

    if(ms) {
      ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
      if(ms == 255) ms = 500;     // If 255, delay for 500 ms
      delay(ms);
    }
  }
}


// Initialization code common to all ST77XX displays
void Adafruit_ST77xx::commonInit(const uint8_t *cmdList) {
  _ystart = _xstart = 0;
  _colstart  = _rowstart = 0; // May be overridden in init func

  pinMode(_dc, OUTPUT);
  pinMode(_cs, OUTPUT);

#if defined(USE_FAST_IO)
  csport    = portOutputRegister(digitalPinToPort(_cs));
  dcport    = portOutputRegister(digitalPinToPort(_dc));
  cspinmask = digitalPinToBitMask(_cs);
  dcpinmask = digitalPinToBitMask(_dc);
#endif

  if(_hwSPI) { // Using hardware SPI
#if defined (SPI_HAS_TRANSACTION)
    SPI.begin();
    mySPISettings = SPISettings(24000000, MSBFIRST, SPI_MODE0);
#elif defined (__AVR__) || defined(CORE_TEENSY)
    SPCRbackup = SPCR;
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV2);
    SPI.setDataMode(SPI_MODE0);
    mySPCR = SPCR; // save our preferred state
    //Serial.print("mySPCR = 0x"); Serial.println(SPCR, HEX);
    SPCR = SPCRbackup;  // then restore
#elif defined (__SAM3X8E__)
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV2); //4MHz//8mhz
    SPI.setDataMode(SPI_MODE0);
#endif
  } else {
    pinMode(_sclk, OUTPUT);
    pinMode(_sid , OUTPUT);
    digitalWrite(_sclk, LOW);
    digitalWrite(_sid, LOW);

#if defined(USE_FAST_IO)
    clkport     = portOutputRegister(digitalPinToPort(_sclk));
    dataport    = portOutputRegister(digitalPinToPort(_sid));
    clkpinmask  = digitalPinToBitMask(_sclk);
    datapinmask = digitalPinToBitMask(_sid);
#endif
  }

  // toggle RST low to reset; CS low so it'll listen to us
  CS_LOW();
  if (_rst != -1) {
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, HIGH);
    delay(500);
    digitalWrite(_rst, LOW);
    delay(500);
    digitalWrite(_rst, HIGH);
    delay(500);
  }

  if(cmdList) 
    displayInit(cmdList);
}



void Adafruit_ST77xx::setRotation(uint8_t m) {

  writecommand(ST77XX_MADCTL);
  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
   case 0:
     writedata(ST77XX_MADCTL_MX | ST77XX_MADCTL_MY | ST77XX_MADCTL_RGB);

     _xstart = _colstart;
     _ystart = _rowstart;
     break;
   case 1:
     writedata(ST77XX_MADCTL_MY | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB);

     _ystart = _colstart;
     _xstart = _rowstart;
     break;
  case 2:
     writedata(ST77XX_MADCTL_RGB);
 
     _xstart = _colstart;
     _ystart = _rowstart;
     break;

   case 3:
     writedata(ST77XX_MADCTL_MX | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB);

     _ystart = _colstart;
     _xstart = _rowstart;
     break;
  }
}

void Adafruit_ST77xx::setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1,
 uint8_t y1) {

  uint16_t x_start = x0 + _xstart, x_end = x1 + _xstart;
  uint16_t y_start = y0 + _ystart, y_end = y1 + _ystart;
  

  writecommand(ST77XX_CASET); // Column addr set
  writedata(x_start >> 8);
  writedata(x_start & 0xFF);     // XSTART 
  writedata(x_end >> 8);
  writedata(x_end & 0xFF);     // XEND

  writecommand(ST77XX_RASET); // Row addr set
  writedata(y_start >> 8);
  writedata(y_start & 0xFF);     // YSTART
  writedata(y_end >> 8);
  writedata(y_end & 0xFF);     // YEND

  writecommand(ST77XX_RAMWR); // write to RAM
}


void Adafruit_ST77xx::pushColor(uint16_t color) {
  SPI_BEGIN_TRANSACTION();
   digitalWriteFast(TFT_CS,HIGH);
   digitalWriteFast(TFT_CS,HIGH);
  //DC_HIGH();
 // CS_LOW();
 digitalWriteFast(TFT_CS,LOW);// CS_LOW();
  digitalWriteFast(TFT_CS,LOW);// CS_LOW();
  spiwrite(color >> 8);
  spiwrite(color);

  CS_HIGH();
  SPI_END_TRANSACTION();
}

void Adafruit_ST77xx::drawPixel(int16_t x, int16_t y, uint16_t color) {

  if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

  setAddrWindow(x,y,x+1,y+1);

  SPI_BEGIN_TRANSACTION();
  //DC_HIGH();
   digitalWriteFast(TFT_DC,HIGH); 
     digitalWriteFast(TFT_DC,HIGH);
 // CS_LOW();
 digitalWriteFast(TFT_CS,LOW);// CS_LOW();
  digitalWriteFast(TFT_CS,LOW);// CS_LOW();
  spiwrite(color >> 8);
  spiwrite(color);
 digitalWriteFast(TFT_CS,HIGH);
  digitalWriteFast(TFT_CS,HIGH);
 // CS_HIGH();
  SPI_END_TRANSACTION();
}


void Adafruit_ST77xx::drawFastVLine(int16_t x, int16_t y, int16_t h,
 uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((y+h-1) >= _height) h = _height-y;
  setAddrWindow(x, y, x, y+h-1);

  uint8_t hi = color >> 8, lo = color;
    
  SPI_BEGIN_TRANSACTION();
  DC_HIGH();
  CS_LOW();

  while (h--) {
    spiwrite(hi);
    spiwrite(lo);
  }

  CS_HIGH();
  SPI_END_TRANSACTION();
}


void Adafruit_ST77xx::drawFastHLine(int16_t x, int16_t y, int16_t w,
  uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((x+w-1) >= _width)  w = _width-x;
  setAddrWindow(x, y, x+w-1, y);

  uint8_t hi = color >> 8, lo = color;

  SPI_BEGIN_TRANSACTION();
  DC_HIGH();
  CS_LOW();

  while (w--) {
    spiwrite(hi);
    spiwrite(lo);
  }

  CS_HIGH();
  SPI_END_TRANSACTION();
}



void Adafruit_ST77xx::fillScreen(uint16_t color) {
  fillRect(0, 0,  _width, _height, color);
}

// fill a rectangle
void Adafruit_ST77xx::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
  uint16_t color) {

  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= _width) || (y >= _height)) return;
  if((x + w - 1) >= _width)  w = _width  - x;
  if((y + h - 1) >= _height) h = _height - y;

  setAddrWindow(x, y, x+w-1, y+h-1);

  uint8_t hi = color >> 8, lo = color;
    
  SPI_BEGIN_TRANSACTION();

  DC_HIGH();
  CS_LOW();
  for(y=h; y>0; y--) {
    for(x=w; x>0; x--) {
      spiwrite(hi);
      spiwrite(lo);
    }
  }
  CS_HIGH();
  SPI_END_TRANSACTION();
}


// Pass 8-bit (each) R,G,B, get back 16-bit packed color

// fill a rectangle
void Adafruit_ST77xx::fillRectFast(int16_t x, int16_t y, int16_t w, int16_t h,
  uint16_t color) {
//this is needed for text but not fills with solid color!
  // rudimentary clipping (drawChar w/big text requires this)
 // if((x >= _width) || (y >= _height)) return;
 // if((x + w - 1) >= _width)  w = _width  - x;
 // if((y + h - 1) >= _height) h = _height - y;

  setAddrWindow(x, y, x+w-1, y+h-1);

  uint8_t hi = color >> 8, lo = color;
    
  SPI_BEGIN_TRANSACTION();

  digitalWriteFast(TFT_DC,HIGH);// old DC_HIGH();
  
  digitalWriteFast(TFT_CS,LOW);// old CS_LOW();

//  y=h;
 
    x=w*h;// most of time is in pixel writes. a dealy is needed for spi to write data

    
   //   spiwrite(hi);//we update color
   //   spiwrite(lo);
//__attribute__((optimize("unroll-loops")));
    while ( x>0) {//we need to unroll this loop and add a command that cycles thru to push address one byte at a time.
 //SPCR = SPCRbackup;  //we place at top for next loop iteration
 SPCRbackup = SPCR; //not sure what this does or if it is really needed, but we keep at begining of loop unroll
      SPCR = mySPCR;
       
      SPDR = hi;
      __asm__("nop\n\t"); 
      __asm__("nop\n\t"); //delayed for fast spi not to need another loop
      __asm__("nop\n\t"); 
      __asm__("nop\n\t"); 
      __asm__("nop\n\t"); 
      __asm__("nop\n\t"); 
      __asm__("nop\n\t"); 
      __asm__("nop\n\t"); 
      __asm__("nop\n\t"); 
      __asm__("nop\n\t"); 
      __asm__("nop\n\t"); 
      __asm__("nop\n\t"); 
      __asm__("nop\n\t"); 
      __asm__("nop\n\t"); 
    //  __asm__("nop\n\t"); 
     // __asm__("nop\n\t"); 
      //while (!(SPSR & _BV(SPIF)));
    //  SPCR = SPCRbackup;

    // SPCRbackup = SPCR;
      SPCR = mySPCR;
      // __asm__("nop\n\t"); 
      SPDR =lo;
      
     __asm__("nop\n\t"); 
      if (x==1){ return;}//shave time off of negating zero
          __asm__("nop\n\t"); 
           __asm__("nop\n\t"); 
           x--;
          
//we are useing the jmp and loop call times to reduce the timing needed here.
// since loop time is at the delay needed for spi, this is as efficient as it can be, as every clock
//per color pixel draw for fill is flooded to spi!

       
    }//x
    digitalWriteFast(TFT_DC,HIGH);// old DC_HIGH();
 
  digitalWriteFast(TFT_CS,HIGH);//old CS_HIGH();
  SPI_END_TRANSACTION();
}

//below we have one that makes subsampling easier
void Adafruit_ST77xx::fillRectFast4colors(int16_t x, int16_t y, int16_t w, int16_t h,
  uint16_t color0,uint16_t color1,uint16_t color2,uint16_t color3) {
//this is needed for text but not fills with solid color!
  // rudimentary clipping (drawChar w/big text requires this)
 // if((x >= _width) || (y >= _height)) return;
 // if((x + w - 1) >= _width)  w = _width  - x;
 // if((y + h - 1) >= _height) h = _height - y;

  setAddrWindow(x, y, x+w-1, y+h-1);
  //uses more memory, but i think it is more efficient
  uint8_t hi0 = color0 >> 8, lo0 = color0;//we set this in advance, and we do the changes in between write cycles of spi! so no time lost!
  uint8_t hi1 = color1 >> 8, lo1 = color1;//we set this in advance, and we do the changes in between write cycles of spi! so no time lost!
  uint8_t hi2 = color2 >> 8, lo2 = color2;//we set this in advance, and we do the changes in between write cycles of spi! so no time lost!
  uint8_t hi3 = color3 >> 8, lo3 = color3;//we set this in advance, and we do the changes in between write cycles of spi! so no time lost!  
  SPI_BEGIN_TRANSACTION();
  //created complex commands in #define so code can be a lot cleaner
  //this is a complex command that complexspi= hi[0] value
#define complexspi SPCRbackup = SPCR; SPCR = mySPCR; SPDR 
//this is small delay
#define shortSpiDelay __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t") 
//last comment out ';' not needed here, but can be added in commands later 
#define longSpiDelay __asm__("nop\n\t"); __asm__("nop\n\t");__asm__("nop\n\t"); __asm__("nop\n\t");  __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t")  
  
digitalWriteFast(TFT_DC,HIGH);// old DC_HIGH();
digitalWriteFast(TFT_CS,LOW);// old CS_LOW();

//  y=h;
  uint16_t countx;//we use these to understand what to draw!
  uint16_t county;

 SPCR = SPCRbackup;  //we place at top for next loop iteration
county=h/2;//+1;
while (county !=0 ){//first rectangle part
         countx=w/2;
          
    while (countx !=0) {//we need to unroll this loop and add a command that cycles thru to push address one byte at a time.
    complexspi = hi0;longSpiDelay; complexspi =lo0; shortSpiDelay;   
           countx--;    
    }
       countx=w/2;
    while (countx !=0) {//we need to unroll this loop and add a command that cycles thru to push address one byte at a time.     
     complexspi = hi1;longSpiDelay;complexspi=lo1;shortSpiDelay;
           countx--;     
    }
    county--;
}//end of first half of rectange [0][1] drawn!


county=h/2;//+1; 
while (county !=0){
     countx=w/2;//we need to reset countx here as well
    while (countx !=0) {//we need to unroll this loop and add a command that cycles thru to push address one byte at a time.
      complexspi = hi2;longSpiDelay;complexspi =lo2;shortSpiDelay;
      countx--;     
    }
 
    countx=w/2;
    while (countx !=0) {//we need to unroll this loop and add a command that cycles thru to push address one byte at a time.
 complexspi = hi3;longSpiDelay;; complexspi =lo3;shortSpiDelay;         
           countx--;
    }
    county--;
}//end of first half of rectange [2][3] drawn!

         

    digitalWriteFast(TFT_DC,HIGH);// old DC_HIGH();
 
  digitalWriteFast(TFT_CS,HIGH);//old CS_HIGH();
  SPI_END_TRANSACTION();
}


void Adafruit_ST77xx::fillRectFast16colors(int16_t x, int16_t y, int16_t w, int16_t h,
             uint16_t color0, uint16_t color1, uint16_t color2, uint16_t color3,
             uint16_t color4, uint16_t color5, uint16_t color6, uint16_t color7,
             uint16_t color8, uint16_t color9, uint16_t color10, uint16_t color11,
             uint16_t color12, uint16_t color13, uint16_t color14, uint16_t color15){
//this is where squares are written together!
  setAddrWindow(x, y, x+w-1, y+h-1);
  //uses more memory, but i think it is more efficient
  uint8_t hi0 = color0 >> 8, lo0 = color0 ;
  uint8_t hi1 = color1 >> 8, lo1 = color1 ;
  uint8_t hi2 = color2 >> 8, lo2 = color2 ;
  uint8_t hi3 = color3 >> 8, lo3 = color3 ;
  uint8_t hi4 = color4 >> 8, lo4 = color4 ;
  uint8_t hi5 = color5 >> 8, lo5 = color5 ;
  uint8_t hi6 = color6 >> 8, lo6 = color6 ;
  uint8_t hi7 = color7 >> 8, lo7 = color7 ;
  uint8_t hi8 = color8 >> 8, lo8 = color8 ;
  uint8_t hi9 = color9 >> 8, lo9 = color9 ; 
  uint8_t hi10= color10>> 8, lo10= color10; 
  uint8_t hi11= color11>> 8, lo11= color11;
  uint8_t hi12= color12>> 8, lo12= color12;
  uint8_t hi13= color13>> 8, lo13= color13;
  uint8_t hi14= color14>> 8, lo14= color14;
  uint8_t hi15= color15>> 8, lo15= color15;  
  SPI_BEGIN_TRANSACTION();
  //created complex commands in #define so code can be a lot cleaner
  //this is a complex command that complexspi= hi[0] value
#define complexspi SPCRbackup = SPCR; SPCR = mySPCR; SPDR 
//this is small delay
#define shortSpiDelay __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); 
//last comment out ';' not needed here, but can be added in commands later 
#define longSpiDelay __asm__("nop\n\t"); __asm__("nop\n\t");__asm__("nop\n\t"); __asm__("nop\n\t");  __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t");  
  //[0][1][2][3] 
  //[4][5][6][7] 
  //[8][9][A][B] //this is how command updates screen
  //[C][D][E][F] //a,b,c,d,e,f are 10,11,12,13,14,15
digitalWriteFast(TFT_DC,HIGH);// old DC_HIGH();
digitalWriteFast(TFT_CS,LOW);// old CS_LOW();

//  y=h;
  uint16_t countx;//we use these to understand what to draw!
  uint16_t county;

 SPCR = SPCRbackup;  //we place at top for next loop iteration
county=h/4;//+1;
while (county !=0 ){//first rectangle part
         countx=w/4;
          
    while (countx !=0) {//we need to unroll this loop and add a command that cycles thru to push address one byte at a time.
    complexspi = hi0;longSpiDelay; complexspi =lo0; shortSpiDelay;   
           countx--;    
                   }
       countx=w/4;
    while (countx !=0) {//we need to unroll this loop and add a command that cycles thru to push address one byte at a time.     
     complexspi = hi1;longSpiDelay;complexspi=lo1;shortSpiDelay;
           countx--;     
                    }
         countx=w/4;          
    while (countx !=0) {//we need to unroll this loop and add a command that cycles thru to push address one byte at a time.
    complexspi = hi2;longSpiDelay; complexspi =lo2; shortSpiDelay;   
           countx--;    
                   }
       countx=w/4;
    while (countx !=0) {//we need to unroll this loop and add a command that cycles thru to push address one byte at a time.     
     complexspi = hi3;longSpiDelay;complexspi=lo3;shortSpiDelay;
           countx--;     
                  }
    
    county--;
};//                     [0][1][2][3] drawn!
county=h/4;//+1;
while (county !=0 ){//first rectangle part
         countx=w/4;
          
    while (countx !=0) {//we need to unroll this loop and add a command that cycles thru to push address one byte at a time.
    complexspi = hi4;longSpiDelay; complexspi =lo4; shortSpiDelay;   
           countx--;    
                   }
       countx=w/4;
    while (countx !=0) {//we need to unroll this loop and add a command that cycles thru to push address one byte at a time.     
     complexspi = hi5;longSpiDelay;complexspi=lo5;shortSpiDelay;
           countx--;     
                    }
         countx=w/4;          
    while (countx !=0) {//we need to unroll this loop and add a command that cycles thru to push address one byte at a time.
    complexspi = hi6;longSpiDelay; complexspi =lo6; shortSpiDelay;   
           countx--;    
                   }
       countx=w/4;
    while (countx !=0) {//we need to unroll this loop and add a command that cycles thru to push address one byte at a time.     
     complexspi = hi7;longSpiDelay;complexspi=lo7;shortSpiDelay;
           countx--;     
                  }
    
    county--;
};  //                        [4][5][6][7] drawn!
county=h/4;//+1;
while (county !=0 ){//first rectangle part
         countx=w/4;
          
    while (countx !=0) {//we need to unroll this loop and add a command that cycles thru to push address one byte at a time.
    complexspi = hi8;longSpiDelay; complexspi =lo8; shortSpiDelay;   
           countx--;    
                   }
       countx=w/4;
    while (countx !=0) {//we need to unroll this loop and add a command that cycles thru to push address one byte at a time.     
     complexspi = hi9;longSpiDelay;complexspi=lo9;shortSpiDelay;
           countx--;     
                    }
         countx=w/4;          
    while (countx !=0) {//we need to unroll this loop and add a command that cycles thru to push address one byte at a time.
    complexspi = hi10;longSpiDelay; complexspi =lo10; shortSpiDelay;   
           countx--;    
                   }
       countx=w/4;
    while (countx !=0) {//we need to unroll this loop and add a command that cycles thru to push address one byte at a time.     
     complexspi = hi11;longSpiDelay;complexspi=lo11;shortSpiDelay;
           countx--;     
                  }
    
    county--;
}; //                         [8][9][A][B] drawn!
county=h/4;//+1;
while (county !=0 ){//first rectangle part
         countx=w/4;
          
    while (countx !=0) {//we need to unroll this loop and add a command that cycles thru to push address one byte at a time.
    complexspi = hi12;longSpiDelay; complexspi =lo12; shortSpiDelay;   
           countx--;    
                   }
       countx=w/4;
    while (countx !=0) {//we need to unroll this loop and add a command that cycles thru to push address one byte at a time.     
     complexspi = hi13;longSpiDelay;complexspi=lo13;shortSpiDelay;
           countx--;     
                    }
         countx=w/4;          
    while (countx !=0) {//we need to unroll this loop and add a command that cycles thru to push address one byte at a time.
    complexspi = hi14;longSpiDelay; complexspi =lo14; shortSpiDelay;   
           countx--;    
                   }
       countx=w/4;
    while (countx !=0) {//we need to unroll this loop and add a command that cycles thru to push address one byte at a time.     
     complexspi = hi15;longSpiDelay;complexspi=lo15;shortSpiDelay;
           countx--;     
                  }
    
    county--;
}; //                         [8][9][A][B] drawn!         

    digitalWriteFast(TFT_DC,HIGH);// old DC_HIGH();
 
  digitalWriteFast(TFT_CS,HIGH);//old CS_HIGH();
  SPI_END_TRANSACTION();             
             }

             
// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t Adafruit_ST77xx::Color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}



void Adafruit_ST77xx::invertDisplay(boolean i) {
  writecommand(i ? ST77XX_INVON : ST77XX_INVOFF);
}


/******** low level bit twiddling **********/


inline void Adafruit_ST77xx::CS_HIGH(void) {
#if defined(USE_FAST_IO)
  *csport |= cspinmask;
#else
  digitalWriteFast(TFT_CS, HIGH);
#endif
}

inline void Adafruit_ST77xx::CS_LOW(void) {
#if defined(USE_FAST_IO)
  *csport &= ~cspinmask;
#else
 digitalWriteFast(TFT_CS,LOW);
#endif
}

inline void Adafruit_ST77xx::DC_HIGH(void) {
#if defined(USE_FAST_IO)
  *dcport |= dcpinmask;
#else
 digitalWriteFast(TFT_DC,HIGH)
#endif
}

inline void Adafruit_ST77xx::DC_LOW(void) {
#if defined(USE_FAST_IO)
  *dcport &= ~dcpinmask;
#else
 digitalWriteFast(TFT_DC,LOW)
#endif
}



////////// stuff not actively being used, but kept for posterity
/*

 uint8_t Adafruit_ST77xx::spiread(void) {
 uint8_t r = 0;
 if (_sid > 0) {
 r = shiftIn(_sid, _sclk, MSBFIRST);
 } else {
 //SID_DDR &= ~_BV(SID);
 //int8_t i;
 //for (i=7; i>=0; i--) {
 //  SCLK_PORT &= ~_BV(SCLK);
 //  r <<= 1;
 //  r |= (SID_PIN >> SID) & 0x1;
 //  SCLK_PORT |= _BV(SCLK);
 //}
 //SID_DDR |= _BV(SID);
 
 }
 return r;
 }
 
 
 void Adafruit_ST77xx::dummyclock(void) {
 
 if (_sid > 0) {
 digitalWrite(_sclk, LOW);
 digitalWrite(_sclk, HIGH);
 } else {
 // SCLK_PORT &= ~_BV(SCLK);
 //SCLK_PORT |= _BV(SCLK);
 }
 }
 uint8_t Adafruit_ST77xx::readdata(void) {
 *portOutputRegister(rsport) |= rspin;
 
 *portOutputRegister(csport) &= ~ cspin;
 
 uint8_t r = spiread();
 
 *portOutputRegister(csport) |= cspin;
 
 return r;
 
 } 
 
 uint8_t Adafruit_ST77xx::readcommand8(uint8_t c) {
 digitalWrite(_rs, LOW);
 
 *portOutputRegister(csport) &= ~ cspin;
 
 spiwrite(c);
 
 digitalWrite(_rs, HIGH);
 pinMode(_sid, INPUT); // input!
 digitalWrite(_sid, LOW); // low
 spiread();
 uint8_t r = spiread();
 
 
 *portOutputRegister(csport) |= cspin;
 
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 
 uint16_t Adafruit_ST77xx::readcommand16(uint8_t c) {
 digitalWrite(_rs, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 
 spiwrite(c);
 pinMode(_sid, INPUT); // input!
 uint16_t r = spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 uint32_t Adafruit_ST77xx::readcommand32(uint8_t c) {
 digitalWrite(_rs, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 spiwrite(c);
 pinMode(_sid, INPUT); // input!
 
 dummyclock();
 dummyclock();
 
 uint32_t r = spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 */
