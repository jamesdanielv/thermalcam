#include "AAlcd_mode.h" //this file tells us what display driver to use
#if lcd_mode == 1
#ifndef _ADAFRUIT_SPITFT_ili9341
#define _ADAFRUIT_SPITFT_ili9341
/*!
* @file Adafruit_SPITFT.cpp
*
* @mainpage Adafruit SPI TFT Displays
*
* @section intro_sec Introduction
  This is our library for generic SPI TFT Displays with
  address windows and 16 bit color (e.g. ILI9341, HX8357D, ST7735...)

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
* @section dependencies Dependencies
*
* This library depends on <a href="https://github.com/adafruit/Adafruit_GFX">
* Adafruit_GFX</a> being present on your system. Please make sure you have
* installed the latest version before using this library.
*
* @section author Author
*
* Written by Limor "ladyada" Fried for Adafruit Industries.
*
* @section license License
*
* BSD license, all text here must be included in any redistribution.
*
*/
#define safety_checks false//this makes every spi speed safe, if we remove checks we can run as fast as possible at max spi
#define TFT_CS     10 //chip select pin for the TFT screen
#define TFT_RST    9  // you can also connect this to the Arduino reset
                      // in which case, set this #define pin to 0!
#define TFT_DC     8
#ifndef __AVR_ATtiny85__ // NOT A CHANCE of this stuff working on ATtiny!
#include "digitalfastwrite.h"
#include "Adafruit_SPITFT_ili9341.h"
#ifndef ARDUINO_STM32_FEATHER
  #include "pins_arduino.h"
#ifndef RASPI
    #include "wiring_private.h"
#endif
#endif

#include <limits.h>

#include "Adafruit_SPITFT_Macros_ili9341.h"
uint16_t  hardcodeh;
uint16_t  hardcodew;
uint16_t  hardcodeToSubtracth; //
uint16_t  hardcodeToSubtractw;
uint16_t temp_x; //these 4 values are used for half mode features
uint16_t temp_y;
uint8_t  temp_h;
uint8_t countline;
/**************************************************************************/
/*!
    @brief  Pass 8-bit (each) R,G,B, get back 16-bit packed color
            This function converts 8-8-8 RGB data to 16-bit 5-6-5
    @param    red   Red 8 bit color
    @param    green Green 8 bit color
    @param    blue  Blue 8 bit color
    @return   Unsigned 16-bit down-sampled color in 5-6-5 format
*/
/**************************************************************************/
uint16_t Adafruit_SPITFT::color565(uint8_t red, uint8_t green, uint8_t blue) {
    return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | ((blue & 0xF8) >> 3);
}


/**************************************************************************/
/*!
    @brief  Instantiate Adafruit SPI display driver with software SPI
    @param    w     Display width in pixels
    @param    h     Display height in pixels
    @param    cs    Chip select pin #
    @param    dc    Data/Command pin #
    @param    mosi  SPI MOSI pin #
    @param    sclk  SPI Clock pin #
    @param    rst   Reset pin # (optional, pass -1 if unused)
    @param    miso  SPI MISO pin # (optional, pass -1 if unused)
*/
/**************************************************************************/
Adafruit_SPITFT::Adafruit_SPITFT(uint16_t w, uint16_t h,
				 int8_t cs, int8_t dc, int8_t mosi,
				 int8_t sclk, int8_t rst, int8_t miso) 
  : Adafruit_GFX(w, h) {
    _cs   = cs;
    _dc   = dc;
    _rst  = rst;
    _sclk = sclk;
    _mosi = mosi;
    _miso = miso;
    _freq = 0;
#ifdef USE_FAST_PINIO
    dcport      = (RwReg *)portOutputRegister(digitalPinToPort(dc));
    dcpinmask   = digitalPinToBitMask(dc);
    clkport     = (RwReg *)portOutputRegister(digitalPinToPort(sclk));
    clkpinmask  = digitalPinToBitMask(sclk);
    mosiport    = (RwReg *)portOutputRegister(digitalPinToPort(mosi));
    mosipinmask = digitalPinToBitMask(mosi);
    if(miso >= 0){
        misoport    = (RwReg *)portInputRegister(digitalPinToPort(miso));
        misopinmask = digitalPinToBitMask(miso);
    } else {
        misoport    = 0;
        misopinmask = 0;
    }
    if(cs >= 0) {
        csport    = (RwReg *)portOutputRegister(digitalPinToPort(cs));
        cspinmask = digitalPinToBitMask(cs);
    } else {
        // No chip-select line defined; might be permanently tied to GND.
        // Assign a valid GPIO register (though not used for CS), and an
        // empty pin bitmask...the nonsense bit-twiddling might be faster
        // than checking _cs and possibly branching.
        csport    = dcport;
        cspinmask = 0;
    }
#endif
}

/**************************************************************************/
/*!
    @brief  Instantiate Adafruit SPI display driver with hardware SPI
    @param    w     Display width in pixels
    @param    h     Display height in pixels
    @param    cs    Chip select pin #
    @param    dc    Data/Command pin #
    @param    rst   Reset pin # (optional, pass -1 if unused)
*/
/**************************************************************************/
Adafruit_SPITFT::Adafruit_SPITFT(uint16_t w, uint16_t h,
				 int8_t cs, int8_t dc, int8_t rst) 
  : Adafruit_GFX(w, h) {
    _cs   = cs;
    _dc   = dc;
    _rst  = rst;
    _sclk = -1;
    _mosi = -1;
    _miso = -1;
    _freq = 0;
#ifdef USE_FAST_PINIO
    clkport     = 0;
    clkpinmask  = 0;
    mosiport    = 0;
    mosipinmask = 0;
    misoport    = 0;
    misopinmask = 0;
    dcport      = (RwReg *)portOutputRegister(digitalPinToPort(dc));
    dcpinmask   = digitalPinToBitMask(dc);
    if(cs >= 0) {
        csport    = (RwReg *)portOutputRegister(digitalPinToPort(cs));
        cspinmask = digitalPinToBitMask(cs);
    } else {
        // See notes in prior constructor.
        csport    = dcport;
        cspinmask = 0;
    }
#endif
}

/**************************************************************************/
/*!
    @brief   Initialiaze the SPI interface (hardware or software)
    @param    freq  The desired maximum SPI hardware clock frequency
*/
/**************************************************************************/
void Adafruit_SPITFT::initSPI(uint32_t freq) {
    _freq = freq;

    // Control Pins
    if(_cs >= 0) {
        pinMode(_cs, OUTPUT);
        digitalWrite(_cs, HIGH); // Deselect
    }
    pinMode(_dc, OUTPUT);
    digitalWrite(_dc, LOW);

    // Software SPI
    if(_sclk >= 0){
        pinMode(_mosi, OUTPUT);
        digitalWrite(_mosi, LOW);
        pinMode(_sclk, OUTPUT);
        digitalWrite(_sclk, HIGH);
        if(_miso >= 0){
            pinMode(_miso, INPUT);
        }
    }

    // Hardware SPI
    SPI_BEGIN();

    // toggle RST low to reset
    if (_rst >= 0) {
        pinMode(_rst, OUTPUT);
        digitalWrite(_rst, HIGH);
        delay(100);
        digitalWrite(_rst, LOW);
        delay(100);
        digitalWrite(_rst, HIGH);
        delay(200);
    }
}

/**************************************************************************/
/*!
    @brief   Read one byte from SPI interface (hardware or software
    @returns One byte, MSB order
*/
/**************************************************************************/
uint8_t Adafruit_SPITFT::spiRead() {
    if(_sclk < 0){
        return HSPI_READ();
    }
    if(_miso < 0){
        return 0;
    }
    uint8_t r = 0;
    for (uint8_t i=0; i<8; i++) {
        SSPI_SCK_LOW();
        SSPI_SCK_HIGH();
        r <<= 1;
        if (SSPI_MISO_READ()){
            r |= 0x1;
        }
    }
    return r;
}

/**************************************************************************/
/*!
    @brief   Write one byte to SPI interface (hardware or software
    @param  b  One byte to send, MSB order
*/
/**************************************************************************/
void Adafruit_SPITFT::spiWrite(uint8_t b) {
    if(_sclk < 0){
        HSPI_WRITE(b);
        return;
    }
    for(uint8_t bit = 0x80; bit; bit >>= 1){
        if((b) & bit){
            SSPI_MOSI_HIGH();
        } else {
            SSPI_MOSI_LOW();
        }
        SSPI_SCK_LOW();
        SSPI_SCK_HIGH();
    }
}


/*
 * Transaction API
 * */

/**************************************************************************/
/*!
    @brief   Begin an SPI transaction & set CS low.
*/
/**************************************************************************/
void inline Adafruit_SPITFT::startWrite(void){
    SPI_BEGIN_TRANSACTION();
    SPI_CS_LOW();
}



//**************************
 //below we have complex code that is less painful to look at if it is in #define
#if safety_checks != false             
#define complexspi while(!(SPSR & (1<<SPIF) ));SPDR//SPCRbackup = SPCR; SPCR = mySPCR; SPDR 
#else
#define complexspi SPDR //SPCRbackup = SPCR; SPCR = mySPCR; SPDR 
#endif
//this is small delay
#define shortSpiDelay __asm__("nop\n\t"); __asm__("nop\n\t");__asm__("nop\n\t") ;//__asm__("nop\n\t") 
#define shortSpiDelay64avg __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t");__asm__("nop\n\t"); //this routine is slightly different
//last comment out ';' not needed here, but can be added in commands later 
#define longSpiDelay __asm__("nop\n\t"); __asm__("nop\n\t");__asm__("nop\n\t"); __asm__("nop\n\t");__asm__("nop\n\t"); __asm__("nop\n\t");  __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t");__asm__("nop\n\t"); //__asm__("nop\n\t");  
#define longSpiDelay64AVG __asm__("nop\n\t"); __asm__("nop\n\t");__asm__("nop\n\t"); __asm__("nop\n\t");__asm__("nop\n\t"); __asm__("nop\n\t");  __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t");__asm__("nop\n\t"); __asm__("nop\n\t");  __asm__("nop\n\t");
 

/**************************************************************************/
/*!
    @brief   Begin an SPI transaction & set CS high.
*/
/**************************************************************************/
void inline Adafruit_SPITFT::endWrite(void){
    SPI_CS_HIGH();
    SPI_END_TRANSACTION();
}

/**************************************************************************/
/*!
    @brief   Write a command byte (must have a transaction in progress)
    @param   cmd  The 8-bit command to send
*/
/**************************************************************************/
void Adafruit_SPITFT::writeCommand(uint8_t cmd){
    SPI_DC_LOW();
    spiWrite(cmd);
    SPI_DC_HIGH();
}

/**************************************************************************/
/*!
    @brief   Push a 2-byte color to the framebuffer RAM, will start transaction
    @param    color 16-bit 5-6-5 Color to draw
*/
/**************************************************************************/

/*
void Adafruit_SPITFT::pushColor(uint16_t color) {
  startWrite();
  SPI_WRITE16(color);
  endWrite();
}
*/
void Adafruit_SPITFT::pushColor(uint16_t color) {
 startWrite();

  spiWrite(color >> 8);//longSpiDelay;
  spiWrite(color);//longSpiDelay;
endWrite();
}

/**************************************************************************/
/*!
    @brief   Blit multiple 2-byte colors  (must have a transaction in progress)
    @param    colors Array of 16-bit 5-6-5 Colors to draw
    @param    len  How many pixels to draw - 2 bytes per pixel!
*/
/**************************************************************************/
void Adafruit_SPITFT::writePixels(uint16_t * colors, uint32_t len){
    SPI_WRITE_PIXELS((uint8_t*)colors , len * 2);
}

/**************************************************************************/
/*!
    @brief   Blit a 2-byte color many times  (must have a transaction in progress)
    @param    color  The 16-bit 5-6-5 Color to draw
    @param    len    How many pixels to draw
*/
/**************************************************************************/
void Adafruit_SPITFT::writeColor(uint16_t color, uint32_t len){
#ifdef SPI_HAS_WRITE_PIXELS
    if(_sclk >= 0){
        for (uint32_t t=0; t<len; t++){
            writePixel(color);
        }
        return;
    }
    static uint16_t temp[SPI_MAX_PIXELS_AT_ONCE];
    size_t blen = (len > SPI_MAX_PIXELS_AT_ONCE)?SPI_MAX_PIXELS_AT_ONCE:len;
    uint16_t tlen = 0;

    for (uint32_t t=0; t<blen; t++){
        temp[t] = color;
    }

    while(len){
        tlen = (len>blen)?blen:len;
        writePixels(temp, tlen);
        len -= tlen;
    }
#else
    uint8_t hi = color >> 8, lo = color;
    if(_sclk < 0){ //AVR Optimization
        for (uint32_t t=len; t; t--){
            HSPI_WRITE(hi);
            HSPI_WRITE(lo);
        }
        return;
    }
    for (uint32_t t=len; t; t--){
        spiWrite(hi);
        spiWrite(lo);
    }
#endif
}

/**************************************************************************/
/*!
   @brief    Write a pixel (must have a transaction in progress)
    @param   x   x coordinate
    @param   y   y coordinate
   @param    color 16-bit 5-6-5 Color to draw with
*/
/**************************************************************************/
void Adafruit_SPITFT::writePixel(int16_t x, int16_t y, uint16_t color) {
    if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;
    setAddrWindow(x,y,1,1);
    writePixel(color);
}

/**************************************************************************/
/*!
   @brief    Write a filled rectangle (must have a transaction in progress)
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    w   Width in pixels
    @param    h   Height in pixels
   @param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void Adafruit_SPITFT::writeFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color){
    if((x >= _width) || (y >= _height)) return;
    int16_t x2 = x + w - 1, y2 = y + h - 1;
    if((x2 < 0) || (y2 < 0)) return;

    // Clip left/top
    if(x < 0) {
        x = 0;
        w = x2 + 1;
    }
    if(y < 0) {
        y = 0;
        h = y2 + 1;
    }

    // Clip right/bottom
    if(x2 >= _width)  w = _width  - x;
    if(y2 >= _height) h = _height - y;

    int32_t len = (int32_t)w * h;
    setAddrWindow(x, y, w, h);
    writeColor(color, len);
}




void Adafruit_SPITFT::fillRectFast4colors(int16_t  x, int16_t y, uint16_t  w, uint16_t  h, uint16_t color0,uint16_t color1,uint16_t color2,uint16_t color3) {
//bool odd=true;//we use to interleave data
uint8_t tempx=x;uint8_t tempy=y; uint8_t countline=0;
uint8_t varx ;uint8_t vary;
while(!(SPSR & (1<<SPIF) ));
 SPI_BEGIN_TRANSACTION();digitalWriteFast(TFT_CS,HIGH);digitalWriteFast(TFT_CS,LOW); ;setAddrWindow(tempx+countline, tempy, w, h); 

vary=h>>1;while (vary !=0){
varx=w>>1;while (varx !=0 ){shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;complexspi = highByte(color0);  longSpiDelay;shortSpiDelay;complexspi = lowByte(color0);varx--;}
varx=w>>1;while (varx !=0 ){shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay ;complexspi = highByte(color1); longSpiDelay;shortSpiDelay;complexspi = lowByte(color1);varx--;} ;

vary--;}
vary=h>>1;while (vary !=0){
varx=w>>1;while (varx !=0 ){shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;complexspi = highByte(color2);  longSpiDelay;shortSpiDelay;complexspi = lowByte(color2);varx--;}
varx=w>>1;while (varx !=0 ){shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;complexspi = highByte(color3); longSpiDelay;shortSpiDelay;complexspi = lowByte(color3);varx--;} ;
vary--;}
digitalWriteFast(TFT_CS,LOW);digitalWriteFast(TFT_CS,HIGH);SPI_END_TRANSACTION();

}

void Adafruit_SPITFT::fillRectFast4colorsHalf(int16_t  x, int16_t y, uint16_t  w, uint16_t  h, uint16_t color0,uint16_t color1,uint16_t color2,uint16_t color3) {

  //to increase performance we will do lines down and up
bool odd=true;//we use to interleave data
uint8_t tempx=x;uint8_t tempy=y; uint8_t countline=0;
uint8_t varx ;uint8_t vary;
 vary=w>>1;while (vary !=0){
 if (odd){
 SPI_BEGIN_TRANSACTION();digitalWriteFast(TFT_CS,HIGH);digitalWriteFast(TFT_CS,LOW); ;setAddrWindow(tempx+countline, tempy, 1, h); 
varx=h>>1;while (varx !=0 ){shortSpiDelay;shortSpiDelay;shortSpiDelay; ;complexspi = highByte(color0);longSpiDelay;complexspi = lowByte(color0);varx--;}
varx=h>>1;while (varx !=0 ){shortSpiDelay;shortSpiDelay;shortSpiDelay; ;complexspi = highByte(color2);longSpiDelay;complexspi = lowByte(color2);varx--;} ;
   }; odd =!odd;
vary--;countline++; }
vary=w>>1;while (vary !=0){
 if (odd){
 SPI_BEGIN_TRANSACTION();digitalWriteFast(TFT_CS,HIGH);digitalWriteFast(TFT_CS,LOW); ;setAddrWindow(tempx+countline, tempy, 1, h); 
varx=h>>1;while (varx !=0 ){shortSpiDelay;shortSpiDelay;shortSpiDelay;;complexspi = highByte(color1);  longSpiDelay;complexspi = lowByte(color1);varx--;}
varx=h>>1;while (varx !=0 ){shortSpiDelay;shortSpiDelay;shortSpiDelay; ;complexspi = highByte(color3); longSpiDelay;complexspi = lowByte(color3);varx--;} ;
 }; odd =!odd;
vary--;countline++; }
digitalWriteFast(TFT_CS,LOW);digitalWriteFast(TFT_CS,HIGH);SPI_END_TRANSACTION();
}




void Adafruit_SPITFT::fillRectFast16colors(uint16_t x,uint16_t y, uint16_t  w, uint16_t  h, uint16_t color0,uint16_t color1,uint16_t color2,uint16_t color3) {
SPI_BEGIN_TRANSACTION();
digitalWriteFast(TFT_DC,HIGH);// old DC_HIGH();
digitalWriteFast(TFT_CS,LOW);

setAddrWindow(x,y, w, h+2); //2 aditional lines in case writes go past which an error currently allows. ill find it later. this is a bandaid.
//to increase performance we will do lines down and up
bool odd=true;//we use to interleave data
uint16_t tempx=x;uint16_t tempy=y; uint8_t countline=0;
hardcodew=w*4;hardcodeh=h*4;//we want to multiply by 2x
hardcodeToSubtractw=hardcodew/8;
hardcodeToSubtracth=hardcodeh/8;

int varx=hardcodew; int vary=hardcodeh;
 vary=hardcodew;while (vary >0){ 
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color0);shortSpiDelay;longSpiDelay;complexspi = lowByte(color0);varx-=hardcodeToSubtracth;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color1);shortSpiDelay;longSpiDelay;complexspi = lowByte(color1);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color2);shortSpiDelay;longSpiDelay;complexspi = lowByte(color2);varx-=hardcodeToSubtracth;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color3);shortSpiDelay;longSpiDelay;complexspi = lowByte(color3);varx-=hardcodeToSubtracth;} ;
  vary-=hardcodeToSubtractw;}

}

void Adafruit_SPITFT::fillRectFast16colors1(uint16_t color0,uint16_t color1,uint16_t color2,uint16_t color3) {

int varx=hardcodew; int vary=hardcodeh;
 vary=hardcodew;while (vary >0){ 
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color0);shortSpiDelay;longSpiDelay;complexspi = lowByte(color0);varx-=hardcodeToSubtracth;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color1);shortSpiDelay;longSpiDelay;complexspi = lowByte(color1);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color2);shortSpiDelay;longSpiDelay;complexspi = lowByte(color2);varx-=hardcodeToSubtracth;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color3);shortSpiDelay;longSpiDelay;complexspi = lowByte(color3);varx-=hardcodeToSubtracth;} ;
  vary-=hardcodeToSubtractw;}             
}
void Adafruit_SPITFT::fillRectFast16colors2(uint16_t color0,uint16_t color1,uint16_t color2,uint16_t color3) {

int varx=hardcodew; int vary=hardcodeh;
 vary=hardcodew;while (vary >0){ 
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color0);shortSpiDelay;longSpiDelay;complexspi = lowByte(color0);varx-=hardcodeToSubtracth;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color1);shortSpiDelay;longSpiDelay;complexspi = lowByte(color1);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color2);shortSpiDelay;longSpiDelay;complexspi = lowByte(color2);varx-=hardcodeToSubtracth;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color3);shortSpiDelay;longSpiDelay;complexspi = lowByte(color3);varx-=hardcodeToSubtracth;} ;
  vary-=hardcodeToSubtractw;}
            
}
void Adafruit_SPITFT::fillRectFast16colors3(uint16_t color0,uint16_t color1,uint16_t color2,uint16_t color3) {
int varx=hardcodew; int vary=hardcodeh;
 vary=hardcodew;while (vary >0){ 
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color0);shortSpiDelay;longSpiDelay;complexspi = lowByte(color0);varx-=hardcodeToSubtracth;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color1);shortSpiDelay;longSpiDelay;complexspi = lowByte(color1);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color2);shortSpiDelay;longSpiDelay;complexspi = lowByte(color2);varx-=hardcodeToSubtracth;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color3);shortSpiDelay;longSpiDelay;complexspi = lowByte(color3);varx-=hardcodeToSubtracth;} ;
  vary-=hardcodeToSubtractw;}

 //while(!(SPSR & (1<<SPIF) ));
digitalWriteFast(TFT_CS,HIGH);SPI_END_TRANSACTION();


}
//this is for half data writes (half as many pixles)
void Adafruit_SPITFT::fillRectFast16colorsHalf(uint16_t x,uint16_t y, uint16_t  w, uint16_t  h, uint16_t color0,uint16_t color1,uint16_t color2,uint16_t color3) {
//SPI_BEGIN_TRANSACTION();
//digitalWriteFast(TFT_DC,HIGH);// old DC_HIGH();
//digitalWriteFast(TFT_CS,LOW);
//setAddrWindow(x,y, w, h+2); //2 aditional lines in case writes go past which an error currently allows. ill find it later. this is a bandaid.
//to increase performance we will do lines down and up
bool odd=true;//we use to interleave data
//uint16_t tempx=x;uint16_t tempy=y; uint8_t countline=0;these are declared already
hardcodew=w*4;hardcodeh=h*4;//we want to multiply by 2x
hardcodeToSubtractw=hardcodew/4;
hardcodeToSubtracth=hardcodeh/8;
countline=0;
//we need to have every other line show and marked by set window
//int temp_x; //int temp_y;//int countline;//these 3 values are used for half mode features and declared at top of page
int varx=hardcodew; int vary=hardcodeh;
temp_x=x;
temp_y=y;
temp_h=h;
 vary=hardcodew;while (vary >0){ 
   if (odd){
  SPI_BEGIN_TRANSACTION();digitalWriteFast(TFT_CS,HIGH);digitalWriteFast(TFT_CS,LOW); ;setAddrWindow(temp_x+countline, temp_y, 1, h+2);
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color0);shortSpiDelay;longSpiDelay;complexspi = lowByte(color0);varx-=hardcodeToSubtracth;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color1);shortSpiDelay;longSpiDelay;complexspi = lowByte(color1);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color2);shortSpiDelay;longSpiDelay;complexspi = lowByte(color2);varx-=hardcodeToSubtracth;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color3);shortSpiDelay;longSpiDelay;complexspi = lowByte(color3);varx-=hardcodeToSubtracth;} ;
  vary-=hardcodeToSubtractw;};odd = !odd;countline++;
 }
 
}

void Adafruit_SPITFT::fillRectFast16colors1Half(uint16_t color0,uint16_t color1,uint16_t color2,uint16_t color3) {
int varx=hardcodew; int vary=hardcodeh;
bool odd=false;//we use to interleave data
int h=temp_h;//restores value
 vary=hardcodew;while (vary >0){ 
   if (odd){
  SPI_BEGIN_TRANSACTION();digitalWriteFast(TFT_CS,HIGH);digitalWriteFast(TFT_CS,LOW); ;setAddrWindow(temp_x+countline, temp_y, 1, h+2);
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color0);shortSpiDelay;longSpiDelay;complexspi = lowByte(color0);varx-=hardcodeToSubtracth;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color1);shortSpiDelay;longSpiDelay;complexspi = lowByte(color1);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color2);shortSpiDelay;longSpiDelay;complexspi = lowByte(color2);varx-=hardcodeToSubtracth;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color3);shortSpiDelay;longSpiDelay;complexspi = lowByte(color3);varx-=hardcodeToSubtracth;} ;
  vary-=hardcodeToSubtractw;};odd = !odd;countline++;
 }
           
}
void Adafruit_SPITFT::fillRectFast16colors2Half(uint16_t color0,uint16_t color1,uint16_t color2,uint16_t color3) {
int varx=hardcodew; int vary=hardcodeh;
bool odd=false;//we use to interleave data
int h=temp_h;//restores value
 vary=hardcodew;while (vary >0){ 
   if (odd){
  SPI_BEGIN_TRANSACTION();digitalWriteFast(TFT_CS,HIGH);digitalWriteFast(TFT_CS,LOW); ;setAddrWindow(temp_x+countline, temp_y, 1, h+2);
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color0);shortSpiDelay;longSpiDelay;complexspi = lowByte(color0);varx-=hardcodeToSubtracth;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color1);shortSpiDelay;longSpiDelay;complexspi = lowByte(color1);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color2);shortSpiDelay;longSpiDelay;complexspi = lowByte(color2);varx-=hardcodeToSubtracth;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color3);shortSpiDelay;longSpiDelay;complexspi = lowByte(color3);varx-=hardcodeToSubtracth;} ;
  vary-=hardcodeToSubtractw;};odd = !odd;countline++;
 }
        
}
void Adafruit_SPITFT::fillRectFast16colors3Half(uint16_t color0,uint16_t color1,uint16_t color2,uint16_t color3) {
int varx=hardcodew; int vary=hardcodeh;
bool odd=false;//we use to interleave data
int h=temp_h;//restores value
 vary=hardcodew;while (vary >0){ 
   if (odd){
  SPI_BEGIN_TRANSACTION();digitalWriteFast(TFT_CS,HIGH);digitalWriteFast(TFT_CS,LOW); ;setAddrWindow(temp_x+countline, temp_y, 1, h+2);
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color0);shortSpiDelay;longSpiDelay;complexspi = lowByte(color0);varx-=hardcodeToSubtracth;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color1);shortSpiDelay;longSpiDelay;complexspi = lowByte(color1);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color2);shortSpiDelay;longSpiDelay;complexspi = lowByte(color2);varx-=hardcodeToSubtracth;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color3);shortSpiDelay;longSpiDelay;complexspi = lowByte(color3);varx-=hardcodeToSubtracth;} ;
  vary-=hardcodeToSubtractw;};odd = !odd;countline++;
 }

 //while(!(SPSR & (1<<SPIF) ));
digitalWriteFast(TFT_CS,HIGH);SPI_END_TRANSACTION();


}
/**************************************************************************/
/*!
   @brief    Write a perfectly vertical line (must have a transaction in progress)
    @param    x   Top-most x coordinate
    @param    y   Top-most y coordinate
    @param    h   Height in pixels
   @param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void inline Adafruit_SPITFT::writeFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color){
    writeFillRect(x, y, 1, h, color);
}

/**************************************************************************/
/*!
   @brief    Write a perfectly horizontal line (must have a transaction in progress)
    @param    x   Left-most x coordinate
    @param    y   Left-most y coordinate
    @param    w   Width in pixels
   @param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void inline Adafruit_SPITFT::writeFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color){
    writeFillRect(x, y, w, 1, color);
}

/**************************************************************************/
/*!
   @brief    Draw a pixel - sets up transaction
    @param   x   x coordinate
    @param   y   y coordinate
   @param    color 16-bit 5-6-5 Color to draw with
*/
/**************************************************************************/
void Adafruit_SPITFT::drawPixel(int16_t x, int16_t y, uint16_t color){
    startWrite();
    writePixel(x, y, color);
    endWrite();
}

/**************************************************************************/
/*!
   @brief    Write a perfectly vertical line - sets up transaction
    @param    x   Top-most x coordinate
    @param    y   Top-most y coordinate
    @param    h   Height in pixels
   @param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void Adafruit_SPITFT::drawFastVLine(int16_t x, int16_t y,
        int16_t h, uint16_t color) {
    startWrite();
    writeFastVLine(x, y, h, color);
    endWrite();
}

/**************************************************************************/
/*!
   @brief    Write a perfectly horizontal line - sets up transaction
    @param    x   Left-most x coordinate
    @param    y   Left-most y coordinate
    @param    w   Width in pixels
   @param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void Adafruit_SPITFT::drawFastHLine(int16_t x, int16_t y,
        int16_t w, uint16_t color) {
    startWrite();
    writeFastHLine(x, y, w, color);
    endWrite();
}

/**************************************************************************/
/*!
   @brief    Fill a rectangle completely with one color.
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    w   Width in pixels
    @param    h   Height in pixels
   @param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
#if safety_Checks != false             
#define complexspi while(!(SPSR & (1<<SPIF) ));SPCRbackup = SPCR; SPCR = mySPCR; SPDR 
#else
#define complexspi SPDR //SPCRbackup = SPCR; SPCR = mySPCR; SPDR 
#endif
//this is small delay
#define shortSpiDelay __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); 
#define shortSpiDelay64avg __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t");__asm__("nop\n\t"); //this routine is slightly different
//last comment out ';' not needed here, but can be added in commands later 
#define longSpiDelay __asm__("nop\n\t"); __asm__("nop\n\t");__asm__("nop\n\t"); __asm__("nop\n\t");__asm__("nop\n\t"); __asm__("nop\n\t");  __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t");__asm__("nop\n\t");// __asm__("nop\n\t");  
#define longSpiDelay64AVG __asm__("nop\n\t"); __asm__("nop\n\t");__asm__("nop\n\t"); __asm__("nop\n\t");__asm__("nop\n\t"); __asm__("nop\n\t");  __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t"); __asm__("nop\n\t");__asm__("nop\n\t");// __asm__("nop\n\t");  

/*************************************************************************/
void Adafruit_SPITFT::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
        uint16_t color) {
    startWrite();   
    writeFillRect(x,y,w,h,color);
    endWrite();
}
void Adafruit_SPITFT::fillRectFast(int16_t x, int16_t y, int16_t w, int16_t h,uint16_t color) {
SPI_BEGIN_TRANSACTION();digitalWriteFast(TFT_CS,HIGH);digitalWriteFast(TFT_CS,LOW);
setAddrWindow(x, y, w, h);
uint8_t varx;uint8_t vary;
varx=h;while (varx !=0){vary=w;
while (vary !=0 ){shortSpiDelay;shortSpiDelay;shortSpiDelay; __asm__("nop\n\t");__asm__("nop\n\t");complexspi = highByte(color);  __asm__("nop\n\t");__asm__("nop\n\t");;longSpiDelay;complexspi = lowByte(color);vary--;}if (x ==1){return;};varx--;
}//we return from last one early!
digitalWriteFast(TFT_CS,LOW);digitalWriteFast(TFT_CS,HIGH);SPI_END_TRANSACTION();
}

void Adafruit_SPITFT::fillRectbarline(int16_t x, int16_t y, int16_t w, int16_t h,uint16_t color) {
SPI_BEGIN_TRANSACTION();digitalWriteFast(TFT_CS,HIGH);digitalWriteFast(TFT_CS,LOW);
setAddrWindow(x, y, w, h);
uint8_t varx;uint8_t vary;
varx=h;while (varx !=0){vary=w;
while (vary !=0 ){shortSpiDelay;shortSpiDelay;shortSpiDelay; __asm__("nop\n\t");__asm__("nop\n\t");complexspi = highByte(color);  __asm__("nop\n\t");__asm__("nop\n\t");;longSpiDelay;complexspi = lowByte(color);vary--;}if (x ==1){return;};varx--;
}//we return from last one early!
digitalWriteFast(TFT_CS,LOW);digitalWriteFast(TFT_CS,HIGH);SPI_END_TRANSACTION();
}



void Adafruit_SPITFT::fillRectFast64colorStart(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color0, uint16_t color1, uint16_t color2, uint16_t color3, uint16_t color4, uint16_t color5, uint16_t color6, uint16_t color7){
SPI_BEGIN_TRANSACTION();
digitalWriteFast(TFT_DC,HIGH);// old DC_HIGH();
digitalWriteFast(TFT_CS,LOW);

setAddrWindow(x,y, w, h+4); //4 aditional lines in case writes go past which an error currently allows. this prevents wrap around to top.
//to increase performance we will do lines down and up
bool odd=true;//we use to interleave data
uint16_t tempx=x;uint16_t tempy=y; uint8_t countline=0;
hardcodew=w*8;hardcodeh=h*8;//we want to multiply by 2x
hardcodeToSubtractw=hardcodew/8;
hardcodeToSubtracth=hardcodeh/8;

int varx=hardcodew; int vary=hardcodeh;
 vary=hardcodew;while (vary >0){ 
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color0);shortSpiDelay;longSpiDelay;complexspi = lowByte(color0);varx-=hardcodeToSubtracth<<1;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color1);shortSpiDelay;longSpiDelay;complexspi = lowByte(color1);varx-=hardcodeToSubtracth<<1;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color2);shortSpiDelay;longSpiDelay;complexspi = lowByte(color2);varx-=hardcodeToSubtracth<<1;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color3);shortSpiDelay;longSpiDelay;complexspi = lowByte(color3);varx-=hardcodeToSubtracth<<1;} ;

  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color4);shortSpiDelay;longSpiDelay;complexspi = lowByte(color4);varx-=hardcodeToSubtracth<<1;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color5);shortSpiDelay;longSpiDelay;complexspi = lowByte(color5);varx-=hardcodeToSubtracth<<1;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color6);shortSpiDelay;longSpiDelay;complexspi = lowByte(color6);varx-=hardcodeToSubtracth<<1;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color7);shortSpiDelay;longSpiDelay;complexspi = lowByte(color7);varx-=hardcodeToSubtracth<<1;} ;
  vary-=hardcodeToSubtractw<<1;}
}//end of part0

void Adafruit_SPITFT::fillRectFast64color(uint16_t color0, uint16_t color1, uint16_t color2, uint16_t color3, uint16_t color4, uint16_t color5, uint16_t color6, uint16_t color7){
int varx=hardcodew; int vary=hardcodeh;
 vary=hardcodew;while (vary >0){ 
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color0);shortSpiDelay;longSpiDelay;complexspi = lowByte(color0);varx-=hardcodeToSubtracth<<1;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color1);shortSpiDelay;longSpiDelay;complexspi = lowByte(color1);varx-=hardcodeToSubtracth<<1;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color2);shortSpiDelay;longSpiDelay;complexspi = lowByte(color2);varx-=hardcodeToSubtracth<<1;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color3);shortSpiDelay;longSpiDelay;complexspi = lowByte(color3);varx-=hardcodeToSubtracth<<1;} ;

  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color4);shortSpiDelay;longSpiDelay;complexspi = lowByte(color4);varx-=hardcodeToSubtracth<<1;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color5);shortSpiDelay;longSpiDelay;complexspi = lowByte(color5);varx-=hardcodeToSubtracth<<1;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color6);shortSpiDelay;longSpiDelay;complexspi = lowByte(color6);varx-=hardcodeToSubtracth<<1;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color7);shortSpiDelay;longSpiDelay;complexspi = lowByte(color7);varx-=hardcodeToSubtracth<<1;} ;
  vary-=hardcodeToSubtractw<<1;}
}//end of part 1

void Adafruit_SPITFT::fillRectFast64colorEnd(){
 //while(!(SPSR & (1<<SPIF) ));
digitalWriteFast(TFT_CS,HIGH);SPI_END_TRANSACTION();

  
}//end of part 7

//256 at a time!
void Adafruit_SPITFT::fillRectFast256colorStart(int16_t x, int16_t y, int16_t w, int16_t h, 
                                                uint16_t color0, uint16_t color1, uint16_t color2, uint16_t color3,
                                                uint16_t color4, uint16_t color5, uint16_t color6, uint16_t color7, 
                                                uint16_t color8, uint16_t color9, uint16_t color10, uint16_t color11,
                                                uint16_t color12,uint16_t color13, uint16_t color14, uint16_t color15){
//we process 16x16 rectangles at a time. this code is too complex here, look at how 64 rectangles work instead
  //this is where squares are written together!
SPI_BEGIN_TRANSACTION();
digitalWriteFast(TFT_DC,HIGH);// old DC_HIGH();
digitalWriteFast(TFT_CS,LOW);

setAddrWindow(x,y, w, h+4); //4 aditional lines in case writes go past which an error currently allows. this prevents wrap around to top.
//to increase performance we will do lines down and up
bool odd=true;//we use to interleave data
uint16_t tempx=x;uint16_t tempy=y; uint8_t countline=0;
hardcodew=w*8;hardcodeh=h*8;//we want to multiply by 2x
hardcodeToSubtractw=hardcodew/2;
hardcodeToSubtracth=hardcodeh/2;

int varx=hardcodew; int vary=hardcodeh;
 vary=hardcodew;while (vary >0){ shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;//delay(10);
  shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;complexspi = highByte(color0);shortSpiDelay;longSpiDelay;complexspi = lowByte(color0);shortSpiDelay;shortSpiDelay;shortSpiDelay;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color1);shortSpiDelay;longSpiDelay;complexspi = lowByte(color1);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color2);shortSpiDelay;longSpiDelay;complexspi = lowByte(color2);varx-=hardcodeToSubtracth;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color3);shortSpiDelay;longSpiDelay;complexspi = lowByte(color3);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color4);shortSpiDelay;longSpiDelay;complexspi = lowByte(color4);varx-=hardcodeToSubtracth;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color5);shortSpiDelay;longSpiDelay;complexspi = lowByte(color5);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color6);shortSpiDelay;longSpiDelay;complexspi = lowByte(color6);varx-=hardcodeToSubtracth;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color7);shortSpiDelay;longSpiDelay;complexspi = lowByte(color7);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color8);shortSpiDelay;longSpiDelay;complexspi = lowByte(color8);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color9);shortSpiDelay;longSpiDelay;complexspi = lowByte(color9);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color10);shortSpiDelay;longSpiDelay;complexspi = lowByte(color10);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color11);shortSpiDelay;longSpiDelay;complexspi = lowByte(color11);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color12);shortSpiDelay;longSpiDelay;complexspi = lowByte(color12);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color13);shortSpiDelay;longSpiDelay;complexspi = lowByte(color13);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color14);shortSpiDelay;longSpiDelay;complexspi = lowByte(color14);varx-=hardcodeToSubtracth;} ;
  shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;
  shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;complexspi = highByte(color15);shortSpiDelay;longSpiDelay;complexspi = lowByte(color15);
 // shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;complexspi = highByte(color15);shortSpiDelay;longSpiDelay;complexspi = lowByte(color15);
  vary-=hardcodeToSubtractw;}
}//end of part0




void Adafruit_SPITFT::fillRectFast256color(     uint16_t color0, uint16_t color1, uint16_t color2, uint16_t color3,
                                                uint16_t color4, uint16_t color5, uint16_t color6, uint16_t color7, 
                                                uint16_t color8, uint16_t color9, uint16_t color10, uint16_t color11,
                                                uint16_t color12,uint16_t color13, uint16_t color14, uint16_t color15){
 //uint16_t countx; uint16_t county;//we use these to understand what to draw!
int varx=hardcodew; int vary=hardcodeh;
  vary=hardcodew;while (vary >0){ shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;//delay(10);
  shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;complexspi = highByte(color0);shortSpiDelay;longSpiDelay;complexspi = lowByte(color0);shortSpiDelay;shortSpiDelay;shortSpiDelay;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color1);shortSpiDelay;longSpiDelay;complexspi = lowByte(color1);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color2);shortSpiDelay;longSpiDelay;complexspi = lowByte(color2);varx-=hardcodeToSubtracth;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color3);shortSpiDelay;longSpiDelay;complexspi = lowByte(color3);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color4);shortSpiDelay;longSpiDelay;complexspi = lowByte(color4);varx-=hardcodeToSubtracth;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color5);shortSpiDelay;longSpiDelay;complexspi = lowByte(color5);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color6);shortSpiDelay;longSpiDelay;complexspi = lowByte(color6);varx-=hardcodeToSubtracth;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color7);shortSpiDelay;longSpiDelay;complexspi = lowByte(color7);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color8);shortSpiDelay;longSpiDelay;complexspi = lowByte(color8);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color9);shortSpiDelay;longSpiDelay;complexspi = lowByte(color9);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color10);shortSpiDelay;longSpiDelay;complexspi = lowByte(color10);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color11);shortSpiDelay;longSpiDelay;complexspi = lowByte(color11);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color12);shortSpiDelay;longSpiDelay;complexspi = lowByte(color12);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color13);shortSpiDelay;longSpiDelay;complexspi = lowByte(color13);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color14);shortSpiDelay;longSpiDelay;complexspi = lowByte(color14);varx-=hardcodeToSubtracth;} ;
  shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;
  shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;complexspi = highByte(color15);shortSpiDelay;longSpiDelay;complexspi = lowByte(color15);
  //shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;shortSpiDelay;complexspi = highByte(color15);shortSpiDelay;longSpiDelay;complexspi = lowByte(color15);
  vary-=hardcodeToSubtractw;}

}//end of part 1



void Adafruit_SPITFT::fillRectFast256colorEnd(){
 //while(!(SPSR & (1<<SPIF) ));
digitalWriteFast(TFT_CS,HIGH);SPI_END_TRANSACTION();

}//end of part 7



//64half driver area**********************************
void Adafruit_SPITFT::fillRectFast64colorStartHalf(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color0, uint16_t color1, uint16_t color2, uint16_t color3, uint16_t color4, uint16_t color5, uint16_t color6, uint16_t color7){

//SPI_BEGIN_TRANSACTION();
//digitalWriteFast(TFT_DC,HIGH);// old DC_HIGH();
//digitalWriteFast(TFT_CS,LOW);
//setAddrWindow(x,y, w, h+2); //2 aditional lines in case writes go past which an error currently allows. ill find it later. this is a bandaid.
//to increase performance we will do lines down and up
bool odd=true;//we use to interleave data
//uint16_t tempx=x;uint16_t tempy=y; uint8_t countline=0;these are declared already
hardcodew=w*4;hardcodeh=h*4;//we want to multiply by 2x
hardcodeToSubtractw=hardcodew/2;
hardcodeToSubtracth=hardcodeh/4;
countline=0;
//we need to have every other line show and marked by set window
//int temp_x; //int temp_y;//int countline;//these 3 values are used for half mode features and declared at top of page
int varx=hardcodew; int vary=hardcodeh;
temp_x=x;
temp_y=y;
temp_h=h;
 vary=hardcodew;while (vary >0){ 
   if (odd){
  SPI_BEGIN_TRANSACTION();digitalWriteFast(TFT_CS,HIGH);digitalWriteFast(TFT_CS,LOW); ;setAddrWindow(temp_x+countline, temp_y, 1, h+2);
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color0);shortSpiDelay;longSpiDelay;complexspi = lowByte(color0);varx-=hardcodeToSubtracth;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color1);shortSpiDelay;longSpiDelay;complexspi = lowByte(color1);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color2);shortSpiDelay;longSpiDelay;complexspi = lowByte(color2);varx-=hardcodeToSubtracth;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color3);shortSpiDelay;longSpiDelay;complexspi = lowByte(color3);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color4);shortSpiDelay;longSpiDelay;complexspi = lowByte(color4);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color5);shortSpiDelay;longSpiDelay;complexspi = lowByte(color5);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color6);shortSpiDelay;longSpiDelay;complexspi = lowByte(color6);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color7);shortSpiDelay;longSpiDelay;complexspi = lowByte(color7);varx-=hardcodeToSubtracth;} ;

  vary-=hardcodeToSubtractw;};odd = !odd;countline++;
 }
 
}

void Adafruit_SPITFT::fillRectFast64colorHalf(uint16_t color0, uint16_t color1, uint16_t color2, uint16_t color3, uint16_t color4, uint16_t color5, uint16_t color6, uint16_t color7){
bool odd=false;//we use to interleave data
//uint16_t tempx=x;uint16_t tempy=y; uint8_t countline=0;these are declared already
//we need to have every other line show and marked by set window
//int temp_x; //int temp_y;//int countline;//these 3 values are used for half mode features and declared at top of page

int varx=hardcodew; int vary=hardcodeh;
int h=temp_h;//restores value
 vary=hardcodew;while (vary >0){ 
   if (odd){
  SPI_BEGIN_TRANSACTION();digitalWriteFast(TFT_CS,HIGH);digitalWriteFast(TFT_CS,LOW); ;setAddrWindow(temp_x+countline, temp_y, 1, h+2);
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color0);shortSpiDelay;longSpiDelay;complexspi = lowByte(color0);varx-=hardcodeToSubtracth;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color1);shortSpiDelay;longSpiDelay;complexspi = lowByte(color1);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color2);shortSpiDelay;longSpiDelay;complexspi = lowByte(color2);varx-=hardcodeToSubtracth;};
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color3);shortSpiDelay;longSpiDelay;complexspi = lowByte(color3);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color4);shortSpiDelay;longSpiDelay;complexspi = lowByte(color4);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color5);shortSpiDelay;longSpiDelay;complexspi = lowByte(color5);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color6);shortSpiDelay;longSpiDelay;complexspi = lowByte(color6);varx-=hardcodeToSubtracth;} ;
  varx=hardcodeh;while (varx >0 ){shortSpiDelay;shortSpiDelay;complexspi = highByte(color7);shortSpiDelay;longSpiDelay;complexspi = lowByte(color7);varx-=hardcodeToSubtracth;} ;

  vary-=hardcodeToSubtractw;};odd = !odd;countline++;
 }
}//end of part 1

void Adafruit_SPITFT::fillRectFast64colorEndHalf(){
 //while(!(SPSR & (1<<SPIF) ));
digitalWriteFast(TFT_CS,HIGH);SPI_END_TRANSACTION();

  
}//end of part 7
//64half driver area********* done *******************



void Adafruit_SPITFT::fillRectFastHalf(int16_t x, int16_t y, int16_t w, int16_t h,uint16_t color) {
bool odd=true;//we use to interleave data
uint8_t tempx=x;uint8_t tempy=y; uint8_t countline=0;
uint8_t varx ;uint8_t vary;
SPI_BEGIN_TRANSACTION();digitalWriteFast(TFT_CS,HIGH);digitalWriteFast(TFT_CS,LOW);
varx=w/2;while (varx !=0){vary=h; if (odd){
 SPI_BEGIN_TRANSACTION();digitalWriteFast(TFT_CS,HIGH);digitalWriteFast(TFT_CS,LOW); ;setAddrWindow(tempx+countline, tempy, 1, h);
while (vary !=0 ){shortSpiDelay;shortSpiDelay;shortSpiDelay; __asm__("nop\n\t");__asm__("nop\n\t");complexspi = highByte(color);  __asm__("nop\n\t");__asm__("nop\n\t");;longSpiDelay;complexspi = lowByte(color);vary--;};varx--;};
odd =!odd;countline++;};
digitalWriteFast(TFT_CS,LOW);digitalWriteFast(TFT_CS,HIGH);SPI_END_TRANSACTION();
}
/**************************************************************************/
/*!
    @brief      Invert the display using built-in hardware command
    @param   i  True if you want to invert, false to make 'normal'
*/
/**************************************************************************/
void Adafruit_SPITFT::invertDisplay(boolean i) {
  startWrite();
  writeCommand(i ? invertOnCommand : invertOffCommand);
  endWrite();
}


/**************************************************************************/
/*!
   @brief   Draw a 16-bit image (RGB 5/6/5) at the specified (x,y) position.  
   For 16-bit display devices; no color reduction performed.
   Adapted from https://github.com/PaulStoffregen/ILI9341_t3 
   by Marc MERLIN. See examples/pictureEmbed to use this.
   5/6/2017: function name and arguments have changed for compatibility
   with current GFX library and to avoid naming problems in prior
   implementation.  Formerly drawBitmap() with arguments in different order.

    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    pcolors  16-bit array with 16-bit color bitmap
    @param    w   Width of bitmap in pixels
    @param    h   Height of bitmap in pixels
*/
/**************************************************************************/
void Adafruit_SPITFT::drawRGBBitmap(int16_t x, int16_t y,
  uint16_t *pcolors, int16_t w, int16_t h) {

    int16_t x2, y2; // Lower-right coord
    if(( x             >= _width ) ||      // Off-edge right
       ( y             >= _height) ||      // " top
       ((x2 = (x+w-1)) <  0      ) ||      // " left
       ((y2 = (y+h-1)) <  0)     ) return; // " bottom

    int16_t bx1=0, by1=0, // Clipped top-left within bitmap
            saveW=w;      // Save original bitmap width value
    if(x < 0) { // Clip left
        w  +=  x;
        bx1 = -x;
        x   =  0;
    }
    if(y < 0) { // Clip top
        h  +=  y;
        by1 = -y;
        y   =  0;
    }
    if(x2 >= _width ) w = _width  - x; // Clip right
    if(y2 >= _height) h = _height - y; // Clip bottom

    pcolors += by1 * saveW + bx1; // Offset bitmap ptr to clipped top-left
    startWrite();
    setAddrWindow(x, y, w, h); // Clipped area
    while(h--) { // For each (clipped) scanline...
      writePixels(pcolors, w); // Push one (clipped) row
      pcolors += saveW; // Advance pointer by one full (unclipped) line
    }
    endWrite();
}

#endif // !__AVR_ATtiny85__
#endif
#endif
