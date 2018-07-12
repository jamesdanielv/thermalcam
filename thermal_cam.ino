//this is better with included zip package that has all needed libraries in it for testing. 
//this is a work in process. 16x16 subsample seems to work ok, trying to verify 32x32 and add up to 512 by 512
//some changes may be needed to have includes location changes also spixx.h just uses libray in file change back to <spi.h>
//this is the only file where code changes are, so just replace old file with this one 
//and change libary include file locations
//32x32 might need work. will need to first increase color table to be sure
//located here on github :https://github.com/jamesdanielv/thermalcam/edit/master/thermal_cam.ino
/***************************************************************************
  This is a library for the AMG88xx GridEYE 8x8 IR camera
  This sketch makes a 64 pixel thermal camera with the GridEYE sensor
  and a 128x128 tft screen https://www.adafruit.com/product/2088
  Designed specifically to work with the Adafruit AMG88 breakout
  ----> http://www.adafruit.com/products/3538
  These sensors use I2C to communicate. The device's I2C address is 0x69
  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!
  Written by Dean Miller for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include "Adafruit_GFX.h"    // Core graphics library
#include "Adafruit_ST7735.h" // Hardware-specific library
#include "SPIxx.h"

#include <Wire.h>
#include "Adafruit_AMG88xx.h"
#include <avr/pgmspace.h>
#define TFT_CS     10 //chip select pin for the TFT screen
#define TFT_RST    9  // you can also connect this to the Arduino reset
                      // in which case, set this #define pin to 0!
#define TFT_DC     8

//low range of the sensor (this will be blue on the screen)
#define MINTEMP 22

//high range of the sensor (this will be red on the screen)
#define MAXTEMP 34


// 0 no optimse |1 pixels only written whe color changed| 2 pixels also optimized for most changed ones first (deals with noise issues)
#define optimize 2

#define interpolatemode 1 //can be 0-2,4,8,16,32,64,128,256,512, 0--> 8x8 resolution,  1--> 16x16 resolution -->512x512 (higher resolution needs more color detail to be tested. 
//const dataType variableName[] PROGMEM = {data0, data1, data3…​};// how to formate table for progmem or the way it is listed. in order for it to work correctly int needs to be reconverted to  (uint16_t)
//the colors we will be using stored into a flash instead of ram which is valuable on arudino just use (uint16_t)pgm_read_word_near(camColors+ instead of Camcolors[]
const PROGMEM uint16_t camColors[] =  {0x480F,
0x400F,0x400F,0x400F,0x4010,0x3810,0x3810,0x3810,0x3810,0x3010,0x3010,
0x3010,0x2810,0x2810,0x2810,0x2810,0x2010,0x2010,0x2010,0x1810,0x1810,
0x1811,0x1811,0x1011,0x1011,0x1011,0x0811,0x0811,0x0811,0x0011,0x0011,
0x0011,0x0011,0x0011,0x0031,0x0031,0x0051,0x0072,0x0072,0x0092,0x00B2,
0x00B2,0x00D2,0x00F2,0x00F2,0x0112,0x0132,0x0152,0x0152,0x0172,0x0192,
0x0192,0x01B2,0x01D2,0x01F3,0x01F3,0x0213,0x0233,0x0253,0x0253,0x0273,
0x0293,0x02B3,0x02D3,0x02D3,0x02F3,0x0313,0x0333,0x0333,0x0353,0x0373,
0x0394,0x03B4,0x03D4,0x03D4,0x03F4,0x0414,0x0434,0x0454,0x0474,0x0474,
0x0494,0x04B4,0x04D4,0x04F4,0x0514,0x0534,0x0534,0x0554,0x0554,0x0574,
0x0574,0x0573,0x0573,0x0573,0x0572,0x0572,0x0572,0x0571,0x0591,0x0591,
0x0590,0x0590,0x058F,0x058F,0x058F,0x058E,0x05AE,0x05AE,0x05AD,0x05AD,
0x05AD,0x05AC,0x05AC,0x05AB,0x05CB,0x05CB,0x05CA,0x05CA,0x05CA,0x05C9,
0x05C9,0x05C8,0x05E8,0x05E8,0x05E7,0x05E7,0x05E6,0x05E6,0x05E6,0x05E5,
0x05E5,0x0604,0x0604,0x0604,0x0603,0x0603,0x0602,0x0602,0x0601,0x0621,
0x0621,0x0620,0x0620,0x0620,0x0620,0x0E20,0x0E20,0x0E40,0x1640,0x1640,
0x1E40,0x1E40,0x2640,0x2640,0x2E40,0x2E60,0x3660,0x3660,0x3E60,0x3E60,
0x3E60,0x4660,0x4660,0x4E60,0x4E80,0x5680,0x5680,0x5E80,0x5E80,0x6680,
0x6680,0x6E80,0x6EA0,0x76A0,0x76A0,0x7EA0,0x7EA0,0x86A0,0x86A0,0x8EA0,
0x8EC0,0x96C0,0x96C0,0x9EC0,0x9EC0,0xA6C0,0xAEC0,0xAEC0,0xB6E0,0xB6E0,
0xBEE0,0xBEE0,0xC6E0,0xC6E0,0xCEE0,0xCEE0,0xD6E0,0xD700,0xDF00,0xDEE0,
0xDEC0,0xDEA0,0xDE80,0xDE80,0xE660,0xE640,0xE620,0xE600,0xE5E0,0xE5C0,
0xE5A0,0xE580,0xE560,0xE540,0xE520,0xE500,0xE4E0,0xE4C0,0xE4A0,0xE480,
0xE460,0xEC40,0xEC20,0xEC00,0xEBE0,0xEBC0,0xEBA0,0xEB80,0xEB60,0xEB40,
0xEB20,0xEB00,0xEAE0,0xEAC0,0xEAA0,0xEA80,0xEA60,0xEA40,0xF220,0xF200,
0xF1E0,0xF1C0,0xF1A0,0xF180,0xF160,0xF140,0xF100,0xF0E0,0xF0C0,0xF0A0,
0xF080,0xF060,0xF040,0xF020,0xF800,};

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

Adafruit_AMG88xx amg;
unsigned long delayTime;

float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
//dont change optimize here, look for value at top
#if optimize > 0
byte pixelsbuf[AMG88xx_PIXEL_ARRAY_SIZE];

#endif
uint16_t displayPixelWidth, displayPixelHeight;


void setup() {
 // Serial.begin(9600);
 Serial.begin(115200);
    Serial.println(F("AMG88xx thermal camera!"));

    tft.initR(INITR_144GREENTAB);   // initialize a ST7735S chip, black tab
    tft.fillScreen(ST7735_BLACK);

#define    displayPixelWidth  tft.width() / 8  //allows values to be hardcoded
#define    displayPixelHeight   tft.height() / 8

    //tft.setRotation(3);

    bool status;
    
    // default settings
    status = amg.begin();
    if (!status) {
        Serial.println(F("Could not find a valid AMG88xx sensor, check wiring!"));
        while (1);
    }
    
    Serial.println( F("-- Thermal Camera Test --"));
   
    delay(100); // let sensor boot up

}


// friends it may be magical bologna, but it speeds things up quite abit!
//it looks at buffered value, and only updates the greated changes pixel locations
//and once changed, the next sets of pixels get updated. this reduces noise and 
//increases data thruput to display by only updating if change, and if overloaded 
//by most change. faster performance if fewer pixels set to priority                          
//      data range  -->     ||  limit data  ||Fuzzy logic data reduction                                           
//=========================]|| to greatest  ||==========================
//   bandwith compressor    ||    changed   || (by james villeneuve 2018)
#define speedUpCompression 8 //lower number is faster (it sets priority pixels amount, however too much means rest of display updates more slowly)
// also includes code to make sure every pixel updates but at a slower rate (1/8)

//higher number precision means more change allowed for pixel noise

//noise suppression, what shows at lower limits goes to lowest color 

 uint8_t compressionnumber;//this counts pixels processed, if to many processed we make compression flux higher
 uint8_t compressionflux=50;//this number goes up and down depending on how many pixesl are changing (usually from noise)
byte runagain=0;//used to refesh different amounts, for example we only process 1/4 of low quality pixes to make sure refesh happens besides with difference
void loop() {


  

  
  amg.readPixels(pixels);


 byte i=0;
 byte j=0; 
 byte k=0;
#if interpolatemode == 0
byte interpolate=0;
#endif
#if interpolatemode == 1
byte interpolate=1;
#endif
#if interpolatemode == 2
byte interpolate=2;
#endif
compressionnumber=0;// we reset each time thru
//need to convert this into numbers that done need changing AMG88xx_PIXEL_ARRAY_SIZE or array witdh and hieght
 runagain++;runagain=runagain&7; //we run this routing to update a few times0,1, maybe more later on
 while(k<8 ){ 
switch(k){// switch more efficient than if compiler tree branches
  case 0: j=0; break;//we interleave page for smoother refresh
  case 1: j=2; break;//this fixes visual anomoly when lots of data
  case 2: j=4; break;//changes at once and frame rate crawls
  case 3: j=6; break;//it takes a few frames for system
  case 4: j=1; break;//to adjust range to filter out noisy
  case 5: j=3; break;//pixels when all of them change at once
  case 6: j=5; break;
  case 7: j=7; break;
}
if (i>7){i=0;k++;}
// | i . j ------>
// | direction of display sensor. helps when figuring out interpolation
// |
//\/

   //bilineral interpolation is based on averaging dat between right, bottom, and bottom right pixel
    int colorIndex = map(pixels[i+j*8], MINTEMP, MAXTEMP, 0, 255); //we resuse this sample 
    
    // Serial.println(deadrange);
 
    //we now compress and only update pixels on screen that change the most! it works and over time they all change!
#if optimize >1 
   if  ((pixelsbuf[i+j*8] -colorIndex)>compressionflux ||(colorIndex-pixelsbuf[i+j*8]>compressionflux ||i+j*8==(i+j*8)|runagain)){//runagain value just esuresallows non priority pixel areas to update as well besides noise
    //we only draw if it is pixle with higher priority of change AND PIXEL IS NOT SAME COLOR ALREADY
#endif    
//below checks the buffer 
#if optimize >0 

if (colorIndex !=pixelsbuf[i+j*8]){
#endif
#if optimize >1   
if (i+j*8 !=(i+j*8)|runagain)    compressionnumber++;// we only count priority pixels, not ones with slower refresh
 
#endif    



#if interpolatemode >0
//how it updates when more than 2x2 sub pixels
int pixelSizeDivide= 2*interpolatemode ; 
//[0][4][8][c] or [0][2] //order reverses depending on sample interpolateSampleDir 
//[1][5][9][d]    [1][3]
//[2][6][a][e]
//[3][7][b][f]
  //fast subdivide low memory pixel enhancing code (by James Villeneuve 7-2018 referencing MIT code and adafruit library )
     
int interpolatesampledir2=1; if(i<4){interpolatesampledir2=1;}else{interpolatesampledir2=-1;}//top(1) or bottom quadrunt (-1)
int interpolateSampleDir =1;// left  (1) or right quadrunt (-1)
int offset=0;
if (j<4){interpolateSampleDir =1;}// we process left to right here . we need to change this so it scales with display resolution
         else{interpolateSampleDir =-1;offset=displayPixelHeight-displayPixelHeight/pixelSizeDivide;}//if past half way on display we sample in other direction
 //long timecount=micros();
for (int raster_x=0;raster_x !=(pixelSizeDivide*interpolateSampleDir)  ;raster_x += 1*interpolateSampleDir){ //done with != instead of <> so i could invert direction ;)
    for (int raster_y=0;raster_y != (pixelSizeDivide* interpolateSampleDir) ;raster_y += 1* interpolateSampleDir){ //0,1  
//we keep sample size from nieghbor pixels even when sample divides increase
int  tempcolor= map(pixels[(i+(interpolatesampledir2+raster_y/(pixelSizeDivide/2)))+(j+(raster_x/(pixelSizeDivide/2)))*8], MINTEMP, MAXTEMP, 0, 255);//we constrain color after subsampling
//next line changes the average of the color between the main pixel and the sub pixels
tempcolor=(( tempcolor*(pixelSizeDivide-raster_y)+ colorIndex*raster_y)/pixelSizeDivide+(tempcolor*(raster_x)+colorIndex*(pixelSizeDivide-raster_x))/pixelSizeDivide)/2;//subsample with real pixel and surounding pixels
//tempcolor=(tempcolor+ colorIndex)/2;//subsample with real pixel and surounding pixels

tempcolor=constrain(tempcolor,0,255);//subsample with real pixel and surounding pixels

//formated line below for easier reading
//*******place pixels*************
 
tft.fillRect(displayPixelWidth *j
+offset+ (interpolateSampleDir*displayPixelWidth/pixelSizeDivide)*(raster_x*interpolateSampleDir), //we reduce pixle size and step over in raster extra pixels created
displayPixelHeight* i+offset+(interpolateSampleDir*displayPixelHeight/pixelSizeDivide)*(raster_y*interpolateSampleDir),
displayPixelWidth/pixelSizeDivide,//we divide width of pixels. /2,4,8,16 is fast and compiler can just do bit shift for it
displayPixelHeight/pixelSizeDivide,//we divide hieght of pixels.
(uint16_t)pgm_read_word_near(camColors+tempcolor));  //we update pixel location with new subsampled pixel.
//would it make sense to subdivide to color of pixel directly? yes except camcolors is set with colors translated for heat.
//maybe this part can be improved in future.
//***^^^^place pixels^^^^*********   

   }//interpolatepixel_y 
}//interpolatepixel_x    





#endif

//below ends the check buffer loop
#if optimize >0 
}
#endif
#if optimize >1
    }
#endif
   //we update each pixel data
 #if optimise > 1
 //pixelsbuf[i+j*8]=colorIndex;
 #endif


      i++;     
//old way
  //         tft.fillRect(displayPixelHeight * (i /8), displayPixelWidth * (i % 8),
    //    displayPixelHeight, displayPixelWidth, camColors[colorIndex]);
 
  }

//we sample one time per frame 
 if (compressionnumber>speedUpCompression){if (compressionflux<255) {compressionflux+=1;};
 }//we increase range more slowly 
 else{
  if (compressionflux>0){compressionflux-=1;};
 }
 




}
