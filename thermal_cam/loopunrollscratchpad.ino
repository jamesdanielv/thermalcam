/*

/////////////////////////////////begin of loop unroll///////////////////////////////////// 
#if interpolatemode > 0
//long timer =micros();//we are timing code for speed
//how it updates when more than 2x2 sub pixels
int  pixelSizeDivide =2; //*interpolatemode; //we may not need to multiply at this point. leaving here for future revisions
//[0][4][8][c] or [0][2] //order reverses depending on sample interpolateSampleDir 
//[1][5][9][d]    [1][3]
//[2][6][a][e]
//[3][7][b][f]
  //fast subdivide low memory pixel enhancing code (by James Villeneuve 7-2018 also referencing MIT code and adafruit library for pixel placement code)
     
int interpolatesampledir2=1; if(i<4){interpolatesampledir2=1;}else{interpolatesampledir2=-1;}//top(1) or bottom quadrunt (-1)
int interpolateSampleDir =1;// left  (1) or right quadrunt (-1)
int offset=0;

//getsubpixelcolor()//side pixel,vertical pixil,LeftOrRigh,current pixel
if (j<4){interpolateSampleDir =1;}// we process left to right here . we need to change this so it scales with display resolution
         else{interpolateSampleDir =-1;offset=displayPixelHeight-displayPixelHeight/pixelSizeDivide;}//if past half way on display we sample in other direction
 //long timecount=micros();
// getsubpixelcolor(pixelSizerDivide);//side pixel,vertical pixil,LeftOrRigh,current pixel
int raster_x=0;
while (raster_x !=(pixelSizeDivide*interpolateSampleDir)){ //done with != instead of <> so i could invert direction ;)  
    int raster_y=0;
    while (raster_y != pixelSizeDivide* interpolateSampleDir) { //0,1  

//we keep sample size from nieghbor pixels even when sample divides increase

#if colorMode <2 //Fasttempcachemap256(pixels[i+j*8]);
int  tempcolor= Fasttempcachemap256(pixels[(i+raster_y+interpolatesampledir2)+(j+raster_x+interpolateSampleDir)*8]);
 #else
int  tempcolor=Fasttempcachemap1024(pixels[(i+raster_y+interpolatesampledir2)+(j+raster_x+interpolateSampleDir)*8]);//we constrain color after subsampling
 
#endif

//next line changes the average of the color between the main pixel and the sub pixels
tempcolor=( tempcolor*(2-raster_y)+ colorIndex*raster_y+tempcolor*raster_x+colorIndex*(2-raster_x))/4;//subsample with real pixel and surounding pixels
#if blur == true
tempcolor=(tempcolor+colorIndex)/2;
#endif

tempcolor=colorClamp(tempcolor);

#endif  //interpolatemode

//***************************************************************************************************************************************************************
//this is end of draw area for sub pixels 32 x32 resolution

#if interpolatemode > 1
//we take data from subpixel routine, and run it to a buffer temporarily. this buffer is 16x16, then we run another subsample. seems repetative, but is needed to visualize how to iterate sub sampling
#define displayPixelHeightx 2
#define displayPixelWidthx 2
#define offsetx 1
//this stores 8x8 sub sample data into a buffer of 16x16********************************************************************************************************
int tempc=0;if (j<4){tempc=-1;}//corrects an error that i have not fixe another way //postbuffer2
postbuffer[((j+j+offsetx+ (interpolateSampleDir)*(raster_x*interpolateSampleDir))+tempc)*16+i+ i+1+(interpolateSampleDir)*(raster_y*interpolateSampleDir)+tempc]=tempcolor;
#endif

#if  interpolatemode == 1
//we use data directly from interpolated sub
   #if  subpixelcoloroptimized !=-1 //we test without lcd writes with -1
fillRectFast(displayPixelWidth *j+
offset+ (interpolateSampleDir*displayPixelWidth/pixelSizeDivide)*(raster_x*interpolateSampleDir), //we reduce pixle size and step over in raster extra pixels created
displayPixelHeight* i+offset+(interpolateSampleDir*displayPixelHeight/pixelSizeDivide)*(raster_y*interpolateSampleDir),
displayPixelWidth/pixelSizeDivide,//we divide width of pixels. /2,4,8,16 is fast and compiler can just do bit shift for it
displayPixelHeight/pixelSizeDivide,//we divide hieght of pixels.
(uint16_t)pgm_read_word_near(camColors+tempcolor));  //we update pixel location with new subsampled pixel.
#endif
#endif

//would it make sense to subdivide to color of pixel directly? yes except camcolors is set with colors translated for heat.

//delay(50);
#if interpolatemode >0
raster_y +=  interpolateSampleDir;
   }//interpolatepixel_y 

   raster_x += interpolateSampleDir;
}//interpolatepixel_x 
#endif
 /////////////////////////////////end of loop unroll/////////////////////////////////////
 */

