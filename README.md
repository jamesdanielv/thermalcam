//this version is rewritten by james villeneuve (jamesdanielv). i am thankful to the MIT code included, and adafruit for the immense time savings they game me for providing functional code. it allowed me to work on efficiency methods.

The included adafruit_amg88xx.cpp file has a #define AMG88xx_PIXEL_MIRROR true. default is false. flips data order.

I designed a method of sub sampling that does not use floats, it works currently to 32x32 uno or equivilent and 64x64 (atmega or better) with st77xx display and arduino. spi mode,and pixel modes optimizations are more important on lower speed processors, however the more power does mean more resolution in the future, and ability to run several tasks together.
the advantage will come for faster processors in the sub pixel method that does not require floats.

with a parallel display more resolution is possible, also i think arduino and st77xx display can go up to 64x64 sub sampling
with little issue (not bad!) but currently as of 8_5 it does up to 32x32.
some of the optimizations are due to multiple rectangle writes at a time. i have posted an article on hackaday and to there blog as well.
article on hackaday https://hackaday.io/project/160498-arduino-spi-up-to-30-times-more-performance
https://www.thingiverse.com/thing:3017556 color table generator javascript page automates table builds
https://youtu.be/tLwYMQjD0l4 video about project details
//https://www.youtube.com/watch?v=PFyu2S1H0v0 info on color tables (some sound issues. sorry)
//https://www.youtube.com/watch?v=3Lv6EnFy4k4 info about buffering
//https://www.thingiverse.com/thing:3050327 a thermal camera pointer
color generator file should be downloaded and opened in a browser. as of 7/22/18 most features can be changes from web page gui.
for compatability with adafruit code, change #define spi_optimized_st77xx true to false, this allows standard libraries to be used with thermalcam.ino file. if use the code in this folder and includeds st77xx.ccp and st77xx.h spi bursts will be 
faster. 

also  subpixelcoloroptimized should be set to 0 if using default st77xx files, as they dont include 4 and 16 write rectanges at a time mode.

adafruit_amg88xx.ccp also has changes from default. it is only needed if sensor needs to be mirrored: 
#define AMG88xx_PIXEL_MIRROR true set to false

this code works out of box (for fast testing), but the best method is to rewrite the includes to use internal libraries, except for st77xx librarys which run slower than the extra ones i include in main directory.
since display update is the most time intensive task (5000-7000 microseconds per pixel)
(with my st77xx update it goes down to under 100 with same display!) i would imagine this code to scale performance across processor lines. much of improvement is on the bandwidth compression and ability to skip updating pixels where information does not change, until it does. some coding can be made more efficient, but the real issues are the display update speed. anything to reduce display writes increases performance. free memory has improved.

there also is an alternate library for st77xx, this includes a fillRectFast, in place of fillRect. this command includes a method of bursting spi, so while spi data being sent, other data is loaded, this way spi is always busy, even at fastest clock speed of 8mhz.

for 16x16 mode:
memory usage from arduino ide: Sketch uses 14,742 bytes (47%) of program storage space. Maximum is 30,720 bytes. Global variables use 927 bytes (45%) of dynamic memory, leaving 1,121 bytes for local variables. Maximum is 2,048 bytes.
faster more memory efficient way of thermal cam imaging //this library does following

1. moves color translate tables to progmem freeing 512 bytes, or allows alternate color tabes with more values
2. incorporates a buffer for temp reads so we only update areas on screen that change
3. adds bandwidth compression so most critical areas get updated first if cpu limited, or lcd slow
4. implements 3 sample interpolation (3 sample makes sub samples) without need for a scaled buffer
5. we have 8x8 , 16x16, 32x32 (32x32 is experimental) sub sampling real time on arduino using spi display.


a few things:it should function if export thermal_cam.ino to adafruit thermal example, however it will be slower as spi is not properly managed in there library for data bursts. further work needs to be done to amg88xx.h file to reduce noise color table needs to be updated with more resolution detail for higher resolution modes 32x32 interpolation is a work in progress.
to change resolution look for interpolatemode at top of page. change it to 1,2, or 3

to optimize work flow change optimize at top of page 0,1,2. 0 is no optimizations, 1 is cached pixles, 2 is bandwidth buffer prioritization with redundant pixel updates (so screen clears faster)

if optimize is set to 2, then change: #define speedUpCompression 8, 
to a lower or higher number, lower than 8 is not recommended, 
and above 63 does little to nothing since there are only 64 pixels of data being sub sampled


