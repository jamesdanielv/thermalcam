//this version is rewritten by james villeneuve (jamesdanielv). i am thankful to the MIT code included, and adafruit for the immense time savings they game me for providing functional code. it allowed me to work on efficiency methods.
it is working and being tested on arudino ide 1.6.5, and ide 1.8.5. 
if using ide 1.8.5 set spi_optimized_st77xx false, true has display coruption if subpixelcoloroptimized =0 (one pixel )
this does not have an effect on subpixelcoloroptimized if >0 (multipixel writes)


updates will be posted first to here https://github.com/jamesdanielv/thermalcam/tree/updates-(possibly-unstable)
to allow updates while keeping a stable buid.

The included adafruit_amg88xx.cpp file has a #define AMG88xx_PIXEL_MIRROR true. default is false. flips data order.

I designed a method of sub sampling that does not use floats, it works currently to 32x32 uno or equivilent and 64x64 (atmega or better) with st77xx display and arduino. spi mode,and pixel modes optimizations are more important on lower speed processors, however the more power does mean more resolution in the future, and ability to run several tasks together.
the advantage will come for faster processors in the sub pixel method that does not require floats.

currently i have redesigned the sub sample system, and it will undergo further change possbily later. for now buffer needs to be about 1/4 size of final resolution, but all that is really needed is extra bytes interpolate side pixel information. for now it is easier to understand with full buffers. so eventually for example 64x64 will work on arduino with 2k of ram, even though an actual 64x64 buffer with 16 bit wide color values would normally be 8k, since output values are directly inerpolated to new values, 32x32 (2k 16 bit color) is max mem needed for 64x64 buffers. so this is why it is possible with atmega currently, even though buffers are not optimized they are 1/4 size! later on buffers will not be needed except for cache to send multiple writes to display at a time.

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

to enable faster method set spi_optimized_st77xx true
however on 1.8.5 this method only works when sending more than one pixel at a time. ( possibly a bug)
subpixelcoloroptimized 1 or greater it works fine still.

