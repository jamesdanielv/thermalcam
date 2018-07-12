//this version is rewritten by james villeneuve. i am thankful to the MIT code included, and adafruit for the immense time savings they game me for providing functional code. it allowed me to work on efficiency methods.

this code works out of box (for fast testing), but the best method is to rewrite the includes to use internal libraries.
since display update is the most time intensive task (5000-7000 microseconds per pixel) i would imagine this code to scale performance across processor lines. much of improvement is on the bandwidth compression and ability to skip updating pixels where information does not change, until it does. some coding can be made more efficient, but the real issues are the display update speed. anything to reduce display writes increases performance. free memory has improved.

memory usage from arduino ide: Sketch uses 14,742 bytes (47%) of program storage space. Maximum is 30,720 bytes. Global variables use 927 bytes (45%) of dynamic memory, leaving 1,121 bytes for local variables. Maximum is 2,048 bytes.
faster more memory efficient way of thermal cam imaging //this library does following

1. moves color translate tables to progmem freeing 512 bytes
2. incorporates a buffer for temp reads so we only update areas on screen that change
3. adds bandwidth compression so most critical areas get updated first if cpu limited, or lcd slow
4. implements 3 sample interpolation (3 sample makes sub samples) without need for a scaled buffer
5. we have 8x8 , 16x16, 32x32 sub sampling real time on arduino using spi display.


a few things: only code changed is in thermal_cam.ino. further work needs to be done to amg88xx.h file to reduce noise color table needs to be updated with more resolution detail for higher resolution modes 32x32 interpolation is a work in progress.
to change resolution look for interpolatemode at top of page. change it to 1,2, or 3
to optimize work flow change optimize at top of page 0,1,2. 0 is no optimizations, 1 is cached pixles, 2 is bandwidth buffer prioritization with redundant pixel updates (so screen clears faster)
if optimize is set to 2, then change: #define speedUpCompression 8, to a lower or higher number, lower than 8 is not recommended, and above 63 does little to nothing since there are only 64 pixels of data being sub sampled
