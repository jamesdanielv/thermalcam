NOTICE. i will be making a new settings code page, to swith settins easily between displays and resolution settings, and sensor types. this will take place in january, and all the zip files will be deleted, and code will once again be stored unzipped, and only 1 version available, with settings changes easily made depending on compatible display, and sensore, and output resolution. again these changes will take place at some point in time after january 2019. 


stable branch is here, but it likely contains less features in favor of stability. https://github.com/jamesdanielv/thermalcam i post versions there after testing them extensively here. one thing i will do by default in stable version is set sketch to use adafruit code library thru a wrapper, this allows standard use of there drivers on many spi and parallel lcd displays. however the wrappper translation uses default spi drivers which are a lot slower than the ones in this branch are. this branch has a set of sketches that uses the wrapper by default as well. it is located here https://github.com/jamesdanielv/thermalcam/blob/updates-(possibly-unstable)/thermal_cam64x64_compatible_adrafruitdisplays.zip

for any sketch files uploaded oct6,2018 or later to use the wrapper just change

#define workAnyAdafruitDisplay false
to #define workAnyAdafruitDisplay true

keeping all develpoment in zip files to allow a central upload that i can link to. when goes to main branch will update files individually. for simplicity only 8x8 and 16x16 can use regular st77xx driver. making several pixel write routines was slowing down dev work. so 32x32 and above uses only advances pixel writes, no longer one at a time (normal driver) 

read me changes here are simplified. the idea is to update this branch and when all modes tested, and features tested post to main branch. seems easier as people understand files are always changing, but it keeps updates to original files quickly (all at same time), and this can be done as i have time with it known some files may have errors.


also in this branch is a timer feature that sends data to serial port, it slows down display updates, so disable it
long tempt=micros()-timer;
Serial.println(tempt);
around line 1285 or so.....



in this iteration:

1) autorange temp feature, great for looking at heat at a distance, or more color details of current object
2) pixel writes to 64 color rectangles at a time, split command up into segments because to much delay in loading and unloading 128 bytes into stack. now it only handles 8 bytes at a time and is 200-300microseconds faster!

3) working on main loops unrolling, and methods to do so nicely. seems up to 760 microseconds in calulations for 64x64

4) added a subsample mode of -1 that ignors display update for timing. this is helpful for loop unroll and optimize calculations
5) enhanced mode, interpits color varations and shadowing to double resolution detail. not just sub sampling.
it also looks for small variations and enhances details. for example a hand with fingers. now fingers show. also reduced gloe effects surounding warm items. it is still there but sharper and more detail.

coming soon or already in some form or another

buffer removal 16x16 [x] if checked done (no longer needed. was used to sub sample for 32x32)

buffer removal 32x32 [x] if checked done (now 40 bytes)

buffer removal 64x64 [x] if checked done (now uses about 132 bytes total for buffer

one thing i realized is that screen updates are quicker than the refresh rate of display. one of the reasons for pixelated effect. for now i switch to updating without interleaving, new method seems to run about 1200 microseconds from start to end of update to lcd. i think i can half this by implimenting a buffer for spi, or using a spi with buffer, this will allow code to run while processing remainder of spi instructions. if im creative about it ill have control over when data sent to spi, so no interupt needed. this is different than the spi bursting in lcd pixel writes. what im thinking of doing is send data to spi, and process most of spi during sub sample processing. with no interupt and direct ctrontrol i can write to spi up to 80 times during execution of code, so only loosing maybe 40-50 microseconds to load spi, to gain 600! (spi runs its own hardware seperate from the mcu.

i have had blockiness issues with 64 but most of these have now been resolved, i have moved on to creating 128x128 resolution and it now sort of works. blockiness issues are back, but can be resolved by averaging adjasent blocks, and averaging outer cell walls of 16x16 pixel blocks. at 16x oversampling this seems to be a good solution.

resolution increased to 128x128 [X]

include code for parallel display[ ]- may include using bitmap function in place of multi pixel mode, although it might not be as optimized.



