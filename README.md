read me changes here are simplified. the idea is to update this branch and when all modes tested, and features tested post to main branch. seems easier as people understand files are always changing, but it keeps updates to original files quickly (all at same time), and this can be done as i have time with it known some files may have errors.


also in this branch is a timer feature that sends data to serial port, it slows down display updates, so disable it
long tempt=micros()-timer;
Serial.println(tempt);
around line 1285 or so.....



in this iteration:

1) autorange temp feature, great for looking at heat at a distance, or more color details of current object
2) pixel writes to 64 color rectangles at a time, however overhead of transfer seems to undermine gains so far
3) working on main loops unrolling, and methods to do so nicely. seems up to 760 microseconds in calulations for 64x64

4) added a subsample mode of -1 that ignors display update for timing. this is helpful for loop unroll and optimize calculations


coming soon or already in some form or another

buffer removal 16x16 [x] if checked done (no longer needed. was used to sub sample for 32x32)

buffer removal 32x32 [x] if checked done (now 40 bytes)
buffer removal 64x64 [x] if checked done (now uses about 132 bytes total
resolution increased to 128x128 [ ]

include code for parallel display[ ]



