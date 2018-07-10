# thermalcam
faster more memory efficient way of thermal cam imaging
//this libray does following
1) moves color translate tables to progmem freeing 512 bytes
2) incorporates a buffer for temp reads so we only update areas on screen that change
3) adds bandwidth compression so most critical areas get updated first if cpu limited, or lcd slow
4) impliments a tripolor implimation of interpolation (3 sample makes sub samples 
5) we have 8x8 , 16x16, 32x32 sub sampling real time on arduin using spi display. 
