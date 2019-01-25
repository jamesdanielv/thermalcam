/*
*
uint16_t x1= sidepixelColor;//this shows how we solve for other pixels with partial information. code should be reduced from compiler
uint16_t x2= (sidepixelColor+diagnalPixelColor)/2;
uint16_t x3= topOrBottomPixelColor;
uint16_t x4= (topOrBottomPixelColor+diagnalPixelColor)/2;
uint16_t x5= diagnalPixelColor;



//upper //left //here is where sub sample is solved
pixelsubsampledata[1]=(pixelsubsampledata[0]+pixelsubsampledata[2] )/2;
pixelsubsampledata[8]=(pixelsubsampledata[0]+pixelsubsampledata[16] )/2;
pixelsubsampledata[9]=(pixelsubsampledata[0]+pixelsubsampledata[18] )/2;

pixelsubsampledata[3]=(pixelsubsampledata[2]+pixelsubsampledata[4] )/2;
pixelsubsampledata[10]=(pixelsubsampledata[2]+pixelsubsampledata[18] )/2;
pixelsubsampledata[11]=(pixelsubsampledata[2]+pixelsubsampledata[20] )/2;

pixelsubsampledata[17]=(pixelsubsampledata[16]+pixelsubsampledata[18] )/2;
pixelsubsampledata[24]=(pixelsubsampledata[16]+pixelsubsampledata[32] )/2;
pixelsubsampledata[25]=(pixelsubsampledata[16]+pixelsubsampledata[34] )/2;

pixelsubsampledata[19]=(pixelsubsampledata[18]+pixelsubsampledata[20] )/2;
pixelsubsampledata[26]=(pixelsubsampledata[18]+pixelsubsampledata[34])/2;
pixelsubsampledata[27]=(pixelsubsampledata[18]+pixelsubsampledata[28] )/2;
//lower
pixelsubsampledata[33]=(pixelsubsampledata[32]+pixelsubsampledata[34] )/2;
pixelsubsampledata[40]=(pixelsubsampledata[32]+pixelsubsampledata[48] )/2;
pixelsubsampledata[41]=(pixelsubsampledata[32]+pixelsubsampledata[50] )/2;

pixelsubsampledata[35]=(pixelsubsampledata[34]+pixelsubsampledata[36] )/2;
pixelsubsampledata[42]=(pixelsubsampledata[34]+pixelsubsampledata[50] )/2;
pixelsubsampledata[43]=(pixelsubsampledata[34]+pixelsubsampledata[52] )/2;

pixelsubsampledata[49]=(pixelsubsampledata[48]+pixelsubsampledata[50] )/2;
pixelsubsampledata[56]=(pixelsubsampledata[48]+x5 )/2;
pixelsubsampledata[57]=(pixelsubsampledata[48]+x6 )/2;

pixelsubsampledata[51]=(pixelsubsampledata[50]+pixelsubsampledata[52] )/2;
pixelsubsampledata[58]=(pixelsubsampledata[50]+x6 )/2;
pixelsubsampledata[59]=(pixelsubsampledata[50]+x7 )/2;
//upper //right //
pixelsubsampledata[5]=(pixelsubsampledata[4]+pixelsubsampledata[6] )/2;
pixelsubsampledata[12]=(pixelsubsampledata[4]+pixelsubsampledata[20] )/2;
pixelsubsampledata[13]=(pixelsubsampledata[4]+pixelsubsampledata[22] )/2;

pixelsubsampledata[7]=(pixelsubsampledata[6] +x1)/2;
pixelsubsampledata[14]=(pixelsubsampledata[6]+pixelsubsampledata[22] )/2;                              
pixelsubsampledata[15]=(pixelsubsampledata[6]+x2 )/2;

pixelsubsampledata[21]=(pixelsubsampledata[20]+pixelsubsampledata[22] )/2;
pixelsubsampledata[28]=(pixelsubsampledata[20]+pixelsubsampledata[36] )/2;
pixelsubsampledata[29]=(pixelsubsampledata[20]+pixelsubsampledata[38] )/2;

pixelsubsampledata[23]=(pixelsubsampledata[22]+x2 )/2;
pixelsubsampledata[30]=(pixelsubsampledata[22]+pixelsubsampledata[38] )/2;
pixelsubsampledata[31]=(pixelsubsampledata[22]+x3 )/2;
//lower
pixelsubsampledata[37]=(pixelsubsampledata[36]+pixelsubsampledata[38] )/2;
pixelsubsampledata[44]=(pixelsubsampledata[36]+pixelsubsampledata[52] )/2;
pixelsubsampledata[45]=(pixelsubsampledata[36]+pixelsubsampledata[54] )/2;

pixelsubsampledata[39]=(pixelsubsampledata[38]+x3 )/2;
pixelsubsampledata[46]=(pixelsubsampledata[38]+pixelsubsampledata[54] )/2;
pixelsubsampledata[47]=(pixelsubsampledata[38]+x4 )/2;

pixelsubsampledata[53]=(pixelsubsampledata[52]+pixelsubsampledata[54] )/2;
pixelsubsampledata[60]=(pixelsubsampledata[52]+x7 )/2;
pixelsubsampledata[61]=(pixelsubsampledata[52]+x8 )/2;

pixelsubsampledata[55]=(pixelsubsampledata[54]+x4 )/2;
pixelsubsampledata[62]=(pixelsubsampledata[54]+x8 )/2;
pixelsubsampledata[63]=(pixelsubsampledata[54]+x9 )/2;








* pixelsubsampledata[0]=mainPixel;pixelsubsampledata[2]=sidePixel;
pixelsubsampledata[4]=(mainPixel+topbottomPixel)/2;  //re
pixelsubsampledata[1]=(mainPixel+sidePixel)/2;    //re     
pixelsubsampledata[6]=(sidePixel+diagnolPixel)/2; 


                                                       
pixelsubsampledata[2]=sidePixel;
pixelsubsampledata[3]=(sidePixel+(sidePixel+sidepixelColor)/2)/2;
pixelsubsampledata[4]=(mainPixel+topbottomPixel)/2;  //re
pixelsubsampledata[5]=(mainPixel+diagnolPixel)/2;  //re
pixelsubsampledata[6]=(sidePixel+diagnolPixel)/2; 
pixelsubsampledata[7]=(sidePixel+(sidepixelColor+diagnalPixelColor)/2)/2;
pixelsubsampledata[8]=(topbottomPixel);
pixelsubsampledata[9]=(topbottomPixel+diagnolPixel)/2;                                                              
pixelsubsampledata[10]=(diagnolPixel); 
pixelsubsampledata[11]=(diagnolPixel+(sidepixelColor+diagnalPixelColor)/2)/2;
pixelsubsampledata[12]=(topbottomPixel+topOrBottomPixelColor/2+topbottomPixel/2)/2;
pixelsubsampledata[13]=(topbottomPixel+(topOrBottomPixelColor+diagnalPixelColor)/2)/2;   
pixelsubsampledata[14]=(diagnolPixel+(topOrBottomPixelColor+diagnalPixelColor)/2)/2;
pixelsubsampledata[15]=(diagnolPixel+diagnalPixelColor)/2;

8x8 for 64 loop. too complex, need to simplify it down.


uint16_t CreateSubPixels8x8(uint16_t pixelColor,uint16_t  sidepixelColor,uint16_t topOrBottomPixelColor,uint16_t diagnalPixelColor,bool EnhancedMode){
//look at 4x4 and 2x2 for explination and diagram examples. this one is going to be complicated, but works on same idea x,s,t,d.
mainPixel=pixelColor;// just shown for sanity.compiler should remove it.
sidePixel= (sidepixelColor+pixelColor)/2;//thes are effectively divided. numbers are uint16_t so no need to worry about carry messing it up.
topbottomPixel=(pixelColor+topOrBottomPixelColor)/2;//ditto 
diagnolPixel=(pixelColor+diagnalPixelColor)/2;//ditto

//these are treated as main pixels. 16 solved samples from 4x4 (except solved here in this loop as other is prob inactive 
pixelsubsampledata[0]=mainPixel;
pixelsubsampledata[2]=(mainPixel+sidePixel)/2;
pixelsubsampledata[4]=sidePixel;
pixelsubsampledata[6]=(sidePixel+(sidePixel+sidepixelColor)/2)/2;
pixelsubsampledata[16]=(mainPixel+topbottomPixel)/2;
pixelsubsampledata[18]=(mainPixel+diagnolPixel)/2;
pixelsubsampledata[20]=(sidePixel+diagnolPixel)/2; 
pixelsubsampledata[22]=(sidePixel+sidePixel/2+(sidepixelColor+diagnalPixelColor)/4)/2;//  4 pixel locations to be sub sampled
pixelsubsampledata[32]=(topbottomPixel);
pixelsubsampledata[34]=(topbottomPixel+diagnolPixel)/2;
pixelsubsampledata[36]=(diagnolPixel); 
pixelsubsampledata[38]=(diagnolPixel+diagnolPixel/2+(sidepixelColor+diagnalPixelColor)/4)/2;//4 pixel locations to be sub sampled
pixelsubsampledata[48]=(topbottomPixel+topOrBottomPixelColor/2+topbottomPixel/2)/2;
pixelsubsampledata[50]=(topbottomPixel+topbottomPixel/2+(topOrBottomPixelColor+diagnalPixelColor)/4)/2;   
pixelsubsampledata[52]=(diagnolPixel+diagnolPixel/2+(topOrBottomPixelColor+diagnalPixelColor)/4)/2;
pixelsubsampledata[54]=(diagnolPixel+diagnolPixel/2+diagnalPixelColor/2)/2;//4 pixel locations to be sub sampled
#if enhancedDetail == true
if (EnhancedMode){//we only enhance at 32x32 and we want to reuse this loop maybe later on, so we have an option to enable and disable it
//please dont change. this was done with experimentation so changes would not look pixelated
#define amountc -15 //enhance around edges
#define amount 20 //this is amount of enhancement if color curve in a negative
#define amounti 9 //this is amount of enhancement if color curve in posative
#define pop 3 //if individual pixel different we pop it (without looking at color curve)
uint16_t center0=(pixelsubsampledata[0]+pixelsubsampledata[1]+pixelsubsampledata[4]+pixelsubsampledata[5])/4;
uint16_t center1=(pixelsubsampledata[2]+pixelsubsampledata[3]+pixelsubsampledata[6]+pixelsubsampledata[7])/4;
uint16_t center2=(pixelsubsampledata[8]+pixelsubsampledata[9]+pixelsubsampledata[12]+pixelsubsampledata[13])/4;
uint16_t center3=(pixelsubsampledata[10]+pixelsubsampledata[11]+pixelsubsampledata[14]+pixelsubsampledata[15])/4;
if (center0>pixelsubsampledata[0]) {pixelsubsampledata[0]=pixelsubsampledata[0]-amount;}else{pixelsubsampledata[0]=pixelsubsampledata[0]+amounti;};
if (center0>pixelsubsampledata[1]) {pixelsubsampledata[1]=pixelsubsampledata[1]-amount;}else{pixelsubsampledata[1]=pixelsubsampledata[1]+amounti;};
if (center0>pixelsubsampledata[4]) {pixelsubsampledata[4]=pixelsubsampledata[4]-amount;}else{pixelsubsampledata[4]=pixelsubsampledata[4]+amounti;};
if (center0>pixelsubsampledata[5]) {pixelsubsampledata[5]=pixelsubsampledata[5]-amount;}else{pixelsubsampledata[5]=pixelsubsampledata[5]+amounti;};

if (center1>pixelsubsampledata[2]) {pixelsubsampledata[2]=pixelsubsampledata[2]-amount;}else{pixelsubsampledata[2]=pixelsubsampledata[2]+amounti;};
if (center1>pixelsubsampledata[3]) {pixelsubsampledata[3]=pixelsubsampledata[3]-amount;}else{pixelsubsampledata[3]=pixelsubsampledata[3]+amounti;};
if (center1>pixelsubsampledata[6]) {pixelsubsampledata[6]=pixelsubsampledata[6]-amount;}else{pixelsubsampledata[6]=pixelsubsampledata[6]+amounti;};
if (center1>pixelsubsampledata[7]) {pixelsubsampledata[7]=pixelsubsampledata[7]-amount;}else{pixelsubsampledata[7]=pixelsubsampledata[7]+amounti;};

if (center2>pixelsubsampledata[8]) {pixelsubsampledata[8]=pixelsubsampledata[8]-amount;}else{pixelsubsampledata[8]=pixelsubsampledata[8]+amounti;};
if (center2>pixelsubsampledata[9]) {pixelsubsampledata[9]=pixelsubsampledata[9]-amount;}else{pixelsubsampledata[9]=pixelsubsampledata[9]+amounti;};
if (center2>pixelsubsampledata[12]) {pixelsubsampledata[12]=pixelsubsampledata[12]-amount;}else{pixelsubsampledata[12]=pixelsubsampledata[12]+amounti;};
if (center2>pixelsubsampledata[13]) {pixelsubsampledata[13]=pixelsubsampledata[13]-amount;}else{pixelsubsampledata[13]=pixelsubsampledata[13]+amounti;};

if (center3>pixelsubsampledata[10]) {pixelsubsampledata[10]=pixelsubsampledata[10]-amount;}else{pixelsubsampledata[10]=pixelsubsampledata[10]+amounti;};
if (center3>pixelsubsampledata[11]) {pixelsubsampledata[11]=pixelsubsampledata[11]-amount;}else{pixelsubsampledata[11]=pixelsubsampledata[11]+amounti;};
if (center3>pixelsubsampledata[14]) {pixelsubsampledata[14]=pixelsubsampledata[14]-amount;}else{pixelsubsampledata[14]=pixelsubsampledata[14]+amounti;};
if (center3>pixelsubsampledata[15]) {pixelsubsampledata[15]=pixelsubsampledata[15]-amount;}else{pixelsubsampledata[15]=pixelsubsampledata[15]+amounti;};
pixelsubsampledata[5]-=amountc/2;pixelsubsampledata[6]-=amountc/2;pixelsubsampledata[9]-=amountc/2;pixelsubsampledata[10]-=amountc/2;
pixelsubsampledata[2]-=amountc;pixelsubsampledata[3]-=amountc;pixelsubsampledata[4]-=amountc;pixelsubsampledata[5]-=amountc;
pixelsubsampledata[9]-=amountc;pixelsubsampledata[11]-=amountc;pixelsubsampledata[13]-=amountc;pixelsubsampledata[14]-=amountc;
//pixel 5
if (pixelsubsampledata[5]>pixelsubsampledata[4]){pixelsubsampledata[5]+=pop;}if (pixelsubsampledata[5]>pixelsubsampledata[6]){pixelsubsampledata[5]+=pop;}
if (pixelsubsampledata[5]>pixelsubsampledata[1]){pixelsubsampledata[5]+=pop;}if (pixelsubsampledata[5]>pixelsubsampledata[9]){pixelsubsampledata[5]+=pop;}
//pixel 6
if (pixelsubsampledata[6]>pixelsubsampledata[5]){pixelsubsampledata[6]+=pop;}if (pixelsubsampledata[6]>pixelsubsampledata[7]){pixelsubsampledata[6]+=pop;}
if (pixelsubsampledata[6]>pixelsubsampledata[2]){pixelsubsampledata[6]+=pop;}if (pixelsubsampledata[6]>pixelsubsampledata[10]){pixelsubsampledata[6]+=pop;}
//pixel 9
if (pixelsubsampledata[9]>pixelsubsampledata[8]){pixelsubsampledata[9]+=pop;}if (pixelsubsampledata[9]>pixelsubsampledata[10]){pixelsubsampledata[9]+=pop;}
if (pixelsubsampledata[9]>pixelsubsampledata[6]){pixelsubsampledata[9]+=pop;}if (pixelsubsampledata[9]>pixelsubsampledata[14]){pixelsubsampledata[9]+=pop;}
//pixel 10
if (pixelsubsampledata[10]>pixelsubsampledata[9]){pixelsubsampledata[10]+=pop;}if (pixelsubsampledata[10]>pixelsubsampledata[11]){pixelsubsampledata[10]+=pop;}
if (pixelsubsampledata[10]>pixelsubsampledata[6]){pixelsubsampledata[10]+=pop;}if (pixelsubsampledata[10]>pixelsubsampledata[14]){pixelsubsampledata[10]+=pop;}

}
#endif



//upper //left //here is where sub sample is solved
pixelsubsampledata[1]=(pixelsubsampledata[0]+pixelsubsampledata[2])/2;   //[0][1] -->>[0][1][2 ][ 3][4 ][ 5][ 6][ 7] solved upper
pixelsubsampledata[8]=(pixelsubsampledata[0]+pixelsubsampledata[16])/2;  //[8][9] -->>[8][9][10][11][12][13][14][15] left sub sample
pixelsubsampledata[9]=(pixelsubsampledata[0]+pixelsubsampledata[18])/2;
pixelsubsampledata[3]=(pixelsubsampledata[2]+pixelsubsampledata[4])/2;
pixelsubsampledata[10]=(pixelsubsampledata[2]+pixelsubsampledata[18])/2;  //[ 2][ 3]
pixelsubsampledata[11]=(pixelsubsampledata[2]+pixelsubsampledata[20])/2;  //[10][11]
pixelsubsampledata[17]=(pixelsubsampledata[16]+pixelsubsampledata[18])/2;
pixelsubsampledata[24]=(pixelsubsampledata[16]+pixelsubsampledata[34])/2;  //[16][17]-->>[16][17][18][19][20][21][22][23] solved upper
pixelsubsampledata[25]=(pixelsubsampledata[16]+pixelsubsampledata[36])/2;  //[24][25]-->>[24][25][26][27][28][29][30][31] left sub sample
pixelsubsampledata[19]=(pixelsubsampledata[18]+pixelsubsampledata[20])/2;
pixelsubsampledata[26]=(pixelsubsampledata[18]+pixelsubsampledata[34])/2;  //[18][19]
pixelsubsampledata[27]=(pixelsubsampledata[18]+pixelsubsampledata[36])/2;  //[26][27]
//lower
pixelsubsampledata[33]=(pixelsubsampledata[32]+pixelsubsampledata[34])/2;   //[32][33] -->>[32][33][34][35][36][37][38][39] solved upper
pixelsubsampledata[40]=(pixelsubsampledata[32]+pixelsubsampledata[48])/2;  //[40][41]  -->>[40][41][42][43][44][45][46][47] left sub sample
pixelsubsampledata[41]=(pixelsubsampledata[32]+pixelsubsampledata[50])/2;
pixelsubsampledata[35]=(pixelsubsampledata[34]+pixelsubsampledata[36])/2;
pixelsubsampledata[42]=(pixelsubsampledata[34]+pixelsubsampledata[50])/2;  //[34][35]
pixelsubsampledata[43]=(pixelsubsampledata[34]+pixelsubsampledata[52])/2;  //[42][43]
pixelsubsampledata[49]=(pixelsubsampledata[48]+pixelsubsampledata[50])/2;
pixelsubsampledata[56]=(pixelsubsampledata[48]+topOrBottomPixelColor)/2;                                                         //[48][49]  -->>[48][49][50][51][52][53][54][55] solved upper
pixelsubsampledata[57]=(pixelsubsampledata[48]+pixelsubsampledata[48]/2+(topOrBottomPixelColor+(topOrBottomPixelColor+diagnalPixelColor)/2)/4)/2;         //[56][57]  -->>[56][57][58][59][60][61][62][63] left sub sample
pixelsubsampledata[51]=(pixelsubsampledata[50]+pixelsubsampledata[52])/2;
pixelsubsampledata[58]=(pixelsubsampledata[50]+pixelsubsampledata[50]/2+(topOrBottomPixelColor+(topOrBottomPixelColor+diagnalPixelColor)/2)/4)/2; //[50][51]
pixelsubsampledata[59]=(pixelsubsampledata[50]+pixelsubsampledata[50]/2+(topOrBottomPixelColor+diagnalPixelColor)/4)/2;                           //[58][59]
//upper //right //
pixelsubsampledata[5]=(pixelsubsampledata[4]+pixelsubsampledata[6])/2;   //[ 4][ 5] -->>[0][1][2 ][ 3][4 ][ 5][ 6][ 7] solved upper
pixelsubsampledata[12]=(pixelsubsampledata[4]+pixelsubsampledata[20])/2;  //[12][13] -->>[8][9][10][11][12][13][14][15] left sub sample
pixelsubsampledata[13]=(pixelsubsampledata[4]+pixelsubsampledata[22])/2;
pixelsubsampledata[7]=(pixelsubsampledata[6]+sidepixelColor)/2;
pixelsubsampledata[14]=(pixelsubsampledata[6]+pixelsubsampledata[22])/2;                                   //[ 6][ 7]
pixelsubsampledata[15]=(pixelsubsampledata[6]+pixelsubsampledata[6]/2+ ((sidepixelColor+diagnalPixelColor)/2+sidepixelColor)/4)/2;  //[14][15]
pixelsubsampledata[21]=(pixelsubsampledata[20]+pixelsubsampledata[22])/2;
pixelsubsampledata[28]=(pixelsubsampledata[20]+pixelsubsampledata[36])/2;  //[20][21]-->>[16][17][18][19][20][21][22][23] solved upper
pixelsubsampledata[29]=(pixelsubsampledata[20]+pixelsubsampledata[38])/2;  //[28][29]-->>[24][25][26][27][28][29][30][31] left sub sample
pixelsubsampledata[23]=(pixelsubsampledata[22]+pixelsubsampledata[22]/2+((sidepixelColor+diagnalPixelColor)/2+sidepixelColor)/4)/2;  //[22][23] -->>[32][33][34][35][36][37][38][39] solved upper
pixelsubsampledata[30]=(pixelsubsampledata[22]+pixelsubsampledata[38])/2;                                  //[30][31]  -->>[40][41][42][43][44][45][46][47] left sub sample
pixelsubsampledata[31]=(pixelsubsampledata[22]+pixelsubsampledata[22]/2+(diagnalPixelColor+ sidepixelColor)/4)/2;
//lower
pixelsubsampledata[37]=(pixelsubsampledata[36]+pixelsubsampledata[38])/2;
pixelsubsampledata[44]=(pixelsubsampledata[36]+pixelsubsampledata[52])/2;
pixelsubsampledata[45]=(pixelsubsampledata[36]+pixelsubsampledata[54])/2;                 //[36][37]
pixelsubsampledata[39]=(pixelsubsampledata[38]+pixelsubsampledata[38]/2+(diagnalPixelColor+ sidepixelColor)/4)/2;  //[44][45]
pixelsubsampledata[46]=(pixelsubsampledata[38]+pixelsubsampledata[54])/2; 
pixelsubsampledata[47]=(pixelsubsampledata[38]+pixelsubsampledata[38]/2+ ((diagnalPixelColor+ sidepixelColor)/2+diagnalPixelColor)/4)/2; //[38][39]  -->>[48][49][50][51][52][53][54][55] solved upper
pixelsubsampledata[53]=(pixelsubsampledata[52]+pixelsubsampledata[54])/2;                                       //[46][47]  -->>[56][57][58][59][60][61][62][63] left sub sample
pixelsubsampledata[60]=(pixelsubsampledata[52]+pixelsubsampledata[52]/2+(topOrBottomPixelColor+diagnalPixelColor)/4)/2;
pixelsubsampledata[61]=(pixelsubsampledata[52]+pixelsubsampledata[52]/2+(topOrBottomPixelColor+(topOrBottomPixelColor+diagnalPixelColor)/2)/4)/2; //[52][53]
pixelsubsampledata[55]=(pixelsubsampledata[54]+pixelsubsampledata[54]/2+((diagnalPixelColor+sidepixelColor)/2+diagnalPixelColor)/4)/2;            //[60][61]
pixelsubsampledata[62]=(pixelsubsampledata[54]+pixelsubsampledata[54]/2+((topOrBottomPixelColor+diagnalPixelColor)/2+diagnalPixelColor)/4)/2;     //[54][55]
pixelsubsampledata[63]=(pixelsubsampledata[54]+diagnalPixelColor)/2;                                                     //[62][63]
}


 */

