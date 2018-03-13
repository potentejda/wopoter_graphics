#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>

// For the breakout, you can use any 2 or 3 pins
// These pins will also work for the 1.8" TFT shield
#define TFT_CS     10
#define TFT_RST    9  // you can also connect this to the Arduino reset
                      // in which case, set this #define pin to 0!
#define TFT_DC     8

// Option 1 (recommended): must use the hardware SPI pins
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)
//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

// Option 2: use any pins but a little slower!
#define TFT_SCLK 13   // set these to be whatever pins you like!
#define TFT_MOSI 11   // set these to be whatever pins you like!
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

/*
 * height cm = 3,6
 * height pixels = 132 or 160
 * 
 * width cm = 2,8
 * width pixels = 162 or 128
 * Plane Ax+By+Cz-D=0; 
 * Line  z = ax+by+c 
 *
  */

  int16_t color = ST7735_YELLOW;
  int16_t i = 0;
  int16_t j = 0;

  /** TODO 
   *  
   */
   
float xyz[6] = {1,1,0, 2,2,0};
float * line;
float * points;

float p = 3.1415926;
float degree = p /180;
float theta = 2 * p * 10 / 360;

boolean dec;
float xe = 0;
float ye = 0;
float ze = 5;
float squareXY[] = { -0.5,-0.5,-1.5,  0.5,-0.5,-1.5,  0.5, 0.5,-1.5, -0.5,0.5,-1.5};
float squareXYback[] = { -0.5,-0.5,-2.5,  0.5,-0.5,-2.5,  0.5, 0.5,-2.5, -0.5,0.5,-2.5 };
float squareXZ[] = {0,0,0, 0,0,100, 100,0,100, 100, 0, 0};
float squareYZ[] = {0,0,0, 0,100,0, 0,100,100, 0,100,0};
float axispoint[] = {0, 0, -2};
float * point2d;

float * squareXYp;
float * squareXYbackp;
float * squareXYpp;
float * squareXYbackpp;
float * squareA2draw;
float * squareB2draw;
float * axispointp;
float * axisvectorA;
float * matrixA;
float * matrixB;
float * matrixC;

float pixelsincm = 160 /  3.2;
float cminpixel = 3.2/160;
float centerpixelsincm_x = 128/2*(3.2/160);
float centerpixelsincm_y = 160/2*(3.2/160);

float square2draw[12];

  float screenAndViewer[9];
             
  float screePlane[4];

  float vertexXscalarOp(float x) {
    float x1 = (x + centerpixelsincm_x) * pixelsincm;
    return x1;
  }

  float vertexYscalarOp(float y) {
    float y1 = (y + centerpixelsincm_y) * pixelsincm;
    return y1;
  }

  float * render(float x, float y, float z, float * xy) {
    float deltax = x - xe;
    float deltay = y - ye;
    float deltaz = z - ze;
    float tgalpha = deltax / deltaz;
    float tgbeta = deltay / deltaz;
    xy[0] = ze * tgalpha;
    xy[1] = ze * tgbeta;
    return xy;
  }

  float * squareVertexOp(float * square2draw, float square[12]) {
    int i,j;
    for (i=0; i < 4; i++) {
      j=i*3;
      square2draw[j] = vertexXscalarOp(square[j]);
      square2draw[j+1] = vertexYscalarOp(square[j+1]);
      //square2draw[j+2] = (squareXY[j+2]) * pixelsincm  + centerpixelsincm_z;
    }
    return square2draw;
  };

  float * squareVertexOpRender(float * square2draw, float square[12]) {
    int i,j;
    float point3d[3];
    for (i=0; i < 4; i++) {
      j=i*3;
      point3d[0] = square[j];
      point3d[1] = square[j+1];
      point3d[2] = square[j+2];
      point2d = render(point3d[0], point3d[1], point3d[2], point2d);
      square2draw[j] = vertexXscalarOp(point2d[0]);
      square2draw[j+1] = vertexYscalarOp(point2d[1]);
    }
    return square2draw;
  };

void drawlineony(float * line, float * points, int16_t color) {
   tft.drawLine(vertexXscalarOp(points[0]),vertexYscalarOp(points[1]),vertexXscalarOp(points[2]),vertexYscalarOp(points[3]),color);
}

float * createRotationMatrix(int whichaxis, float * axisvector, float * matrix, float theta) {
  float x,y,z,xx,yy,zz,xy,yz,xz,thetaprim,costheta,sintheta,oneminuscos,xmalsin,ymalsin,zmalsin;
  x = axisvector[0];
  y = axisvector[1];
  z = axisvector[2];
  xx = x*x;
  yy = y*y;
  zz = z*z;
  xy = x * y;
  yz = y * z;
  xz = x * z;
  thetaprim = theta;
  costheta = cos(thetaprim);
  sintheta = sin(thetaprim);
  oneminuscos = 1 - costheta;
  xmalsin = x * sintheta;
  ymalsin = y * sintheta;
  zmalsin = z * sintheta;
             
  if (whichaxis == 0) {
      matrix[0] = xx + costheta * (1 - xx);
      matrix[1] = xy * oneminuscos + zmalsin;
      matrix[2] = xz * oneminuscos - ymalsin;
      matrix[3] = 0;
      matrix[4] = xy * oneminuscos - zmalsin;
      matrix[5] = yy + costheta * (1 - yy);
      matrix[6] = yz * oneminuscos + xmalsin;
      matrix[7] = 0;
      matrix[8] = xz * oneminuscos + ymalsin;
      matrix[9] = yz * oneminuscos - xmalsin;
      matrix[10] = zz + costheta * (1 - zz);
      matrix[11] = 0;
      matrix[12] = 0;
      matrix[13] = 0;
      matrix[14] = 0;
      matrix[15] = 1;
  } else if (whichaxis == 1) {
      matrix[0] = 1;
      matrix[1] = 0;
      matrix[2] = 0;
      matrix[3] = 0;
      matrix[4] = 0;
      matrix[5] = costheta;
      matrix[6] = - sintheta;
      matrix[7] = 0;
      matrix[8] = 0;
      matrix[9] = sintheta;
      matrix[10] = costheta;
      matrix[11] = 0;
      matrix[12] = 0;
      matrix[13] = 0;
      matrix[14] = 0;
      matrix[15] = 1;
  } else if (whichaxis == 2) {
      matrix[0] = costheta;
      matrix[1] = 0;
      matrix[2] = sintheta;
      matrix[3] = 0;
      matrix[4] = 0;
      matrix[5] = 1;
      matrix[6] = 0;
      matrix[7] = 0;
      matrix[8] = - sintheta;
      matrix[9] = 0;
      matrix[10] = costheta;
      matrix[11] = 0;
      matrix[12] = 0;
      matrix[13] = 0;
      matrix[14] = 0;
      matrix[15] = 1;
  } else if (whichaxis == 3) {
      matrix[0] = costheta;
      matrix[1] = - sintheta;
      matrix[2] = 0;
      matrix[3] = 0;
      matrix[4] = sintheta;
      matrix[5] = costheta;
      matrix[6] = 0;
      matrix[7] = 0;
      matrix[8] = 0;
      matrix[9] = 0;
      matrix[10] = 1;
      matrix[11] = 0;
      matrix[12] = 0;
      matrix[13] = 0;
      matrix[14] = 0;
      matrix[15] = 1;
    }
    return matrix;
}

float * matrixByVectorAndMove(float * matrix, float * vertex4D, float * axispoint, float * result) {
  int i,j;
  vertex4D[0] -= axispoint[0];
  vertex4D[1] -= axispoint[1];
  vertex4D[2] -= axispoint[2];
  for (i=0; i<4; i++) {
    j= i * 4;
    result[i] = matrix[j] * vertex4D[0] + matrix[j+1] * vertex4D[1] + matrix[j+2] * vertex4D[2] + matrix[j+3] * vertex4D[3];
  }
  result[0] += axispoint[0];
  result[1] += axispoint[1];
  result[2] += axispoint[2];
  return result;
}  
         
float * rotate_square3D(float * matrix, float * square, float * axispoint ) {
    int i,j;
    float xyz[4];
    float * result;
    result = (float*) malloc(sizeof(float) * 4);
             
    for (i=0; i<4; i++) {
      j = i * 3;
      xyz[0] = square[j];
      xyz[1] = square[j+1];
      xyz[2] = square[j+2];
      xyz[3] = 0;
      result = matrixByVectorAndMove(matrix, xyz, axispoint, result);
      square[j] = result[0];
      square[j+1] = result[1];
      square[j+2] = result[2];
    }
    free(result);
    return square;
  }

float * makeAxisVector(float alpha, float beta, float * axisvector) {
  float x = cos(alpha);
  float y = sin(alpha);
  float z = cos(beta);
  float r = sqrt((x*x) + (y*y) + (z*z));
  axisvector[0] = x/r;
  axisvector[1] = y/r;
  axisvector[2] = z/r;
  return axisvector;
}

void setup() {

  screenAndViewer[0] = -centerpixelsincm_x;
  screenAndViewer[0] = -centerpixelsincm_y;
  screenAndViewer[0] = 0;
  screenAndViewer[0] = centerpixelsincm_x;
  screenAndViewer[0] = -centerpixelsincm_y;
  screenAndViewer[0] = 0;
  screenAndViewer[0] = centerpixelsincm_x;
  screenAndViewer[0] = centerpixelsincm_y;
  screenAndViewer[0] = 0;
  screenAndViewer[0] = 0;
  screenAndViewer[0] = 0;
  screenAndViewer[0] = -40;
   
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.print("Hello! ST7735 TFT Test");

  // Use this initializer if you're using a 1.8" TFT
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab

  // Use this initializer (uncomment) if you're using a 1.44" TFT
  //tft.initR(INITR_144GREENTAB);   // initialize a ST7735S chip, black tab

  Serial.println("Initialized");

  tft.fillScreen(ST7735_BLUE);

  line = (float*) malloc(sizeof(float) * 6);

  points = (float*) malloc(sizeof(float)*9);

  point2d = (float*) malloc(sizeof(float)*2);
  
  squareA2draw = (float*) malloc(sizeof(float) * 12);
  squareB2draw = (float*) malloc(sizeof(float) * 12);
  squareXYp = (float*) malloc(sizeof(float) * 12);
  squareXYbackp = (float*) malloc(sizeof(float) * 12);
  squareXYpp = (float*) malloc(sizeof(float) * 12);
  squareXYbackpp = (float*) malloc(sizeof(float) * 12);
  axispointp = (float*) malloc(sizeof(float) * 3);
  axisvectorA = (float*) malloc(sizeof(float) * 3);
  matrixA = (float*) malloc(sizeof(float) * 16);
  matrixB = (float*) malloc(sizeof(float) * 16);
  matrixC = (float*) malloc(sizeof(float) * 16);

  axisvectorA = makeAxisVector(30*degree, 30*degree, axisvectorA);

  axispointp[0]=axispoint[0];
  axispointp[1]=axispoint[1];
  axispointp[2]=axispoint[2];
  
  matrixA = createRotationMatrix(0, axisvectorA, matrixA, theta);
 
  color = ST7735_YELLOW;
 
  for (i=0; i<12; i++) {
    squareXYp[i] = squareXY[i];
    squareXYbackp[i] = squareXYback[i];
  }

  color = ST7735_RED;

  tft.drawLine(0,0,127,0,color);
  tft.drawLine(127,0,127,159,color);
  tft.drawLine(127,159,0,159,color);
  tft.drawLine(0,159,0,0,color);

  //dec=true;
}

void loop() {
  int16_t color;
  color = ST7735_YELLOW;
  squareA2draw = (float*) squareVertexOpRender(squareA2draw, squareXYp);
    
  squareB2draw = (float*) squareVertexOpRender(squareB2draw, squareXYbackp);

  //draw

  for (i=0; i < 3; i++) { 
    j=i*3;
    tft.drawLine(squareA2draw[j], squareA2draw[j+1], squareA2draw[j+3], squareA2draw[j+4], color);
  }
        
  tft.drawLine(squareA2draw[j+3], squareA2draw[j+4], squareA2draw[0], squareA2draw[1], color);

  for (i=0; i < 3; i++) { 
    j=i*3;
    tft.drawLine(squareB2draw[j], squareB2draw[j+1], squareB2draw[j+3], squareB2draw[j+4], color);
  }
  tft.drawLine(squareB2draw[j+3], squareB2draw[j+4], squareB2draw[0], squareB2draw[1], color);

  for (i=0; i < 3; i++) { 
    j=i*3;
    tft.drawLine(squareA2draw[j], squareA2draw[j+1], squareB2draw[j], squareB2draw[j+1], color);
    tft.drawLine(squareA2draw[j+3], squareA2draw[j+4], squareB2draw[j+3], squareB2draw[j+4], color);
  }

  for (i=0; i<12; i++) {
    squareXYpp[i] = squareXYp[i];
    squareXYbackpp[i] = squareXYbackp[i];
  }

  //delay(10);

  squareXYp = rotate_square3D(matrixA, squareXYp, axispointp);
  squareXYbackp = rotate_square3D(matrixA, squareXYbackp, axispointp);

  //clear
  color = ST7735_BLUE;

  for (i=0; i < 3; i++) { 
    j=i*3;
    tft.drawLine(squareA2draw[j], squareA2draw[j+1], squareA2draw[j+3], squareA2draw[j+4], color);
  }
        
  tft.drawLine(squareA2draw[j+3], squareA2draw[j+4], squareA2draw[0], squareA2draw[1], color);

  for (i=0; i < 3; i++) { 
    j=i*3;
    tft.drawLine(squareB2draw[j], squareB2draw[j+1], squareB2draw[j+3], squareB2draw[j+4], color);
  }
  tft.drawLine(squareB2draw[j+3], squareB2draw[j+4], squareB2draw[0], squareB2draw[1], color);

  for (i=0; i < 3; i++) { 
    j=i*3;
    tft.drawLine(squareA2draw[j], squareA2draw[j+1], squareB2draw[j], squareB2draw[j+1], color);
    tft.drawLine(squareA2draw[j+3], squareA2draw[j+4], squareB2draw[j+3], squareB2draw[j+4], color);
  }
  
  if (dec == true) {
    dec = false;
  }
}
