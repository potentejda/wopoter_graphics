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

float p = 3.1415926;
boolean dec;
float xe = 0;
float ye = 0;
float ze = 5;
float squareXY[] = { -0.5,-0.5,-0.5,  0.5,-0.5,-0.5,  0.5, 0.5,-0.5, -0.5,0.5,-0.5};
float squareXYback[] = { -0.5,-0.5,-1.5,  0.5,-0.5,-1.5,  0.5, 0.5,-1.5, -0.5,0.5,-1.5 };
float squareXZ[] = {0,0,0, 0,0,100, 100,0,100, 100, 0, 0};
float squareYZ[] = {0,0,0, 0,100,0, 0,100,100, 0,100,0};
float * point2d;

  int16_t color = ST7735_YELLOW;
  int16_t i = 0;
  int16_t j = 0;

  /** TODO 
   *  
   */

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

float xyz[6] = {1,1,0, 2,2,0};
float * line;
float * points;

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

  float * squareA2draw;
  float * squareB2draw;

  line = (float*) malloc(sizeof(float) * 6);

  points = (float*) malloc(sizeof(float)*9);

  point2d = (float*) malloc(sizeof(float)*2);
  
  squareA2draw = (float*) malloc(sizeof(float) * 12);
  squareB2draw = (float*) malloc(sizeof(float) * 12);
 
  color = ST7735_YELLOW;
 
  squareA2draw = (float*) squareVertexOpRender(squareA2draw, squareXY);
    
  for (i=0; i < 3; i++) { 
    j=i*3;
    tft.drawLine(squareA2draw[j], squareA2draw[j+1], squareA2draw[j+3], squareA2draw[j+4], color);
  }
        
  tft.drawLine(squareA2draw[j+3], squareA2draw[j+4], squareA2draw[0], squareA2draw[1], color);

  color = ST7735_RED;

  tft.drawLine(0,0,127,0,color);
  tft.drawLine(127,0,127,159,color);
  tft.drawLine(127,159,0,159,color);
  tft.drawLine(0,159,0,0,color);

  color = ST7735_YELLOW;

  squareB2draw = (float*) squareVertexOpRender(squareB2draw, squareXYback);

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

  dec=true;
}

void loop() {
  int16_t color;
  
  if (dec == true) {
    dec = false;
  }
}
