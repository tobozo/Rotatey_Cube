/*
  
  3D Rotatey-Cube is placed under the MIT license
  
  Copyleft (c+) 2016 tobozzo 
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  
  This project is heavily inspired from the work of GoblinJuicer https://www.reddit.com/user/GoblinJuicer 
  who developed and implemented the idea.
  Started from this sub https://www.reddit.com/r/arduino/comments/3vmw1k/ive_been_playing_with_a_gyroscope_and_an_lcd/
  The code is mainly a remix to make it work with esp8266

  Deps:
    I2Cdev library grabbed from: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev
    MPU650 library grabbed from: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
  
  Pinout OLED and MPU (shared)
    VCC => 5V
    GND => GND
    SCL => A5
    SDA => A4
  
  Pinout MPU
    AD0 => GND
    INT => D2

 */


#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"`
#include "OLEDDisplayUi.h"

#define DEBUG false

// Accel and gyro data
int16_t  ax, ay, az, gx, gy, gz;
double MMPI = 32768*M_PI;

long int timeLast = -100, period = 1;
// Overall scale and perspective distance
uint8_t sZ = 4, scale = 16, scaleMax = 16;
// screen center
uint8_t centerX = 64;
uint8_t centerY = 32;

// Initialize cube point arrays
double C1[] = {  1,  1,  1 };
double C2[] = {  1,  1, -1 };
double C3[] = {  1, -1,  1 };
double C4[] = {  1, -1, -1 };
double C5[] = { -1,  1,  1 };
double C6[] = { -1,  1, -1 };
double C7[] = { -1, -1,  1 };
double C8[] = { -1, -1, -1 };

// Initialize cube points coords
uint8_t P1[] = { 0, 0 };
uint8_t P2[] = { 0, 0 };
uint8_t P3[] = { 0, 0 };
uint8_t P4[] = { 0, 0 };
uint8_t P5[] = { 0, 0 };
uint8_t P6[] = { 0, 0 };
uint8_t P7[] = { 0, 0 };
uint8_t P8[] = { 0, 0 };

SSD1306 display(0x3c, D3, D4);
OLEDDisplayUi ui ( &display );
MPU6050 mpu;


void setup() {
  Serial.println("hello");
  // assign default color value
  Serial.begin(115200);  
  display.init();
  display.clear();   // clears the screen and buffer
  display.display();
  display.setColor(WHITE);

  pinMode(A0, INPUT);

  //Wire.begin(D3,D4);

  mpu.initialize();

  if (mpu.dmpInitialize() == 0) {
      Serial.println("MPU initialized!");
      // turn on the DMP, now that it's ready
      mpu.setDMPEnabled(true);
      // supply your own gyro offsets here, scaled for min sensitivity
      mpu.setXGyroOffset(62);
      mpu.setYGyroOffset(100);
      mpu.setZGyroOffset(82);

      
      mpu.setZAccelOffset(-580); // 1688 factory default for my test chip
  }

}


int framecount = 0;
int ispeed = 6000; // interpolation speed
double bouncespeed = 1;
double lastspeedpot;

void loop() {
  display.clear();
  cubeloop();
  Serial.print(scale);
  Serial.print("\t");
  double speedpot = analogRead(A0);
  Serial.print(speedpot);
  Serial.print("\t");
  
  if(abs(speedpot-lastspeedpot)>0.01) {
    lastspeedpot = speedpot;
    bouncespeed = speedpot;
  }
  Serial.println( bouncespeed );
}


void cubeloop() {

  period = (millis()- timeLast);
  timeLast = millis(); 
  // precalc
  double MMPI_TIME = MMPI*period;
  double interpolation = cos(++framecount/MMPI*ispeed);

  scale = ceil( (interpolation * 3) + (scaleMax-3) );
  
  //Read gyro, apply calibration, ignore small values
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // ignore low values (supply uour own values here, based on Serial console output)

  
  if(abs(gx)<20){
    gx = (400 * interpolation)+600;
  }
  if(abs(gy)<15){
    gy = (400 * interpolation)+600;
  }
  if(abs(gz)<30){
    gz = (400 * interpolation)+600;
  }
  
  // scale angles down, rotate
  vectRotXYZ((double)-gy/MMPI_TIME, 1); // X
  vectRotXYZ((double)-gx/MMPI_TIME, 2); // Y
  vectRotXYZ((double)-gz/MMPI_TIME, 3); // Z

  #if DEBUG == true
    Serial.print(scale);
    Serial.print("\t");  
    Serial.print(ax);
    Serial.print("\t");
    Serial.print(ay);
    Serial.print("\t");
    Serial.print(az);
    Serial.print("\t");
    Serial.print(gx);
    Serial.print("\t");
    Serial.print(gy);
    Serial.print("\t");
    Serial.println(gz);
  #endif

  // calculate each point coords
  P1[0] = centerX + scale/(1+C1[2]/sZ)*C1[0]; P1[1] = centerY + scale/(1+C1[2]/sZ)*C1[1];
  P2[0] = centerX + scale/(1+C2[2]/sZ)*C2[0]; P2[1] = centerY + scale/(1+C2[2]/sZ)*C2[1];
  P3[0] = centerX + scale/(1+C3[2]/sZ)*C3[0]; P3[1] = centerY + scale/(1+C3[2]/sZ)*C3[1];
  P4[0] = centerX + scale/(1+C4[2]/sZ)*C4[0]; P4[1] = centerY + scale/(1+C4[2]/sZ)*C4[1];
  P5[0] = centerX + scale/(1+C5[2]/sZ)*C5[0]; P5[1] = centerY + scale/(1+C5[2]/sZ)*C5[1];
  P6[0] = centerX + scale/(1+C6[2]/sZ)*C6[0]; P6[1] = centerY + scale/(1+C6[2]/sZ)*C6[1];
  P7[0] = centerX + scale/(1+C7[2]/sZ)*C7[0]; P7[1] = centerY + scale/(1+C7[2]/sZ)*C7[1];
  P8[0] = centerX + scale/(1+C8[2]/sZ)*C8[0]; P8[1] = centerY + scale/(1+C8[2]/sZ)*C8[1];

  // draw each cube edge
  display.drawLine(P1[0], P1[1], P2[0], P2[1]); //1-2
  display.drawLine(P1[0], P1[1], P3[0], P3[1]); //1-3
  display.drawLine(P1[0], P1[1], P5[0], P5[1]); //1-5
  display.drawLine(P2[0], P2[1], P4[0], P4[1]); //2-4
  display.drawLine(P2[0], P2[1], P6[0], P6[1]); //2-6
  display.drawLine(P3[0], P3[1], P4[0], P4[1]); //3-4
  display.drawLine(P3[0], P3[1], P7[0], P7[1]); //3-7
  display.drawLine(P4[0], P4[1], P8[0], P8[1]); //4-8
  display.drawLine(P5[0], P5[1], P6[0], P6[1]); //5-6
  display.drawLine(P5[0], P5[1], P7[0], P7[1]); //5-7
  display.drawLine(P6[0], P6[1], P8[0], P8[1]); //6-8
  display.drawLine(P7[0], P7[1], P8[0], P8[1]); //7-8

/*
  // draw cross on each cube face
  display.drawLine(P1[0], P1[1], P4[0], P4[1]); //1-4
  display.drawLine(P2[0], P2[1], P3[0], P3[1]); //2-3

  display.drawLine(P1[0], P1[1], P6[0], P6[1]); //1-6
  display.drawLine(P2[0], P2[1], P5[0], P5[1]); //2-5

  display.drawLine(P2[0], P2[1], P8[0], P8[1]); //2-8
  display.drawLine(P6[0], P6[1], P4[0], P4[1]); //6-4

  display.drawLine(P4[0], P4[1], P7[0], P7[1]); //4-7
  display.drawLine(P3[0], P3[1], P8[0], P8[1]); //3-8

  display.drawLine(P1[0], P1[1], P7[0], P7[1]); //1-7
  display.drawLine(P3[0], P3[1], P5[0], P5[1]); //3-5

  display.drawLine(P5[0], P5[1], P8[0], P8[1]); //5-8
  display.drawLine(P7[0], P7[1], P6[0], P6[1]); //7-6

*/
  
  display.display();
}


void vectRotXYZ(double angle, int axe) { 
  int8_t m1; // coords polarity
  uint8_t i1, i2; // coords index
  
  switch(axe) {
    case 1: // X
      i1 = 1; // y
      i2 = 2; // z
      m1 = -1;
    break;
    case 2: // Y
      i1 = 0; // x
      i2 = 2; // z
      m1 = 1;
    break;
    case 3: // Z
      i1 = 0; // x
      i2 = 1; // y
      m1 = 1;
    break;
  }

  double t1 = C1[i1];
  double t2 = C1[i2];
  C1[i1] = t1*cos(angle)+(m1*t2)*sin(angle);
  C1[i2] = (-m1*t1)*sin(angle)+t2*cos(angle);
  
  t1 = C2[i1]; 
  t2 = C2[i2];
  C2[i1] = t1*cos(angle)+(m1*t2)*sin(angle);
  C2[i2] = (-m1*t1)*sin(angle)+t2*cos(angle);

  t1 = C3[i1]; 
  t2 = C3[i2];
  C3[i1] = t1*cos(angle)+(m1*t2)*sin(angle);
  C3[i2] = (-m1*t1)*sin(angle)+t2*cos(angle);

  t1 = C4[i1]; 
  t2 = C4[i2];
  C4[i1] = t1*cos(angle)+(m1*t2)*sin(angle);
  C4[i2] = (-m1*t1)*sin(angle)+t2*cos(angle);

  t1 = C5[i1]; 
  t2 = C5[i2];
  C5[i1] = t1*cos(angle)+(m1*t2)*sin(angle);
  C5[i2] = (-m1*t1)*sin(angle)+t2*cos(angle);

  t1 = C6[i1]; 
  t2 = C6[i2];
  C6[i1] = t1*cos(angle)+(m1*t2)*sin(angle);
  C6[i2] = (-m1*t1)*sin(angle)+t2*cos(angle);

  t1 = C7[i1]; 
  t2 = C7[i2];
  C7[i1] = t1*cos(angle)+(m1*t2)*sin(angle);
  C7[i2] = (-m1*t1)*sin(angle)+t2*cos(angle);

  t1 = C8[i1]; 
  t2 = C8[i2];
  C8[i1] = t1*cos(angle)+(m1*t2)*sin(angle);
  C8[i2] = (-m1*t1)*sin(angle)+t2*cos(angle);

}

