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
  The code is mainly a remix to make it work with u8glib and mpu6050

  Deps:
    U8glib library grabbed from: https://github.com/olikraus/u8glib
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



#include "U8glib.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define DEBUG true

// Accel and gyro data
int16_t  ax, ay, az, gx, gy, gz;

long int timeLast = -100, period = 1;
int scale = 16, sZ = 4; // Overall scale and perspective distance

// Initialize cube point arrays
double C1[] = {  1,  1,  1 };
double C2[] = {  1,  1, -1 };
double C3[] = {  1, -1,  1 };
double C4[] = {  1, -1, -1 };
double C5[] = { -1,  1,  1 };
double C6[] = { -1,  1, -1 };
double C7[] = { -1, -1,  1 };
double C8[] = { -1, -1, -1 };

U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NO_ACK);	// Display which does not send ACK
MPU6050 mpu;


void setup() {
  
  // assign default color value
  
  if ( u8g.getMode() == U8G_MODE_R3G3B2 ) {
    u8g.setColorIndex(255);     // white
  }
  else if ( u8g.getMode() == U8G_MODE_GRAY2BIT ) {
    u8g.setColorIndex(3);         // max intensity
  }
  else if ( u8g.getMode() == U8G_MODE_BW ) {
    u8g.setColorIndex(1);         // pixel on
  }
  else if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
    u8g.setHiColorByRGB(255,255,255);
  }
  
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  #if DEBUG == true
      // initialize serial communication
      Serial.begin(115200);
  #endif

  mpu.initialize();

  if (mpu.dmpInitialize() == 0) {
      // turn on the DMP, now that it's ready
      mpu.setDMPEnabled(true);
      // supply your own gyro offsets here, scaled for min sensitivity
      mpu.setXGyroOffset(161);
      mpu.setYGyroOffset(48);
      mpu.setZGyroOffset(43);
      mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  }

}



void loop() {
  u8g.firstPage();  
  do {
    cubeloop();
  } 
  while( u8g.nextPage() );  
}



void cubeloop()
{

  period = millis()- timeLast;
  timeLast = millis(); 

  //Read gyro, apply calibration, ignore small values
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  #if DEBUG == true
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

  // ignore low values
  if(abs(gx)<2){
    gx = 0;
  }
  if(abs(gy)<2){
    gy = 0;
  }
  if(abs(gz)<2){
    gz = 0;
  }

  // scale angles down, rotate
  vectRotX((double)gy/(1000*M_PI)/period);
  vectRotY((double)-gx/(1000*M_PI)/period);
  vectRotZ((double)gz/(1000*M_PI)/period);

  // draw each cube edge
  u8g.drawLine(64 + scale/(1+C1[2]/sZ)*C1[0], 32 + scale/(1+C1[2]/sZ)*C1[1], 64 + scale/(1+C2[2]/sZ)*C2[0], 32 + scale/(1+C2[2]/sZ)*C2[1]); //1-2
  u8g.drawLine(64 + scale/(1+C1[2]/sZ)*C1[0], 32 + scale/(1+C1[2]/sZ)*C1[1], 64 + scale/(1+C3[2]/sZ)*C3[0], 32 + scale/(1+C3[2]/sZ)*C3[1]); //1-3
  u8g.drawLine(64 + scale/(1+C1[2]/sZ)*C1[0], 32 + scale/(1+C1[2]/sZ)*C1[1], 64 + scale/(1+C5[2]/sZ)*C5[0], 32 + scale/(1+C5[2]/sZ)*C5[1]); //1-5
  u8g.drawLine(64 + scale/(1+C2[2]/sZ)*C2[0], 32 + scale/(1+C2[2]/sZ)*C2[1], 64 + scale/(1+C4[2]/sZ)*C4[0], 32 + scale/(1+C4[2]/sZ)*C4[1]); //2-4
  u8g.drawLine(64 + scale/(1+C2[2]/sZ)*C2[0], 32 + scale/(1+C2[2]/sZ)*C2[1], 64 + scale/(1+C6[2]/sZ)*C6[0], 32 + scale/(1+C6[2]/sZ)*C6[1]); //2-6
  u8g.drawLine(64 + scale/(1+C3[2]/sZ)*C3[0], 32 + scale/(1+C3[2]/sZ)*C3[1], 64 + scale/(1+C4[2]/sZ)*C4[0], 32 + scale/(1+C4[2]/sZ)*C4[1]); //3-4
  u8g.drawLine(64 + scale/(1+C3[2]/sZ)*C3[0], 32 + scale/(1+C3[2]/sZ)*C3[1], 64 + scale/(1+C7[2]/sZ)*C7[0], 32 + scale/(1+C7[2]/sZ)*C7[1]); //3-7
  u8g.drawLine(64 + scale/(1+C4[2]/sZ)*C4[0], 32 + scale/(1+C4[2]/sZ)*C4[1], 64 + scale/(1+C8[2]/sZ)*C8[0], 32 + scale/(1+C8[2]/sZ)*C8[1]); //4-8
  u8g.drawLine(64 + scale/(1+C5[2]/sZ)*C5[0], 32 + scale/(1+C5[2]/sZ)*C5[1], 64 + scale/(1+C6[2]/sZ)*C6[0], 32 + scale/(1+C6[2]/sZ)*C6[1]); //5-6
  u8g.drawLine(64 + scale/(1+C5[2]/sZ)*C5[0], 32 + scale/(1+C5[2]/sZ)*C5[1], 64 + scale/(1+C7[2]/sZ)*C7[0], 32 + scale/(1+C7[2]/sZ)*C7[1]); //5-7
  u8g.drawLine(64 + scale/(1+C6[2]/sZ)*C6[0], 32 + scale/(1+C6[2]/sZ)*C6[1], 64 + scale/(1+C8[2]/sZ)*C8[0], 32 + scale/(1+C8[2]/sZ)*C8[1]); //6-8
  u8g.drawLine(64 + scale/(1+C7[2]/sZ)*C7[0], 32 + scale/(1+C7[2]/sZ)*C7[1], 64 + scale/(1+C8[2]/sZ)*C8[0], 32 + scale/(1+C8[2]/sZ)*C8[1]); //7-8

}


void vectRotX(double angle)  //Rotates all the points around the x axis.
{
  double yt = C1[1]; 
  double zt = C1[2];
  C1[1] = yt*cos(angle)-zt*sin(angle);
  C1[2] = yt*sin(angle)+zt*cos(angle);

  yt = C2[1]; 
  zt = C2[2];
  C2[1] = yt*cos(angle)-zt*sin(angle);
  C2[2] = yt*sin(angle)+zt*cos(angle);

  yt = C3[1]; 
  zt = C3[2];
  C3[1] = yt*cos(angle)-zt*sin(angle);
  C3[2] = yt*sin(angle)+zt*cos(angle);

  yt = C4[1]; 
  zt = C4[2];
  C4[1] = yt*cos(angle)-zt*sin(angle);
  C4[2] = yt*sin(angle)+zt*cos(angle);

  yt = C5[1]; 
  zt = C5[2];
  C5[1] = yt*cos(angle)-zt*sin(angle);
  C5[2] = yt*sin(angle)+zt*cos(angle);

  yt = C6[1]; 
  zt = C6[2];
  C6[1] = yt*cos(angle)-zt*sin(angle);
  C6[2] = yt*sin(angle)+zt*cos(angle);

  yt = C7[1]; 
  zt = C7[2];
  C7[1] = yt*cos(angle)-zt*sin(angle);
  C7[2] = yt*sin(angle)+zt*cos(angle);

  yt = C8[1]; 
  zt = C8[2];
  C8[1] = yt*cos(angle)-zt*sin(angle);
  C8[2] = yt*sin(angle)+zt*cos(angle);
}

void vectRotY(double angle)  //Rotates all the points around the y axis.
{
  double xt = C1[0]; 
  double zt = C1[2];
  C1[0] = xt*cos(angle)+zt*sin(angle);
  C1[2] = -xt*sin(angle)+zt*cos(angle);

  xt = C2[0]; 
  zt = C2[2];
  C2[0] = xt*cos(angle)+zt*sin(angle);
  C2[2] = -xt*sin(angle)+zt*cos(angle);

  xt = C3[0]; 
  zt = C3[2];
  C3[0] = xt*cos(angle)+zt*sin(angle);
  C3[2] = -xt*sin(angle)+zt*cos(angle);

  xt = C4[0]; 
  zt = C4[2];
  C4[0] = xt*cos(angle)+zt*sin(angle);
  C4[2] = -xt*sin(angle)+zt*cos(angle);

  xt = C5[0]; 
  zt = C5[2];
  C5[0] = xt*cos(angle)+zt*sin(angle);
  C5[2] = -xt*sin(angle)+zt*cos(angle);

  xt = C6[0]; 
  zt = C6[2];
  C6[0] = xt*cos(angle)+zt*sin(angle);
  C6[2] = -xt*sin(angle)+zt*cos(angle);

  xt = C7[0]; 
  zt = C7[2];
  C7[0] = xt*cos(angle)+zt*sin(angle);
  C7[2] = -xt*sin(angle)+zt*cos(angle);

  xt = C8[0]; 
  zt = C8[2];
  C8[0] = xt*cos(angle)+zt*sin(angle);
  C8[2] = -xt*sin(angle)+zt*cos(angle);

}

void vectRotZ(double angle)  //Rotates all the points around the z axis.
{
  double xt = C1[0]; 
  double yt = C1[1];
  C1[0] = xt*cos(angle)+yt*sin(angle);
  C1[1] = -xt*sin(angle)+yt*cos(angle);

  xt = C2[0]; 
  yt = C2[1];
  C2[0] = xt*cos(angle)+yt*sin(angle);
  C2[1] = -xt*sin(angle)+yt*cos(angle);

  xt = C3[0]; 
  yt = C3[1];
  C3[0] = xt*cos(angle)+yt*sin(angle);
  C3[1] = -xt*sin(angle)+yt*cos(angle);

  xt = C4[0]; 
  yt = C4[1];
  C4[0] = xt*cos(angle)+yt*sin(angle);
  C4[1] = -xt*sin(angle)+yt*cos(angle);

  xt = C5[0]; 
  yt = C5[1];
  C5[0] = xt*cos(angle)+yt*sin(angle);
  C5[1] = -xt*sin(angle)+yt*cos(angle);

  xt = C6[0]; 
  yt = C6[1];
  C6[0] = xt*cos(angle)+yt*sin(angle);
  C6[1] = -xt*sin(angle)+yt*cos(angle);

  xt = C7[0]; 
  yt = C7[1];
  C7[0] = xt*cos(angle)+yt*sin(angle);
  C7[1] = -xt*sin(angle)+yt*cos(angle);

  xt = C8[0]; 
  yt = C8[1];
  C8[0] = xt*cos(angle)+yt*sin(angle);
  C8[1] = -xt*sin(angle)+yt*cos(angle);

}

