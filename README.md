# 3D Rotatey Cube

  A 3D rotating cube animated on a tiny OLED screen like a gyroscope. [[demo]](https://www.youtube.com/watch?v=1GbFJb48saU)

  This project is heavily inspired from the work of [GoblinJuicer](https://www.reddit.com/user/GoblinJuicer)
  who developed and implemented [the idea](http://imgur.com/gallery/fQUAx/new) and posted on [this subreddit](https://www.reddit.com/r/arduino/comments/3vmw1k/ive_been_playing_with_a_gyroscope_and_an_lcd/).
  
  The code is a remix of the original, it's using u8glib and mpu6050 libraries instead of Adafruit and ITG-3200.
```
  Pinout OLED
    VCC => 5V
    GND => GND
    SCL => A5
    SDA => A4
  
  Pinout MPU
    VCC => 5V
    GND => GND
    SCL => A5
    SDA => A4
    AD0 => GND
    INT => D2
```
<p align="center">
<img src="https://raw.githubusercontent.com/tobozo/Rotatey_Cube/master/rotatey-cube.jpg" />
</p>


<h3 align="center">Front</h3>
<p align="center">
<img src="https://raw.githubusercontent.com/tobozo/Rotatey_Cube/master/front.jpg" />
</p>
<h3 align="center">Back</h3>
<p align="center">
<img src="https://raw.githubusercontent.com/tobozo/Rotatey_Cube/master/back.jpg" />
</p>