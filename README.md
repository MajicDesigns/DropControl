# Liquid Drop Photography Controller Sketch

This code accompanies the blog post at https://arduinoplusplus.wordpress.com/2022/02/13/liquid-drop-photography-controller/

This drop controller manages:
* creating up to two liquid drops that will create a splash
* triggering a camera using its IR interface to capture the splash image.

Basic hardware modules/components required are:
* Arduino Uno or other similar controller
* Solenoid controller valve (see below)
* 12V power for the Solenoid valve (eg, a LiPO battery pack)
* MOSFET or simple Transistor to switch the solenoid
* Rotary Encoder with built-in switch
* Infrared Transmitter LED for trigger signal to the camera
* 4-line by 20 character (2004) LCD mode with I2C interface
* Tact switch

If you like and use this software please consider making a small donation using [PayPal](https://paypal.me/MajicDesigns/4USD)
