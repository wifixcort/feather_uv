// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// released under the GPLv3 license to match the rest of the AdaFruit NeoPixel library

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1
#define PIN            12

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      5

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

int delayval = 500; // delay for half a second

int32_t ledIndex[15] = {0, 3302400, 65280, 16777010, 13158400, 16776960, 6566400, 6566400, 16724530, 13107200, 16711680, 13107400};

void setup() {
  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
//  while(!Serial);
  // End of trinket special code

  Serial.println(pixels.Color(255,255,0));

  pixels.begin(); // This initializes the NeoPixel library.
  pixels.setBrightness(80);
  setLedIndex(1);
}

void loop() {
//
//  // For a set of NeoPixels the first NeoPixel is 0, second is 1, all the way up to the count of pixels minus one.
////pixels.setPixelColor(0, ledIndex[2]); // Moderately bright green color.
//  for(int i=0;i<NUMPIXELS;i++){
//
//    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
//    pixels.setPixelColor(i, ledIndex[i]); // Moderately bright green color.
//
//    pixels.show(); // This sends the updated pixel color to the hardware.
//
//    delay(delayval); // Delay for a period of time (in milliseconds).
//
//  }
//  for(int i=0;i<NUMPIXELS;i++){
//
//    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
//    pixels.setPixelColor(i, ledIndex[5+i]); // Moderately bright green color.
//
//    pixels.show(); // This sends the updated pixel color to the hardware.
//
//    delay(delayval); // Delay for a period of time (in milliseconds).
//  }
//
//  for(int i=0;i<NUMPIXELS;i++){
//
//    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
//    pixels.setPixelColor(i, ledIndex[10]); // Moderately bright green color.
//
//    pixels.show(); // This sends the updated pixel color to the hardware.
//    delay(10); // Delay for a period of time (in milliseconds).
//    
//  }
//  delay(delayval); // Delay for a period of time (in milliseconds).
  
}

void setLedIndex(uint8_t _ledIndex){
  if(_ledIndex == 0){
      for(int i= 0;i < NUMPIXELS;i++){

    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, ledIndex[0]); // Moderately bright green color.

    pixels.show(); // This sends the updated pixel color to the hardware.

    delay(delayval); // Delay for a period of time (in milliseconds).
    }    
  }elseif(_ledIndex == 11){
      for(int i= 0;i < NUMPIXELS;i++){

    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, ledIndex[10]); // Moderately bright green color.

    pixels.show(); // This sends the updated pixel color to the hardware.

    delay(10); // Delay for a period of time (in milliseconds).    
    }      
    delay(delayval); // Delay for a period of time (in milliseconds).
  }else if(_ledIndex <= 5){
      for(int i=1;i < _ledIndex+1;i++){

    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i-1, ledIndex[i]); // Moderately bright green color.

    pixels.show(); // This sends the updated pixel color to the hardware.

    delay(delayval); // Delay for a period of time (in milliseconds).
    }
  }else if(){
      for(int i=0;i < NUMPIXELS;i++){

    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i+1, ledIndex[5+i]); // Moderately bright green color.

    pixels.show(); // This sends the updated pixel color to the hardware.

    delay(delayval); // Delay for a period of time (in milliseconds).
    }
  
  }
}
