
#include "FastLED.h"
#define NUM_LEDS 80
CRGB leds[NUM_LEDS];
void setup() { 
  pinMode(11, OUTPUT);
  FastLED.addLeds<WS2801, RGB>(leds, NUM_LEDS); }
void loop() {
    digitalWrite(11, HIGH);
  
    //int colors[6] = {CRGB::Red, CRGB::Orange, CRGB::Yellow, CRGB::Green, CRGB::Blue, CRGB::Purple};
     CRGB hi[7] = {CRGB::Red, CRGB::Orange, CRGB::Yellow, CRGB::Green, CRGB::Blue, CRGB::Purple, CRGB::Red};

    //}

     for(int j = 0; j < 256; j++) {
   for(int i = 0; i < NUM_LEDS; i++) {
      leds[i] = Scroll((i * 256 / NUM_LEDS + j) % 256);      
    } 

    FastLED.show();
    delay(100);    
  } 

}

CRGB Scroll(int pos) {
  CRGB color (0,0,0);
  if(pos < 85) {
    color.g = 0;
    color.r = ((float)pos / 85.0f) * 255.0f;
    color.b = 256 - color.r;
  } else if(pos < 170) {
    color.g = ((float)(pos - 85) / 85.0f) * 255.0f;
    color.r = 256 - color.g;
    color.b = 0;
  } else if(pos < 400) {
    color.b = ((float)(pos - 170) / 85.0f) * 255.0f;
    color.g = 256 - color.b;
    color.r = 1;
  }
  return color;
}
