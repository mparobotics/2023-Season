
#include "FastLED.h"
#define NUM_LEDS 80
#define buttonPin 2
bool oldButton;
CRGB leds[NUM_LEDS];
//0: Blue pattern
//1: Red pattern
//2: purple/yellow
//3:  blue/white
//4: rainbow
int color = 3;
int j = 0;
void setup() { 
  pinMode(buttonPin, INPUT_PULLUP);
  FastLED.addLeds<WS2801, RGB>(leds, NUM_LEDS); 
  for(int i = 0; i < NUM_LEDS; i++) {
      
    
  
    leds[i] = CRGB(0,0,0);  
      
         
  }

    FastLED.show();
}
void button(){
  bool buttonstate = digitalRead(buttonPin);
  if(buttonstate == HIGH && oldButton == LOW){
    color = (color + 1) % 3;
    j = 0;
    for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0,0,0);  
    }
    FastLED.show();
    delay(10);
  }
  oldButton = buttonstate;
}

void loop() {
   button();
  
    //int colors[6] = {CRGB::Red, CRGB::Orange, CRGB::Yellow, CRGB::Green, CRGB::Blue, CRGB::Purple};
     CRGB hi[7] = {CRGB::Red, CRGB::Orange, CRGB::Yellow, CRGB::Green, CRGB::Blue, CRGB::Purple, CRGB::Red};

    //}

    j = (j+1) % 2550;
   for(int i = 0; i < NUM_LEDS; i++) {
      
      //leds[i] = Scroll((i * 256 / NUM_LEDS + j) % 256);  
      int length = NUM_LEDS/2;
      if(color == 0){
        leds[i] = CRGB(0,0,127 + ((i-j) % length)*128/length);   
      }
      else if(color == 1){
        leds[i] = CRGB(127 + ((NUM_LEDS-i-j) % length)*128/length,0,0);  
      }
      else if (color == 2){
        leds[i] = makeColor(float(i+j)/NUM_LEDS * 360);
      }
    } 

    FastLED.show();
    delay(25);    
  

}
CRGB makeColor(int pos) {
  CRGB color (255,255,255);
  pos %= 360;
  if(pos < 120) {
    color.g = 0;
    color.r = (float(pos)/ 120.0f) * 255.0f;
    color.b = 255 - color.r;
  } else if(pos < 240) {
    color.g = (float(pos - 120) / 120.0f) * 255.0f;
    color.r = 256 - color.g;
    color.b = 0;
  } else {
    color.b = (float(pos - 240) / 120.0f) * 255.0f;
    color.g = 256 - color.b;
    color.r = 0;
  }
  return color;
}
