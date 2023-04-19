
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
int color = 4;
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
    color = (color + 1) % 5;
    Serial.println("button");
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
      else if(color == 2){
        CRGB purple = CRGB(50,0,255);
        CRGB yellow = CRGB(255,255,0);
        int border1 =  j % NUM_LEDS;
        int border2 = (j + NUM_LEDS/2) % NUM_LEDS;

        if( (i < border1|| i > border2) && (border2 > border1) || (i > border2 && i < border1 && border2 < border1)){
          leds[i] = purple;
        }
        else{
          leds[i] = yellow;
        }
      }
      else if (color == 3){
        CRGB white = CRGB(255,255,255);
        CRGB blue = CRGB(0,127,255);
        int border1 =  j % NUM_LEDS;
        int border2 = (j + NUM_LEDS/2) % NUM_LEDS;
        
        if( (i < border1|| i > border2) && (border2 > border1) || (i > border2 && i < border1 && border2 < border1)){
          
          
          
          leds[i] = blue;
        }
        
        else{
          leds[i] = white;
        }
       
        
      }
      else if (color == 4){
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
