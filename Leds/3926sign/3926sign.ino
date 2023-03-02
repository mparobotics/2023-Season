  // put your setup code here, to run once:
#include <FastLED.h>

/* 
  layout:
  ---13---<   v--11---<   ---11---    
          |   |       |           |   |
          11  10      10          10  10
          |   |       |           |   |
  ^--13---<   >--11---^   ---11---     ---11---
          |           |   |           |        |
          11          10  10          10       10
          |           |   |           |        |
  >--13---^           ^   ---11---     ---11---

*/
//3: 61 leds
//9: 52 leds
//2: 53 leds
//6: 52 leds
//TOTAL: 218 leds
#define LED_PIN     5
#define NUM_LEDS    218
const int buttonPin = 2;     // the number of the pushbutton pin
int buttonState = 0;         // variable for reading the pushbutton status
int state = 0;
//order = int[NUM_LEDS];
float frames = 0;
const float ratio = 255/60;
CRGB leds[NUM_LEDS];
CRGB hsv(int h, int s, int v){
  int r = 0;
  int g = 0;
  int b = 0;

  v /= 100;
  h %= 360;

  if(h < 60){
    r = 255;
    g = ratio * h;
  }
  else if(h < 120){
    r = ratio * (120 - h);
    g = 255;
  }
  else if(h < 180){
    g = 255;
    b = ratio * (h - 120);
  }
  else if(h < 240){
    g = ratio * (240 - h);
    b = 255;
  }
  else if(h < 300){
    b = 255;
    r = ratio * (h - 240);
  }
  else{
    r = 255;
    b = ratio * (360 - h);
  }
  r *= v;
  g *= v;
  b *= v;

  return CRGB(r,g,b);
}
void setup() {
  delay(1000);
  Serial.begin (9600);
  pinMode(buttonPin, INPUT_PULLUP);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  attachInterrupt(digitalPinToInterrupt(buttonPin), button, CHANGE);
  for (int i = 0; i < NUM_LEDS; i++) {
    
    leds[i] = CRGB(0, 0, 0);
    //order[random(0,NUM_LEDS)] = i;
  
  }
  FastLED.show();
}

struct c2d{
  int x;
  int y;
};

c2d pos;
//given a light index, determine the light's position in an xy grid
struct c2d getLightPosition(int i){
  c2d c;
  //the 3
  //0 - 12
       if(i < 13){    c.x = i;        c.y = 0;        } 
  //13 - 23
  else if(i < 24){    c.x = 13;       c.y = i - 12;   }
  //24 - 36
  else if(i < 37){    c.x = 36 - i;   c.y = 13;       }
  //37 - 47
  else if(i < 48){    c.x = 13;       c.y = i - 23;   }
  //48 - 60
  else if(i < 61){    c.x = 60 - i;   c.y = 26;       }
  
  //the 9
  //61 - 70
  else if(i < 71){    c.x = 32;       c.y = i - 59;   }
  //71 - 80
  else if(i < 81){    c.x = 32;       c.y = i - 56;   }
  //81 - 91
  else if(i < 92){    c.x = 111 - i;  c.y = 26;       }
  //92 - 101
  else if(i < 102){   c.x = 18;       c.y = 116 - i;  }
  //102 - 112
  else if(i < 113){   c.x = i - 82;   c.y = 13;       }
  
  //the 2
  //113 - 123
  else if(i < 124){    c.x = i - 74;       c.y = 26;   }
  //124 - 133
  else if(i < 134){    c.x = 51;       c.y = 148 - i;   }
  //134 - 144
  else if(i < 145){    c.x = 183 - i;  c.y = 13;       }
  //145 - 154
  else if(i < 155){    c.x = 37;       c.y = 156 - i;  }
  //155 - 165
  else if(i < 166){    c.x = i - 116;   c.y = 0;       }
  
  //the 6
  //166 - 175
  else if(i < 176){    c.x = 56;       c.y = 190 - i;   }
  //176- 185
  else if(i < 186){    c.x = 56;       c.y = 187 - i;   }
  //186 - 196
  else if(i < 197){    c.x = i - 128;  c.y = 0;       }
  //197 - 206
  else if(i < 207){   c.x = 70;       c.y = i - 195;  }
  //207 - 217
  else if(i < 218){   c.x = 275 - i;   c.y = 13;   }

  else{c.x = -1;c.y = -1;}
  return c;
}
void button() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 70){
   buttonState = digitalRead(buttonPin);

  if (buttonState == LOW) {
    state ++;
    state = state & 3;
  } 
  Serial.println(state);
  }
  last_interrupt_time = interrupt_time;
}


void loop() {
  for (int i = 0; i < NUM_LEDS; i++) {
    pos = getLightPosition(i); 
    CRGB color = CRGB(0,0,0);
    
    

    

    int time = 400;
    float f = 0;
    float fi = float(int(frames) % time);
    if(fi < time/4){
      if(i < 61){
       
        color.b = 255;
        
      }
      else{
        color.b = 0; 
      }
      
    }
    else if(fi < time/2){
      if(i < 113 && i > 60){
       
       
        color.b = 255;
        
      }
      else{
        color.b = 0; 
      }
    }
    else if(fi < 3*time/4){
      if(i < 166 && i > 112){
        
      
        color.b = 255;
        
      }
      else{
        color.b = 0; 
      }
    }
    else{
      if(i < 218 && i > 165){
       
        color.b = 255;
        
      }
      else{
        color.b = 0; 
      }
    }

    
    

    color.b = 100 * sin(float(pos.x + pos.y)/5 + (float(frames)/5)) + 155;
    
    
    c2d centered;
    centered.x = pos.x - 34;
    centered.y = pos.y - 13;
    
    
    leds[i] = color;
    
    
  }
  FastLED.show();

  frames++;
  
}