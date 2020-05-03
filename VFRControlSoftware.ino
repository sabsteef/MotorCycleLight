/*
 Version 0.3
 Author:
 - Sabsteef (Sabsteef@hotmail.com)
 
 Hardware:
 - Arduino Nano (Compatible board)
 Software:
 - Arduino 1.8.10

 Arduino Pin Mapping:
 - 00 = Serial RX USB Serial
 - 01 = Serial TX USB Serial
 - 04 = Switch blinker Right
 - 05 = Switch Brake
 - 06 = Switch blinker Left
 - 10 = LED tail light
 - 11 = LED mirror Left
 - 12 = LED mirror Right


 
*/

#include <FastLED.h>  // Control LED
#include <Chrono.h>   // Control Multitasking

//Left Front Mirror
#define LFM_NUM_LEDS 40
#define LFM_DATA_PIN 11

//Right Front Mirror
#define RFM_NUM_LEDS 40
#define RFM_DATA_PIN 12

//Tail Light
#define TAIL_NUM_LEDS 92
#define TAIL_DATA_PIN 10

//CRGB Settings
CRGB TAIL_Leds[TAIL_NUM_LEDS];
CRGB LFM_Leds[LFM_NUM_LEDS];
CRGB RFM_Leds[RFM_NUM_LEDS];

//consistant values

const int buttonblinkLSwitch = 6;
const int buttonblinkRSwitch = 4;
const int buttonBrakeSwitch  = 5; 


//Values
int buttonblinkR = 0;
int buttonblinkL = 0; 
int buttonBrake  = 0;
int Debug = 0;




// Instantiate Chronos treats
Chrono chrono_TailLight; 
Chrono chrono_Blinkers;
Chrono chronoC; //not used
Chrono chronoD; // not used
Chrono chrono_blinkerR;
Chrono chrono_blinkerL;
Chrono chronoG; //not used
Chrono chrono_brakelight;

// Check interters

int RV = 100;
int LV = 100;
int LA = 100;
int RA = 100;

 
void setup() {
  // sanity check delay - allows reprogramming if accidently blowing power w/leds
    delay(1000);
 
  //PinLayouts
    //inputs switches
    pinMode(buttonblinkRSwitch, INPUT);
    pinMode(buttonblinkLSwitch, INPUT);
    pinMode(buttonBrakeSwitch,  INPUT);

 

  //Set Led Matrix
    
    // Left Front Mirror
    FastLED.addLeds<WS2812B, LFM_DATA_PIN, GRB>(LFM_Leds, LFM_NUM_LEDS).setCorrection(TypicalSMD5050);

    // Right Front Mirror
    FastLED.addLeds<WS2812B, RFM_DATA_PIN, GRB>(RFM_Leds, RFM_NUM_LEDS).setCorrection(TypicalSMD5050);

    // Tail Light
    FastLED.addLeds<WS2812B, TAIL_DATA_PIN, GRB>(TAIL_Leds, TAIL_NUM_LEDS).setCorrection(TypicalSMD5050);
    
    // Set brightness all LED
    FastLED.setBrightness(255);
    

  /* FastLED provides these pre-conigured incandescent color profiles:
     Candle, Tungsten40W, Tungsten100W, Halogen, CarbonArc,
     HighNoonSun, DirectSunlight, OvercastSky, ClearBlueSky,
     FastLED provides these pre-configured gaseous-light color profiles:
     WarmFluorescent, StandardFluorescent, CoolWhiteFluorescent,
     FullSpectrumFluorescent, GrowLightFluorescent, BlackLightFluorescent,
     MercuryVapor, SodiumVapor, MetalHalide, HighPressureSodium,
     FastLED also provides an "Uncorrected temperature" profile
     UncorrectedTemperature; */

    //FastLED.setTemperature(Candle); 
    FastLED.setTemperature(HighPressureSodium); 

    FastLED.clear();
   
    // Turn on DRL
    DRLLichtR(0,40,100);
    DRLLichtL(0,40,100);

     // Turn on Tail-Light
    Tail_Light(0,92);
    // Enable debug  
    Serial.begin(115200);
    Serial.println ("startup");
}


void loop() {
  // check buttons
  buttonblinkL        = digitalRead(buttonblinkLSwitch);  
  buttonblinkR        = digitalRead(buttonblinkRSwitch);
  buttonBrake         = digitalRead(buttonBrakeSwitch);
  
  // Check if Brake light is needed
  if ( chrono_TailLight.hasPassed(50) ) { 
    buttonBrake = digitalRead(buttonBrakeSwitch);
    chrono_TailLight.restart();
    if (buttonBrake == HIGH && buttonblinkL == LOW && buttonblinkR == LOW){
      Brake_Light(0,93,0);  
    }
    if (buttonBrake == HIGH && buttonblinkL == HIGH && buttonblinkR == LOW){
      Brake_Light(0,46,0);
    }
    if (buttonBrake == HIGH && buttonblinkL == LOW && buttonblinkR == HIGH){
      Brake_Light(46,92,0); 
    }
  }

  // Check if Blinkers are needed
  if ( chrono_Blinkers.hasPassed(60) ) { 
    if (buttonblinkL == HIGH){    //Check if Blinker links
     Serial.println ("startup knipperlicht L detected");
      MirrorL_run(46,91,0);
      chrono_blinkerL.restart(); 
    }  
    else if (buttonblinkL == LOW){
        LA = 100;
        LV = 100;
    }
    if (buttonblinkR == HIGH ){
       MirrorR_run(0,46,0);
       chrono_blinkerR.restart(); 
    }  
      else if (buttonblinkR == LOW){
        RA = 100;
        RV = 100;
    }
    chrono_Blinkers.restart();
  }
 // check if default light can be reset.
 if ( chrono_brakelight.hasPassed(101) ) {
    chrono_brakelight.restart();
    if (buttonBrake == LOW || buttonblinkL == LOW || buttonblinkR == LOW){
      if (buttonBrake == LOW && buttonblinkL == LOW){
      Tail_Light(46,92);
      }
      if (buttonBrake == LOW && buttonblinkR == LOW){
      Tail_Light(0,46);
      }
      if(buttonblinkL == LOW ){
      DRLLichtL(0,40,100);
      }
      if(buttonblinkR == LOW ) {
      DRLLichtR(0,40,100); 
      }
    }   
  }
}


// Tail-Light
void Tail_Light(int st,int en){
  for(int dot = st; dot < en; dot++) { 
      TAIL_Leds[dot] = CRGB::Red;
      TAIL_Leds[dot].fadeToBlackBy(164);   
   }
   FastLED.show();    
}

//Brake-Light
void Brake_Light(int st,int en, int fade){
  for(int dot = st; dot < en; dot++) { 
      TAIL_Leds[dot] = CRGB::Red;
      TAIL_Leds[dot].fadeToBlackBy(fade); 

  }
  FastLED.show();
}

//dag rij verlighting L voorkant
void DRLLichtL(int st,int en, int fade ){
  //FastLED.clear(); 
  for(int dot = st; dot >= en; dot--) { 
      LFM_Leds[dot] = CRGB::Black;
  }
  FastLED.show();
  for(int dot = st; dot < en; dot++) { 
      LFM_Leds[dot] = CRGB::White;
      LFM_Leds[dot].fadeToBlackBy(fade);
      NoLED();
   }
   FastLED.show();
}

//dag rij verlighting R voorkant
void DRLLichtR(int st,int en, int fade ){
  //FastLED.clear(); 
  for(int dot = st; dot >= en; dot--) { 
      RFM_Leds[dot] = CRGB::Black;
  }
  FastLED.show();
  for(int dot = st; dot < en; dot++) { 
      RFM_Leds[dot] = CRGB::White;
      RFM_Leds[dot].fadeToBlackBy(fade);
      NoLED();
   }
   FastLED.show();
}
// Runnig lights Left side
void MirrorL_run(int st,int en, int fade){
    if ( chrono_blinkerL.hasPassed(10) ) {
     //chrono_blinkerL.restart(); // restart the crono so that it triggers again later
      if (LA == 100){
        for(int dot = st; dot < 92; dot++) { //reset tail left
           TAIL_Leds[dot] = CRGB::Black;
        } 
         for(int dot = 0; dot < 40; dot++) {  //reset front left
           LFM_Leds[dot] = CRGB::Black;
        } 
        FastLED.show();
        LA = 1;
      }
      else if (LA != 100){
        if (LA >= en){ 
        LA = 100;
        }
        else {
           if (LA == 1){
            
           TAIL_Leds[88]  = CRGB::DarkOrange;
           TAIL_Leds[81]  = CRGB::DarkOrange;
           TAIL_Leds[68]  = CRGB::DarkOrange;
           TAIL_Leds[59]  = CRGB::DarkOrange;
           TAIL_Leds[51]  = CRGB::DarkOrange;
           TAIL_Leds[50]  = CRGB::DarkOrange;

           LFM_Leds[24]  = CRGB::DarkOrange;
           LFM_Leds[23]  = CRGB::DarkOrange;
           LFM_Leds[8]   = CRGB::DarkOrange;
           LFM_Leds[7]   = CRGB::DarkOrange;
           
           NoLED();
           FastLED.show();
           LA = 2; 
          }
          else if (LA == 2){
            
           TAIL_Leds[89]  = CRGB::DarkOrange;
           TAIL_Leds[82]  = CRGB::DarkOrange;
           TAIL_Leds[69]  = CRGB::DarkOrange;
           TAIL_Leds[60]  = CRGB::DarkOrange;
           TAIL_Leds[52]  = CRGB::DarkOrange;
           TAIL_Leds[49]  = CRGB::DarkOrange;

           LFM_Leds[25]  = CRGB::DarkOrange;
           LFM_Leds[22] = CRGB::DarkOrange;
           LFM_Leds[9] = CRGB::DarkOrange;
           LFM_Leds[6] = CRGB::DarkOrange;
           
           NoLED();
           FastLED.show();
           LA = 3; 
          }
          else if (LA == 3){
 
           TAIL_Leds[90]  = CRGB::DarkOrange;
           TAIL_Leds[83]  = CRGB::DarkOrange;
           TAIL_Leds[70]  = CRGB::DarkOrange;
           TAIL_Leds[61]  = CRGB::DarkOrange;
           TAIL_Leds[53]  = CRGB::DarkOrange;
           TAIL_Leds[48]  = CRGB::DarkOrange;
  
           LFM_Leds[26]  = CRGB::DarkOrange;
           LFM_Leds[21] = CRGB::DarkOrange;
           LFM_Leds[10] = CRGB::DarkOrange;
           LFM_Leds[5] = CRGB::DarkOrange;
           
           NoLED();
           FastLED.show();
           LA = 4; 
          }
          else if (LA == 4){

           TAIL_Leds[91]  = CRGB::DarkOrange;
           TAIL_Leds[84]  = CRGB::DarkOrange;
           TAIL_Leds[71]  = CRGB::DarkOrange;
           TAIL_Leds[62]  = CRGB::DarkOrange;
           TAIL_Leds[54]  = CRGB::DarkOrange;
           TAIL_Leds[47]  = CRGB::DarkOrange;

           LFM_Leds[27]  = CRGB::DarkOrange;
           LFM_Leds[20] = CRGB::DarkOrange;
           LFM_Leds[11] = CRGB::DarkOrange;
           LFM_Leds[4] = CRGB::DarkOrange;
           
           NoLED();
           FastLED.show();
           LA = 5; 
          }
          else if (LA == 5){

           TAIL_Leds[85]  = CRGB::DarkOrange;
           TAIL_Leds[72]  = CRGB::DarkOrange;
           TAIL_Leds[63]  = CRGB::DarkOrange;
           TAIL_Leds[55]  = CRGB::DarkOrange;
           TAIL_Leds[46]  = CRGB::DarkOrange;
 
           LFM_Leds[28]  = CRGB::DarkOrange;
           LFM_Leds[19] = CRGB::DarkOrange;
           LFM_Leds[12] = CRGB::DarkOrange;
           LFM_Leds[3] = CRGB::DarkOrange;
           NoLED();
                      
           FastLED.show();
           LA = 6; 
          }
          else if (LA == 6){

           TAIL_Leds[86]  = CRGB::DarkOrange;
           TAIL_Leds[73]  = CRGB::DarkOrange;
           TAIL_Leds[64]  = CRGB::DarkOrange;
           TAIL_Leds[56]  = CRGB::DarkOrange;
           // back
           LFM_Leds[29]  = CRGB::DarkOrange;
           LFM_Leds[18] = CRGB::DarkOrange;
           LFM_Leds[13] = CRGB::DarkOrange;
           LFM_Leds[2] = CRGB::DarkOrange;
           
           NoLED();
           FastLED.show();
           LA = 7; 
          }
          else if (LA == 7){

           TAIL_Leds[87]  = CRGB::DarkOrange;
           TAIL_Leds[74]  = CRGB::DarkOrange;
           TAIL_Leds[65]  = CRGB::DarkOrange;
           TAIL_Leds[57]  = CRGB::DarkOrange;

           LFM_Leds[30]  = CRGB::DarkOrange;
           LFM_Leds[17] = CRGB::DarkOrange;
           LFM_Leds[14] = CRGB::DarkOrange;
           LFM_Leds[1] = CRGB::DarkOrange;
           
           NoLED();
           FastLED.show();
           LA = 8; 
          }
          else if (LA == 8){

           TAIL_Leds[75]  = CRGB::DarkOrange;
           TAIL_Leds[66]  = CRGB::DarkOrange;
           TAIL_Leds[58]  = CRGB::DarkOrange;

           LFM_Leds[31]  = CRGB::DarkOrange;
           LFM_Leds[16] = CRGB::DarkOrange;
           LFM_Leds[15] = CRGB::DarkOrange;
           LFM_Leds[0] = CRGB::DarkOrange;
           NoLED();
           FastLED.show();
           LA = 9; 
          }
          else if (LA == 9){

           TAIL_Leds[67]  = CRGB::DarkOrange;
           TAIL_Leds[76]  = CRGB::DarkOrange;

           LFM_Leds[36]  = CRGB::DarkOrange;
           LFM_Leds[35] = CRGB::DarkOrange;
           
           NoLED();
           FastLED.show();
           LA = 10; 
          }
          else if (LA == 10){
 
           TAIL_Leds[77]  = CRGB::DarkOrange;

           LFM_Leds[37]  = CRGB::DarkOrange;
           LFM_Leds[34] = CRGB::DarkOrange;
           
           NoLED();

           FastLED.show();
           LA = 11; 
          }
          else if (LA == 11){

           TAIL_Leds[78]  = CRGB::DarkOrange;

           LFM_Leds[38]  = CRGB::DarkOrange;
           LFM_Leds[33] = CRGB::DarkOrange;
           
           NoLED();
           FastLED.show();
           LA = 12; 
          }
          else if (LA == 12){

           TAIL_Leds[79]  = CRGB::DarkOrange;

           LFM_Leds[39] = CRGB::DarkOrange;
           LFM_Leds[32] = CRGB::DarkOrange;

           NoLED();
           FastLED.show();
           LA = 13; 
          }
          else if (LA == 13){
            // front
           TAIL_Leds[80]  = CRGB::DarkOrange;
           
           FastLED.show();
           LA = 15; 
          }
          else if (LA == 15){

           NoLED();
           FastLED.show();
           LA = 91; 
          }    
       }
     }
   }  
}


void MirrorR_run(int st,int en, int fade){
   if ( chrono_blinkerR.hasPassed(10) ){
     //chrono_blinkerR.restart(); // restart the crono so that it triggers again later
      if (RA == 100){
        for(int dot = st; dot < en; dot++) { 
           TAIL_Leds[dot] = CRGB::Black;
        } 
         for(int dot = 0; dot < 40; dot++) { 
           RFM_Leds[dot] = CRGB::Black;
        } 
        FastLED.show();
        RA = 1;
      }
      
      else if (RA != 100){
        if (RA >= en){ 
           
        RA = 100;
        }
        else {
           if (RA == 1){
           TAIL_Leds[45]  = CRGB::DarkOrange;
           TAIL_Leds[35]  = CRGB::DarkOrange;
           TAIL_Leds[22]  = CRGB::DarkOrange;
           TAIL_Leds[21]  = CRGB::DarkOrange;
           TAIL_Leds[5]  = CRGB::DarkOrange;
           TAIL_Leds[4]  = CRGB::DarkOrange;

           RFM_Leds[24]  = CRGB::DarkOrange;
           RFM_Leds[23] = CRGB::DarkOrange;
           RFM_Leds[8] = CRGB::DarkOrange;
           RFM_Leds[7] = CRGB::DarkOrange;
           NoLED();
           FastLED.show();
           RA = 2; 
          }
          else if (RA == 2){

           TAIL_Leds[44]  = CRGB::DarkOrange;
           TAIL_Leds[36]  = CRGB::DarkOrange;
           TAIL_Leds[23]  = CRGB::DarkOrange;
           TAIL_Leds[20]  = CRGB::DarkOrange;
           TAIL_Leds[6]  = CRGB::DarkOrange;
           TAIL_Leds[3]  = CRGB::DarkOrange;

           RFM_Leds[25]  = CRGB::DarkOrange;
           RFM_Leds[22] = CRGB::DarkOrange;
           RFM_Leds[9] = CRGB::DarkOrange;
           RFM_Leds[6] = CRGB::DarkOrange;
           NoLED();
           FastLED.show();
           RA = 3; 
          }
          else if (RA == 3){

           TAIL_Leds[43]  = CRGB::DarkOrange;
           TAIL_Leds[37]  = CRGB::DarkOrange;
           TAIL_Leds[24]  = CRGB::DarkOrange;
           TAIL_Leds[19]  = CRGB::DarkOrange;
           TAIL_Leds[7]  = CRGB::DarkOrange;
           TAIL_Leds[2]  = CRGB::DarkOrange;

           RFM_Leds[26]  = CRGB::DarkOrange;
           RFM_Leds[21] = CRGB::DarkOrange;
           RFM_Leds[10] = CRGB::DarkOrange;
           RFM_Leds[5] = CRGB::DarkOrange;
           NoLED();
           FastLED.show();
           RA = 4; 
          }
          else if (RA == 4){
 
           TAIL_Leds[42]  = CRGB::DarkOrange;
           TAIL_Leds[38]  = CRGB::DarkOrange;
           TAIL_Leds[25]  = CRGB::DarkOrange;
           TAIL_Leds[18]  = CRGB::DarkOrange;
           TAIL_Leds[8]  = CRGB::DarkOrange;
           TAIL_Leds[1]  = CRGB::DarkOrange;

           RFM_Leds[27]  = CRGB::DarkOrange;
           RFM_Leds[20] = CRGB::DarkOrange;
           RFM_Leds[11] = CRGB::DarkOrange;
           RFM_Leds[4] = CRGB::DarkOrange;
           NoLED();
           FastLED.show();
           RA = 5; 
          }
          else if (RA == 5){

           TAIL_Leds[39]  = CRGB::DarkOrange;
           TAIL_Leds[26]  = CRGB::DarkOrange;
           TAIL_Leds[17]  = CRGB::DarkOrange;
           TAIL_Leds[9]  = CRGB::DarkOrange;
           TAIL_Leds[0]  = CRGB::DarkOrange;

           RFM_Leds[28]  = CRGB::DarkOrange;
           RFM_Leds[19] = CRGB::DarkOrange;
           RFM_Leds[12] = CRGB::DarkOrange;
           RFM_Leds[3] = CRGB::DarkOrange;
           NoLED();
                      
           FastLED.show();
           RA = 6; 
          }
          else if (RA == 6){

           TAIL_Leds[40]  = CRGB::DarkOrange;
           TAIL_Leds[27]  = CRGB::DarkOrange;
           TAIL_Leds[16]  = CRGB::DarkOrange;
           TAIL_Leds[10]  = CRGB::DarkOrange;

           RFM_Leds[29]  = CRGB::DarkOrange;
           RFM_Leds[18] = CRGB::DarkOrange;
           RFM_Leds[13] = CRGB::DarkOrange;
           RFM_Leds[2] = CRGB::DarkOrange;
           NoLED();
           FastLED.show();
           RA = 7; 
          }
          else if (RA == 7){
            
           TAIL_Leds[41]  = CRGB::DarkOrange;
           TAIL_Leds[28]  = CRGB::DarkOrange;
           TAIL_Leds[15]  = CRGB::DarkOrange;
           TAIL_Leds[11]  = CRGB::DarkOrange;

           RFM_Leds[30]  = CRGB::DarkOrange;
           RFM_Leds[17] = CRGB::DarkOrange;
           RFM_Leds[14] = CRGB::DarkOrange;
           RFM_Leds[1] = CRGB::DarkOrange;
           NoLED();
           FastLED.show();
           RA = 8; 
          }
          else if (RA == 8){

           TAIL_Leds[29]  = CRGB::DarkOrange;
           TAIL_Leds[14]  = CRGB::DarkOrange;
           TAIL_Leds[12]  = CRGB::DarkOrange;
 
           RFM_Leds[31]  = CRGB::DarkOrange;
           RFM_Leds[16] = CRGB::DarkOrange;
           RFM_Leds[15] = CRGB::DarkOrange;
           RFM_Leds[0] = CRGB::DarkOrange;
           NoLED();
           FastLED.show();
           RA = 9; 
          }
          else if (RA == 9){

           TAIL_Leds[30]  = CRGB::DarkOrange;
           TAIL_Leds[13]  = CRGB::DarkOrange;

           RFM_Leds[39]  = CRGB::DarkOrange;
           RFM_Leds[32] = CRGB::DarkOrange;
           NoLED();
           FastLED.show();
           RA = 10; 
          }
          else if (RA == 10){

           TAIL_Leds[31]  = CRGB::DarkOrange;

           RFM_Leds[38]  = CRGB::DarkOrange;
           RFM_Leds[33] = CRGB::DarkOrange;
           NoLED();
           FastLED.show();
           RA = 11; 
          }
          else if (RA == 11){

           TAIL_Leds[32]  = CRGB::DarkOrange;

           RFM_Leds[37]  = CRGB::DarkOrange;
           RFM_Leds[34] = CRGB::DarkOrange;
           NoLED();
           FastLED.show();
           RA = 12; 
          }
          else if (RA == 12){

            TAIL_Leds[33]  = CRGB::DarkOrange;

           RFM_Leds[36] = CRGB::DarkOrange;
           RFM_Leds[35] = CRGB::DarkOrange;
           FastLED.show();
           RA = 13; 
          }
          else if (RA == 13){
            
           TAIL_Leds[34]  = CRGB::DarkOrange;

           FastLED.show();
           RA = 15; 
          }
          else if (RA == 15){

           NoLED();
           FastLED.show();
           RA = 46; 
          }    
        }
      }
   }  
}

// normal blinking Left side //still in development
void MirrorL_normal(int st,int en, int fade){ 
    if ( chrono_blinkerL.hasPassed(59) ) { // need to use a other Chrono
     //chrono_blinkerL.restart(); // restart the crono so that it triggers again later
     Serial.println ("MirrorL_normal Fuction");
      if (LV == 100){
        Serial.println ("MirrorL_normal LV = 100");
        for(int dot = st; dot < en; dot++) { 
              RFM_Leds[dot] = CRGB::Blue;
              RFM_Leds[dot].fadeToBlackBy(fade);
              LFM_Leds[dot] = CRGB::Black;
              LFM_Leds[dot].fadeToBlackBy(fade);
        }
       
        FastLED.show();
        LV = 1;
      }
      else if (LV == 1){
         Serial.println ("MirrorL_normal LV = 1");
            
           LV = 2; 
      }   
      else if (LV == 2){
         Serial.println ("MirrorL_normal LV = 2");
           for(int dot = st; dot < en; dot++) { 
              LFM_Leds[dot] = CRGB::Blue;
              LFM_Leds[dot].fadeToBlackBy(fade);
              RFM_Leds[dot] = CRGB::Black;
              RFM_Leds[dot].fadeToBlackBy(fade);
              NoLED();
           }
           NoLED();
           FastLED.show();
           LV = 100; 
          }
    }
}
     

// Normal blinker Right ( still in development)
void MirrorR_normal(int st,int en, int fade){
 
}






// Fuction that does not turn on LEDS that are not seen needed for DRL
void NoLED(){
 LFM_Leds[24] = CRGB::Black;
  LFM_Leds[7] = CRGB::Black;
  RFM_Leds[24] = CRGB::Black;
  RFM_Leds[7] = CRGB::Black;
  
}


// turn off taillights
void Tail_LightOFF(int st,int en){
  //FastLED.clear(); 
  for(int dot = st; dot < en; dot++) { 
            TAIL_Leds[dot] = CRGB::Black;
            
        }
        FastLED.show();
}

// traffic - warning lights ( still in development)
void TrafficLights(int st,int en, int fade){
    if ( chrono_blinkerL.hasPassed(59) ) { // returns true if it passed 250 ms since it was started
     //chrono_blinkerL.restart(); // restart the crono so that it triggers again later
     Serial.println ("MirrorL_normal Fuction");
      if (LV == 100){
        //DRL
        for(int dot = st; dot < en; dot++) { 
              RFM_Leds[dot] = CRGB::DarkOrange;
              RFM_Leds[dot].fadeToBlackBy(fade);
              LFM_Leds[dot] = CRGB::Black;
              LFM_Leds[dot].fadeToBlackBy(fade);
        }
        //tail left
         for(int dot = 0; dot < 46; dot++) { 
               TAIL_Leds[dot] = CRGB::DarkOrange;
               TAIL_Leds[dot].fadeToBlackBy(fade);
        }
        //tail right
         for(int dot = st; 46 < 92; dot++) { 
              TAIL_Leds[dot] = CRGB::Black;
              TAIL_Leds[dot].fadeToBlackBy(fade);
        }
       NoLED();
        FastLED.show();
        LV = 1;
      }
      else if (LV == 1){
         Serial.println ("MirrorL_normal LV = 1");
            
           LV = 2; 
      }   
      else if (LV == 2){
         Serial.println ("MirrorL_normal LV = 2");
           for(int dot = st; dot < en; dot++) { 
              LFM_Leds[dot] = CRGB::DarkOrange;
              LFM_Leds[dot].fadeToBlackBy(fade);
              RFM_Leds[dot] = CRGB::Black;
              RFM_Leds[dot].fadeToBlackBy(fade);
           }
           //tail left
         for(int dot = 46; dot < 91; dot++) { 
               TAIL_Leds[dot] = CRGB::DarkOrange;
               TAIL_Leds[dot].fadeToBlackBy(fade);
        }
        //tail right
         for(int dot = st; 0 < 45; dot++) { 
              TAIL_Leds[dot] = CRGB::Black;
              TAIL_Leds[dot].fadeToBlackBy(fade);
        }
           NoLED();
           FastLED.show();
           LV = 100; 
          }
    }
}
