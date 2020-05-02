//working version
#include <FastLED.h>  //control LED
#include <Chrono.h>   // Control Multitasking

 //Left Front Mirror
 #define LFM_NUM_LEDS 40
 #define LFM_DATA_PIN 11

//Right Front Mirror
 #define RFM_NUM_LEDS 40
 #define RFM_DATA_PIN 12

//Tail Light
 #define NUM_LEDS 92
 #define DATA_PIN 10

 //CRGB Settings
CRGB leds[NUM_LEDS];
CRGB LFM_Leds[LFM_NUM_LEDS];
CRGB RFM_Leds[RFM_NUM_LEDS];

//consistant values

const int buttonblinkLSwitch = 6;
const int buttonblinkRSwitch = 4;
const int buttonBrakeSwitch  = 5; 
const int buttonWarningSwitch  = 3;  //not used


//Values
int buttonblinkR = 0;
int buttonblinkL = 0; 
int buttonBrake  = 0;
int buttonWarninglichts  = 0; //not used



// Instantiate Chronos treats
Chrono chronoA; 
Chrono chronoB;
Chrono chronoC;
Chrono chronoD;
Chrono chronoE;
Chrono chronoF;
Chrono chronoG; //not used
Chrono chronoX;

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
    pinMode(buttonWarningSwitch,  INPUT); //not used

 

    //Set Led Matrix
    
    // Tail Light
    FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalSMD5050);
    
    // Left Front Mirror
    FastLED.addLeds<WS2812B, LFM_DATA_PIN, GRB>(LFM_Leds, LFM_NUM_LEDS).setCorrection(TypicalSMD5050);

    // Right Front Mirror
    FastLED.addLeds<WS2812B, RFM_DATA_PIN, GRB>(RFM_Leds, RFM_NUM_LEDS).setCorrection(TypicalSMD5050);
    
    //set brightness all LED
    FastLED.setBrightness(255);
    

  // FastLED provides these pre-conigured incandescent color profiles:
  //     Candle, Tungsten40W, Tungsten100W, Halogen, CarbonArc,
  //     HighNoonSun, DirectSunlight, OvercastSky, ClearBlueSky,
  // FastLED provides these pre-configured gaseous-light color profiles:
  //     WarmFluorescent, StandardFluorescent, CoolWhiteFluorescent,
  //     FullSpectrumFluorescent, GrowLightFluorescent, BlackLightFluorescent,
  //     MercuryVapor, SodiumVapor, MetalHalide, HighPressureSodium,
  // FastLED also provides an "Uncorrected temperature" profile
  //    UncorrectedTemperature;

    FastLED.setTemperature(Candle); 
  //FastLED.setTemperature(DirectSunlight); 

    FastLED.clear();
  //zet achterlight aan
    AchterLicht(0,92);
  //zet Dag rij verlichting aan  
    DRLLichtR(0,40,100);
    DRLLichtL(0,40,100);
  //debug  
    Serial.begin(115200);
    Serial.println ("startup");
}


void loop() {
  // check buttons
  buttonblinkR        = digitalRead(buttonblinkRSwitch);
  buttonblinkL        = digitalRead(buttonblinkLSwitch);   
  buttonWarninglichts = digitalRead(buttonWarningSwitch); 
  buttonBrake         = digitalRead(buttonBrakeSwitch);
  
  if ( chronoA.hasPassed(50) ) { //Brake Licht
    buttonBrake = digitalRead(buttonBrakeSwitch);
    chronoA.restart();
    if (buttonBrake == HIGH && buttonblinkL == LOW && buttonblinkR == LOW){
      REMLicht(0,93,0);  
    }
    if (buttonBrake == HIGH && buttonblinkL == HIGH && buttonblinkR == LOW){
      REMLicht(0,46,0);
    }
    if (buttonBrake == HIGH && buttonblinkL == LOW && buttonblinkR == HIGH){
      REMLicht(46,92,0); 
    }
  }

  
  if ( chronoB.hasPassed(200) ) { //Blinkers
    chronoB.restart(); 
    if (buttonblinkL == HIGH){    //Check if Blinker links
     //Serial.println ("startup knipperlicht L detected");
      KnipperLichtLNEW(0,40,0);
      KnipperLichtLAchterNEW(46,91,0);
      chronoF.restart();
    }  
      else if (buttonblinkL == LOW && buttonWarninglichts == LOW){
        LA = 100;
        LV = 100;
    }
    if (buttonblinkR == HIGH){
     // Serial.println ("startup knipperlicht R detected");
       KnipperLichtRNEW(0,40,0);
       KnipperLichtRAchterNEW(0,46,0);
       chronoE.restart(); 
       //Police(0,40,0);
    }  
      else if (buttonblinkR == LOW && buttonWarninglichts == LOW){
        RA = 100;
        RV = 100;
    }
  }

 if ( chronoX.hasPassed(101) ) { // returns true if it passed 250 ms since it was started
    chronoX.restart(); // restart the crono so that it triggers again later
    if (buttonBrake == LOW || buttonblinkL == LOW || buttonblinkR == LOW){
      if (buttonBrake == LOW && buttonblinkL == LOW){
      AchterLicht(46,92);
      }
      if (buttonBrake == LOW && buttonblinkR == LOW){
      AchterLicht(0,46);
      }
      if(buttonblinkL == LOW && buttonWarninglichts  == LOW ){
      DRLLichtL(0,40,100);
      }
      if(buttonblinkR == LOW && buttonWarninglichts  == LOW ) {
      DRLLichtR(0,40,100); 
      }
    }   
  }
}


// Tail-Light
void AchterLicht(int st,int en){
  for(int dot = st; dot < en; dot++) { 
      leds[dot] = CRGB::Red;
      leds[dot].fadeToBlackBy(164);   
   }
   FastLED.show();    
}

//Brake-Light
void REMLicht(int st,int en, int fade){
  for(int dot = st; dot < en; dot++) { 
      leds[dot] = CRGB::Red;
      leds[dot].fadeToBlackBy(fade); 

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




void KnipperLichtRAchterNEW(int st,int en, int fade){
    if ( chronoE.hasPassed(100) ) { // returns true if it passed 100 ms since it was started
     //chronoE.restart(); // restart the crono so that it triggers again later
       //Serial.println ("knipperlicht R Fuction");
      if (RA == 100){
        for(int dot = st; dot < en; dot++) { 
           leds[dot] = CRGB::Black;
        } 
        leds[2]  = CRGB::DarkOrange;
        leds[3]  = CRGB::DarkOrange;
        leds[4]  = CRGB::DarkOrange;
        leds[5]  = CRGB::DarkOrange;
        leds[6]  = CRGB::DarkOrange;
        leds[7]  = CRGB::DarkOrange;
        leds[19]  = CRGB::DarkOrange;
        leds[20]  = CRGB::DarkOrange;
        leds[21]  = CRGB::DarkOrange;
        leds[22]  = CRGB::DarkOrange;
        leds[23]  = CRGB::DarkOrange;
        leds[24]  = CRGB::DarkOrange;
        leds[35]  = CRGB::DarkOrange;
        leds[36]  = CRGB::DarkOrange;
        leds[37]  = CRGB::DarkOrange;
        leds[43]  = CRGB::DarkOrange;
        leds[44]  = CRGB::DarkOrange;
        leds[45]  = CRGB::DarkOrange;

        FastLED.show();
        RA = 4;
      }
      else if (RA != 100){
        if (RA >= en){ 
           
        RA = 100;
        }
        else {
          if (RA == 4){
           leds[0]  = CRGB::DarkOrange;
           leds[1]  = CRGB::DarkOrange;
           leds[8]  = CRGB::DarkOrange;
           leds[9]  = CRGB::DarkOrange;
           leds[10]  = CRGB::DarkOrange;
           leds[16]  = CRGB::DarkOrange;
           leds[17]  = CRGB::DarkOrange;
           leds[18]  = CRGB::DarkOrange;
           leds[25]  = CRGB::DarkOrange;
           leds[26]  = CRGB::DarkOrange;
           leds[27]  = CRGB::DarkOrange;
           leds[38]  = CRGB::DarkOrange;
           leds[39]  = CRGB::DarkOrange;
           leds[40]  = CRGB::DarkOrange;
           leds[42]  = CRGB::DarkOrange;

           FastLED.show();
           RA = 2; 
          }
           else if (RA == 2){

           leds[11]  = CRGB::DarkOrange;
           leds[12]  = CRGB::DarkOrange;
           leds[13]  = CRGB::DarkOrange;
           leds[14]  = CRGB::DarkOrange;
           leds[15]  = CRGB::DarkOrange;
           leds[30]  = CRGB::DarkOrange;
           leds[29]  = CRGB::DarkOrange;
           leds[28]  = CRGB::DarkOrange;
           leds[41]  = CRGB::DarkOrange;

           FastLED.show();
           RA = 0; 
          }
          else if (RA == 0){

             leds[31]  = CRGB::DarkOrange;
             leds[32]  = CRGB::DarkOrange;

             FastLED.show();
             RA = 36; 
          }
          else if (RA == 36){

           leds[33]  = CRGB::DarkOrange;
           leds[34]  = CRGB::DarkOrange;

           FastLED.show();
           RA = 38; 
          }
          else if (RA == 38){

           NoLED();
           FastLED.show();
           RA = 46; 
          }    
       }
     }
   }  
}

void KnipperLichtLNEW(int st,int en, int fade){
    if ( chronoF.hasPassed(100) ) { // returns true if it passed 250 ms since it was started
    // chronoF.restart(); // restart the crono so that it triggers again later
     //  Serial.println ("knipperlicht L Fuction");
      if (LV == 100){
        for(int dot = st; dot < en; dot++) { 
           LFM_Leds[dot] = CRGB::Black;
        }
        LFM_Leds[6]  = CRGB::DarkOrange;
        LFM_Leds[7]  = CRGB::DarkOrange;
        LFM_Leds[8]  = CRGB::DarkOrange;
        LFM_Leds[9]  = CRGB::DarkOrange;
        LFM_Leds[22] = CRGB::DarkOrange;
        LFM_Leds[23] = CRGB::DarkOrange;
        LFM_Leds[24] = CRGB::DarkOrange;
        LFM_Leds[25] = CRGB::DarkOrange;

        NoLED();
        FastLED.show();
        LV = 4;
      }
      else if (LV != 100){
        if (LV >= en){ 
           
        LV = 100;
        }
        else {
          if (LV == 4){
           LFM_Leds[5]  = CRGB::DarkOrange;
           LFM_Leds[10] = CRGB::DarkOrange;
           LFM_Leds[21] = CRGB::DarkOrange;
           LFM_Leds[26] = CRGB::DarkOrange;
           LFM_Leds[4]  = CRGB::DarkOrange;
           LFM_Leds[11] = CRGB::DarkOrange;
           LFM_Leds[20] = CRGB::DarkOrange;
           LFM_Leds[27] = CRGB::DarkOrange;

           NoLED();
           FastLED.show();
           LV = 2; 
          }
           else if (LV == 2){
           LFM_Leds[3]  = CRGB::DarkOrange;
           LFM_Leds[12] = CRGB::DarkOrange;
           LFM_Leds[19] = CRGB::DarkOrange;
           LFM_Leds[28] = CRGB::DarkOrange;
           LFM_Leds[2]  = CRGB::DarkOrange;
           LFM_Leds[13] = CRGB::DarkOrange;
           LFM_Leds[18] = CRGB::DarkOrange;
           LFM_Leds[29] = CRGB::DarkOrange;


  

           NoLED();
           FastLED.show();
           LV = 0; 
          }
          else if (LV == 0){
             LFM_Leds[1]  = CRGB::DarkOrange;
             LFM_Leds[14] = CRGB::DarkOrange;
             LFM_Leds[17] = CRGB::DarkOrange;
             LFM_Leds[30] = CRGB::DarkOrange;
             LFM_Leds[0]  = CRGB::DarkOrange;
             LFM_Leds[15] = CRGB::DarkOrange;
             LFM_Leds[16] = CRGB::DarkOrange;
             LFM_Leds[31] = CRGB::DarkOrange;

             NoLED();
             FastLED.show();
             LV = 36; 
          }
          else if (LV == 36){
           LFM_Leds[34] = CRGB::DarkOrange;
           LFM_Leds[37] = CRGB::DarkOrange;
           LFM_Leds[35] = CRGB::DarkOrange;
           LFM_Leds[36] = CRGB::DarkOrange;

           
           NoLED();
           FastLED.show();
           LV = 38; 
          }
          else if (LV == 38){
           LFM_Leds[32] = CRGB::DarkOrange;
           LFM_Leds[39] = CRGB::DarkOrange;
           LFM_Leds[33] = CRGB::DarkOrange;
           LFM_Leds[38] = CRGB::DarkOrange;
           NoLED();
           FastLED.show();
           LV = 40; 
          }    
       }
     }
   }  
}

void KnipperLichtRNEW(int st,int en, int fade){
    if ( chronoE.hasPassed(100) ) { // returns true if it passed 250 ms since it was started
     //chronoE.restart(); // restart the crono so that it triggers again later
     //  Serial.println ("knipperlicht R Fuction");
      if (RV == 100){
        for(int dot = st; dot < en; dot++) { 
           RFM_Leds[dot] = CRGB::Black;
        }
        RFM_Leds[6]  = CRGB::DarkOrange;
        RFM_Leds[7]  = CRGB::DarkOrange;
        RFM_Leds[8]  = CRGB::DarkOrange;
        RFM_Leds[9]  = CRGB::DarkOrange;
        RFM_Leds[22] = CRGB::DarkOrange;
        RFM_Leds[23] = CRGB::DarkOrange;
        RFM_Leds[24] = CRGB::DarkOrange;
        RFM_Leds[25] = CRGB::DarkOrange;
        NoLED();
        FastLED.show();
        RV = 4;
      }
      else if (RV != 100){
        if (RV >= en){ 
        RV = 100;
        }
        else {
          if (RV == 4){
           RFM_Leds[5]  = CRGB::DarkOrange;
           RFM_Leds[10] = CRGB::DarkOrange;
           RFM_Leds[21] = CRGB::DarkOrange;
           RFM_Leds[26] = CRGB::DarkOrange;
           RFM_Leds[4]  = CRGB::DarkOrange;
           RFM_Leds[11] = CRGB::DarkOrange;
           RFM_Leds[20] = CRGB::DarkOrange;
           RFM_Leds[27] = CRGB::DarkOrange;
           NoLED();
           FastLED.show();
           RV = 2; 
          }
           else if (RV == 2){
           RFM_Leds[3]  = CRGB::DarkOrange;
           RFM_Leds[12] = CRGB::DarkOrange;
           RFM_Leds[19] = CRGB::DarkOrange;
           RFM_Leds[28] = CRGB::DarkOrange;
           RFM_Leds[2]  = CRGB::DarkOrange;
           RFM_Leds[13] = CRGB::DarkOrange;
           RFM_Leds[18] = CRGB::DarkOrange;
           RFM_Leds[29] = CRGB::DarkOrange;
           NoLED();
           FastLED.show();
           RV = 0; 
          }
          else if (RV == 0){
             RFM_Leds[1]  = CRGB::DarkOrange;
             RFM_Leds[14] = CRGB::DarkOrange;
             RFM_Leds[17] = CRGB::DarkOrange;
             RFM_Leds[30] = CRGB::DarkOrange;
             RFM_Leds[0]  = CRGB::DarkOrange;
             RFM_Leds[15] = CRGB::DarkOrange;
             RFM_Leds[16] = CRGB::DarkOrange;
             RFM_Leds[31] = CRGB::DarkOrange;
             NoLED();
             FastLED.show();
             RV = 36; 
          }
          else if (RV == 36){
           RFM_Leds[32] = CRGB::DarkOrange;
           RFM_Leds[39] = CRGB::DarkOrange;
           RFM_Leds[33] = CRGB::DarkOrange;
           RFM_Leds[38] = CRGB::DarkOrange;
           NoLED();
           FastLED.show();
           RV = 38; 
          }
          else if (RV == 38){
           RFM_Leds[34] = CRGB::DarkOrange;
           RFM_Leds[37] = CRGB::DarkOrange;
           RFM_Leds[35] = CRGB::DarkOrange;
           RFM_Leds[36] = CRGB::DarkOrange;
           NoLED();
           FastLED.show();
           RV = 40; 
          }    
       }
     }
   }  
}

void KnipperLichtLAchterNEW(int st,int en, int fade){
    if ( chronoF.hasPassed(100) ) { // returns true if it passed 100 ms since it was started
     chronoD.restart(); // restart the crono so that it triggers again later
    //   Serial.println ("knipperlicht L Fuction");

      if (LA == 100){
        for(int dot = st; dot < en; dot++) { 
           leds[dot] = CRGB::Black;
        } 

        leds[48]  = CRGB::DarkOrange;
        leds[49]  = CRGB::DarkOrange;
        leds[50]  = CRGB::DarkOrange;
        leds[51]  = CRGB::DarkOrange;
        leds[52]  = CRGB::DarkOrange;
        leds[53]  = CRGB::DarkOrange;
        leds[59]  = CRGB::DarkOrange;
        leds[60]  = CRGB::DarkOrange;
        leds[61]  = CRGB::DarkOrange;
        leds[68]  = CRGB::DarkOrange;
        leds[69]  = CRGB::DarkOrange;
        leds[70]  = CRGB::DarkOrange;
        leds[81]  = CRGB::DarkOrange;
        leds[82]  = CRGB::DarkOrange;
        leds[83]  = CRGB::DarkOrange;
        leds[88]  = CRGB::DarkOrange;
        leds[89]  = CRGB::DarkOrange;
        leds[90]  = CRGB::DarkOrange;

        FastLED.show();
        LA = 4;
      }
      else if (LA != 100){
        if (LA >= en){ 
           
        LA = 100;
        }
        else {
          if (LA == 4){
           leds[46]  = CRGB::DarkOrange;
           leds[47]  = CRGB::DarkOrange;
           leds[54]  = CRGB::DarkOrange;
           leds[55]  = CRGB::DarkOrange;
           leds[56]  = CRGB::DarkOrange;
           leds[62]  = CRGB::DarkOrange;
           leds[63]  = CRGB::DarkOrange;
           leds[64]  = CRGB::DarkOrange;
           leds[71]  = CRGB::DarkOrange;
           leds[72]  = CRGB::DarkOrange;
           leds[73]  = CRGB::DarkOrange;
           leds[84]  = CRGB::DarkOrange;
           leds[85]  = CRGB::DarkOrange;
           leds[86]  = CRGB::DarkOrange;
           leds[91]  = CRGB::DarkOrange;

           FastLED.show();
           LA = 2; 
          }
           else if (LA == 2){

           leds[57]  = CRGB::DarkOrange;
           leds[58]  = CRGB::DarkOrange;
           leds[65]  = CRGB::DarkOrange;
           leds[66]  = CRGB::DarkOrange;
           leds[67]  = CRGB::DarkOrange;
           leds[74]  = CRGB::DarkOrange;
           leds[75]  = CRGB::DarkOrange;
           leds[76]  = CRGB::DarkOrange;
           leds[87]  = CRGB::DarkOrange;

           FastLED.show();
           LA = 0; 
          }
          else if (LA == 0){

             leds[77]  = CRGB::DarkOrange;
             leds[78]  = CRGB::DarkOrange;

             FastLED.show();
             LA = 36; 
          }
          else if (LA == 36){

           leds[79]  = CRGB::DarkOrange;
           leds[80]  = CRGB::DarkOrange;

           FastLED.show();
           LA = 38; 
          }
          else if (LA == 38){

           //NoLED();
           FastLED.show();
           LA = 91; 
          }    
       }
     }
   }  
}




// Fuction that does not turn on LEDS that are not seen needed for DRL
void NoLED(){
 LFM_Leds[24] = CRGB::Black;
  LFM_Leds[7] = CRGB::Black;
  RFM_Leds[24] = CRGB::Black;
  RFM_Leds[7] = CRGB::Black;
  
}

// do not uses ;-)
void Police(int st,int en, int fade ){
  //FastLED.clear(); 
  delay(140);
  for(int dot = st; dot < en; dot++) { 
              LFM_Leds[dot] = CRGB::Blue;
              LFM_Leds[dot].fadeToBlackBy(fade);
              RFM_Leds[dot] = CRGB::Black;
              RFM_Leds[dot].fadeToBlackBy(fade);
              NoLED();
  }
  FastLED.show();
  delay(140);
  for(int dot = st; dot < en; dot++) { 
            LFM_Leds[dot] = CRGB::Black;
             LFM_Leds[dot].fadeToBlackBy(fade);
             RFM_Leds[dot] = CRGB::Blue;
             RFM_Leds[dot].fadeToBlackBy(fade);
  }
  FastLED.show();
        
}
// if taillights needs to be turned off
void AchterLichtOFF(int st,int en){
  //FastLED.clear(); 
  for(int dot = st; dot < en; dot++) { 
            leds[dot] = CRGB::Black;
            
        }
        FastLED.show();
}

// KnipperLicht voledig ( nog aanpassen heeft nog delay) 
void KnipperLicht(int st,int en, int fade ){
  
  for(int dot = st; dot < en; dot++) { 
      leds[dot] = CRGB::DarkOrange;
      leds[dot].fadeToBlackBy(fade);    
  }
  FastLED.show();
  delay(400);
  
  for(int dot = st; dot < en; dot++) { 
      leds[dot] = CRGB::Black;
      leds[dot].fadeToBlackBy(fade);
  }
  FastLED.show();
  
  delay(400);
        
}
