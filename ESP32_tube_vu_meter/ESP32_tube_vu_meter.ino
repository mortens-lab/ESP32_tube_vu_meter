// *********************************************************************************************************************************************
//
//
// VU meter and LED effects code for ESP32 WROOM module
// @morten's lab 2021
// 
// Please link, like,comment and subscribe here : https://www.youtube.com/user/moffe1234
//
// *********************************************************************************************************************************************

#include <Adafruit_NeoPixel.h>
#include "water_torture.h"

#define PIN 18
#define N_PIXELS  59
#define BG 0
#define COLOR_ORDER RBG  // Try mixing up the letters (RGB, GBR, BRG, etc) for a whole new world of color combinations
#define BRIGHTNESS 255   // 0-255, higher number is brighter.
#define LED_TYPE WS2812B
#define MIC_PIN   15  // Microphone is attached to this analog pin
#define DC_OFFSET 1250 // DC offset in mic signal - if unusure, leave 0
#define NOISE     150  // Noise/hum/interference in mic signal
#define SAMPLES   60  // Length of buffer for dynamic level adjustment
#define TOP       (N_PIXELS + 2) // Allow dot to go slightly off scale
#define TOP2      (N_PIXELS + 1) // Allow dot to go slightly off scale
#define LAST_PIXEL_OFFSET N_PIXELS-1
#define PEAK_FALL 20  // Rate of peak falling dot
#define PEAK_FALL_MILLIS 10  // Rate of peak falling dot
#define N_PIXELS_HALF (N_PIXELS/2)
#define GRAVITY           -9.81              // Downward (negative) acceleration of gravity in m/s^2
#define h0                1                  // Starting height, in meters, of the ball (strip length)
#define NUM_BALLS         3                  // Number of bouncing balls you want (recommend < 7, but 20 is fun in its own way)
#define SPEED .20       // Amount to increment RGB color by each cycle

// Modes
enum 
{
} MODE;
bool reverse2 = true;
int BRIGHTNESS_MAX = 255;
int brightness = 120;


// cycle variables
int CYCLE_MIN_MILLIS = 2;
int CYCLE_MAX_MILLIS = 1000;
int cycleMillis = 20;
bool paused = false;
long lastTime = 0;
bool boring = true;
bool gReverseDirection = false;

float
  greenOffset = 30,
  blueOffset = 150;
 
byte
  peak      = 0,      // Used for falling dot
  dotCount  = 0,      // Frame counter for delaying dot-falling speed
  volCount  = 0;      // Frame counter for storing past volume data
int
  vol[SAMPLES],       // Collection of prior volume samples
  lvl       = 10,      // Current "dampened" audio level
  minLvlAvg = 0,      // For dynamic adjustment of graph low & high
  maxLvlAvg = 512;
 
int brightnessValue, prevBrightnessValue;
int sensorDeviationBrightness = 1;
int sensitivityValue = 228;                               // 0 - 255, initial value (value read from the potentiometer if useSensorValues = true)
int maxSensitivity = 2 * 255;                             // let the 'volume' go up to 200%!
int ledBrightness = 255;                                   // 0 - 255, initial value (value read from the potentiometer if useSensorValues = true)
int val;

int time_cnt=0;
int x=0;
int global_j=0;


Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_PIXELS, PIN, NEO_GRB + NEO_KHZ800);

// Water torture
WaterTorture water_torture = WaterTorture(&strip);

 
// FOR SYLON ETC
uint8_t thisbeat =  23;
uint8_t thatbeat =  28;
uint8_t thisfade =   2;                                     // How quickly does it fade? Lower = slower fade rate.
uint8_t thissat = 255;                                     // The saturation, where 255 = brilliant colours.
uint8_t thisbri = 255; 
 
//FOR JUGGLE
uint8_t numdots = 4;                                          // Number of dots in use.
uint8_t faderate = 2;                                         // How long should the trails be. Very low value = longer trails.
uint8_t hueinc = 16;                                          // Incremental change in hue between each dot.
uint8_t thishue = 0;                                          // Starting hue.
uint8_t curhue = 0; 
uint8_t thisbright = 255;                                     // How bright should the LED/display be.
uint8_t basebeat = 5; 
uint8_t max_bright = 255; 
 
// Twinkle
float redStates[N_PIXELS];
float blueStates[N_PIXELS];
float greenStates[N_PIXELS];
float Fade = 0.96;
 
unsigned int sample;
int          myhue = 0;

int buttonPushCounter=0;
 
//Ripple variables
int color;
int center = 0;
int step = -1;
int maxSteps = 8;
float fadeRate = 0.80;
int diff;
 
//background color
uint32_t currentBg = random(256);
uint32_t nextBg = currentBg;
 

//*******************************************************************************************************************
//
// TOUCH STUFF
//
//*******************************************************************************************************************

int mode_keycnt=0;
int bright_keycnt=20;
int onoff_keycnt=0;

int mode_lowdet=0;
int brightUP_lowdet=0;
int brightDN_lowdet=0;
int onoff_lowdet=0;
int onoff=-1;

const int TOUCH_NUM_READINGS = 3; // number of positive read for switch on
int touchReadIndex = 0;

const int touchPin_MO = 12; // MODE            T5
const int touchPin_ON = 13; // ON/OFF          T4 
const int touchPin_UP = 32; // BRIGHTNESS UP   T9
const int touchPin_DN = 14; // BRIGHTNESS DN   T6

const int ledPin_1 = 22; // Led connected to GPIO 22

// variable for storing the touch pin value 
int touchValue_ON;
int touchValue_MO;
int touchValue_UP;
int touchValue_DN;

volatile int interruptCounter;
int totalInterruptCounter;
 
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
 
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup()
{
  pinMode(touchPin_ON, INPUT);
  pinMode(touchPin_MO, INPUT);
  pinMode(touchPin_DN, INPUT);
  pinMode(touchPin_UP, INPUT);
 
  Serial.begin(115200);

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 50000, true);
  timerAlarmEnable(timer);

  delay(2000); // give me time to bring up serial monitor

  // initialize the LED pins as an output:
  pinMode (ledPin_1, OUTPUT);
  digitalWrite(ledPin_1, HIGH);

//*******************************************************************************************************************
// VU meter
//*******************************************************************************************************************
 
  strip.begin();
 // strip.setPixelColor(1,strip.Color(255, 0,0));
 // strip.setPixelColor(2,strip.Color(255, 0,0));
  strip.show(); // Initialize all pixels to 'off'  
    
  //analogReference(EXTERNAL);
  memset(vol, 0, sizeof(vol));
 
  // colorWipe(strip.Color(255, 0, 0), 20); // A Black
  delay(2000);

  colorWipe(strip.Color(0, 0, 0), 0); 
  delay(200);

  colorWipe(strip.Color(255, 0, 0), 0);
  delay(100);

  colorWipe(strip.Color(0, 0, 0), 0); 
  delay(200);

  colorWipe(strip.Color(255, 0, 0), 0); 
  delay(100);

  colorWipe(strip.Color(0, 0, 0), 0); 
  delay(200);

  colorWipe(strip.Color(255, 0, 0), 0); 
  delay(100);

  colorWipe(strip.Color(0, 0, 0), 0); 
  delay(100);

  touchAttachInterrupt(T4, gotTouch, 20);
  touchAttachInterrupt(T5, gotTouch, 20);
  touchAttachInterrupt(T6, gotTouch, 20);
  touchAttachInterrupt(T9, gotTouch, 20);

  Serial.println("morten's lab was here in 2021 :-)");
  Serial.println("enjoy!");
  Serial.println("ready .... \n");
}


void gotTouch(){
  Serial.println("Touched\n");
  x=1;
}


void loop()
{
  global_j++;
  if (global_j>255) global_j=0; 
  if (interruptCounter > 0) 
  {
    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);
 
    totalInterruptCounter++;
 
    int touchValue_MO = runningMedian(touchPin_MO);   
    int touchValue_ON = runningMedian(touchPin_ON);   
    int touchValue_DN = runningMedian(touchPin_DN);   
    int touchValue_UP = runningMedian(touchPin_UP);   

//*************************************************************************************
//
// ON / OFF
//
//*************************************************************************************

    if (touchValue_ON>20 && onoff_lowdet==1)
    {
      onoff_keycnt=onoff_keycnt+1;
      onoff_lowdet=0; 
      if (onoff_keycnt>10) { onoff_keycnt=1; }
      Serial.println(" ONOFF keypressed");
      onoff=~onoff;
      Serial.println(onoff);
    }

    if (touchValue_ON<20)  
    { 
       onoff_lowdet=1;
    } 


    if (onoff==-1)
    {
       setAll(0,0,0);  
       buttonPushCounter=0;
       mode_keycnt=0;
    }
 
    if (onoff==0)
    {
   

//*************************************************************************************
//
// MODE
//
//*************************************************************************************

     
    if (touchValue_MO>20 && mode_lowdet==1)
    {
      mode_keycnt=mode_keycnt+1;
      mode_lowdet=0; 
      if (mode_keycnt>16) { mode_keycnt=1; }
      Serial.println(" MODE keypressed");
      Serial.println(mode_keycnt);
    }

    if (touchValue_MO<20)  
    { 
      mode_lowdet=1;
    } 
  

//*************************************************************************************
//
// BRIGHT UP
//
//*************************************************************************************

  if (touchValue_UP>20 && brightUP_lowdet==1)
    {
      bright_keycnt=bright_keycnt+1;
      brightUP_lowdet=0; 
      if (bright_keycnt>20) { bright_keycnt=1; }
      Serial.println(" BRIGHT keypressed");
      Serial.println(bright_keycnt);
    }

    if (touchValue_UP<20)  
    { 
      brightUP_lowdet=1;
    } 
  
//*************************************************************************************
//
// BRIGHT DN
//
//*************************************************************************************

  if (touchValue_DN>20 && brightDN_lowdet==1)
    {
      bright_keycnt=bright_keycnt-1;
      brightDN_lowdet=0; 
      if (bright_keycnt<1) { bright_keycnt=20; }
      Serial.println(" BRIGHT keypressed");
      Serial.println(bright_keycnt);
    }

    if (touchValue_DN<20)  
    { 
      brightDN_lowdet=1;
    } 
    
//*************************************************************************************

     }
   }

   if (onoff==0)
   {
     if (brightDN_lowdet==1 || brightUP_lowdet==1 || mode_lowdet==1)
       digitalWrite(ledPin_1, HIGH);
     else
       digitalWrite(ledPin_1, LOW);
    }
    else
      digitalWrite(ledPin_1, HIGH);


// *****************************************************************************************************************************
//
// VU meter stuff main loop
//
// *****************************************************************************************************************************


    brightnessValue = map(bright_keycnt, 1, 20, 0, 255);
    strip.setBrightness(brightnessValue);
  
    //for mic
    uint8_t  i;
    uint16_t minLvl, maxLvl;
    int      n, height;
    // end mic
 
    buttonPushCounter=mode_keycnt;
 
    switch (buttonPushCounter){

    case 0:  // STANDBY
      buttonPushCounter==0; {
      break;}
           
    case 1:  // STANDARD RAINBOW
      buttonPushCounter==1; {
      vu1();
      break;}
       
    case 2:  // STANDARD CENTER
      buttonPushCounter==2; {
      vu2();
      break;}
      
    case 3:  // STANDARD CHANGING COLOR
      buttonPushCounter==3; {
      Vu3();  
      break;}
           
    case 4:  // COLOR CENTER CHANGING  
      buttonPushCounter==4; {
      Vu4();  
      break;}  
      
    case 5:  // SHOOTING START
      buttonPushCounter==5; {
      Vu5();
      break;}
       
    case 6:  // FALLING START
      buttonPushCounter==6; {
      Vu6();
      break;}
       
    case 7:  // SNOW FLAKES  
      buttonPushCounter==7; {
      SnowSparkle(0x10, 0x10, 0x10, 20, random(100,1000));
      break;}
       
    case 8:  // RIPPLE 
      buttonPushCounter==8; {
      ripple2();
      break;}
       
    case 9:  // RIPPLE WITH BACKGROUND
      buttonPushCounter==9; {
      ripple();
      break;}
      
    case 10:  // TWINKLE
      buttonPushCounter==10; {
      Twinkle();
      break;}

    case 11:  // TWINKLE RANDOM COLORS
      buttonPushCounter==11; {
      TwinkleRandom(40, 100, false);
      break;}

    case 12:  // FIRE 
      buttonPushCounter==12; {
      Fire(55,120,15);
      break;}

   case 13:  // RAINBOW
      buttonPushCounter==13; {
      rainbow(20);
      break;}
     
    case 14: // WATER DROPS  
      buttonPushCounter==14; {
      Drip();
      break;}
       
    case 15:  // BOUNCING BALLS
      buttonPushCounter==15; {
      Ball(0xff,0,0,3); // 
      break;}    
       
    case 16:  // ALL WHITE
      buttonPushCounter==16; {
      setAll(255,255,255);
      delay(50);
      break;}
  }
}


int runningMedian(int pin) 
{
  const int sampleSize = 3;
  int samples[sampleSize];
  int temp;
  for (int i = 0; i < sampleSize; i++) 
  {
    samples[i] = touchRead(pin);
  }
  for (int i = 0; i < sampleSize - 1; i++) 
  {
    for (int j = 0; j < sampleSize - 1 - i; j++) 
    {
      if (samples[j] < samples[j + 1]) 
      {
        temp = samples[j];
        samples[j] = samples[j + 1];
        samples[j + 1] = samples[j];
      }
    }
  }
  return samples[sampleSize / 2 + 1];
}


//*******************************************************************************************************************************
//
//
// VU meter functions
//
//
//*******************************************************************************************************************************



void TwinkleRandom(int Count, int SpeedDelay, boolean OnlyOne) 
{
  time_cnt++;
  delay(2);
  if (time_cnt>100)
  {
    setPixel(random(N_PIXELS),random(0,255),random(0,255),random(0,255));
    strip.show();
    time_cnt=0;
  }
}



void SnowSparkle(byte red, byte green, byte blue, int SparkleDelay, int SpeedDelay) {

  setAll(red,green,blue);

  int Pixel = random(N_PIXELS);

  setPixel(Pixel,0xff,0xff,0xff);

  strip.show();

  delay(SparkleDelay);

  setPixel(Pixel,red,green,blue);

  strip.show();

  delay(SpeedDelay);

}

void Fire(int Cooling, int Sparking, int SpeedDelay) {

  static byte heat[N_PIXELS];

  int cooldown;
  
  for( int i = 0; i < N_PIXELS; i++) {
    cooldown = random(0, ((Cooling * 10) / N_PIXELS) + 2);
    if(cooldown>heat[i]) {
    heat[i]=0;
    } else {
    heat[i]=heat[i]-cooldown;
}}

  for( int k= N_PIXELS - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
}


  if( random(255) < Sparking ) {
    int y = random(7);
    heat[y] = heat[y] + random(160,255);
    //heat[y] = random(160,255);
}


  for( int j = 0; j < N_PIXELS; j++) {
    setPixelHeatColor(j, heat[j] );
}

strip.show();
delay(SpeedDelay);

}

void setPixel(int Pixel, byte red, byte green, byte blue) {
    strip.setPixelColor(Pixel, strip.Color(red, green, blue));

}


void setPixelHeatColor (int Pixel, byte temperature) {


  byte t192 = round((temperature/255.0)*191);

  byte heatramp = t192 & 0x3F; // 0..63
  heatramp <<= 2; // scale up to 0..252


  if( t192 > 0x80) {                     // hottest
  setPixel(Pixel, 255, 255, heatramp);

  } else if( t192 > 0x40 ) {             // middle
    setPixel(Pixel, 255, heatramp, 0);

  } else {                               // coolest
    setPixel(Pixel, heatramp, 0, 0);

  }
}


void Drip()
{
  MODE_WATER_TORTURE: 
  if (cycle())
  {
    strip.setBrightness(255); // off limits
    water_torture.animate(reverse2);
    strip.show();
  //strip.setBrightness(brightness); // back to limited
  }
}

      
bool cycle()
{
  if (paused)
  {
    return false;
  }
  
  if (millis() - lastTime >= cycleMillis)
  {
    lastTime = millis();
    return true;
  }
  return false;
}


void Vu5()
{
  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int      n, height;

  val=513;
  val= map(val, 0, 4095, -10, 6);

  n   = analogRead(MIC_PIN);                        // Raw reading from mic 
  n   = abs(n - 0 - DC_OFFSET); // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE);             // Remove noise/hum

 if(val<0){
       n=n/(val*(-1));
        }
       if(val>0){
        n=n*val;
        }
  
  lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP2 * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if(height < 0L)       height = 0;      // Clip output
  else if(height > TOP2) height = TOP2;
  if(height > peak)     peak   = height; // Keep 'peak' dot at top

#ifdef CENTERED
 // Color pixels based on rainbow gradient
  for(i=0; i<(N_PIXELS/2); i++) {
    if(((N_PIXELS/2)+i) >= height)
    {
      strip.setPixelColor(((N_PIXELS/2) + i),   0,   0, 0);
      strip.setPixelColor(((N_PIXELS/2) - i),   0,   0, 0);
    }
    else
    {
      strip.setPixelColor(((N_PIXELS/2) + i),Wheel(map(((N_PIXELS/2) + i),0,strip.numPixels()-1,30,150)));
      strip.setPixelColor(((N_PIXELS/2) - i),Wheel(map(((N_PIXELS/2) - i),0,strip.numPixels()-1,30,150)));
    }
  }
  
  // Draw peak dot  
  if(peak > 0 && peak <= LAST_PIXEL_OFFSET)
  {
    strip.setPixelColor(((N_PIXELS/2) + peak),255,255,255); // (peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
    strip.setPixelColor(((N_PIXELS/2) - peak),255,255,255); // (peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
  }
#else
  // Color pixels based on rainbow gradient
  for(i=0; i<N_PIXELS; i++)
  {
    if(i >= height)
    {
      strip.setPixelColor(i,   0,   0, 0);
    }
    else
    {
      strip.setPixelColor(i,Wheel(map(i,0,strip.numPixels()-1,30,150)));
    }
  }

  // Draw peak dot  
  if(peak > 0 && peak <= LAST_PIXEL_OFFSET)
  {
    strip.setPixelColor(peak,255,255,255); // (peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
  }
  
#endif  

  // Every few frames, make the peak pixel drop by 1:

  if (millis() - lastTime >= PEAK_FALL_MILLIS)
  {
    lastTime = millis();

    strip.show(); // Update strip

    //fall rate 
    if(peak > 0) peak--;
    }

  vol[volCount] = n;                      // Save sample for dynamic leveling
  if(++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter

  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for(i=1; i<SAMPLES; i++)
  {
    if(vol[i] < minLvl)      minLvl = vol[i];
    else if(vol[i] > maxLvl) maxLvl = vol[i];
  }
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if((maxLvl - minLvl) < TOP2) maxLvl = minLvl + TOP2;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}

void Vu6()
{
  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int      n, height;

  val=513;
  val= map(val, 0, 4095, -10, 6);

  n   = analogRead(MIC_PIN);                        // Raw reading from mic 
  n   = abs(n - 9 - DC_OFFSET); // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE);             // Remove noise/hum

   if(val<0){
        n=n/(val*(-1));
        }
       if(val>0){
        n=n*val;
        }
  
  
  lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)

  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP2 * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);

  if(height < 0L)       height = 0;      // Clip output
  else if(height > TOP2) height = TOP2;
  if(height > peak)     peak   = height; // Keep 'peak' dot at top


#ifdef CENTERED
  // Draw peak dot  
  if(peak > 0 && peak <= LAST_PIXEL_OFFSET)
  {
    strip.setPixelColor(((N_PIXELS/2) + peak),255,255,255); // (peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
    strip.setPixelColor(((N_PIXELS/2) - peak),255,255,255); // (peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
  }
#else
  // Color pixels based on rainbow gradient
  for(i=0; i<N_PIXELS; i++)
  {
    if(i >= height)
    {
      strip.setPixelColor(i,   0,   0, 0);
    }
    else
    {
     }
  }

  // Draw peak dot  
  if(peak > 0 && peak <= LAST_PIXEL_OFFSET)
  {
    strip.setPixelColor(peak,0,0,255); // (peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
  }
  
#endif  

  // Every few frames, make the peak pixel drop by 1:

  if (millis() - lastTime >= PEAK_FALL_MILLIS)
  {
    lastTime = millis();

    strip.show(); // Update strip

    //fall rate 
    if(peak > 0) peak--;
    }

  vol[volCount] = n;                      // Save sample for dynamic leveling
  if(++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter

  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for(i=1; i<SAMPLES; i++)
  {
    if(vol[i] < minLvl)      minLvl = vol[i];
    else if(vol[i] > maxLvl) maxLvl = vol[i];
  }
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if((maxLvl - minLvl) < TOP2) maxLvl = minLvl + TOP2;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}


void Ball(byte red, byte green, byte blue, int BallCount) {
  float Gravity = -9.81;
  int StartHeight = 1;
 
  float Height[BallCount];
  float ImpactVelocityStart = sqrt( -2 * Gravity * StartHeight );
  float ImpactVelocity[BallCount];
  float TimeSinceLastBounce[BallCount];
  int   Position[BallCount];
  long  ClockTimeSinceLastBounce[BallCount];
  float Dampening[BallCount];
 
  for (int i = 0 ; i < BallCount ; i++) {  
    ClockTimeSinceLastBounce[i] = millis();
    Height[i] = StartHeight;
    Position[i] = 0;
    ImpactVelocity[i] = ImpactVelocityStart;
    TimeSinceLastBounce[i] = 0;
    Dampening[i] = 0.90 - float(i)/pow(BallCount,2);
  }

  while (true) {
    for (int i = 0 ; i < BallCount ; i++) {
      TimeSinceLastBounce[i] =  millis() - ClockTimeSinceLastBounce[i];
      Height[i] = 0.5 * Gravity * pow( TimeSinceLastBounce[i]/1000 , 2.0 ) + ImpactVelocity[i] * TimeSinceLastBounce[i]/1000;
 
      if ( Height[i] < 0 ) {                      
        Height[i] = 0;
        ImpactVelocity[i] = Dampening[i] * ImpactVelocity[i];
        ClockTimeSinceLastBounce[i] = millis();
 
        if ( ImpactVelocity[i] < 0.01 ) {
          ImpactVelocity[i] = ImpactVelocityStart;
        }
      }
      Position[i] = round( Height[i] * (N_PIXELS - 1) / StartHeight);
    }
 
    for (int i = 0 ; i < BallCount ; i++) {
      //setPixel(,red,green,blue);
      if (i==0) strip.setPixelColor(Position[i], strip.Color(255, 0,0));   // Red
      if (i==1) strip.setPixelColor(Position[i], strip.Color(255, 255,0)); // Yellow
      if (i==2) strip.setPixelColor(Position[i], strip.Color(0, 0,255));   // Blue
    }
    strip.show();
    setAll(0,0,0);
     if (x==1)
     { 
       x=0;
       return; 
     }       
  }
}


void setAll(byte red, byte green, byte blue) {
  for(int i = 0; i < N_PIXELS; i++ ) {
    strip.setPixelColor(i, strip.Color(red, green, blue));
  }
  strip.show();
}


void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      if (x==1)
      { 
        x=0;
        return; 
      }        
      delay(wait);
  }
}


void Vu4() {
  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int      n, height;

  
//  val = (analogRead(potPin));  
  val=513;
  val= map(val, 0, 4095, -10, 6);
  
  n   = analogRead(MIC_PIN);                        // Raw reading from mic 
  n   = abs(n - 0 - DC_OFFSET); // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE);             // Remove noise/hum
  
    if(val<0){
        n=n/(val*(-1));
        }
       if(val>0){
        n=n*val;
        }
  
  lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)
 
  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);
 
  if(height < 0L)       height = 0;      // Clip output
  else if(height > TOP) height = TOP;
  if(height > peak)     peak   = height; // Keep 'peak' dot at top
  greenOffset += SPEED;
  blueOffset += SPEED;
  if (greenOffset >= 255) greenOffset = 0;
  if (blueOffset >= 255) blueOffset = 0;
 
  // Color pixels based on rainbow gradient
  for(i=0; i<N_PIXELS_HALF; i++) {
    if(i >= height) {              
      strip.setPixelColor(N_PIXELS_HALF-i-1,   0,   0, 0);
      strip.setPixelColor(N_PIXELS_HALF+i,   0,   0, 0);
    }
    else {
      uint32_t color = Wheel(map(i,0,N_PIXELS_HALF-1,(int)greenOffset, (int)blueOffset));
      strip.setPixelColor(N_PIXELS_HALF-i-1,color);
      strip.setPixelColor(N_PIXELS_HALF+i,color);
    }
  }
 
  // Draw peak dot  
  if(peak > 0 && peak <= N_PIXELS_HALF-1) {
    uint32_t color = Wheel(map(peak,0,N_PIXELS_HALF-1,30,150));
    strip.setPixelColor(N_PIXELS_HALF-peak-1,color);
    strip.setPixelColor(N_PIXELS_HALF+peak,color);
  }
  
   strip.show(); // Update strip
 
// Every few frames, make the peak pixel drop by 1:
 
    if(++dotCount >= PEAK_FALL) { //fall rate 
      
      if(peak > 0) peak--;
      dotCount = 0;
    }
 
 
  vol[volCount] = n;                      // Save sample for dynamic leveling
  if(++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter
 
  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for(i=1; i<SAMPLES; i++) {
    if(vol[i] < minLvl)      minLvl = vol[i];
    else if(vol[i] > maxLvl) maxLvl = vol[i];
  }
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
 
}
 
 
void Vu3() {
  uint8_t i;
  uint16_t minLvl, maxLvl;
  int n, height;
 
  
//  val = (analogRead(potPin));  
  val=512;
  val= map(val, 0, 4095, -10, 6);
 
  n = analogRead(MIC_PIN);             // Raw reading from mic
  n = abs(n - 0 - DC_OFFSET);        // Center on zero
  n = (n <= NOISE) ? 0 : (n - NOISE);  // Remove noise/hum
 
      if(val<0){
        n=n/(val*(-1));
        }
       if(val>0){
        n=n*val;
        }
        
  lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)
 
  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);
 
  if (height < 0L)       height = 0;      // Clip output
  else if (height > TOP) height = TOP;
  if (height > peak)     peak   = height; // Keep 'peak' dot at top
 
  greenOffset += SPEED;
  blueOffset += SPEED;
  if (greenOffset >= 255) greenOffset = 0;
  if (blueOffset >= 255) blueOffset = 0;
 
  // Color pixels based on rainbow gradient
  for (i = 0; i < N_PIXELS; i++) {
    if (i >= height) {
      strip.setPixelColor(i, 0, 0, 0);
    } else {
      strip.setPixelColor(i, Wheel(
        map(i, 0, strip.numPixels() - 1, (int)greenOffset, (int)blueOffset)
      ));
    }
  }
  // Draw peak dot  
  if(peak > 0 && peak <= N_PIXELS-1) strip.setPixelColor(peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
  
   strip.show(); // Update strip
 
// Every few frames, make the peak pixel drop by 1:
 
    if(++dotCount >= PEAK_FALL) { //fall rate 
      
      if(peak > 0) peak--;
      dotCount = 0;
    }
  strip.show();  // Update strip
 
  vol[volCount] = n;
  if (++volCount >= SAMPLES) {
    volCount = 0;
  }
 
  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for (i = 1; i < SAMPLES; i++) {
    if (vol[i] < minLvl) {
      minLvl = vol[i];
    } else if (vol[i] > maxLvl) {
      maxLvl = vol[i];
    }
  }
 
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if ((maxLvl - minLvl) < TOP) {
    maxLvl = minLvl + TOP;
  }
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}
 
 
 
 
// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;
  int temp_key;

  temp_key=mode_keycnt;
 
  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();

    if( mode_keycnt != temp_key)
 
       return;        
      delay(wait);
        }
    }

 
void vu1() {
 
  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int      n, height;
 
//  val = (analogRead(potPin));  
  val=512;
  val= map(val, 0, 4095, -10, 6);
  
  n   = analogRead(MIC_PIN);                        // Raw reading from mic 
  n   = abs(n - 0 - DC_OFFSET); // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE);             // Remove noise/hum
     if(val<0){
        n=n/(val*(-1));
        }
       if(val>0){
        n=n*val;
        }
  lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)
 
  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);
 
  if(height < 0L)       height = 0;      // Clip output
  else if(height > TOP) height = TOP;
  if(height > peak)     peak   = height; // Keep 'peak' dot at top
 
 
  // Color pixels based on rainbow gradient
  for(i=0; i<N_PIXELS; i++) {
    if(i >= height)               strip.setPixelColor(i,   0,   0, 0);
    else strip.setPixelColor(i,Wheel(map(i,0,strip.numPixels()-1,30,150)));
  }
 
 
  // Draw peak dot  
  if(peak > 0 && peak <= N_PIXELS-1) strip.setPixelColor(peak,Wheel(map(peak,0,strip.numPixels()-1,30,150)));
  
   strip.show(); // Update strip
 
// Every few frames, make the peak pixel drop by 1:
 
    if(++dotCount >= PEAK_FALL) { //fall rate 
      
      if(peak > 0) peak--;
      dotCount = 0;
    }
 
  vol[volCount] = n;                      // Save sample for dynamic leveling
  if(++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter
 
  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for(i=1; i<SAMPLES; i++) {
    if(vol[i] < minLvl)      minLvl = vol[i];
    else if(vol[i] > maxLvl) maxLvl = vol[i];
  }
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
 
}
 
// Input a value 0 to 255 to get a color value.
// The colors are a transition r - g - b - back to r.


uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
 
 
void vu2() {
  
  uint8_t  i;
  uint16_t minLvl, maxLvl;
  int      n, height;
 
//  val = (analogRead(potPin));  
  val= 1351;
  val= map(val, 0, 4095, -10, 6);
  n   = analogRead(MIC_PIN);                        // Raw reading from mic 
  n   = abs(n - 0 - DC_OFFSET); // Center on zero
  n   = (n <= NOISE) ? 0 : (n - NOISE);             // Remove noise/hum
 
      if(val<0){
        n=n/(val*(-1));
        }
       if(val>0){
        n=n*val;
        }
  lvl = ((lvl * 7) + n) >> 3;    // "Dampened" reading (else looks twitchy)
 
  // Calculate bar height based on dynamic min/max levels (fixed point):
  height = TOP * (lvl - minLvlAvg) / (long)(maxLvlAvg - minLvlAvg);
 
  if(height < 0L)       height = 0;      // Clip output
  else if(height > TOP) height = TOP;
  if(height > peak)     peak   = height; // Keep 'peak' dot at top
 
  // Color pixels based on rainbow gradient
  for(i=0; i<N_PIXELS_HALF; i++) {
    if(i >= height) {              
      strip.setPixelColor(N_PIXELS_HALF-i-1,   0,   0, 0);
      strip.setPixelColor(N_PIXELS_HALF+i,   0,   0, 0);
    }
    else {
      uint32_t color = Wheel(map(i,0,N_PIXELS_HALF-1,30,150));
      strip.setPixelColor(N_PIXELS_HALF-i-1,color);
      strip.setPixelColor(N_PIXELS_HALF+i,color);
    }
  }
 
  // Draw peak dot  
  if(peak > 0 && peak <= N_PIXELS_HALF-1) {
    uint32_t color = Wheel(map(peak,0,N_PIXELS_HALF-1,30,150));
    strip.setPixelColor(N_PIXELS_HALF-peak-1,color);
    strip.setPixelColor(N_PIXELS_HALF+peak,color);
  }
  
   strip.show(); // Update strip
 
// Every few frames, make the peak pixel drop by 1:
 
    if(++dotCount >= PEAK_FALL) { //fall rate 
      
      if(peak > 0) peak--;
      dotCount = 0;
    }

  vol[volCount] = n;                      // Save sample for dynamic leveling
  if(++volCount >= SAMPLES) volCount = 0; // Advance/rollover sample counter
 
  // Get volume range of prior frames
  minLvl = maxLvl = vol[0];
  for(i=1; i<SAMPLES; i++) {
    if(vol[i] < minLvl)      minLvl = vol[i];
    else if(vol[i] > maxLvl) maxLvl = vol[i];
  }
  // minLvl and maxLvl indicate the volume range over prior frames, used
  // for vertically scaling the output graph (so it looks interesting
  // regardless of volume level).  If they're too close together though
  // (e.g. at very low volume levels) the graph becomes super coarse
  // and 'jumpy'...so keep some minimum distance between them (this
  // also lets the graph go to zero when no sound is playing):
  if((maxLvl - minLvl) < TOP) maxLvl = minLvl + TOP;
  minLvlAvg = (minLvlAvg * 63 + minLvl) >> 6; // Dampen min/max levels
  maxLvlAvg = (maxLvlAvg * 63 + maxLvl) >> 6; // (fake rolling average)
}
 
 
void ripple() {

    if (currentBg == nextBg) {
      nextBg = random(256);
    }
    else if (nextBg > currentBg) {
      currentBg++;
    } else {
      currentBg--;
    }
    for(uint16_t l = 0; l < N_PIXELS; l++) {
     strip.setPixelColor(l, Wheel(currentBg, 0.1));
    }
 
  if (step == -1) {
    center = random(N_PIXELS);
    color = random(256);
    step = 0;
  }
 
  if (step == 0) {
    strip.setPixelColor(center, Wheel(color, 1));
    step ++;
  }
  else {
    if (step < maxSteps) {
      Serial.println(pow(fadeRate,step));
 
      strip.setPixelColor(wrap(center + step), Wheel(color, pow(fadeRate, step)));
      strip.setPixelColor(wrap(center - step), Wheel(color, pow(fadeRate, step)));
      if (step > 3) {
        strip.setPixelColor(wrap(center + step - 3), Wheel(color, pow(fadeRate, step - 2)));
        strip.setPixelColor(wrap(center - step + 3), Wheel(color, pow(fadeRate, step - 2)));
      }
      step ++;
    }
    else {
      step = -1;
    }
  }
  strip.show();
  delay(25);
}
 
 
int wrap(int step) {
  if(step < 0) return N_PIXELS + step;
  if(step > N_PIXELS - 1) return step - N_PIXELS;
  return step;
}
 

 
void ripple2() {
  if (BG){
    if (currentBg == nextBg) {
      nextBg = random(256);
    } 
    else if (nextBg > currentBg) {
      currentBg++;
    } else {
      currentBg--;
    }
    for(uint16_t l = 0; l < N_PIXELS; l++) {
      strip.setPixelColor(l, Wheel(currentBg, 0.1));
    }
  } else {
    for(uint16_t l = 0; l < N_PIXELS; l++) {
      strip.setPixelColor(l, 0, 0, 0);
    }
  }
 
  if (step == -1) {
    center = random(N_PIXELS);
    color = random(256);
    step = 0;
  }
 
  if (step == 0) {
    strip.setPixelColor(center, Wheel(color, 1));
    step ++;
  } 
  else {
    if (step < maxSteps) {
      strip.setPixelColor(wrap(center + step), Wheel(color, pow(fadeRate, step)));
      strip.setPixelColor(wrap(center - step), Wheel(color, pow(fadeRate, step)));
      if (step > 3) {
        strip.setPixelColor(wrap(center + step - 3), Wheel(color, pow(fadeRate, step - 2)));
        strip.setPixelColor(wrap(center - step + 3), Wheel(color, pow(fadeRate, step - 2)));
      }
      step ++;
    } 
    else {
      step = -1;
    }
  }
  
  strip.show();
  delay(25);
}
 
 
 
// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos, float opacity) {
  
  if(WheelPos < 85) {
    return strip.Color((WheelPos * 3) * opacity, (255 - WheelPos * 3) * opacity, 0);
  } 
  else if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color((255 - WheelPos * 3) * opacity, 0, (WheelPos * 3) * opacity);
  } 
  else {
    WheelPos -= 170;
    return strip.Color(0, (WheelPos * 3) * opacity, (255 - WheelPos * 3) * opacity);
  }
}
  
  
void Twinkle () {
   if (random(25) == 1) {
      uint16_t i = random(N_PIXELS);
      if (redStates[i] < 1 && greenStates[i] < 1 && blueStates[i] < 1) {
        redStates[i] = random(256);
        greenStates[i] = random(256);
        blueStates[i] = random(256);
      }
    }
    
    for(uint16_t l = 0; l < N_PIXELS; l++) {
      if (redStates[l] > 1 || greenStates[l] > 1 || blueStates[l] > 1) {
        strip.setPixelColor(l, redStates[l], greenStates[l], blueStates[l]);
        
        if (redStates[l] > 1) {
          redStates[l] = redStates[l] * Fade;
        } else {
          redStates[l] = 0;
        }
        
        if (greenStates[l] > 1) {
          greenStates[l] = greenStates[l] * Fade;
        } else {
          greenStates[l] = 0;
        }
        
        if (blueStates[l] > 1) {
          blueStates[l] = blueStates[l] * Fade;
        } else {
          blueStates[l] = 0;
        }
  
      } else {
        strip.setPixelColor(l, 0, 0, 0);
      }
    }
    strip.show();
    delay(10);
}
 
 
void rainbow(uint8_t wait) {
  uint16_t i, j;

 int temp_key;

  temp_key=mode_keycnt;
 
 // for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+global_j) & 255));
    }
    strip.show();
    // check if a button pressed
     if (x==1)
     { 
       x=0;
       return; 
     }        
     
     Serial.println(j);
     Serial.println(i);

     delay(wait);
    
 // }
  Serial.println("T2");
}
