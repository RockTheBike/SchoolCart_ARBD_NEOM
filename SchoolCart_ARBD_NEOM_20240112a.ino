#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>

#define VOLT_PIN        A0
#define AMPS_IN_PIN     A3      // labeled PLUSRAIL/PLUSOUT IC2
#define AMPS_OUT_PIN    A2      // labeled MINUSRAIL/MINUSOUT IC3
#define MATRIX01_PIN    11
#define MATRIX02_PIN    12
#define STRIP01_PIN     13

#define VOLTCOEFF       13.36   // convert ADC value to voltage
#define AMPS_IN_COEFF   13.05   // PLUSOUT = OUTPUT, PLUSRAIL = PEDAL INPUT
#define AMPS_IN_OFFSET  118.0   // when current sensor is at 0 amps this is the ADC value
#define AMPS_OUT_COEFF  13.05   // PLUSOUT = OUTPUT, PLUSRAIL = PEDAL INPUT
#define AMPS_OUT_OFFSET 118.0   // when current sensor is at 0 amps this is the ADC value

#define BRIGHTNESS      32
#define mh              8       // matrix height
#define mw              20      // matrix width
#define STRIP_COUNT     60      // how many LEDs

Adafruit_NeoMatrix matrix02 =  Adafruit_NeoMatrix(mw, mh, MATRIX02_PIN,  NEO_MATRIX_BOTTOM + NEO_MATRIX_RIGHT + NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG, NEO_GRB + NEO_KHZ800);
Adafruit_NeoMatrix matrix01 =  Adafruit_NeoMatrix(mw, mh, MATRIX01_PIN,  NEO_MATRIX_BOTTOM + NEO_MATRIX_RIGHT + NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip01(STRIP_COUNT, STRIP01_PIN, NEO_GRB + NEO_KHZ800);

#define LED_BLACK		      0
#define LED_RED_HIGH 		  (31 << 11)
#define LED_GREEN_HIGH 		(63 << 5)  
#define LED_BLUE_HIGH 		31
#define LED_WHITE_HIGH		(LED_RED_HIGH    + LED_GREEN_HIGH    + LED_BLUE_HIGH)

//uint16_t V_Value = 0;
//uint16_t I_Value = 0;
//uint16_t P_Value = 0;
uint32_t V_Avg = 0;
uint32_t I_Avg = 0;
uint32_t P_Avg = 0;

uint16_t FAvgReads = 50;

void setup() {
  Serial.begin(115200);
  matrix01.begin();
  matrix01.setTextWrap(false);
  matrix01.setBrightness(BRIGHTNESS);
  matrix01.clear();
  matrix02.begin();
  matrix02.setTextWrap(false);
  matrix02.setBrightness(BRIGHTNESS);
  matrix02.clear();
  strip01.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip01.show();            // Turn OFF all pixels ASAP
  strip01.setBrightness(BRIGHTNESS); // Set BRIGHTNESS to about 1/5 (max = 255)
}

void loop() {
  V_Avg = constrain(map(((constrain(analogRead(A4),0,1023) + (V_Avg * (FAvgReads-1))) / FAvgReads), 0, 999, 0, 999),0,999);
  I_Avg = constrain(map(((constrain(analogRead(A5),0,1023) + (I_Avg * (FAvgReads-1))) / FAvgReads), 0, 999, 0, 999),0,999);
  P_Avg = map(((V_Avg/10)*(I_Avg/10))/10, 0, 949, 0, 60);

  disNeostring01(intAlignRigiht(V_Avg), LED_WHITE_HIGH);
  disNeostring02(intAlignRigiht(I_Avg), LED_WHITE_HIGH);
  disNeowipe(Wheel(80), P_Avg);
  Serial.print(" V_Avg : ");
  Serial.print(V_Avg);
  Serial.print(" I_Avg : ");
  Serial.print(I_Avg);
  Serial.print(" P_Avg : ");
  Serial.println(P_Avg);
}

String intAlignRigiht(int num) {
  int cursorPos = 0;
  int    space  = 0;
  String spaces = "";

  if      (num < 10.00)                       {space=2;    cursorPos=2;} //3
  else if (num >= 10.00 && num < 100.00)      {space=1;    cursorPos=2;}
  else if (num >= 100.00 && num < 1000.00)    {space=0;    cursorPos=2;}
  else if (num >= 1000.00 && num < 10000.00)  {space=0;    cursorPos=2;}
  
  for (uint8_t s=0; s<space; s++) spaces += F(" ");
  
  return spaces+String(num);
}

void disNeostring01(String nval, uint32_t col) {
  int cursorPos = 0;
  //uint8_t dval = bval; //uint8_t dval = map(constrain(V_Value, 0, 1200), 0, 1200, 0, 255);
  matrix01.clear();
  matrix01.setCursor(3, 0);
  matrix01.setTextColor(col);
  matrix01.print(nval);
  //matrix01->drawLine(24, 7, map(dval, 0, 255, 24, 0), 7, matrix01->Color(map(dval, 0, 255, 255, 150),dval,0));
  matrix01.show();
}

void disNeostring02(String nval, uint32_t col) {
  int cursorPos = 0;
  //uint8_t dval = bval; //uint8_t dval = map(constrain(V_Value, 0, 1200), 0, 1200, 0, 255);
  matrix02.clear();
  matrix02.setCursor(3, 0);
  matrix02.setTextColor(col);
  matrix02.print(nval);
  //matrix02->drawLine(24, 7, map(dval, 0, 255, 24, 0), 7, matrix02->Color(map(dval, 0, 255, 255, 150),dval,0));
  matrix02.show();
}

void disNeowipe(uint32_t color, int pixlevel) {
  if (pixlevel >= 60) {pixlevel = 59;}
  for(int i=0; i<strip01.numPixels(); i++) { // For each pixel in strip...
    if (i <= pixlevel) {strip01.setPixelColor(i, color);}
    else {strip01.setPixelColor(i, 0);}         //  Set pixel's color (in RAM)
    //strip.show();                          //  Update strip to match
    //delay(wait);                           //  Pause for a moment
  }
  strip01.show();
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
   return strip01.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if(WheelPos < 170) {
    WheelPos -= 85;
   return strip01.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip01.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}
