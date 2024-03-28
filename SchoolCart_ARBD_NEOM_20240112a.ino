#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>

#define VOLTAGE_PROTECT   29.2 // if voltage > 29.2 open relay
#define VOLTAGE_UNPROTECT 27.2 // if relay has been opened and  voltage is <=27.2, close relay
#define VOLTAGE_SHUTDOWN  20.0 // if relay has been opened and  voltage is <=27.2, close relay
#define HYSTERESIS_WATTS        25 // how many watts above inverter consumption is considered gaining
#define TIMEOUT_UTILITYMODE     15*60*1000 // 15 minutes
#define TIMEOUT_ENERGYBANKING   15*60*1000 // 15 minutes
#define IDLE_THRESHOLD_PEDAL_WATTS      25 // below this wattage input is considered idle
#define IDLE_THRESHOLD_INVERTER_WATTS   25 // below this wattage output is considered idle
#define RED_LIGHTS_WATTAGE_FULL_BRIGHTNESS 500 // inverter wattage at which LEDs are at full brightness
#define RED_LIGHTS_WATTAGE_BLINKING        1200 // inverter wattage at which LEDs are BLINKING

#define VOLT_PIN        A0
#define AMPS_IN_PIN     A3      // labeled PLUSRAIL/PLUSOUT IC2
#define AMPS_OUT_PIN    A2      // labeled MINUSRAIL/MINUSOUT IC3
#define INVERTER_AMPS1_PIN     A4      // one of two current sensors for inverter
#define INVERTER_AMPS2_PIN     A5      // two of two current sensors for inverter
#define MATRIX01_PIN      11 // we switch between 11 and 12 in software
#define MATRIX02_PIN      12
#define PEDALOMETER_PIN     13
#define RED_LED_LIGHTS  5
#define BUTTONLEFT      6
#define SWITCHMODE      7
#define BUTTONRIGHT     8
#define RELAY_OVERPEDAL  2 // this relay disconnects pedal power input when activated
#define RELAY_INVERTERON 3 // this relay turns on the inverter by its own power switch
#define RELAY_DROPSTOP   4 // this relay connects this arbduino to the battery power

#define VOLTCOEFF       13.233   // convert ADC value to voltage
#define AMPS_IN_COEFF   11.94   // PLUSOUT = OUTPUT, PLUSRAIL = PEDAL INPUT
#define AMPS_IN_OFFSET 124.5   // when current sensor is at 0 amps this is the ADC value
#define AMPS_OUT_COEFF  11.97   // PLUSOUT = OUTPUT, PLUSRAIL = PEDAL INPUT
#define AMPS_OUT_OFFSET 122.0   // when current sensor is at 0 amps this is the ADC value
#define INVERTER_AMPS1_COEFF   12.30   // one of two current sensors for inverter
#define INVERTER_AMPS1_OFFSET 119.5
#define INVERTER_AMPS2_COEFF   12.63  // two of two current sensors for inverter
#define INVERTER_AMPS2_OFFSET 120.5

#define INTERVAL_PRINT  6000    // time between printInfo() events
#define INTERVAL_NEOPIXELS 250  // time between neopixel update events WHICH CORRUPTS millis()
#define BRIGHTNESS      20
#define MATRIX_HEIGHT   8       // matrix height
#define MATRIX_WIDTH    18      // matrix width
#define STRIP_COUNT     60      // how many LEDs
#define WATTHOURS_EEPROM_ADDRESS 20

Adafruit_NeoMatrix matrix(MATRIX_WIDTH, MATRIX_HEIGHT, MATRIX01_PIN,  NEO_MATRIX_BOTTOM + NEO_MATRIX_RIGHT + NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pedalometer(STRIP_COUNT, PEDALOMETER_PIN, NEO_GRB + NEO_KHZ800);

#define LED_BLACK		      0
#define LED_RED_HIGH 		  (31 << 11)
#define LED_GREEN_HIGH 		(63 << 5)
#define LED_BLUE_HIGH 		31
#define LED_WHITE_HIGH		(LED_RED_HIGH    + LED_GREEN_HIGH    + LED_BLUE_HIGH)

uint32_t lastPrintInfo = 0;  // last time printInfo() happened
uint32_t lastNeopixels = 0;  // last time NeoPixels updated
uint32_t lastGetAnalogs = 0;  // last time getAnalogs() happened
uint32_t animation_start_time = 0; // what time trending animation started
uint32_t lastInteractionTime = 0; // last time there was charging or discharging
float voltage = 0;              // system DC voltage
float current_pedal = 0;        // pedal input current
float current_inverter = 0;     // inverter output DC current
uint32_t energy_pedal = 0;         // energy accumulators ALL IN MILLIJOULES
uint32_t energy_inverter = 0;      // energy accumulators
uint32_t energy_balance;           // energy banking account value, loaded from EEPROM
int trend = 0;                  // animation on energy banking pedalometer

#define AVG_CYCLES 50 // how many times to average analog readings over

void setup() {
  pinMode(RELAY_DROPSTOP, OUTPUT);
  digitalWrite(RELAY_DROPSTOP, HIGH); // turn on relay so we stay on until we decide otherwise
  pinMode(RELAY_INVERTERON, OUTPUT);
  pinMode(RELAY_OVERPEDAL, OUTPUT);
  pinMode(RED_LED_LIGHTS, OUTPUT);
  Serial.begin(115200);
  Serial.println("SchoolCart_ARBD_NEOM_20240112a.ino");
  digitalWrite(BUTTONLEFT,HIGH);  // enable internal pull-up resistor
  digitalWrite(SWITCHMODE,HIGH);  // enable internal pull-up resistor
  digitalWrite(BUTTONRIGHT,HIGH); // enable internal pull-up resistor
  matrix.begin();
  matrix.setTextWrap(false);
  matrix.setBrightness(BRIGHTNESS);
  matrix.clear();
  matrix.setCursor(0, 0); // 3 allows 3 character, greater moves pixels to the right and allows fewer characters
  matrix.setTextColor(LED_BLUE_HIGH);
  matrix.print("mx0102mx");
  matrix.show();

  delay(2000);
  pedalometer.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  pedalometer.show();            // Turn OFF all pixels ASAP
  pedalometer.setBrightness(BRIGHTNESS); // Set BRIGHTNESS to about 1/5 (max = 255)
  load_energy_balance(); // load energy_balance from EEPROM
  lastGetAnalogs = millis(); // initialize integrator timer before first getAnalogs() call
}

void loop() {
  getAnalogs(); // read voltages and currents, integrate energy counts, calculate energy_balance
  doProtectionRelay(); // disconnect pedallers if necessary, shutoff inverter if necessary
  if (switchInUtilityMode()) {
    utilityModeLoop();
  } else {
    energyBankingModeLoop();
  }
  if (millis() - lastPrintInfo > INTERVAL_PRINT) {
    lastPrintInfo = millis();
    printInfo();
  }
}

void utilityModeLoop() {
  if (millis() - lastNeopixels > INTERVAL_NEOPIXELS) { // update neopixels at a reasonable rate
    lastNeopixels = millis(); // reset interval
    doInverterLeds(); // control LEDs under plastic panel
    if (! digitalRead(BUTTONRIGHT)) {
      disNeostring(MATRIX01_PIN,"off", LED_WHITE_HIGH);
      disNeostring(MATRIX02_PIN,"off", LED_WHITE_HIGH);
      delay(500);
      if (! digitalRead(BUTTONRIGHT)) attemptShutdown(); // if button is still being held down, try to shut down
    } else if (! digitalRead(BUTTONLEFT)) {
      disNeostring(MATRIX01_PIN,"---", LED_WHITE_HIGH);
      disNeostring(MATRIX02_PIN,"---", LED_WHITE_HIGH);
    } else {
      disNeostring(MATRIX01_PIN,intAlignRigiht(watts_pedal()), LED_WHITE_HIGH);
      disNeostring(MATRIX02_PIN,intAlignRigiht(energy_pedal/3600000), LED_WHITE_HIGH);
      //disNeostring(MATRIX01_PIN,intAlignRigiht(millis()%1000), LED_WHITE_HIGH);
      //disNeostring(MATRIX02_PIN,intAlignRigiht(1000 - (millis()%1000)), LED_WHITE_HIGH);
    }
    int soc = estimateStateOfCharge();
    if (soc > 10) {
      digitalWrite(RELAY_INVERTERON, HIGH); // turn on inverter
    } else if (soc < 5) {
      digitalWrite(RELAY_INVERTERON, LOW); // shut inverter OFF
    }
    //soc = (millis() / 200) % 100; // TODO: this is for testing only
    utilityModePedalometer(soc);
  }
  if ((watts_pedal() > IDLE_THRESHOLD_PEDAL_WATTS) || (watts_inverter() > IDLE_THRESHOLD_INVERTER_WATTS)) {
    lastInteractionTime = millis(); // update idle detector
  }
  if (millis() - lastInteractionTime > TIMEOUT_UTILITYMODE) attemptShutdown();
}

void energyBankingModeLoop() {
  if (Serial.available()) {
    if (Serial.read() == 'e') {
      uint32_t serial_integer = Serial.parseInt();
      energy_balance = serial_integer * 3600000; // set the number of watt-hours
      store_energy_balance();
    } else {
      while (Serial.available()) Serial.read(); // flush serial buffer
    }
  }
  if (millis() - lastNeopixels > INTERVAL_NEOPIXELS) { // update neopixels at a reasonable rate
    lastNeopixels = millis(); // reset interval
    doInverterLeds(); // control LEDs under plastic panel
    if (! digitalRead(BUTTONRIGHT)) {
      disNeostring(MATRIX01_PIN,"off", LED_WHITE_HIGH);
      disNeostring(MATRIX02_PIN,"off", LED_WHITE_HIGH);
      delay(500);
      if (! digitalRead(BUTTONRIGHT)) attemptShutdown(); // if button is still being held down, try to shut down
    } else if (! digitalRead(BUTTONLEFT)) {
      disNeostring(MATRIX01_PIN,"rst", LED_WHITE_HIGH);
      disNeostring(MATRIX02_PIN,"rst", LED_WHITE_HIGH);
      delay(1000);
      if (! digitalRead(BUTTONLEFT)) {
        disNeostring(MATRIX01_PIN,"RST", LED_WHITE_HIGH);
        disNeostring(MATRIX02_PIN,"RST", LED_WHITE_HIGH);
        reset_energy_balance();
      }
    } else { //Show watts_pedal() and energy_pedal on main signs.
      disNeostring(MATRIX01_PIN,intAlignRigiht(watts_pedal()), LED_WHITE_HIGH);
      disNeostring(MATRIX02_PIN,intAlignRigiht(energy_pedal/3600000), LED_WHITE_HIGH);
    }
    if (energy_balance > 600000) { //If we have just begun a new session, and energy_balance is <600, don't turn on inverter yet
      digitalWrite(RELAY_INVERTERON, HIGH); // turn on inverter
    } else if (energy_balance == 0) {
      digitalWrite(RELAY_INVERTERON, LOW); // shut inverter OFF
    }
    //uint32_t energy_balance = (millis()*250000UL) % 3690000000UL; // TODO: this is for testing only
    //uint32_t energy_balance = 3600000000UL/2; // TODO: this is for testing only
    if ((watts_pedal() >  (watts_inverter() + HYSTERESIS_WATTS) ) && ( trend != 1)) { // determine animation pattern on pedalometer
      trend = 1;
      animation_start_time = millis();
    }
    if (watts_pedal() < watts_inverter() ) {
      if ( trend != -1) {
        trend = -1;
        animation_start_time = millis();
      }
    } else if ((watts_pedal() <= (watts_inverter() + HYSTERESIS_WATTS) ) && ( trend != 0)) {
      trend = 0;
      animation_start_time = millis();
    }
    //trend = millis() % 9000 / 3000 - 1; // TODO: for testing
    energyBankPedalometer(energy_balance/(    3600000000UL/59UL), trend); // 59 is max pedalometer, 60*60*1000000 is 1kwh
  }
  if ((watts_pedal() > IDLE_THRESHOLD_PEDAL_WATTS) || (watts_inverter() > IDLE_THRESHOLD_INVERTER_WATTS)) {
    lastInteractionTime = millis(); // update idle detector
  }
  if (millis() - lastInteractionTime > TIMEOUT_ENERGYBANKING) attemptShutdown();
}

void energyBankPedalometer(int pixlevel, int trend) {
  //Display energyBankBalance as a % of the available pixels, in green.
  //If watts_pedal() > watts_inverter then the pedalers are putting in more than
  //the inverter. Show them things are heading higher. Do this by making the
  //next 3 pixels up chase up on a 0.2s interval .  Else if watts_pedal() <
  //watts_inverter, the pedalers aren't keeping up. Show them things are
  //heading lower.  Make the 3 pixels down chase down in red.
  //If watt_pedals = watt_inverter, students are keeping up with the inverter.
  //Show this with a 2 px chase up and chase down happening at the same time,
  //in green.   OR JUST BLINK thE TOP LINE ON AND OFF
  if (pixlevel >= 60) {pixlevel = 59;}
  for(int i=0; i<pedalometer.numPixels(); i++) { // For each pixel in strip...
    if (i <= pixlevel) {pedalometer.setPixelColor(i, pedalometer.Color(0,255,0));}
    else {pedalometer.setPixelColor(i, 0);}         //  Set pixel's color to black
  }
  int animationtime = (millis() - animation_start_time) % 1000; // animation sequence counter
  if (trend == 1) { // upward animation
    if (animationtime > 750)                         pedalometer.setPixelColor(pixlevel + 3, pedalometer.Color(0,255,0));
    if (animationtime > 500 && animationtime <= 750) pedalometer.setPixelColor(pixlevel + 2, pedalometer.Color(0,255,0));
    if (animationtime > 250 && animationtime <= 500) pedalometer.setPixelColor(pixlevel + 1, pedalometer.Color(0,255,0));
  }
  if (trend == 0) { // in-place animation
    if (animationtime > 750)                         pedalometer.setPixelColor(pixlevel, pedalometer.Color(0,170,0)); // two-thirds-bright
    if (animationtime > 500 && animationtime <= 750) pedalometer.setPixelColor(pixlevel, pedalometer.Color(0,85,0)); // third-bright green
    if (animationtime > 250 && animationtime <= 500) pedalometer.setPixelColor(pixlevel, pedalometer.Color(0,0,0));
  }
  if (trend == -1) { // downward animation in red
    if (animationtime > 750)                         pedalometer.setPixelColor(pixlevel - 3, pedalometer.Color(255,0,0)); // red
    if (animationtime > 500 && animationtime <= 750) pedalometer.setPixelColor(pixlevel - 2, pedalometer.Color(255,0,0)); // red
    if (animationtime > 250 && animationtime <= 500) pedalometer.setPixelColor(pixlevel - 1, pedalometer.Color(255,0,0)); // red
  }
  pedalometer.show();
}

void utilityModePedalometer(int soc) {
  //Display stateOfCharge as a %
  // Light up a single ring. This pedalometer is not cumulative.
  // for example, If state of charge is 50%, rather than turn on the bottom 30 rows, light up only the 30th row.
  int pixlevel = soc * 59 / 100; // soc is 0-100, there are 60 pixels numbered 0-59
  for(int i=0; i<pedalometer.numPixels(); i++) { //     color setting: RRRRRRRRRRRRRRRRR, GGGGGGGGGGGGGGGGG, Blue
    if (i == pixlevel) {pedalometer.setPixelColor(i, pedalometer.Color(255 * (soc <= 15), 255 * (soc >= 15), 0));} //If state of charger is less than 15, make the ring red, otherwise it's green
    else {pedalometer.setPixelColor(i, 0);}         //  Set pixel's color to black
  }
  pedalometer.show();
}

boolean switchInUtilityMode() { // LEFT is LOW/FALSE, RIGHT is HIGH/TRUE
  return digitalRead(SWITCHMODE);
}

int estimateStateOfCharge() {
  int soc = 0;
  if (voltage >= 20.0)  soc =   0 +  (voltage - 20.0 ) * (10 / 4.0);  //  0,   20.0
  if (voltage >= 24.0)  soc =  10 +  (voltage - 24.0 ) * (10 / 1.6);  //  10,  24.0
  if (voltage >= 25.6)  soc =  20 +  (voltage - 25.6 ) * (10 / 0.2);  //  20,  25.6
  if (voltage >= 25.8)  soc =  30 +  (voltage - 25.8 ) * (10 / 0.2);  //  30,  25.8
  if (voltage >= 26.0)  soc =  40 +  (voltage - 26.0 ) * (10 / 0.2);  //  40,  26.0
  if (voltage >= 26.2)  soc =  50 +  (voltage - 26.2 ) * (10 / 0.1);  //  50,  26.2
  if (voltage >= 26.3)  soc =  60 +  (voltage - 26.3 ) * (10 / 0.05); //  60,  26.3
  if (voltage >= 26.35) soc =  66 +  (voltage - 26.35) * (10 / 0.05); //  66,  26.35
  if (voltage >= 26.4)  soc =  70 +  (voltage - 26.4 ) * (10 / 0.2);  //  70,  26.4
  if (voltage >= 26.6)  soc =  80 +  (voltage - 26.6 ) * (10 / 0.2);  //  80,  26.6
  if (voltage >= 26.8)  soc =  90 +  (voltage - 26.8 ) * (10 / 0.4);  //  90,  26.8
  if (voltage >= 27.2)  soc =  100;                                   //  100, 27.2
  return soc;
}

void printInfo() {
  // voltage = (millis() % 7200) / 1000.0 + 20.0; // TODO: take out this debugging feature
#ifdef DEBUG
  if (digitalRead(RELAY_INVERTERON)) Serial.print("INV,");
  if (digitalRead(RELAY_OVERPEDAL)) Serial.print("OVP,");
  if (digitalRead(RELAY_DROPSTOP)) Serial.print("DSR,");
  Serial.println(String(millis()/1000)+"	volt:"+String(voltage)+
      "	SOC:"+String(estimateStateOfCharge())+
      "	watts_pedal: "+String(watts_pedal())+
      "	watts_inverter: "+String(watts_inverter())+
      "	_pedal:"+String(float(energy_pedal)/3600000)+
      "	_inverter:"+String(float(energy_inverter)/3600000)+
      "	_balance:"+String(float(energy_balance)/3600000));
#else
  //23:45:00. 23.5V. Power In: 223. Power Out: 400. Bank: 480Wh
  char timestamp[10]; // string to hold timestamp
  sprintf(timestamp, "%02d:%02d:%02d", int(millis() / 3600000), int((millis() % 3600000) / 60000), int((millis() % 60000) / 1000));
  Serial.println(String(timestamp)+". "+String(voltage,1)+"V. Power In: "+String(watts_pedal())+". Power Out: "+String(watts_inverter())+". Bank: "+String(energy_balance / 3600000)+"Wh");
#endif
}

void getAnalogs() {
  uint32_t integrationTime = millis() - lastGetAnalogs; // time since last integration
  lastGetAnalogs = millis();

  voltage = average(analogRead(VOLT_PIN) / VOLTCOEFF, voltage);

  int amps_in_pin_reading = analogRead(AMPS_IN_PIN);
  current_pedal = average(( amps_in_pin_reading - AMPS_IN_OFFSET ) / AMPS_IN_COEFF , current_pedal);
  if (amps_in_pin_reading - AMPS_IN_OFFSET < 6) current_pedal = 0;
  if (current_pedal < 0) current_pedal = 0;

  int inverter_amps1_pin_reading = analogRead(INVERTER_AMPS1_PIN);
  float inverter_amps1_calc = ( inverter_amps1_pin_reading - INVERTER_AMPS1_OFFSET ) / INVERTER_AMPS1_COEFF;
  if (inverter_amps1_pin_reading - INVERTER_AMPS1_OFFSET < 6) inverter_amps1_calc = 0;
  if (inverter_amps1_calc < 0)     inverter_amps1_calc = 0;

  int inverter_amps2_pin_reading = analogRead(INVERTER_AMPS2_PIN);
  float inverter_amps2_calc = ( inverter_amps2_pin_reading - INVERTER_AMPS2_OFFSET ) / INVERTER_AMPS2_COEFF;
  if (inverter_amps2_pin_reading - INVERTER_AMPS2_OFFSET < 6) inverter_amps2_calc = 0;
  if (inverter_amps2_calc < 0)     inverter_amps2_calc = 0;

  current_inverter = average(inverter_amps1_calc + inverter_amps2_calc, current_inverter);

  energy_pedal    += watts_pedal()    * integrationTime;
  energy_inverter += watts_inverter() * integrationTime;

  energy_balance  += watts_pedal()    * integrationTime; // adjust energy_balance
  energy_balance  -= watts_inverter() * integrationTime; // adjust energy_balance
  if ((energy_balance > 3600000000 ) && (energy_balance < 3960000000)) { // we went past 1000 (from 999)
    energy_balance = 3600000000 ; // don't let it go over 1000, we don't do that
  } // uint32_t maxes out at 1193 * 3600000
  if (energy_balance > 3960000000) {
    energy_balance = 0; // don't let it go negative, we don't do that
  }
}

float watts_pedal() { return voltage * current_pedal; }

float watts_inverter() { return voltage * current_inverter; }

float average(float val, float avg) {
  if (avg == 0) avg = val;
  return (val + (avg * (AVG_CYCLES - 1))) / AVG_CYCLES;
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

void disNeostring(int pin_number, String nval, uint32_t col) { // https://forums.adafruit.com/viewtopic.php?t=101790
  //uint8_t dval = bval; //uint8_t dval = map(constrain(V_Value, 0, 1200), 0, 1200, 0, 255);
  matrix.setPin(pin_number); // set which pin to connect to
  if (pin_number == MATRIX01_PIN) pinMode(MATRIX02_PIN, OUTPUT);
  if (pin_number == MATRIX02_PIN) pinMode(MATRIX01_PIN, OUTPUT);
  matrix.clear();
  matrix.setCursor(0, 0); // 3 allows 3 character, greater moves pixels to the right and allows fewer characters
  matrix.setTextColor(col);
  matrix.print(nval);
  //matrix.drawLine(24, 7, map(dval, 0, 255, 24, 0), 7, matrix01->Color(map(dval, 0, 255, 255, 150),dval,0));
  matrix.show();
}

void doProtectionRelay() {
  if (voltage > VOLTAGE_PROTECT) { digitalWrite(RELAY_OVERPEDAL, HIGH); } // disconnect pedallers
  if (voltage < VOLTAGE_UNPROTECT) { digitalWrite(RELAY_OVERPEDAL, LOW); } // don't disconnect pedallers
  if ((voltage < VOLTAGE_SHUTDOWN) && (watts_pedal() < IDLE_THRESHOLD_PEDAL_WATTS)) {
    Serial.println("Shutdown due to undervoltage");
    reset_energy_balance();
    attemptShutdown();
  }
}

void doInverterLeds() {
  static int led_brightness = 0;
  if (digitalRead(RELAY_INVERTERON)) {
    int new_led_brightness = constrain(watts_inverter() * 255 / RED_LIGHTS_WATTAGE_FULL_BRIGHTNESS, 0, 255);
    if (led_brightness != new_led_brightness) { // only update if led_brightness it has changed
      analogWrite(RED_LED_LIGHTS, new_led_brightness);
      led_brightness = new_led_brightness;
    }
  } else {
    digitalWrite(RED_LED_LIGHTS, LOW); // turn off red LEDs
  }
  if (watts_inverter() > RED_LIGHTS_WATTAGE_BLINKING) { // blink the red LEDs as a warning
    digitalWrite(RED_LED_LIGHTS, (millis() % 1000 > 500)); // blink on/off once per second
  }
}

void attemptShutdown() {
  if (switchInUtilityMode() == false) {
    int ebsoc = ( energy_balance / 3600000 ) / 10; // calculate percentage
    Serial.println("Setting energy_balance to "+String(ebsoc)+"% and storing to EEPROM...");
    store_energy_balance(); // save our present energy bank account
  }
  disNeostring(MATRIX01_PIN,"OFF", LED_WHITE_HIGH);
  disNeostring(MATRIX02_PIN,"OFF", LED_WHITE_HIGH);
  digitalWrite(RELAY_INVERTERON, LOW); // shut inverter OFF
  digitalWrite(RED_LED_LIGHTS, LOW); // shut red lights OFF
  digitalWrite(RELAY_DROPSTOP, LOW); // turn off
  delay(5000);  // power will disappear by now unless we're connected to USB
  // the next lines only run if we're connected to USB and power didn't disappear
  digitalWrite(RELAY_DROPSTOP, HIGH); // if we're still on might as well own it
  // also the above line prevents a situation where charging continues and we don't want the diode taking all the current
  while(digitalRead(BUTTONRIGHT)); // wait until unless right button is pressed
}

union float_and_byte { // https://www.tutorialspoint.com/cprogramming/c_unions
  float f; // accessed as fab.f
  unsigned char bs[sizeof(float)]; // accessed as fab.bs
} fab; // a union is multiple vars taking up the same memory location

void store_energy_balance() {
  fab.f = energy_balance;
  for( int i=0; i<sizeof(float); i++ )
    EEPROM.write( WATTHOURS_EEPROM_ADDRESS+i, fab.bs[i] );
}

void load_energy_balance() {
  Serial.print( "Loading watthours bytes 0x" );
  bool blank = true;
  for( int i=0; i<sizeof(float); i++ ) {
    fab.bs[i] = EEPROM.read( WATTHOURS_EEPROM_ADDRESS+i );
    Serial.print( fab.bs[i], HEX );
    if( blank && fab.bs[i] != 0xff )  blank = false;
  }
  energy_balance = blank ? 0 : fab.f;
  Serial.println( ", so energy_balance is "+String(energy_balance));
}

void reset_energy_balance() {
/* call estimateStateOfCharge() to see if energyBankBalance needs to be higher than 0 at start.
   If battery state of charge is higher than 66% then adjust initial bank account according to the following table.
   As a function, for every percent higher than 66, multiply that difference by 3.
   70 - 66 = 4 * 3 = 12. 12 is the starting energy bank balance. */
  int ebsoc = 0; // energy bank state of charge, default 0 unless battery is above 66% SOC
  int batterySOC = estimateStateOfCharge(); // get battery state of charge
  if ( batterySOC > 66 ) ebsoc = 3 * (batterySOC - 66);
  energy_balance = ( 3600000000 * ebsoc ) / 100 ;
  energy_pedal = 0;    // reset these too
  energy_inverter = 0; // reset these too
  Serial.println("Setting energy_balance to "+String(ebsoc)+"% and storing to EEPROM...");
  store_energy_balance();
  delay(1000); // otherwise it resets a million times each press
}
