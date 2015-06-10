//LIBRARIES
#include <LiquidCrystal.h> //LCD library
#include <SPI.h> //Serial
#include "MAX31855.h" //This is RobTillaart's (from github) MAX31855 library.  Much faster than Adafruit's library
#include <TimerOne.h> //Allows for high resolution PWM on pins 9 & 10, which can be used as DAC.

//CONSTANTS
//Thermocouple initialization.  Specify data pins for thermocouple.
const int doPin = 3;
const int csPin = 4;
const int clPin = 5;


MAX31855 tc(clPin, csPin, doPin); // Data pins for thermocouple library
LiquidCrystal lcd(2, 6, 7, 8, 12, 13); // Data pins for LCD library

//TimerOne initialization
const byte PWMDAC1pin = 9; // PWM DAC for peltier.  Need to add smoothing for peltier, especially if we use cooling.
const byte PWMDAC2pin = 10; // PWM DAC for BNC output.  Using a 100nF cap and 10k resistor for smoothing
const byte period = 32; // (8 bit DAC)

//Pins
int inPin1 = A4;   // choose the input pin (for pushbutton1)
int inPin2 = A5;   // chooes the input pin (for pushbutton2)
int analogPin1 = 0; //hold
int analogPin2 = 1; //maximum
int analogPin3 = 2; //ramp
int analogPin4 = 3; //wait

//Intervals
//const int interval_Buttons = 5;   //ms between checks for buttons.  Make fast to detect start and reset reliably.
const int interval_BNC = 50;    //ms between BNC outputs.  Make as fast as time resolution needed.
const int interval_LCD = 300;    //LCD redraw interval.  Slow is okay.
const int interval_Sensors = 50; //ms between checks for pots/sensors.  Needs to be faster than tc requests


//VARIABLES
int start = 0;     // variable for reading the pin status for start button press.
int reset = 0;     // variable for reading the second pin status for reset button press.
int peltiertemp = 0;    //peltier variable for PWM output
int hold = 0; //peltier hold temperature/power (should be correlated but plan to add PID).
int maximum = 0; //maximum peltier power
int fadeAmount = 1;    // how many points to adjust PWM.  1 should be highest resolution.  change delay instead
int ramp = 0;         //how long to delay in ms (ramp if linear)
int wait = 0;        //how long to hold high temperature at maximum
int tc_temp = 0;     //thermocouple temperature
int start_seq = 0;
int reset_seq = 0;

//Timing
unsigned long currentMillis = 0;        // stores the value of millis() in each iteration of loop()
unsigned long previousMillis_LCD = 0;   // variable to store last time LCD was updated
unsigned long previousMillis_BNC = 0;   // variable to store last time BNC was updated
unsigned long previousMillis_Sensors = 0; //variable to store the last time Sensors were updated
unsigned long previousMillis_Buttons = 0; //variable to store the last time Sensors were updated
unsigned long previousMillis_Peltier = 0; //variable to store the last time Peltier loop was accessed
unsigned long previousMillis_Wait = 0; //variable to store the last time Peltier loop was accessed

void setup() {
  Timer1.initialize(period);
  pinMode(PWMDAC1pin, OUTPUT);  //declare pin 9 as output. Peltier output (via MOSFET).
  pinMode(PWMDAC2pin, OUTPUT);  //declare pin 10 as output. BNC output with temperature info.
  pinMode(inPin1, INPUT);       //declare pushbutton as input. Start button.
  pinMode(inPin2, INPUT);       //declare pushbutton as input. Reset button.
  lcd.begin(20, 4);             //setup LCD
  Serial.begin(115200);          //setup Serial
  tc.begin();
  lcd.print("Peltier proto 5.0"); //print version
  delay(500);   // wait for maximum chip to stabilize (at least 500ms)
}


//===============================
void loop() {

  currentMillis = millis();   // capture the latest value of millis()
  inputButtons();
  inputSensors();
  outputLCD();
  outputBNC();
  outputPeltier();
}

//===============================
// No need to debounce because sequences are quite long.
void inputButtons() {
  //  if (currentMillis - previousMillis_Buttons >= interval_Buttons) {
  start = analogRead(inPin1);  // read input value for start button press
  if (start > 500) {
    start_seq = 1;
  }
  reset = analogRead(inPin2);  // read input value for reset button press
  if (reset > 500) {
    reset_seq = 1;
  }
  //    previousMillis_Buttons = currentMillis;
}
//}

//===============================
void inputSensors() {
  if (currentMillis - previousMillis_Sensors >= interval_Sensors) {
    hold = analogRead(analogPin1);    //read hold time
    maximum = analogRead(analogPin2); //read maximum
    ramp = analogRead(analogPin3);    //read ramp
    ramp = ((ramp+1));                //transform to nonzero
    wait = analogRead(analogPin4);    //read wait time pot
    wait = (wait*4);                  //transform * 4
    tc.read();                        //sample from thermocouple.  Must use tc.read() before tc.getTemperature()
    //    tc_temp = (tc.getTemperature());
    previousMillis_Sensors = currentMillis;
  }
}

//===============================
void outputLCD() {
  if (currentMillis - previousMillis_LCD >= interval_LCD) {
    lcd.setCursor(0, 0);
    lcd.print("INTtemp = ");
    lcd.println(tc.getInternal());  //internal temp
    double c = tc.getTemperature(); //TC temp.  Note: getTemperature() isn't refreshed until tc.read() is called so make sure inputSensors() is at = or higher rate than outputLCD()
    lcd.setCursor(0, 1);
    if (isnan(c))
    {
      lcd.print("T/C Problem");
    }
    else
    {
      lcd.print("TCtemp = ");
      lcd.print(c);
      lcd.print("  ");
    }
    lcd.setCursor(0, 2);
    lcd.print("Hold ");
    lcd.setCursor(5, 2);
    lcd.print(hold);
    lcd.print("   ");
    lcd.setCursor(10, 2);
    lcd.print("R ");
    lcd.print(ramp);
    lcd.print("   ");
    lcd.setCursor(0, 3);
    lcd.print("Max ");
    lcd.print(maximum);
    lcd.print("   ");
    lcd.setCursor(10, 3);
    lcd.print("Wait ");
    lcd.print(wait);
    lcd.print("   ");
    previousMillis_LCD = currentMillis; //would be better to not use this method for better timing, so says some dude online.
  }
}

//===============================
void outputBNC() {
  if ( (currentMillis - previousMillis_BNC >= interval_BNC) ) {
    Timer1.pwm(PWMDAC2pin, tc.getTemperature()*10); //*10 is important because decimals will be rounded.
    previousMillis_BNC = millis();
  }
}

//===============================
void outputPeltier() {
  if (start_seq == 0) {
    (peltiertemp = hold);
    Timer1.pwm(PWMDAC1pin, peltiertemp);
  }
  if (start_seq == 1) {
    if(currentMillis - previousMillis_Peltier >= ramp) {
      previousMillis_Peltier = currentMillis;
      if (peltiertemp < maximum) {
        peltiertemp = (peltiertemp + fadeAmount);
        Timer1.pwm(PWMDAC1pin, peltiertemp);
        if (reset_seq == 1) {
          Timer1.pwm(PWMDAC1pin, hold);
          start_seq = 0;
          reset_seq = 0;
        }
      }
      if (peltiertemp == maximum) {
        start_seq = 2;
      };
    }
  }
  if (start_seq == 2) {
    if(currentMillis - previousMillis_Peltier >= wait) {
      start_seq = 0;
      Timer1.pwm(PWMDAC1pin, hold);
    }
    if (reset_seq == 1) {
      Timer1.pwm(PWMDAC1pin, hold);
      start_seq = 0;
      reset_seq = 0;
    }
  }
}


