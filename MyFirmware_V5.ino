//LCD config
#include <Wire.h> 
#include <Arduino.h>
#include <BasicEncoder.h>
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>

//Thermistor needed libraries
#include <thermistor.h>  //Download it here: https://electronoobs.com/eng_arduino_thermistor.php

LiquidCrystal_I2C lcd(0x27,16,2);  //sometimes the adress is not 0x3f. Change to 0x27 if it dosn't work.

        
thermistor therm1(A0,1);          //Connect thermistor on A0, 1 represents TEMP_SENSOR_0 ( configuration.h for more)


//I/Oint 
int PWM_pin = 5;                   //Pin for PWM signal to the MOSFET driver (the BJT npn with pullup)
int temp_pin  = A0;                // Pin to read temperature
int speed_pot = A1;                // Pin to read setting speed
int temp_set  = A2;                // Pin to read temperature setpoint

int but1 = 7;
int but2 = 10;    
// int but3 = 11;    // Diverged to A1 for speed_pot (input from Speed_pot)
int sw = 12;        // Used to for Heater ON/OFF

int EN = 2; 
int STEP = 3;
int DIR = 4;
int LED = 13;
int FAN = 6;

//Rotary Encoder
const int8_t OUTA = 8; 
const int8_t OUTB = 9;

volatile int curCount = 0;

// Variables
float set_temperature = 0;      //Default temperature setpoint. Leave it 0 and control it with rotary encoder
float initial_temperature = 100;
float max_temperature = 250;
float temperature_read = 0.0;
// float error_int = 0;
// int   menu_activated=0;
// float last_set_temperature = 0;
int   max_PWM = 255;
bool  activate_heat = false;
bool  activate_fan = false;
bool  sw_state = false;
bool  but2_state = true;

//Stepper Variables
int max_speed = 2000;     // In the accellstep lib this is the max frequency
int min_speed = 0;       // The actual speed 
bool but1_state = true;
bool activate_stepper = true;
int rotating_speed = 0;
int pot_read = 0;

// Define a stepper and the pins it will use
AccelStepper stepper1(1, STEP, DIR); // (Type of driver: with 2 pins, STEP, DIR)

//######################## ENCODER #################################
// Block for the Encoder and interrupts
//Define Encoder
BasicEncoder encoder(OUTB, OUTA, HIGH, 2);

//PID constants 
//////////////////////////////////////////////////////////
	  int Tu = 32; // period in seconds Obtained by experimentation
    int Ku = 45;
	  // Classic PID :  float kp = 0.5 * Ku; float ki = kp/(0.3 * Tu ) ; float  kd = 0.125 * kp * Tu ; 
 float kp = 0.6 * Ku; float ki = 1.2 * Ku/Tu;   // float kd = 0.075 * Ku*Tu (not implemented); 
//////////////////////////////////////////////////////////

// Variables for the PID controller - with IIR filter in the derivative component
float PID_p = 0;    
float PID_i = 0;      
// float PID_d = 0;
float PID_error = 0;
float previous_error = 0;
float previous_error2 = 0;
float elapsedTime, Time, timePrev;
float  PID_output= 0;

// float pi0 = 0, pi1 = 0, pi2 = 0; 

// #############  Auxiliary functions ###############
//  Read potentiometer
int Get_Speed() {
  int Read = 0;
  for(int i = 0; i<10 ;i++)         // Get 10 readings to average
      Read += analogRead(speed_pot);
  Read /= 10 ;
  // Serial.println(Read); // used for debug
  return map(Read,0,1023,min_speed,max_speed);
  
}

// Encoder related functions
void pciSetup(byte pin)  // Setup pin change interrupt on pin
{
  *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR |= bit(digitalPinToPCICRbit(pin));                    // clear outstanding interrupt
  PCICR |= bit(digitalPinToPCICRbit(pin));                    // enable interrupt for group
}

void setup_encoders(int a, int b) {
  uint8_t old_sreg = SREG;     // save the current interrupt enable flag
  noInterrupts();
  pciSetup(a);
  pciSetup(b);
  encoder.reset();
  SREG = old_sreg;    // restore the previous interrupt enable flag state
}

ISR(PCINT0_vect)  // pin change interrupt for D8 to D13
{
  encoder.service();
}
//######################## End of Aux functions #################################


void setup() { 
  Serial.begin(115200); 
  setup_encoders(OUTA,OUTB);
  encoder.set_reverse();
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);     //Stepper driver is disbled
  pinMode(LED, OUTPUT); 
  pinMode(but1, INPUT_PULLUP);
  pinMode(but2, INPUT_PULLUP);
  pinMode(speed_pot, INPUT);
  pinMode(sw, INPUT_PULLUP);
  stepper1.setMaxSpeed(max_speed); 
  digitalWrite(LED, LOW);
  digitalWrite(DIR,LOW); // Rotate clocwise
  digitalWrite(EN, HIGH); // Disable Stepper
  pinMode(PWM_pin,OUTPUT);
  Time = millis();

  //############################################################################################################
  // This sets a Timer Interrupt in TIMER1 at 8kHz
  //This is essential to drive the stepper motor. It will be pulsed whenever a step is due for the selected speed.
  // ############################################################################################################

  cli() ;                             // Clear all interrupts
  TCCR1A = 0;                         //Reset entire TCCR1A register
  TCCR1B = 0;                         //Reset entire TCCR1B register
  TCNT1  = 0;                         //Reset Timer 1 value to 0. Must be Timer 1 since it is 16 bits and the CTC is greater than 256
  //set the CTC for aprox 8kHz increments
  OCR1A = 0.953;                      //  = (16*10^6) / (8000*1024) -1  (must be < 65536)
  TCCR1B |= (1 << WGM12);             //Turn on CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10); // set CS10 and CS12 bits for 1024 prescaler
  TIMSK1 = (1 << OCIE1A);              // enable timer compare interrupt
  sei(); // allow interrupts

  // End of interrupt setting

  set_temperature = initial_temperature;
  digitalWrite(LED,HIGH);
  lcd.begin();
  lcd.backlight();
  lcd.clear();
}

void loop() {
   
   // HEATER ON/OFF
    if(!digitalRead(sw) && sw_state){
      sw_state = false;
      activate_heat = !activate_heat;
      delay(10);
    }
    else 
      if (digitalRead(sw) && !sw_state) {
        sw_state = true;
      }

  // Motor ON/OFF
  if(!digitalRead(but1) && but1_state){
    but1_state = false;
    activate_stepper = !activate_stepper;
    delay(10);
  }
  else if(digitalRead(but1) && !but1_state){
    but1_state = true;
  }

// FAN ON/OFF
 if(!digitalRead(but2) && but2_state){
    but2_state = false;
    activate_fan = !activate_fan;
    delay(10);
  }
  else if(digitalRead(but2) && !but2_state){
    but2_state = true;
  }

 if (activate_fan) 
    digitalWrite(FAN, HIGH);
  else
    digitalWrite(FAN, LOW);
    
  if(activate_stepper){
    digitalWrite(LED, HIGH);
    digitalWrite(EN, LOW);    //We activate stepper driver
    rotating_speed = Get_Speed();
    stepper1.setSpeed(rotating_speed);
   // stepper1.setSpeed(200);
    //stepper1.runSpeed();
  }
  else {
    digitalWrite(EN, HIGH);    //Deactivate stepper driver
    digitalWrite(LED, LOW);
    rotating_speed = 0;
    stepper1.setSpeed(rotating_speed);
    stepper1.runSpeed();  
    // digitalWrite(FAN,LOW);
  }
   // First we read the real value of temperature
  temperature_read = therm1.analog2temp(); // read temperature

  if (activate_heat){
     // #############  Reading Encoder
    int encoder_change = encoder.get_change();
    if (encoder_change) {
      curCount = encoder.get_count();
      if (curCount > max_temperature - initial_temperature ) {
          curCount = max_temperature - initial_temperature;
          // Serial.println(curCount); 
        }
      else
        if(curCount < 0 ) {
           curCount = 0;
        }
      set_temperature = initial_temperature + curCount;
    }

    timePrev = Time;    // hold time of previous loop
    temperature_read = therm1.analog2temp(); // read temperature

    PID_error = set_temperature - temperature_read + 6 ;  
    Time = millis();                      // actual time read
    elapsedTime = (Time - timePrev) / 1000 ;   // conversion to seconds
  
    //##########  Calculation of the control output 
   
    // Proportional contribution
    PID_p  = PID_p +  kp * (PID_error - previous_error);
    //  Integral contribution   
    PID_i = ki * PID_error * elapsedTime;
    // PI output (averaging the last 3 values)
    PID_output =  PID_p + PID_i;   
    // Thermal Load compensation
    PID_output +=  5; 
           
    Serial.print(PID_error); // Plotting
    Serial.print("   ");    
    // Serial.print(PID_error-previous_error); // Plotting
    // Serial.print("   ");      
    Serial.print(PID_p); // Plotting
    Serial.print("   ");    
    Serial.println(PID_i); // Plotting
   
    previous_error2 = previous_error;
    previous_error = PID_error;

    if(PID_output < 0){
      PID_output = 0;
    }
    if(PID_output > max_PWM){
      PID_output = max_PWM;
    }
    //Now we can write the PWM signal to the mosfet on digital pin D5
    analogWrite(PWM_pin,PID_output);
  
  }
    else {
      PID_output = 0;
      analogWrite(PWM_pin,PID_output);
      set_temperature = initial_temperature;
      activate_heat = false;
    }

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Tsp:");
  lcd.print(set_temperature,0);
  lcd.setCursor(8,0);
  lcd.print("T:");  
  lcd.print(temperature_read,1);
  lcd.setCursor(0,1);
  lcd.print("PID:");
  lcd.print(PID_output,0);
  lcd.setCursor(8,1);
  lcd.print("S:");  
  lcd.print((float)rotating_speed * 0.0365,0); //Conversion of Hz to rpm    (Hz * 60)/ (8*200)
  lcd.print(" rpm");
  delay(300); //Refresh rate + delay of LCD print  

}//Void loop end

// Interrupt Routine
ISR(TIMER1_COMPA_vect) 
{
  stepper1.runSpeed();
}
