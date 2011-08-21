/*
EMW SmartCharger-10000
A 10kW+ charging system

DIY charger inspired by the work of SimonRafferty & jackbauer on DIYelectriccar.com:
http://www.diyelectriccar.com/forums/showthread.php/200-build-your-own-intelligent-charger-36627.html. 
Including display driver in cabin (serial display)

Controller: Arduino Uno, Dec 2010 version (based on a ATmega328P-PU microcontroller)

Major pinout assignments:
ANALOG
* Analog Ref - +5V
* Analog In #0 - High Current hall sensor (100A)
* Analog In #1 - Low Current hall sensor (40mA, used to measure output / battery voltage)
* Analog In #2 - charger heatsink temperature sensor
* Analog In #3 - battery bank 1 temp sensor
* Analog In #4 - battery bank 2 temp sensor
* Analog In #5 - mains voltage - used to toggle power output
DIGITAL
* Digital out/in 0/1 - reserved for serial comms with display
* Digital In #2 - control button input
* Digital In #3 - select button input
* Digital Out #4 - charger fan control
* Digital Out #6 - max current reference voltage (using PWM output)
* Digital Out #9 - charger PWM output
* Digital Out #12 - End of charge indicator
* Digital In #13 - BMS HVC input (active high)

REMAINING SPARES (as of July 13, 2011):
Analog Inputs: A3, A4 (space for pull-up resistors to 5V and filtering caps are on board)
Digital inputs / outputs (pin linked to via): None
Digital relay drivers from deprecated outputs (space for driver transistors is on board, 
                                       can also be used as digital inputs): D5, 7, 8, 10, 11 

Basic code structure:
Startup:
* check battery voltage, if >minimum battery voltage, proceed, otherwise stop
* set duty cycle to 0
Charging (CV or CC):
* increase duty cycle until the condition is met (target voltage or target current)
* monitor condition by taking frequent samples averaging over 120Hz ripple waveform
* based on average value of condition, change duty cycle accordingly
* break when exit condition satisfied

Created Jan 2011 by Valery Miftakhov, Copyright 2011
Commercial use prohibited. Contact Author for commercial rights inquiries.
*/

// need this to remap PWM frequency
#include <EEPROM.h>
#include "EEPROM_VMcharger.h"
#include <TimerOne.h>
// my LCD library for 4D systems display (http://www.4dsystems.com.au/prod.php?id=121)
#include <uLCD_144.h>
uLCD_144 *myLCD;

//---------------- pin-out constants ----------------
// analog pis
const int pin_C=0; // output current pin
const int pin_V=1; // output / battery voltage pin
const int pin_heatSinkT=2; // charger heatsink temp - for thermal derating 
const int pin_mainsV=5;
// digital pins
const int pin_pwrCtrlButton=2; // this is wired to the button (used for menu step)
const int pin_pwrCtrl2Button=3; // this is wired to the button2 (used for menu select)
const int pin_fan=4; // fan control
const int pin_maxC=6; // max current reference voltage (using PWM) -  SHOULD BE CHANGED TO 6 FOR THE NEXT PCB RUN! 
const int pin_PWM=9; // PWM pin
const int pin_EOC=12; // end-of-charge output (GREEN WIRE) - high when charge is complete
const int pin_HVC=13; // this accepts end-of-charge signal from BMS (RED WIRE). Active high (pulled down by 10k on the PCB so ensure <5k source impedance when providing input to this pin)
//---------------- END PINOUTS -----------------------


//============= BATTERY INFO - YOU MUST CHANGE THIS TO YOUR PACK PARAMETERS OR RISK BLOWING UP YOUR PACK! =====
struct config_t {
  int battType;
  int nCells;
  int AH;
  int CV; // per cell
  int CC; // max output current
  int mainsC; // max input current
  // sensor config
  float Vcal;
  float Vcal_k;
  float mVcal;
  float Ccal;
} configuration;

// battery type 
// 0 = LiFePo4, 1 = NiMh/NiCad, 2 = Lead Acid (in dev), 3 = LiCo (in dev)
const char * battTypeLabel[] = { "    LiFePo4   ", "  NiMh/NiCad  " };
int battTypeLen = 2;
//============= END BATTERY INFO USER EDITABLE PARAMETERS =====================================================


//------------------------------- default constants for various battery chemistries
// min battery voltage - if below this, do not start the charger
const float minBattVs[4]={2.5, 0.9, 10., 3.0};
const int enableChargerLowBattLockout=0; // set to 1 to enable 

// CV constant (N/A for LA, Ni)
const float CVs[4]={3.5, -1, -1, 4.2}; // charging voltage for CALB 3.6V per cell. Using 3.55 here to ensure reliable detecion of end-of-charge for a bottom-balanced pack

// absolute maximum voltage
const float maxBattVs[4]={3.8, 1.6, 15., 4.5};

// Nickel chemistries dVdt cutoff
const float dVdt_stop=0.; // in %/s. at 1C, safe value is between -1E-05 and +1E-05

//===================== charger cycle timers =====================================
// for stepDelay=1000, use measCycle_len=300, dVdt_measCycles=150
// when changing stepDelay, change the other 2 variables so that stepDelay*measCycle_len = 0.5-1 sec
// and stepDelay*measCycle_len*dVdt_measCycles = 100-200 sec
const int stepDelay=3000; // primary charger loop delay in microseconds
const int measCycle_len=200; // how many primary loop cycles per display cycle
const int dVdt_measCycles=150; // how many measurement cycles to calculate the dVdT ove
//===================== end charger cycle timers =================================

// DO NOT CHANGE THESE!
float CV;
const float min_CV_Crating=0.05; // wait until the current goes to XC (use values from your battery's datasheet)
// spread for duty cycle ramp conditions to avoid jitter - in volts
// With 5k resistor, 40ma sensor, 25V/A constant, voltage sensitivity is ~1V so no point setting this lower
const float spreadV=2.;
// With 100A sensor, 0.016V/A constant, current sensitivity is ~0.3A. But current being off is not a big deal...
const float spreadC=1.0; 
float maxOutV=0; // absolute maximum output voltage - will be set later in the code
const float charger_efficiency=0.95; // really? need to test. Right now, based on voltage drops (IGBT=3V, diode=2V, bridge=1V, relay=2V)=8V out of 200 ->4% loss 
// ------------------------------- END battery constants -----------------------------


//---------------- MAX CURRENTS
// absolute maximum average output current (used in CV mode) - leave at 0 here - will be set via power button
float maxOutC=0.; 
// with current inductor AND H/W current limiting, can use up to 50-60A here 
// (in older version of the charger, needed to limit to 30A as no hardware current limiter was used 
// - otherwise 120Hz ripple saturates the inductor and kills IGBTs)
const float absMaxChargerCurrent=60; // 60A rating 
const float absMaxChargerPower=10000; // 10kW rating
// when does the current limiter kick in? 1.2-1.3 is a good compromise to get the most out of 
// the input caps while keeping overall ripple below 50% 
// this is mostly relevant for 120Hz ripple. Switching frequency ripple is controlled by the automatic
// frequency selection and low-ESR high-freq output cap
const float instantMaxCRatio=1.3; 
  
//---------------------- timer levels. 9999=unlimited (charger will stop automatically at 100% SOC)
const int timerLevels[9]={9999, 900, 600, 300, 180, 120, 60, 30, 10}; // in minutes
const char * timerLevelLabel[9] = { "No Timeout", "15 hours  ", "10 hours  ", "5 hours   ", "3 hours   ", "2 hours   ", "1 hour    ", "30 min    ", "10 min    " }; 
const int nTimerLevels=9;
int timeOut=0;
int xt = 0; // timing index var

//------------- THERMAL DERATING OF CHARGER 
// for now, simple protection by pausing charger until cooldown to certain temp
// note that heatSink temp at the point of measurement is generally 20-30 deg C LOWER than temperature 
// of critical components attached to heatsink (due to distance from components to probe)
// use maxHeatSinkT of <60 to ensure <85 deg C temp of components
// this assumes thermistor placement near the heat generating components
// BTW, modest airflow (120mm PC fan) should be sufficient for up to 30A output at max power 
//          (assuming adequate heatsink - see site for some references)
const int maxHeatSinkT=60; // in Centigrades - will stop the charger for derating here
const int okToRestartT=50; // restarting charger here
const int midHeatSinkT=40; // turn on the fans here; also wait until cool down to this temp before resuming at the prev power level 
const int lowHeatSinkT=35; // turn off the fans here 
//--------------------------------------------------------

// SAFETY - mostly for testing. 
const int PWM_res=1024;
const int MAXDUTY=PWM_res*0.95; // very short off pulses are bad (diode does not recover by the time IGBT turns on again - generally limit Toff to MORE than 1us)
int period=60; // set PWM period in microseconds (max without driver cooling for IGBT half-bridge seems to be 20kHz)

// 5V supply voltage. On the V4+ boards (with 5V regulator), 4.97-4.98 most of the time
const float Vcc=4.97; 

//========== battery voltage sensor ==============================
// offests - 2.5V for Honeywell CSLW6B40M 
float V_o_V0=Vcc/2;
// V/A constant for the hall sensor 
// 25.5 is the datasheet V/A for Honeywell CSLW6B40M with single 5V supply
float k_V_V=25.5; 
// Vout @ 0 AT
float V_o_V=V_o_V0; 
// resistance of the calibration resistor - approx is fine - will be calibrated away
const float sensorV_R=10000.0;
//========== mains voltage sensor ==============================
// V/A constant for the hall sensor - 40mA
// 25.5 is the datasheet V/A for Honeywell CSLW6B40M with single 5V supply
const float k_mV_V=25.5; 
// Vout @ 0 AT
float mV_o_V=V_o_V0; 
// resistance of the calibration resistor - approx is fine - will be calibrated away
const float sensormV_R=82000.0;

//========== charger current sensor ==============================
// V/A constant for the charger current sensor 
const float k_V_C=0.016; 
// Vout @ 0 AT
// 2.5V for Honeywell CSLT6B100
float V_o_C=Vcc/2; 

//=================== TEMP sensor block================
const float T0=298; // standard reference temp for NTC thermistors = 25 centigrade
//=========== charger heatsink temp sensor ========================
const float B_hST=4540; // this is the B value
const float R0_hST=100000; // nominal (at T=25) resistance of the thermistor
const float R1_hST=100000; // voltage divider resistor
const float V_hST=5; // voltage applied to the divider
//====================== END SENSOR CONSTANTS block ==============

// this should be global vars  -----------------------------
// reference voltage for the analog convertor
const float Aref=Vcc; // default for Arduino
const int nSamplesStopVar=40; // how many samples for moving averages of output voltage / current
const int stopCycles=50; // how many primary charger cycles to require stop condition to exist before exiting
const int nReadSamples=1; // how many samples to average in a single call of readX() functions
const int waitReadSamples=0; // wait between samples for voltage / current readouts in microseconds
float duty=0;
float mainsV=0, outV=0, outC=0;
int charger_run=0;
unsigned long timer=0, timer_ch=0;
float AH_in=0, AH_charger=0;
unsigned int min_up=0;
char str[100];
byte state;
float temp;

//------------------------- Running Averages for dV/dt calcs -------------
float V_ravg[2];
unsigned long t_ms = 0;
unsigned long ele_counter=0;
float dVdt = 0.0;
//-----------------------Navigate Menu--------------------
const char * configMenu[] = { " Run Charger  ", " Config Power ", " Config Time  "  };
const unsigned int configMenuLen = 3;
const char * menuNavigate[] = { "     Yes      ", "      No      " };
const unsigned int menuNavigateLen = 2;
// ------------- end global vars ---------------------------

void setup() {
  // set analog input pins
  pinMode(pin_C, INPUT);
  pinMode(pin_V, INPUT);
  pinMode(pin_heatSinkT, INPUT);
  pinMode(pin_mainsV, INPUT);

  // digital inputs
  pinMode(pin_pwrCtrlButton, INPUT);
  pinMode(pin_pwrCtrl2Button, INPUT);
  pinMode(pin_HVC, INPUT);

  // set output digital pins
  pinMode(pin_PWM, OUTPUT);
  pinMode(pin_maxC, OUTPUT);
  pinMode(pin_EOC, OUTPUT);
  pinMode(pin_fan, OUTPUT);

  // set pins to the right initial values
  digitalWrite(pin_EOC, LOW);

  // get the display going
  *myLCD=uLCD_144(9600); // max is 100kbps and is dependent on the noise levels in the charger
  myLCD->setBgColor(0, 0, 0);
  myLCD->setContrast(0x0f);
  myLCD->clrScreen();
  myLCD->setOpacity(1);
  
  //==================================== ONE-TIME CONFIG =================================
  // check if needed to go into config 
  int forceConfig=0;
  EEPROM_readAnything(0, configuration);
  if(configuration.CC<=0) forceConfig=1; // first time running the charger after assembly
    
  int x = 0;
  const byte STATE_DONE = 0xff;
  const byte STATE_BT = 0x0;
  const byte STATE_CV = 0x1;
  const byte STATE_CELLS = 0x2;
  const byte STATE_CONFIRM = 0x3;
  const byte STATE_CAPACITY = 0x4;
  const byte STATE_CALIBRATE = 0x5; // sensitivity calibration only. zero point calibration done automatically on power-on
  state = STATE_BT;
    
  while(state != STATE_DONE)
  {
    switch(state)
   {
     case STATE_BT:
       myLCD->printStr(0, 0, 2, 0, 0x3f, 0x00, "Thank you for choosing EMW Charger! Press any button to configure"); 
       // if config is not forced, just timeout and send to end of config. Else, wait until button press
       if(forceConfig==0) {
         forceConfig=BtnTimeout(5, 5); // -1 if no button pressed; 1 otherwise
       }
       if(forceConfig==-1) {
         state=STATE_DONE;
       } else { // forceConfig=1 here
         myLCD->clrScreen();
         myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x00, "Cell Type:          ");
         configuration.battType=MenuSelector2(battTypeLen, battTypeLabel);
         state = STATE_CV;
       }
       break;
     case STATE_CV:
       myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x00, "CV cutoff:         ");
       configuration.CV = DecimalDigitInput3(CVs[configuration.battType]*100); 
       state = STATE_CELLS;       
       break;
     case STATE_CELLS:
       myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x00, "Number of cells:   ");
       configuration.nCells = DecimalDigitInput3(configuration.nCells); 
       state = STATE_CAPACITY;       
       break;
     case STATE_CAPACITY:
       myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x00, "Capacity:          ");
       configuration.AH = DecimalDigitInput3(configuration.AH); 
       state = STATE_CALIBRATE;       
       break;
     case STATE_CALIBRATE:
       // first, zero calibration
       myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x00, "Short output and press any button");
       while(!(digitalRead(pin_pwrCtrlButton) || digitalRead(pin_pwrCtrl2Button)));
       outV=readV();
       outC=readC();
       if(abs(outV)<20) { // if too far off, fault out
         // output voltage calibration
         temp=outV/sensorV_R*k_V_V;
         V_o_V+=temp; // this needs to be adjusted HERE because we are calling readV() again below for sensitivity calibration
         configuration.Vcal=temp; 
         // output current calibration
         configuration.Ccal=outC*k_V_C;
         myLCD->printStr(0, 5, 2, 0x1f, 0x3f, 0x00, "Calibrated zero");
         delay(1000);
       }
       // now at voltage. first, double-check we have reset to zero point
       outV=readV(); // get the readings with zero-point already calibrated
       if(abs(outV)<3) { // should be pretty tight after zero calibration
         myLCD->clrScreen();
         myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x00, "Connect battery to calibrate");
         delay(1000); // to avoid reading same button state as in prev step
         while(1) {
           outV=readV();
           if(outV>10) { // loop until battery not connected
             delay(5000); // let settle
             outV=readV(); // read settled voltage
             // calibrate
             myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x00, "Measure & enter actual battery voltage:");
             // calibration routine here - if actual voltage > shown, REDUCE the constant
             configuration.Vcal_k=DecimalDigitInput3(int(outV))/outV;
             break; // from while() loop
           }
         }
       }
       // now mains? problem is - PFC is 380VDC, rectified/doubled in older design are 330V...
       // skip for now - mains sensing is good enough without calibration
       configuration.mVcal=0;
       state = STATE_CONFIRM;
       break;
     case STATE_CONFIRM:
       myLCD->clrScreen();
       myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x00, "Confirm:        ");
       sprintf(str, "%d %s cells, %dAH", configuration.nCells, battTypeLabel[configuration.battType], configuration.AH);       myLCD->printStr(0, 1, 2, 0x1f, 0x3f, 0x00, str);
       x=MenuSelector2(menuNavigateLen, menuNavigate);
       if(x == 0) state = STATE_DONE;
       if(x == 1) state = STATE_BT;
       break;
     default: break;
   } 
  }

  // parameters calculated from config variables go here
  maxOutV=maxBattVs[configuration.battType]*configuration.nCells;
  // adjust core sensor constants
  V_o_V=V_o_V0+configuration.Vcal;
  mV_o_V+=configuration.mVcal;
  V_o_C+=configuration.Ccal;
  k_V_V/=configuration.Vcal_k;
}
  

void loop() {  
  // ---------------real loop()
  float pwr;
  mainsV=read_mV();
  outV=readV();

  // run charger if: 
  //                       (1) charger has NOT been run yet in this cycle, or 
  //                       (2) has been run over a week ago
  if(outV>configuration.nCells*minBattVs[configuration.battType] || !enableChargerLowBattLockout) {
    if((charger_run==0 || (charger_run==1 && millis()-timer_ch>1000*3600*24*7))) {
      // get the charger going
      int x=0;
      myLCD->clrScreen();
      
      //----------------------------
      // run state machine:
      const byte STATE_TOP_MENU = 0x0;
      const byte STATE_CONFIG_PWR = 0x1;
      const byte STATE_CONFIG_TIMER = 0x2;
      const byte STATE_RUN_CHARGER = 0x3;
      const byte STATE_CHARGE = 0x4;
      const byte STATE_WAIT_TIMEOUT = 0x5;
      const byte STATE_SHUTDOWN = 0xff;
      state = STATE_WAIT_TIMEOUT;
      if(configuration.CC<=0) state=STATE_CONFIG_PWR;
      
      while(state != STATE_SHUTDOWN)
      {
        myLCD->printStr(0, 6, 2, 0x1f, 0x3f, 0, "Params      ");
        sprintf(str, "IN: %dV, %dA", int(mainsV/1.4), configuration.mainsC); myLCD->printStr(1, 7, 2, 0x1f, 0x3f, 0, str);
        sprintf(str, "OUT: %dA", configuration.CC); myLCD->printStr(1, 8, 2, 0x1f, 0x3f, 0, str);
        myLCD->printStr(0, 9, 2, 0x1f, 0x3f, 0, "TIME:"); myLCD->printStr(6, 9, 2, 0x1f, 0x3f, 0, timerLevelLabel[xt]);
        
        switch(state)
        {
        case STATE_WAIT_TIMEOUT:
          myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x00, "press BTN to change parameters");
          x=BtnTimeout(10, 3);
          if(x == 1) state = STATE_TOP_MENU;
          if(x == -1) // nothing pressed
           { 
            state = STATE_CHARGE;
           }
          break;
        case STATE_TOP_MENU:
          myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x00, "Action:                         ");
          x=MenuSelector2(configMenuLen, configMenu);
          switch(x)
          {
            case 0: state = STATE_RUN_CHARGER; break;
            case 1: state = STATE_CONFIG_PWR; break;
            case 2: state = STATE_CONFIG_TIMER; break;
            default: break;
          }
          break;
        case STATE_CONFIG_PWR:
          myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x00, "max INput current ");      
          configuration.mainsC = DecimalDigitInput3(configuration.mainsC); 
          myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x00, "max OUTput current");      
          configuration.CC = DecimalDigitInput3(configuration.CC); 
          state = STATE_TOP_MENU;
          break;
        case STATE_CONFIG_TIMER:
            // now set the timer using the same button       
            myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x1f, "timeout (min):       ");
            xt=MenuSelector2(nTimerLevels, timerLevelLabel);
           state = STATE_TOP_MENU;
           break;
        case STATE_RUN_CHARGER:
            myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x00, "Confirm CHARGE [Y/n]:");
            x=MenuSelector2(menuNavigateLen, menuNavigate);
          if(x == 1) state = STATE_TOP_MENU;
          if(x == 0) 
          {
            state = STATE_CHARGE;
          }
           break;
         case STATE_CHARGE:
            pwr=configuration.mainsC; 
            // curb power on 110VAC
            // 110VAC=160VDC rectified, 220VAC=320VDC - choose 240VDC as midpoint - safest value
            if(mainsV<240) {
              pwr/=2; // later, pwr is assumed to be a 220VAC-equivalent current
              pwr=min(pwr, 7.5); // equivalent 15A from 110VAC // DEBUG
            }
            maxOutC=min(pwr*charger_efficiency*230/maxOutV, absMaxChargerCurrent);
            maxOutC=min(maxOutC, absMaxChargerPower/maxOutV );
            // curb further if user-spec'ed current is less
            maxOutC=min(maxOutC, configuration.CC); 
            timeOut=timerLevels[xt];
            
            // write out the configuration to EEPROM for next time
            EEPROM_writeAnything(0, configuration);
            
            CV=float(configuration.CV)/100;
  
            timer_ch=millis(); // reset the timer
            runCharger();
            charger_run=1; // charger has run this mains cycle...
            state = STATE_SHUTDOWN; //STATE_TOP_MENU;   
          // the real state machine should rather enclose the arduino loop()
          // cause some logic for charger cycle is done via it  
           break; 
           default: break;
        }
      }
    }
  } else {
    // if battery voltage too low, we get here
    myLCD->clrScreen();
    sprintf(str, "mains: %dV", int(mainsV)); 
    myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x00, str);      
    sprintf(str, "battery: %dV", int(outV)); 
    myLCD->printStr(0, 1, 2, 0x1f, 0x3f, 0x00, str);
    myLCD->printStr(0, 3, 2, 0, 0x3f, 0x1f, "Connect mains AND battery to start charger");  
    charger_run=0;
    delay(1000); 
  }
  
}


//-------------------------------- main charger routine ------------------
int runCharger() {
  // initialize timer here - this way will reset every time when returning from no-mains break
  Timer1.initialize(period); 
  Timer1.pwm(pin_PWM, 0);           

  // reset AH counter
  AH_charger=0;
  digitalWrite(pin_EOC, LOW);
  
  //----------------- MAIN CHARGING ENTRY POINT -------------------------------------
  if(configuration.battType==0 || configuration.battType==3) {
    //---------------- CCCV for LiFePo4 or LiPoly --------------------
    // CC step, end condition - voltage goes to CV
    if(!runChargeStep(1, maxOutC, 1, configuration.nCells*CV)) {
      // CV step
      runChargeStep(2, configuration.nCells*CV, 2, configuration.AH*min_CV_Crating);
    }
  }
  if(configuration.battType==1) {
    //---------------- CC with dVdt termination for NiMh --------------------
    // CC step, end condition - dVdt goes below pre-determined value
    runChargeStep(1, maxOutC, 3, dVdt_stop);
  }
  if(configuration.battType==2) {
    //---------------- Lead Acid - CV forever --------------------
    CV+=10.0; // convention for CV parameter storage in flash
    runChargeStep(2, configuration.nCells*CV, 2, configuration.AH*min_CV_Crating);
  }
  //------------------------------------------------------

  Timer1.setPwmDuty(pin_PWM, 0);           
  digitalWrite(pin_fan, LOW);    
  digitalWrite(pin_EOC, HIGH);
  myLCD->clrScreen();
  myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x1f, "Charging Complete! Disconnect mains");      
  sprintf(str, "%dAH in", int(AH_charger)); 
  myLCD->printStr(0, 5, 2, 0x1f, 0x3f, 0x1f, str);      

  AH_in+=AH_charger;
}


//---------------------------- basic charging step function ----------------------------
// cycle type = 1 for CC, 2 for CV
// universal charging stage function - stop variable is different from start (i.e. if cycleType=1 (CC), CX is amps, 
// stop is voltage 
int runChargeStep(int cycleType, float CX, int stopType, float stopValue) {
  float outC_avg=0.;
  float outV_avg=0.;
  float out1=0.;
  float out2=0.;
  float heatSinkT=0.;
  float spread=0;
  int breakCnt=0;
  int breakCycle=0;
  int out1Reached=0;
  V_ravg[0] = 0;
  V_ravg[1] = 0;
  t_ms = 0; 
  duty=0; // start with the duty cycle = 0 until you hit the constraint
  
  myLCD->clrScreen();
  myLCD->printStr(0, 1, 2, 0, 0x3f, 0, "Starting step (after 5 sec)");      
  sprintf(str, "type=%d, CX=%d, smin=%d, smax=%d", cycleType, int(CX), stopType, int(stopValue)); 
  myLCD->printStr(0, 4, 2, 0x1f, 0x3f, 0x1f, str);      
  delay(5000);
  myLCD->clrScreen();
  
  // reset timer for AH metering
  timer=millis();
  int n=0;
  int tcounter=0;
  
  // here, out1 is what is being controlled (kept constant), out2 is a termination criterion
  if(cycleType==1) {
    // CC - constant current, stop by max voltage
    spread=spreadC;
  } else if(cycleType==2) {
    // CV - constant voltage, stop by min current
    spread=spreadV;
  } else {
    // wrong charge profile
    return -1;
  }

  // start sensor readouts at some value so we can feed the averages
  outV_avg=outV=readV();
  outC_avg=outC=readC();
  float maxC=maxOutC*instantMaxCRatio;
  
  while(1) {
    // loop counter
    n++;
    tcounter++;
    min_up=(unsigned int)1.*(millis()-timer_ch)/60000;

    delayMicroseconds(stepDelay); // reasonable delay but not so that everything slows down too much
    
    // modulate depending on current - can afford lower frequency for higher currents
    // this will result in the minimal possible AC losses in IGBT and freewheeling diodes 
    // assuming standard design with a 0.4mH inductor (derated to 0.2mH at high current to account for saturation). 
    // Target R<0.4
    // general formula: R=Vin*D(1-D)/(f*L*Iout). For Vin=400V, L=400uH, this becomes
    // R=period (in microseconds) *duty*(1-duty)/outC_avg -> at duty=50%, 10kHz, output current 50A => R=0.5
    if(tcounter>10000) { // only every 20-40 sec when duty is fully ramped to steady-state value
      float indSatC=1+outC_avg/45; // 0.4mH at 0A, 50% saturation at 45A (from datasheet)
      float R=period*(duty/PWM_res)*(1-duty/PWM_res)/outC_avg*indSatC; // 10kHz means period=100. lower period - lower R
      int dp=100;
      if(R<=0.3 && period<400) dp=130; // low ripple - can increase period - down to 2 kHz
      if(R>=0.5 && period>=60) dp=80; // too much ripple - reduce the period - up to 20 kHz
      if(dp!=100) {
        period*=dp/100.;
        Timer1.setPeriod(period);
      }
      tcounter=0;
    }
    Timer1.setPwmDuty(pin_PWM, duty);           
    
    // here, out1 is what is being controlled (kept constant), out2 is a termination criterion
    outC=readC(); // every cycle
    outC_avg=(outC_avg*(nSamplesStopVar-1)+outC)/nSamplesStopVar; // moving average
    outV=readV();
    outV_avg=(outV_avg*(nSamplesStopVar-1)+outV)/nSamplesStopVar; // moving average
    if(cycleType==1) {
      // CC - constant current, stop by max voltage
      out1=outC_avg; // controlled variable
      out2=outV_avg; // stop variable
    } else if(cycleType==2) {
      // CV - constant voltage, stop by min current
      out1=outV_avg; // controlled variable
      out2=outC_avg;  // stop variable
    }

    
    // print out stats - but only every few hundred cycles; n=300 generally corresponds to ~2-3Hz
    if(n==measCycle_len) {
      n=0;

      // AH meter
      unsigned long timer_now=millis();
      unsigned int delta=int(timer_now-timer);
      AH_charger+=outC_avg*delta/1000/3600;
      timer=timer_now;

      // this preps for the dVdT etc
      updateMovingAverages(outV_avg);
      
      // check thermal parameters
      heatSinkT=read_heatSinkT();
      if(heatSinkT<lowHeatSinkT) {
        // turn off the fans
        digitalWrite(pin_fan, LOW);
      } 
      if(heatSinkT>midHeatSinkT) {
        // warming up. turn on the fans
        digitalWrite(pin_fan, HIGH);
        if(heatSinkT>maxHeatSinkT) {
          // overheating. stop charger.  wait until drops enough to restart
          duty=0;
          out1=out2=outV=outC=outC_avg=outV_avg=0; // force duty ramp
          out1Reached=0; // otherwise immediate exit since out2 is set to 0 in the prev line 
          Timer1.setPwmDuty(pin_PWM, duty);
          myLCD->clrScreen();        
          while(1) {
            sprintf(str, "Overheating, T=%dC).", (int)heatSinkT);
            myLCD->printStr(0, 1, 2, 0x1F, 0, 0, str);
            sprintf(str, "Pausing until T=%dC", (int)okToRestartT);
            myLCD->printStr(0, 4, 2, 0, 0x3F, 0, str);
            delay(1000);
            heatSinkT=read_heatSinkT();
            if(heatSinkT<okToRestartT) {
              myLCD->clrScreen();
              break; // exit cycle when the temp drops enough
            }
          }
        }
      }
      
      // check HVC signal from BMS
      if(digitalRead(pin_HVC)==HIGH) {
        // BMS commanding charger to stop
        // noise protection - ensure signal stays on for 100ms or so
        int m=0;
        while(1) {
          m++;
          if(digitalRead(pin_HVC)==LOW) break; // false alarm - just exit the loop and continue
          delay(1);
          if(m>100) {
            // this is for real
            // printClrMsg("BMS Stop Signal. Exiting in 5 sec...", 5000, 0x1f, 0x3f, 0);
            return 1; // assume this is a normal end-of charge but do not allow any more steps (return 1)
          }
        }
      } 
      
      // check the timer
      unsigned int runtime=(unsigned int)(1.*(millis()-timer_ch))/60000; // in minutes
      if(runtime>timeOut) {
        // timer run out
        // printClrMsg("Timeout. Exiting in 5 sec...", 5000, 0x1f, 0x3f, 0);
        return 1; // assume this is a normal end-of charge but do not let any more steps (return 1)
      }
      
      // print all parameters
      printParams(duty, outV_avg, outC_avg, heatSinkT, AH_charger, dVdt);
      
      setMaxC(maxC); // has to be set here as has to be in the 'slow feedback loop' (defined earlier)
    }
        
    //-------------- MAIN DUTY CYCLE MANAGEMENT LOGIC ----------------------------
    // use small hysteresis (spread*2) to avoid jitter   
    // if current or voltage too LOW, INcrease duty cycle
    if(out1 < CX-spread) {
      if(duty<MAXDUTY && outC<maxOutC && outV<maxOutV) {
        duty++; 
      }   
    } else {
      out1Reached=1;
    }
    // if current or voltage too HIGH, DEcrease duty cycle       
    if(out1 > CX+spread) {
      if(duty>0) {
        duty--;
      }
    }
    //---------- END MAIN DUTY CYCLE MANAGEMENT LOGIC ----------------------------

    // check for break conditions - only on secondary variable!
    breakCycle=0;
    if(stopType==1 && out2 > stopValue) {
      breakCycle=1;
    } 
    // also, no point to break on stopMin before the CX condition has been reached
    if(stopType==2 && out2 < stopValue && out1Reached==1) {
      breakCycle=1;
    }
    // check dV/dt and break if it is too small - stopValue is in % of pack voltage change per second
    // on a 216V nominal pack, 1E-05 stopValue corresponds to dVdt=2mV/s
    if(stopType==3 && dVdt/maxOutV < stopValue) {
      if(min_up > 5) { // ignore first few min of the charge
        breakCycle=1;        
      }
    }
 
    // do we REALLY need to break?
    if(breakCycle) {
      breakCnt++;
      if(breakCnt>stopCycles) {
        // printClrMsg("Step done. Exiting", 5000, 0, 0x3f, 0);
        break;
      }
    } else {
      breakCnt=0;
    }
    
    
  }; 

  duty=0;
  Timer1.setPwmDuty(pin_PWM, 0);           

  return 0;
}


//============================ HELPER FUNCTIONS ====================================

//============================ voltage readout functions =====================
// read output voltage
float readV() {
  // read voltage pin
  return sensorV_R*(sampleRead(pin_V)-V_o_V)/k_V_V;
}

// read mains voltage
float read_mV() {
  // read voltage pin
  return sensormV_R*(sampleRead(pin_mainsV)-mV_o_V)/k_mV_V;
}
//============================ end voltage readout functions =====================

void setMaxC(float maxC) {
  analogWrite(pin_maxC, 256/Aref*(V_o_C+k_V_C*maxC));
}

//============================ current readout functions =====================
// read output charger current
float readC() {
  // read current pin
  return (sampleRead(pin_C)-V_o_C)/k_V_C;
}
//================================= end current functions ========================

//====================== temperature readout functions ===========================
// read the charger heatsink temp
float read_heatSinkT() {
  return read_T(pin_heatSinkT, R0_hST, R1_hST, V_hST, B_hST);
}
// master temp readout, using formulas from http://en.wikipedia.org/wiki/Thermistor
float read_T(int pin, float R0, float R1, float V, float B) {
  float sensor_T_V=sampleRead(pin);
  float R=R1/(V/sensor_T_V-1);
  return 1/(log(R/R0)/B+1/T0)-273; 
}

float sampleRead(int pin) {
  float sum=0;
  for(int i=0; i<nReadSamples; i++) {
    sum+=analogRead(pin)*Aref/1024.; // 10-bit ADC
    if(waitReadSamples!=0) delayMicroseconds(waitReadSamples);
  }
  return sum/nReadSamples;
}

// SerialLCD Functions
void printParams(float duty, float outV, float outC, float b1T, float curAH, float dVdt) {
  char tempstr[16];
  // myLCD->setOpacity(1); // so that we override previous text
  sprintf(str, "Duty = %s%%  ", ftoa(tempstr, 100.*duty/PWM_res, 1)); myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x1f, str);      
  sprintf(str, "Freq = %dkHz ", int(1000/period)); myLCD->printStr(0, 1, 2, 0x1f, 0x3f, 0x1f, str);      
  sprintf(str, "Out = %dA, %dV   ", int(outC), int(outV)); myLCD->printStr(0, 3, 2, 0x1f, 0x3f, 0, str);      
  sprintf(str, "Temp = %dC  ", int(b1T)); myLCD->printStr(0, 5, 2, 0x1f, 0, 0, str);      
  sprintf(str, "AH in = %sAH", ftoa(tempstr, curAH, 1)); myLCD->printStr(0, 6, 2, 0x1f, 0x3f, 0, str);      
  sprintf(str, "Runtime = %umin", min_up); myLCD->printStr(0, 8, 2, 0, 0, 0x1f, str);
  if(min_up>=5) {
    // print dVdt only if we are past initial settling period
    sprintf(str, "dVdt = %s     ", ftoa(tempstr, dVdt*1000., 1)); myLCD->printStr(0, 9, 2, 0, 0, 0x1f, str);
  }
}

void printClrMsg(const char *str, const int del, const byte red, const byte green, const byte blue) {
  myLCD->clrScreen();
  myLCD->printStr(0, 2, 2, red, green, blue, str);      
  delay(del);
}

char *ftoa(char *a, double f, int precision)
{
  long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};
  
  char *ret = a;
  long heiltal = (long)f;
  itoa(heiltal, a, 10);
  while (*a != '\0') a++;
  *a++ = '.';
  long desimal = abs((long)((f - heiltal) * p[precision]));
  itoa(desimal, a, 10);
  return ret;
}

//--------------------Calculate moving Averages-------------
void updateMovingAverages(float V) {
  float avg = V_ravg[1];
  unsigned int k = ele_counter;
  float updated_avg = avg * (k/float(k+1)) + V/float(k+1);
  if( ele_counter >= dVdt_measCycles /*|| abs(updated_avg - avg) < 0.00001*/) {
    // switch averages and calculate dVdt
    unsigned long now = millis();
    if(t_ms > 0) {
      float time_interval = (now - t_ms) * 0.001; 
      dVdt = (V_ravg[1] - V_ravg[0]) / time_interval;
    }
    ele_counter = 0;
    V_ravg[0] = V_ravg[1];
    t_ms = now;
  } 
  else {
    V_ravg[1] = updated_avg;
    ++ele_counter;
  }
}

unsigned int MenuSelector2(unsigned int selection_total, const char * labels[])
{
  unsigned int selection = 0;
  unsigned int temp_selection = 1;
  
  //sprintf(str, "[%s]", labels[temp_selection-1] ); 
  //myLCD->printStr(0, 3, 2, 0x1f, 0x3f, 0x1f, str);
  myLCD->printStr(0, 3, 2, 0x1f, 0x3f, 0x1f, "[              ]");
  myLCD->printStr(1, 3, 2, 0x1f, 0x3f, 0x1f, labels[temp_selection-1]);

  while(!selection)
  {
    int step_btn = digitalRead(pin_pwrCtrlButton);
    int select_btn = digitalRead(pin_pwrCtrl2Button);
    if(step_btn == HIGH)
    {
      ++temp_selection;
      if(temp_selection > selection_total) temp_selection = 1;
      myLCD->printStr(0, 3, 2, 0x1f, 0x3f, 0x1f, "[              ]");
      myLCD->printStr(1, 3, 2, 0x1f, 0x3f, 0x1f, labels[temp_selection-1]);
      
      // ideally, this should call a StatusDisplay method and simply pass selection index
      // StatusDisplay should encapsulate all the complexities of drawing status info onto the screen
      // alternatively myLCD can be re-purposed for this
    }
    else
    if(select_btn == HIGH)
    {
      selection = temp_selection;
      myLCD->printStr(0, 3, 2, 0x1f, 0x0, 0x0, "(              )");
      myLCD->printStr(1, 3, 2, 0x1f, 0x0, 0x0, labels[selection-1]);
      // similar to the above, should delegate display to StatusDisplay object
    } 
    delay(80);
  }

  delay(200);
  return selection - 1;
}

int BtnTimeout(int n, int line)
{
  while(n > 0)
  {
    sprintf(str, "(%d sec left) ", n); 
    myLCD->printStr(0, line, 2, 0x1f, 0x3f, 0, str);

    for(int k=0; k<100; k++) {
      if(digitalRead(pin_pwrCtrlButton)==HIGH || digitalRead(pin_pwrCtrl2Button) == HIGH) return 1;
      delay(10);
    }

    --n;
  }
  
  return -1;
}

int DecimalDigitInput3(int preset)
{
  //  myLCD->setOpacity(1);
  int d3=preset/100;
  int d2=(preset/10)%10;
  int d1=abs(preset%10);
  int digit[3] = { d3, d2, d1 };
  int x = 0; // 0-1-2-3-4
  // 0x30 ascii for "0"
  str[1] = 0x0; // eol 

  while(x < 4)
  {
    int step_btn = digitalRead(pin_pwrCtrlButton); // increments digit
    int select_btn = digitalRead(pin_pwrCtrl2Button); // moves to next digit
 
    if(step_btn == HIGH) {
      if(x > 2) x = 0;
      else {
        // increment digit
        ++digit[x];
        // wrap at 3 (for 100s) or at 9 (for 10s and 1s) 
        if(x == 0 && digit[x] > 3) digit[x] = 0;
        if(digit[x] > 9) digit[x] = 0;
      }      
    } else 
    if(select_btn == HIGH) {
      ++x;
    } 

    printDigits(0, digit, 1);
  
    if(x < 3) {
      // still on digits. Reprint the digit we are on in a different color now so we see what's being changed
      str[0] = 0x30+digit[x];
      printDigit(x, 0, str);
    } else 
    if(x == 3) {
      // selection made - show all in the 'changing' color
      printDigits(0, digit, 0);
    }
    
    delay(150);
  }
  
  printDigits(8, digit, 0);

  return (digit[0]*100+digit[1]*10+digit[2]);
}

void printDigits(int start, int * digit, int stat) {
  str[0] = 0x30+digit[0];
  printDigit(start++, stat, str);
  str[0] = 0x30+digit[1];
  printDigit(start++, stat, str);
  str[0] = 0x30+digit[2];
  printDigit(start, stat, str);
}
void printDigit(int x, int stat, char * str) {
  if(stat==0) myLCD->printStr(x, 5, 2, 0x1f, 0x3f, 0x0, str); // yellow
  if(stat==1) myLCD->printStr(x, 5, 2, 0x8, 0x8, 0x1f, str); // blue
}
