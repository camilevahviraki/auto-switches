 #include <PCD8544.h> 
#include <EEPROM.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include "EmonLib.h"
#include <TimeLib.h>
#include <DS1307RTC.h>
#include "AT24C256.h"
//Create Object b
AT24C256 eeprom = AT24C256();
//SoftwareSerial mySerial(15, 19); // RX, TX the pin used to communuccate zith bluetooth

PCD8544 lcd; 

#define output 10 // main output pin
#define inPv A3// put here input pin of pv or A1
#define inReg  A2// put here input pin of reg A2
#define lcdLed 8 //  put here output lcd led pin
#define pvLed 13 //  put here output pv led pin
#define regLed 12 //  put here output reg led pin
#define pvButton 1// put here input button for manual switching of pv
#define regButton 0// put here input button for manual switching of reg
#define lcdButton A7// put here input button for manual lighting of the lcd
#define DS1307_CTRL_ID 0x68 // rtc address

const unsigned long longClickDuration = 1000; // Define the duration for a long click in milliseconds

int buttonState1 = 0, buttonState2 = 0; // Current state of the button (LOW or HIGH)
int lastButtonState = 0; // Previous state of the button
unsigned long buttonPressTime = 0; // Timestamp when the button was last pressed
bool longClickTriggered = false;

float pvValu;
float regValu;
float lcdButtonValu; // the button that command the lcd light
int ledState;// variable that stores the lcd led state;
float volt,crnt,c = 2.7,pwr,enrgy;//variable used for the meesurement of the electricity
EnergyMonitor emon1;//Emon object
tmElements_t tm,ptm,ptm1; // variable used to store data
double pvEnrgy ;  //variable used to store the energy for pv
double regEnrgy;  //variable used to store the energy for reg
double getPvEnrgy ;  //variable used to store the energy for pv
double getRegEnrgy;  //variable used to store the energy for reg
double s0,s1,h; // variable for calculating t he energy for the device
double f0,f1;
double pvStoredValu,regStoredValu;
int x=0;int y=1; // variable used for switching
int z ; //variable to reset the lcd led light
char switchState ;//variable that stores data sent by the smartphone
long startTime = millis();

long t_write [14]; //write
long t_read [14]; //read
long tab0[7]; // PD
long tab1[7]; //PE
long t0[7] ; // decimal value get from eeprom
long t1[7] ; //integer  value get from eeprom

long t_write1 [14]; //write
long t_read1 [14]; //read
long tab2[7]; // PD
long tab3[7]; //PE
long t2[7] ; // decimal value get from eeprom
long t3[7] ; //integer  value get from eeprom

double Irms,Vrms,realPower;
 int sampleI,sampleV,startV;
// int inPinI = A1;
 double filteredI,filteredV,lastFilteredV,phaseShiftedV;
 boolean lastVCross, checkVCross;
 double  sqI,sumI,sqV,sumV,instP,sumP;
 double ICAL= 79.9 ,VCAL = 275,PHASECAL = 1.7;
//  int SupplyVoltage=5000;
 int SupplyVoltage = 3300;
 #define  ADC_BITS 10
 #define ADC_COUNTS  (1<<ADC_BITS)
 double offsetI = ADC_COUNTS>>1;
 double offsetV = ADC_COUNTS>>1;
 double _I_RATIO,_V_RATIO; 
 ///Added pins
 int remotePv = 60;
// int remoteReg  = 12;
 int remote_switch = 0;
 int previous = 0;
 int prev_rem_reg; 


void setup() {
  // put your setup code here, to run once:
//  Serial.begin(9600); 
  lcd.begin(84, 48); //Initialization of the 84x48 Nokia
//  while (!Serial);
  pinMode (output,OUTPUT);
  pinMode (lcdLed,OUTPUT); // led lcd button
  pinMode (pvLed,OUTPUT); // lcd led
  pinMode (regLed,OUTPUT); // lcd led 
//  pinMode (regButton,INPUT); // snel button
  pinMode (pvButton,INPUT); // pv button
//  pinMode (ledLcd,OUTPUT); // remote pv
  emon1.voltage(1, 270, 1.7);  // Voltage: input pin, calibration, phase_shift
  emon1.current(0, 43);
  /// READ HERE STORED VALUE OF ENERGY BEFORE MOVING IN THE MAIN LOOP
  Read ();
  Read1 ();
  getPvEnrgy = getNumber(t1,t0);
  getRegEnrgy = getNumber(t3,t2);
  digitalWrite(lcdLed,1);
}

void loop() {
  // put your main code here, to run repeatedly:
  volt =  calcVrmse(20,20);
  crnt =   calcIrmse(1400); // 1480
  lcd.setContrast(50);
  if (crnt <= 0.36){
    crnt =  0.000;
  }
  if (volt <= 120) {
     volt = 0.0;
  }  
  pwr =  volt* crnt; 
   pvValu = analogRead(inPv);
   regValu = analogRead(inReg);
   lcdButtonValu = digitalRead(lcdButton);
 
 //PROGRAM THAT CONSISTS TO SWICTH EITHER MANUALLY OR AUTOMATICALLY
  if(pvValu > 100.0 && regValu > 100.0)
  {
    //MANUAL SWITCHING BY BUTTON

    if(!digitalRead(pvButton))
    {
      y = 1;
    }
    else if(!digitalRead(regButton))
    {
      y = 0;
    }
    // MANUAL SWITCHING BY SMARTPHONE //RECEIVE DATA FROM BLUETOOTH
    if(y){
     x = 1 ;
    }
    else if(!y){
      x = 0 ;
    }
  }
  else
  {
    // AUTOMATIC SWITCHING
    if( pvValu > 100.0 && regValu < 100.0 ){
        x = 0;  y = 1 ;
    }
    else if(regValu > 100.0 && pvValu < 100.0){
      y = 1;
       x = 1;
    }
    else if (pvValu < 100.0 && regValu < 100.0){
      ///when there no available source at all
      // store data here
      //PROGRAM SAVE ENERGYY IN THE EEPROM MEMORY WHEN THE POWER GOES OUT
   save (pvEnrgy,tab0);
   saveInteger (pvEnrgy,tab1); // complete tables  tab1 and tab0
   Write();
   save (regEnrgy,tab2);
   saveInteger (regEnrgy,tab3); // complete tables  tab1 and tab0
   Write1();
   x = y = 1 ;
    }
  }

  if (x)
  {
    digitalWrite(output,1);
    digitalWrite(pvLed,0);
    digitalWrite(regLed,1);

  }
  else if (!x)
  {
    digitalWrite(output,0);
    digitalWrite(pvLed,1);
    digitalWrite(regLed,0);
  }

//PROGRAM TO DISPLAY ON THE LCD
   if (!x)
  {
     lcd.clear();
     lcdDisplay ();
     lcdDisplayRegEnergy ();
  }
  else if (x)
  {
     lcd.clear();
     lcdDisplay ();
     lcdDisplayPvEnergy ();
   }

  //PROGRAM TO COMMAND THE LCD LIGHT
  if (!digitalRead(pvButton) || !digitalRead(regButton)){ 
  // z = 1 ;
  lightLcd ();
  }
 //PROGRAM TO CALCULATE THE ENERGY WITH THE RTC
 if (x){
   if (RTC.read(tm)) {
    if (tm.Second != ptm.Second){
      h  = pwr /1000.0;
       s0 += h ;
    }
   f0 = s0/3600.0 ;
  ptm.Second = tm.Second;
  }
}
if (!x){
     if (RTC.read(tm)) {
    if (tm.Second != ptm.Second){
      h  = pwr /1000.0;
       s1 += h ;
    }
    f1 = s1/3600.0;
    ptm.Second = tm.Second; 
  }
} 
   pvEnrgy = f0 + getPvEnrgy ; // add current value of energy to the stored value of pv energy
   regEnrgy = f1 + getRegEnrgy;// add current value of energy to the stored value of pv energy
  //PROGRAM TO WRITE DATA IN THE SD CARDS ONE TIME A DAY

   if (RTC.read(tm)) {
  if(tm.Hour != ptm1.Hour){
      // send PV & REG data to the server each hour
    }
  ptm1.Hour = tm.Hour;
 }
}

void lightLcd (){

// /////////////////////////////////////////////////////
  buttonState1 = digitalRead(pvButton);
  buttonState2 = digitalRead(regButton);

   if (buttonState1 != lastButtonState) {
    // Button state has changed
    if (buttonState1 == LOW ) {
      // Button is pressed
      buttonPressTime = millis();
    } else {
      // Button is released
      if (millis() - buttonPressTime >= longClickDuration) {
        // Long click detected
        longClickTriggered = true;
      }
    }

    lastButtonState = buttonState1;
  }else if(buttonState2 != lastButtonState){
    // Button state has changed
    if (buttonState1 == LOW ) {
      // Button is pressed
      buttonPressTime = millis();
    } else {
      // Button is released
      if (millis() - buttonPressTime >= longClickDuration) {
        // Long click detected
        longClickTriggered = true;
      }
    }

    lastButtonState = buttonState2;
  }

   if (longClickTriggered) {
      if (millis() - startTime > 2000) { 
    startTime  = millis();  
     if (ledState ==  LOW) {
        ledState = HIGH;
     }
    else if (ledState == HIGH){
       ledState =  LOW;
     } 
        digitalWrite(lcdLed, ledState);
      z = 0 ;
  }

    longClickTriggered = false;
  }
// //////////////////////////////////////////////////

}


void lcdDisplay () { 
 lcd.setCursor(0,1);
 lcd.print("V:");lcd.print(volt);
 lcd.setCursor(48,1);lcd.print (" V");
 lcd.setCursor(0,2);
 lcd.print("I: ");lcd.print(crnt);
 lcd.setCursor(48,2);lcd.print (" A");
 lcd.setCursor(0,3);
 lcd.print("P: ");
 lcd.print( pwr);
 lcd.setCursor(48,3);
 lcd.print (" W");
}
void lcdDisplayPvEnergy () {
  lcd.setCursor(0,0);
  lcd.print("PV IS RUNNING");
  lcd.setCursor(0,4);
  lcd.print("E: ");
  lcd.print( pvEnrgy,2);
  lcd.print ( " kwh ");
}
void lcdDisplayRegEnergy () {
  lcd.setCursor(0,0);
  lcd.print("GRID S RUNNING");
  lcd.setCursor(0,4);
  lcd.print("E : ");
  lcd.print( regEnrgy,2);
  lcd.print ( " kwh ");
}

void Write1 (){
  int add [14] = {20,21,22,23,24,25,26,27,28,29,30,31,32,33};
  concate1 ();
  for (int i = 0 ; i < 14 ; i++){
    EEPROM.write(add[i],t_write1[i]);
  }
}

void Read1  (){
 int add [14] = {20,21,22,23,24,25,26,27,28,29,30,31,32,33};
  for (int i = 0 ; i<14 ; i++)
  {
    t_read1[i] = EEPROM.read(add[i]);
    deconcat1 ();
  }
}


void Write  (){
  int add [14] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13};
  // here , concatener tab0 et tab1
  concate ();
  for (int i = 0 ; i < 14 ; i++){
    EEPROM.update(add[i],t_write[i] );
  }
}

void Read (){
  for (int i = 0 ; i<14 ; i++)
  {
    t_read[i] = EEPROM.read(i);
    deconcat ();
  }
}

 void deconcat ()
{
  int _i;
   for (int j = 0 ;j<7;j++)
    {
       t1[j] = t_read[_i];
       _i++;
    }
    for (int j = 0 ;j<7;j++)
    {
        t0[j] =t_read[_i];
        _i++;
    }
}


void deconcat1 ()
{
  int _i;
   for (int j = 0 ;j<7;j++)
    {
       t3[j] = t_read1[_i];
       _i++;
    }
    for (int j = 0 ;j<7;j++)
    {
        t2[j] =t_read1[_i];
        _i++;
    }
}

void concate ()
{
    int k  = 0;
    for (int j = 0 ;j<=7;j++)
    {
        t_write [j] = tab1 [k];
       k++;
    }

    for (int j = 7 ;j<=14;j++)
    {
        t_write [j] = tab0 [j-7];
    }
}

void concate1 ()
{
    int k  = 0;
    for (int j = 0 ;j<=7;j++)
    {
        t_write1 [j] = tab3 [k];
       k++;
    }

    for (int j = 7 ;j<=14;j++)
    {
        t_write1 [j] = tab2 [j-7];
    }
}

void save (double ln , long *n  )
{

    long number  = ln;
    float  i = ln - number;
    long _n = i * 10000000;
     n[0] = _n / 1000000;
     n[1] = (_n - ( n[0] * 1000000))/100000;
     n[2] = (_n -  ( n[0]*1000000) -  ( n[1]*100000))/10000;
     n[3] = (_n -  ( n[0] *1000000) -  ( n[1] *100000) - ( n[2] *10000)) / 1000;
     n[4] = (_n -  ( n[0] *1000000) -  ( n[1] *100000) - ( n[2]*10000)- ( n[3]*1000) ) / 100;
     n[5] = (_n -  ( n[0] *1000000) -  ( n[1] *100000) - ( n[2]*10000) - ( n[3]*1000) - ( n[4]*100) ) / 10;
     n[6] = (_n -  ( n[0] *1000000) -  ( n[1] *100000) - ( n[2]*10000) - ( n[3]*1000) - ( n[4]*100) - ( n[5]*10) ) ;

}


void saveInteger (double k , long *n )
{
    long _n = k;
     n[0] = _n / 1000000;
     n[1] = (_n - ( n[0]  * 1000000))/100000;
     n[2] = (_n -  ( n[0] * 1000000) -  ( n[1] *100000))/10000;
     n[3] = (_n -  ( n[0] * 1000000) -  ( n[1] *100000) - ( n[2] *10000)) / 1000;
     n[4] = (_n -  ( n[0] * 1000000) -  ( n[1] *100000) - ( n[2] *10000)- ( n[3]*1000) ) / 100;
     n[5] = (_n -  ( n[0] *1000000) -  ( n[1] *100000) - ( n[2] *10000) - ( n[3]*1000) - ( n[4]* 100) ) / 10;
     n[6] = (_n -  ( n[0] *1000000) -  ( n[1] *100000) - ( n[2]  *10000) - ( n[3]*1000) - ( n[4]*100) - (n[5]*10) ) ;

}

double getNumber (long *a,long *b)
{
    double n1;
    double n0 = a [0] * 1000000 + a[1] * 100000 + a[2] * 10000 + a [3] * 1000 + a [4] * 100 + a [5] * 10+ a [6];
    double i = (b [0] * 1000000 + b[1] * 100000 + b[2] * 10000 + b [3] * 1000 + b [4] * 100 + b [5] * 10+ b [6]);
    n1 = i /10000000;
    return n0+n1;
}

double  calcIrmse(  int Number_of_Samples )
{
 for (  int n = 0; n < Number_of_Samples; n++)
  {
    sampleI = analogRead(A0);

    // Digital low pass filter extracts the 2.5 V or 1.65 V dc offset,
    //  then subtract this - signal is now centered on 0 counts.
    offsetI = (offsetI + (sampleI-offsetI)/ADC_COUNTS);
    filteredI = sampleI - offsetI;
    // Root-mean-square method current
    // 1) square current values
    sqI = filteredI * filteredI;
    // 2) sum
    sumI += sqI;
  }

  double I_RATIO = ICAL *((SupplyVoltage/1000.0) / (1024));
  Irms = I_RATIO * sqrt(sumI / Number_of_Samples);

  //Reset accumulators
  sumI = 0;
  //--------------------------------------------------------------------------------------

  return Irms;
}

double  calcVrmse(int crossings,int timeout)
{
  int crossCount = 0;
  int numberOfSamples = 0;
  long start = millis();
  while(1)                                   //the while loop...
  {
    startV = analogRead(A1);                    //using the voltage waveform
    if ((startV < (ADC_COUNTS*0.55)) && (startV > (ADC_COUNTS*0.45))) break;  //check its within range
    if ((millis()-start)>timeout) break;
  }
  start = millis();
  while ((crossCount < crossings) && ((millis()-start)<timeout))
  {
    numberOfSamples++;                       //Count number of times looped.
    lastFilteredV = filteredV;               //Used for delay/phase compensation
    sampleV = analogRead(A1);                 //Read in raw voltage signal
    offsetV = offsetV + ((sampleV-offsetV)/1024);
    filteredV = sampleV - offsetV;
    sqV= filteredV * filteredV;                 //1) square voltage values
    sumV += sqV;
    phaseShiftedV = lastFilteredV + PHASECAL * (filteredV - lastFilteredV);
    lastVCross = checkVCross;
    if (sampleV > startV) checkVCross = true;
                     else checkVCross = false;
    if (numberOfSamples==1) lastVCross = checkVCross;

    if (lastVCross != checkVCross) crossCount++;
  }
  double V_RATIO = VCAL *((SupplyVoltage/1000.0) / (ADC_COUNTS));
  Vrms = V_RATIO * sqrt(sumV / numberOfSamples);
  V_RATIO = _V_RATIO;
  sumV = 0;
  return Vrms ;
}