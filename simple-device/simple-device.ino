#define output 10 // main output pin
#define inPv A3// put here input pin of pv or A1
#define inReg  A2// put here input pin of reg A2
#define pvLed 13 //  put here output pv led pin
#define pvButton 1// put here input button for manual switching of pv
#define regButton 0// put here input button for manual switching of reg
#define lcdButton A7// put here input button for manual lighting of the lcd

int buttonState1 = 80, buttonState2 = 80; // Current state of the button (LOW or HIGH)
int lastButtonState = 80; // Previous state of the button
unsigned long buttonPressTime = 0; // Timestamp when the button was last pressed
bool longClickTriggered = false;

float pvValu;
float regValu;
float lcdButtonValu; // the button that command the lcd light
int ledState;// variable that stores the lcd led state;
double pvStoredValu,regStoredValu;
int x=0;int y=1; // variable used for switching
int z ; //variable to reset the lcd led light
char switchState ;//variable that stores data sent by the smart
int remote_switch = 0;
int previous = 0;
int prev_rem_reg;

void setup() {
  pinMode (output,OUTPUT);// led lcd button
  pinMode (pvLed,OUTPUT); // out led
  pinMode (regButton,INPUT);
  pinMode (pvButton,INPUT); 

}

void loop() {
   pvValu = analogRead(inPv);
   regValu = analogRead(inReg);
   lcdButtonValu = digitalRead(lcdButton);

  if(pvValu > 100.0 && regValu > 100.0)
  {

    if(!digitalRead(pvButton))
    {
      y = 1;
    }
    else if(!digitalRead(regButton))
    {
      y = 0;
    }
    // MANUAL SWITCHING BY SMARTPHONE //RECEIVE DATA FROM BLUETOOTH
    //////////////////////////////////////
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

   x = y = 1 ;
    }
  }

  if (x)
  {
    digitalWrite(output,1);
    digitalWrite(pvLed,0);
  }
  else if (!x)
  {
    digitalWrite(output,0);
    digitalWrite(pvLed,1);
  }

}
