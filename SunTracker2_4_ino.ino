
/*

 ***********************************************************************************
 ******************************SUNTRACKER*******************************************
 ***********************************************************************************
 
 Version:  2.4
 Datestamp:  2012-09-24
  Author: Lawrence Lile
 lawrencelile@gmail.com
 Copyright 2012
 
 Revision History:
 
 1.0   Ported Petey Botts' "Evilrobot" code, a dark following robot.  
 Add header and comments, 
 add code to drive the motor controllers.
 First draft of code  
 
 1.1 reconfigure photoresistors so there is a common pin.  Rebuilt power board. 
 Fix code so that the common pin betwen photosensors represents the balancebetween them, absolute light level is now irrelevant
 use difference  betwen two sensors as indication of night mode
 Add idle state if sensors are satisfied
 
 1.2 Implement full blown state machine.  
 There will be a state for motor movement, a state for button press, for moving east, west, etc. 
 
 2.1  Converted to relay based code
 2.2 added a startup delay because of motorboating.  also needed snubbers and diodes on the board. 
 2.3 Added some hysteresis to reduce hunting
 2.4 calibrated in actual field condirtions
 
 
 
 Suntracker is a program designed to drive the positioning motor on a solar electric array. 
 It uses two CDS photocells to produce a differential signal, that is processed to find a point where 
 both cells are producing the smae output within a tolerance. 
 
 If it becomes dark below a certain threshold, the program returns the solar panels to the East to wait for morning. 
 
 
 Almost any C-Band satelite actuator motor could be used for this purpose.  We used a Venture mfg co Maxi Ball Screw motor, because of 
 high durability. 
 
 This motor is rated 36VDC, although it will run well on less.  We are using the motor on 24VDC, where it draws about 1 amp continuous, and 2.4 amps peak. 
 Motor takes about 2 minutes to return to the East from the West.
 
 
 This motor contains an internal limit switch,which prevents overtravel. Therefore, there is no limiting of position in the code. THe motor also contains a feedback pulse signal, which is about 
 40 pulses per rotation.  THis is also not sensed by the program, and there is no positional feedback.  
 However, this limit switch will NOT accomplish anything if the moving tracker hits something stationary, like a lawnmower or a kid's toy or the ground. 
 Version 1.2 implemented current control and a fuse in the hardware to prevent burning out the electronics if the motor jgets locked out.  Guess how I know this?  
 
 Reversing the polarity on the motor leads reverses the motor.  This allows a simple pair of relays to handle the high currects to the motor, and to be controlled by a 
 rather simple digital signal from the Arduino controler. 
 When there is no voltage on the motor the gears hold the tracker firmly, so there is no need to lock down the motor. In idle statre the motor controller is not activated. 
 
 V+ is connected to relay NO.  Ground is connected fto V-  If relays are opposite, one on, one off, then the motor moves.  Switching the two inpouts move it the other way.  Any time the motor relays are either off or on, both, then 
 the motor is locked. 
 
 
 Other imputs to the microcontroller:
 
 Sensor 1, Sensor Common, and Sensor 2.  THese are analog inputs used to determine the position of the sun and absolute light level.  If light is low, then unit should return to the east to wait for sunrise.
 Current:  THis is an analog input from the motor sense terminal of the L298, taken off of a 0.5 ohm 1 watt resistor.  At the maximum rated 2 amps this resistor should have 1 volt on it.  
 
 There is a switch that can allow test moves of the motor. 
 
 Relays caused a lot of noise issues.  Finally solved them with antiparallel diodes on the motor output, and a ferrous shield between the arduino and the relay board, plus .1 uf capacitors on all power supplies, reset buttons, etc. 
 
 Note:  Mechanical relays finally failed.  Need to reprogram to use MOSFETS as I should have donwe in the first place.

 */



#define EXTEND 3           //Motor extends (turns to the east) when Digital Pin is positive
#define RETRACT 2          //Motor retracts ( turns to the West when Digital pin is positive


#define PHOTORESISTOR1 A1   // EAST sensor.  160 ohms to 5V, then sensor to common
#define PCOMMON A2  // common pin between two photoresisotrs
#define PHOTORESISTOR2 A3   // WEST swensor - 160 ohms to ground, then phtotoresistor to common
#define AMPS A4             // Amp sensor - o.5 ohm resistor connected to current sense terminal of L298
#define WEST_BUTTON 6	    //	pushbutton connected pin 6 to 5V moves array to the West manually
#define EAST_BUTTON 7	    //  pushbutton connected pin 7 to 5V moves array to the East manually
#define DEADBAND 10       // this needs to be calibrated to prevent hunting. 
const int BLINKER = 13;     /// Am I alive? blinker on pin 13
const int blinkInterval = 1000;      // interval between blinks  a second
int blinkerstate = LOW;    // initial state for blinker
long prevMillis = 0;       // millisecond counter for led blinker 
unsigned long currentMillis = 0;
int testState;
char Messagestr[15] = "Startup";

int EASTSensor;            // 0-1023 value from EAST.  
int WESTSensor;           // WEST sensor.  If unit tracks backward, you may sw3ap these pins 
int SensorCommon;           // Common signal
int statevariable;          // checks for various states in the program
int scratchpad;              // placeholder

void setup() {
   
  Serial.print("Suntracker Version 2.3\n");
  
  delay(1000);                 // 1 second startup delay for stability
  pinMode(EXTEND, OUTPUT);    //EXTEND is an output
  pinMode(RETRACT, OUTPUT);     //RETRACT is an output   
  
  pinMode(PHOTORESISTOR1, INPUT); //Photoresistor1 is an input
  pinMode(PHOTORESISTOR2, INPUT); //Photoresistor2 is an input
  pinMode(PCOMMON, INPUT);
  pinMode(WEST_BUTTON, INPUT);  //WEST_Button is and input
  pinMode(EAST_BUTTON, INPUT);  //EAST_BUTTON is an input
  pinMode(BLINKER, OUTPUT);    // blinker LED signals I am alive
  Serial.begin(9600);
  pinMode(1, OUTPUT);
  pinMode(0, OUTPUT);   // serial pins
  
}



//Note to Lile: int is a 2-byte 2's complement range -32768 to +32767  I can never keep this straight from one platform to the other.

void loop() {



  switch (statevariable){

    case 0:  // idle state - motor does not move
  
    digitalWrite(EXTEND, LOW);
    digitalWrite(RETRACT, LOW);
   
    break;

  case 1:  // move West
    digitalWrite (EXTEND, HIGH);
    digitalWrite (RETRACT, LOW);  // this will set up to move West
    delay(100);
     break;

  case 2:   // move east
    digitalWrite (EXTEND, LOW);
    digitalWrite (RETRACT, HIGH);
    delay(100);
         
    break;
  
  case 3: // Return To East
    digitalWrite (EXTEND, LOW);
    digitalWrite (RETRACT, HIGH);
    delay(100);                   // and continue at least .1 sec to limit hunting and clicking 
    break;

  default:   // this case should not occur.  Added for safety
    digitalWrite(EXTEND, LOW);
    digitalWrite(RETRACT, LOW);
   
    break;
  }

  // check the time and see what needs to be done right now:
  currentMillis = millis();   // Note current millisecond timer
  
  if(currentMillis - prevMillis > blinkInterval){    // test for exceed blinker interval 500 ms, if so, 
    prevMillis = currentMillis;
     Serial.print(" Statevariable = ");
     Serial.print(statevariable);
     Serial.print("  Difference = ");
     Serial.print(scratchpad); 
    Serial.print(" East Reading = ");
    Serial.print(EASTSensor);
    Serial.print(" Common = ");
    Serial.print(SensorCommon);
    Serial.print(" West Reading = ");
    Serial.print(WESTSensor);
    Serial.print("\n");
    
    if (digitalRead(WEST_BUTTON) == 0){  // this will override statevariable settings from sensors above if button is pressed
    
    Serial.print("West Button\n");
    
  }else 

  if (digitalRead(EAST_BUTTON) == 0){
    
    Serial.print("East Button\n");

  }  
    
    statevariable = 0;  //StATEVARIABLE is returned to 0 every 500 ms.  this prevents a motor movement from lasting more than a half a second without re-initialization from a button or a sensor      
    
    if (blinkerstate == LOW)                    // switch blinker to the other state every 500 ms
      blinkerstate = HIGH;
    else
      blinkerstate = LOW;
    
    digitalWrite (BLINKER, blinkerstate);  
  }  // end if currentMillis
  
  
    
    EASTSensor = analogRead(PHOTORESISTOR1);
    delay(1);
    WESTSensor = analogRead(PHOTORESISTOR2); 
    delay(1);
    SensorCommon = analogRead(PCOMMON);
    delay(1);
    
      scratchpad = EASTSensor - WESTSensor;  // calculates absolute light level
    
    if( scratchpad > 1005){   // was 975 in v 2.1 not dark enough   RETURN TO EAST 975 is calibrated to be a very low level of light typical of evening or night
        statevariable = 3;    // full dark with a quarter moon reads 1023.  
      
       } 
       else      
       if(scratchpad > 800){ // calibrated on a dark cloudy day measured 840
         statevariable = 0; // do nothing, wait for more light, but do not return to weast yet.  May be a cloud. 
       }
       else   
       if (SensorCommon > 480){    // TOO FAR TO THE EAST - GO WEST YOUNG Man.  
       // too far to the eaST
         statevariable = 1;
     
       }
       else
       if (SensorCommon < 450){
          statevariable = 0;        // reset to idle state with hysteresis
       }
  
  
  if (digitalRead(WEST_BUTTON) == 0){  // this will override statevariable settings from sensors above if button is pressed
    
   
    statevariable = 2;  
    
  }else 

  if (digitalRead(EAST_BUTTON) == 0){
    
  
    statevariable = 1; // last statement to prevent processing of photosensor signals after button press
  }  
  
    
    
    
  
   



} // end loop
// ba de ba de ba de THAT'S ALL , FOLKS!  


