#include <SoftwareSerial.h>
#include <avr/io.h>
#include<string.h>

SoftwareSerial BTSerial(2, 4); // Initialize Software serial with RX on Pin 2 and TX on Pin 4 to enable communication with the BT Module

boolean done = false; //flag to indicate that an address with a signal strength is detected

unsigned long fwdMeas = 0; // average value of the signal strength in the forward direction
unsigned long backMeas = 0; // average value of the signal strength in the backward direction
unsigned long rightMeas = 0; // average value of the signal strength in the right direction
unsigned long  leftMeas = 0; // average value of the signal strength in the left direction

int dir = 0; 
/* dir:
 *  variable to indicate the stronger signal between the forward and backward measurements
 *  dir = 1 => fwdMeas > backMeas
 *  dir = 2 => fwdMeas < backMeas
*/

char a; // character received from the BT Module one at a time
char input[50]; //array of characters received from the BT Module
int i = 0; //counter
int compare = 1; // compare match flag. If the address the BT Module should be finding matches the address received in the input array, the compare is set to 1

int signalStrength = 0;

char address[14] = ""; // address received in the input from the BT Module
char device[14] = "98D3:51:FD9EDF"; // Address that the device should be moving towards --> SET TO REQUIRED ADDRESS

int total = 0; // variable to average the signalStrengths received
int averageValue = 0; // average value that the "averager()" function returns

/* BLUETOOTH READINGS
 *  -> gets the result AT+INQ from the Bluetooth Module and returns the signalStrength only when the 
 *     address of the compared input address and the "device" array match.
 */
unsigned long BTReading(){
  while(true){
    BTSerial.println("AT+INQM = 1,20,48"); // BT Module set to RSSI mode and to look for 20 devices with limited inquiry time of 48
    delay(500);
    BTSerial.println("AT+INQ"); // check for devices around the car
    delay(1000);
    while (BTSerial.available()){
      a = BTSerial.read(); // read characters from the BT module while it returns some values
      if(a == '\n'){ // look for \newline character in the BTSerial
        done = true;
        i = 0;
      }
      else {
        input[i] = a; // if the input character is not a newline, it is added to the input array
        i++;
        }
      }
    
    if(done == true){ //input array received
      done = false;
  
    if(input[0] == '+') { // look for the plus tag in the input (+INQ:)
      for (int j = 0; j < 14; j++) //save address in array
        {
          address[j] = input[j+5]; // write the address from the input to the address array for comparison. Omits first 5 characters - +INQ:
        }
      }
    }

    /*
     * The following loop compares the address array to the device array to see if the address found matches the address required
     */
    
    for (int j = 0; j < 14; j++){
  
      if (device[j] == address[j]){
        compare = 1;
      }
      else {
        compare = 0; // flag set to 1 if the 2 addresses match, otherwise it is set to 0
        break;
      }
    }

    /*
     * If the compare flag is 1, get the signalStrength from the input array and return the value from the function which also breaks the while(true) loop.
     */
    
    if (compare == 1) //get signal strength
    {
      signalStrength = (input[29] << 8) | input[30];
      return signalStrength;
    }
  }
}




// MOTOR CONTROL

/*
 * The Motor Driver requires PWM input to run the 2 motors while having the phase for each which decides the direction that the motors would move in.
 * The team decided to use a common PWM signal on Timer2 (refer to setup() for details) while only changing the phase to implement turns.
 * 
 * Phase:
 *        0 => backward movement
 *        1 => forward movement
 *        
 * MOTOR A --> Right motor - Phase Pin on PORT C1
 * MOTOR B --> Left motor - Phase Pin on PORT C3
 */


void brake(){
  PORTC &= ~(1 << PORTC1); // set phase for motor A to be 0 => moving backward
  PORTC &= ~(1 << PORTC3); // set phase for motor B to be 0 => moving backward
  
  OCR2A = 0; //Wheel move set to 0
  OCR2B = 0; //0 just to ensure the brake
}

// 0 < pwm < 255. However, a minimum of 140 is required for the motors to be able to move with the resistance and weight.

void forward(unsigned char pwm) 
{
  PORTC |= (1 << PORTC1); // set phase for motor A to be 1 => moving forward
  PORTC |= (1 << PORTC3); // set phase for motor B to be 1 => moving forward
  OCR2A = pwm; //Wheel move with speed set to input value
  OCR2B = 0;
}

void reverse(unsigned char pwm)
{
  Serial.println("reverse");
  PORTC &= ~(1 << PORTC1); // set phase for motor A to be 0 => moving backwards
  PORTC &= ~(1 << PORTC3); // set phase for motor B to be 0 => moving backwards
  
  OCR2A = pwm; //Wheel move with speed set to input value
  OCR2B = 0;
}

void left(unsigned char pwm){

  PORTC |= (1 << PORTC1); // set phase for motor A to be 1 => moving forward
  PORTC &= ~(1 << PORTC3); // set phase for motor B to be 0 => moving backwards
  
  OCR2A = pwm;
  OCR2B = 0;
}

void right(unsigned char pwm){

  PORTC &= ~(1 << PORTC1); // set phase for motor A to be 0 => moving backwards
  PORTC |= (1 << PORTC3); // set phase for motor B to be 1 => moving forward
  
  OCR2A = pwm;
  OCR2B = 0;
}



void setup()
{
  Serial.println("Enter Setup");
  pinMode(9, OUTPUT);  // this pin will pull the HC-05 Enable pin (EN)(key pin) HIGH to switch module to AT mode
  digitalWrite(9, HIGH);
  Serial.begin(9600);
  Serial.println("Enter AT commands:");
  BTSerial.begin(38400);  // HC-05 default speed in AT command more
  delay(500);
  BTSerial.println("AT+RESET");
  delay(500);
  BTSerial.println("AT+ROLE = 1"); // SET ROLE to MASTER
  delay(500);
  BTSerial.println("AT+INIT"); // Initialize module
  delay(500);
  BTSerial.println("AT+CMODE = 1"); // Set to 1 to allow searching and pairing with unpaired devices
  delay(500);
  BTSerial.println("AT+INQM  = 1, 20, 48");
  delay(500);

  /*
   * Timer2 set up to have a clock prescale of 8 (010) with a Fast PWM signal with clear on compare match
   */

  TCCR2A |= (1 << COM0A1) | (1 << WGM02) | (1 << WGM01) | (1 << WGM00); 
  TCCR2B = (1 << CS01);

  /* 
   *  set PWM pin as digital output (the PWM signals will not appear on the lines if they are digital inputs) 
   */
  DDRB |= (1 << PORTB3);
  PORTB &= ~(1 << PORTB3);

  /*
   * set the phase pins for the motors to be outputs and initialize output to LOW
   */
  DDRC |= (1 << PORTC1) | (1 << PORTC3);// Analog pins 1 and 3 for Phase control
  PORTC &= ~(1 << PORTC1);
  PORTC &= ~(1 << PORTC3);
}

/*
 * The averager function calls the BTReading() function 4 times to get 4 signalStrength readings each time the function is called. 
 * These are averaged to output a more consistent reading for signalStrength at any point.
 */

int averager(){

  total = BTReading();
  delay(1000);
  for (i = 0; i < 3; i++){
    total += BTReading();
    delay(1000);
  }
  averageValue = total/4;

  return averageValue;
}

/*
 * ALGORITHM:
 *  The car is equipped with a metal foil on one side of the HC-05 Module which blocks out the signals on one side of the module. 
 *  This helps the car get readings in different directions without actually moving around.
 *  The algorithm used by the car is the following :-
 *    
 *    1) Check reading facing forward when the car starts (done by calling the averager() function) --> fwdMeas
 *    2) Turn 180 degrees and get another reading --> backMeas
 *    3) Turn to the right and get a reading --> rightMeas
 *    4) Turn to the left and get a reading --> leftMeas
 *    5) Compare the readings on the left and right while also comparing the readings of forward and backward
 *    6) Move in between (half angle - 45 degrees) the 2 directions with greates signals (eg. Left and Forward, Left and Backward or Right and Backward)
 */

  void loop()
{
  fwdMeas = 0;
  backMeas = 0;
  rightMeas = 0;
  leftMeas = 0;

  
  Serial.println("ENTER LOOP");
  //measure forward signal and turn 180 degrees
  fwdMeas = averager();
  delay(1000);

  right(150);
  delay(970);
  brake();
  delay(1000);

  //measure signal facing backward and turn to the right position
  backMeas = averager();//Add measure function
  delay(1000);
  
  left(200);
  delay(525);
  brake();
  delay(1000);
  
  //measure signal facing right and turn to the right position
  rightMeas = averager();//Add measure function
  delay(1000);

  right(200);
  delay(970);
  brake();
  delay(1000);

  // get measurement facing leftward
  leftMeas = averager();//Add measure function

  if (fwdMeas == 0xFFDF | backMeas == 0xFFDF | leftMeas == 0xFFDF | rightMeas == 0xFFDF){
    while (true){
      // stops the car when it reaches the other module
    }
  }

  // The next 2 conditional statements check which direction the signal is stronger in between forward and backward and turn to face that direction
  if (fwdMeas > backMeas){
    right(200);
    delay(510);
    brake();
    delay(1000); //Back to relative forward
    dir = 1;
  }

  if (backMeas > fwdMeas){
    //turn left 90deg
    left(200);
    delay(525);
    brake();
    delay(1000);
    dir = 2;
  }

  // Find max(signal at left, signal at right) and depending on the stronger signal between front and back, turn to 45 degrees between the two.
  if (rightMeas > leftMeas && dir == 1){
    //turn right 45deg
    right(200);
    delay(255);
    brake();
    delay(1000);
  }
  if (leftMeas > rightMeas && dir == 1){
    //turn left 45deg
    left(200);
    delay(250);
    brake();
    delay(1000);
  }
  if (rightMeas > leftMeas && dir == 2){
    //turn left 45deg
    left(200);
    delay(250);
    brake();
    delay(1000);
  }
  if (leftMeas > rightMeas && dir == 2){
    //turn right 45deg
    right(200);
    delay(510);
    brake();
    delay(1000);
  }
  // move forward ~1 foot
  forward(200);
  delay(2500);
  brake();
  delay(5000);
  dir = 0;
  // redo check after a little more than a foot!
}
