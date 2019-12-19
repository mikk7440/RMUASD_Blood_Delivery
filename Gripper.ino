/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards


int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(3);  // attaches the servo on pin 9 to the servo object

  pinMode(4,INPUT);
  pinMode(5,INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(4,HIGH);
  digitalWrite(5,HIGH);

}

void loop() {
 
  if (digitalRead(4) == LOW && digitalRead(5) == HIGH){
    pos = 180;
    myservo.write(pos);
    
  }
  else if (digitalRead(4) == HIGH && digitalRead(5) == LOW){
    pos = 90;
    myservo.write(pos);
    
  }
  delay(100);

  //digitalWrite(LED_BUILTIN,digitalRead(4));

}
