#include <Arduino.h>
// ------------------------------------------------------------------------------
// Replace Standard Servo Library with this to enable audio recording with TMRpcm Library
#include <Servo.h>
//#include <MobaTools.h> 
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
//#define Servo Servo8
// ------------------------------------------------------------------------------
#include <Wire.h>

#define Console Serial
#define CONSOLE_BAUDRATE    115200       // baudrate used for console
#define DFPLAYER_BAUDRATE   9600        // baudrate used for communication with DFPlayerMini

 //Die Aufrufe sind kompatibel zur Standard-Servo Library. 
 //zusätzliche Aufrufe:
 //Servo2.setSpeed( wert ); Vorgabe der Geschwindigkeit. Je größer die Zahl, umso
 //                         größer ist die Geschwindigkeit. Bei 0 (defaultwert)
 //                         verhält das Servo sich wie bei der Standard Bibliothek
 //byte = Servo2.moving();  gibt den noch verbleibenden Fahrweg in % vom gesamten
 //                         Verfahrweg an. Bei 0 hat das Servo die Zielposition
 //                          erreicht und steht.
#define SERVO_PIN_LIFT 3 // pin number on UNO for left Servo. Must be one of the six PWM Pins!
#define SERVO_OPEN 90
#define SERVO_CLOSE 0
Servo servoLift;

#define BUTTON_PIN 2

// DFPlayer RX pin is connected to UNO Pin 12 and TX pin to UNO Pin 13
SoftwareSerial mp3Serial(10,11); // RX, TX
DFRobotDFPlayerMini mp3;

bool lidOpen = true;
int buttonState = 0;
bool buttonPressed = false;

uint8_t currentSong = 0;
uint8_t currentVolume = 0;
const uint8_t MAX_VOLUME = 26;
long songStarted =0;

long lidClosingStarted = 0;
long lidOpeningStarted = 0;

uint8_t lastServoAngle =0;
void setup() {

  Serial.begin(CONSOLE_BAUDRATE); // Open serial monitor at 9600 baud to see state changes an distances measured.
  
  mp3Serial.begin(DFPLAYER_BAUDRATE);
  if (!mp3.begin(mp3Serial)) {  //If connection to DFPlayer Module not available, stop here
    Console.println(F("Connection to DFPlayer not possible. Check and Reset."));
    while (true); // loop forever
  }
  mp3.setTimeOut(500); //Set serial communictaion time out 500ms
  mp3.volume(6);  //Set volume value (0~30).
  mp3.outputDevice(DFPLAYER_DEVICE_SD);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  servoLift.attach(SERVO_PIN_LIFT);
  servoLift.write(SERVO_OPEN);
  lidOpen = true;

  Serial.println(F("setup() done"));
}

void loop() {

  buttonState = digitalRead(BUTTON_PIN);
  
  if (buttonState == LOW) {
    if (!buttonPressed) {
      Serial.println(F("Button was pressed. Waiting for Release."));
    }
    buttonPressed = true;
    delay(50);
  } else if (buttonPressed) {
    buttonPressed = false;
    delay(200);
    Serial.println(F("Button released. Starting procedures."));
    // now start action after button was released
    //
    if (lidOpen) {
      Serial.println(F("Lid was open. Closing and stopping Music."));
      servoLift.detach();
      delay(50);
      
      mp3.stop();
      songStarted = 0;
      delay(300);
      
      servoLift.attach(SERVO_PIN_LIFT);
      
      lidOpen=false;
      lidOpeningStarted = 0;
      lidClosingStarted = millis();
    } else {
      //Serial.println(F("Starting Music and opening Lid."));
      servoLift.detach();
      delay(10);
      
      startMusic();
      songStarted = millis();
      delay(4000);
      
      servoLift.attach(SERVO_PIN_LIFT);
      
      lidOpen=true;
      lidOpeningStarted = millis();
      lidClosingStarted = 0;
    }
  }

  if (lidOpen && lidOpeningStarted>0) {
    
    int angle = servoLift.read();
    Serial.print("Opening Servo. Current angle ");
    Serial.println(angle);
    if (angle >= SERVO_OPEN) {
      lidOpeningStarted = 0;
      return;
    }
    long diff = (millis() - lidOpeningStarted) / 150;
    Serial.print("current diff ");
    Serial.println(diff);
    if (diff > 0 && diff > angle) {
      servoLift.write(diff);
    }
  }
  if (!lidOpen && lidClosingStarted>0) {
    
    int angle = servoLift.read();
    Serial.print("Closing Servo. Current angle ");
    Serial.println(angle);
    if (angle <= SERVO_CLOSE) {
      lidClosingStarted = 0;
      return;
    }
    long diff = (millis() - lidClosingStarted) / 150;
    Serial.print("current diff ");
    Serial.println(diff);
    int nextAngle = SERVO_OPEN - diff;
    if (diff > 0 && nextAngle < angle) {
      servoLift.write(nextAngle);
    }
  }
}


// #####################################################################################
// #####################################################################################

void startMusic() {
  if (mp3.readCurrentFileNumber() >= 0) {
    mp3.stop();
  }
  currentSong++;
  if (currentSong > 3) {
    currentSong = 1;
  }
  currentVolume = 25;
  mp3.volume(currentVolume);  //Set volume value (0~30).
  mp3.playMp3Folder(currentSong); // TODO: Some music
}
