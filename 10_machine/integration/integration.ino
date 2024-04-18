#include <AccelStepper.h>

//code for connecting to firebase/controls
//control website: https://rsfarnsworth.github.io/PS70/10/current_firebase.html
#include <Arduino.h>

#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>


//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"
// Insert your network credentials but take them out when you put them online
#define WIFI_SSID "MAKERSPACE"
#define WIFI_PASSWORD "12345678"

// Insert Firebase project API Key
#define API_KEY "AIzaSyDG3KNz1eb2WGkVcVcGtKDUvM-KrSqsgJY"

// Insert RTDB URLefine the RTDB URL /
#define DATABASE_URL "ps70-machine-default-rtdb.firebaseio.com" 

//Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
int intValue;
float floatValue;
bool signupOK = false;

int dist2transl;   
int dist2rot;
int circ_rad;                
String color; 

// Motor pin definitions
#define MOTOR1_STEP_PIN 14
#define MOTOR1_DIR_PIN 12
#define MOTOR2_STEP_PIN 25
#define MOTOR2_DIR_PIN 26

int motor1_pin1 = 32;
int motor1_pin2 = 33;

int motor2_pin1 = 19;
int motor2_pin2 = 23;



// Initialize two stepper objects
AccelStepper stepper1(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN);

void setup() {

  pinMode(motor1_pin1, OUTPUT);
  pinMode(motor1_pin2, OUTPUT);
  pinMode(motor2_pin1, OUTPUT);
  pinMode(motor2_pin2, OUTPUT);

  digitalWrite(motor1_pin1, LOW);
  digitalWrite(motor1_pin2, LOW);
  digitalWrite(motor2_pin1, HIGH);
  digitalWrite(motor2_pin2, HIGH);


  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  config.api_key = API_KEY;

  config.database_url = DATABASE_URL;

  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("firebase");
    signupOK = true;
  }
  else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  Firebase.RTDB.setInt(&fbdo, "/TRANSLATION_AMOUNT", 0);  
  Firebase.RTDB.setInt(&fbdo, "/ROTATION_AMOUNT", 0);
  Firebase.RTDB.setInt(&fbdo, "/CIRCLE_RADIUS", 0);

  // Set up the maximum speed and acceleration for both motors
  stepper1.setMaxSpeed(100); // speed can be adjusted as necessary
  stepper1.setAcceleration(100);
  stepper2.setMaxSpeed(100);
  stepper2.setAcceleration(100);
}

void loop() {
  dist2transl = Firebase.RTDB.getInt(&fbdo, "/TRANSLATION_AMOUNT");  
  dist2transl = fbdo.to<int>(); 
  dist2rot = Firebase.RTDB.getInt(&fbdo, "/ROTATION_AMOUNT");
  dist2rot = fbdo.to<int>();
  circ_rad = Firebase.RTDB.getInt(&fbdo, "/CIRCLE_RADIUS");                  // get led status input from firebase
  circ_rad = fbdo.to<int>();               // get led status input from firebase
  color = Firebase.RTDB.getString(&fbdo, "/COLOR_VAL");                        // change to e.g. intData() or boolData()
  color = fbdo.to<String>();


  while (dist2transl != 0) {
    // To rotate the car in place, we set one motor to move forward and the other to move backward
    int steps = int(dist2transl * (200/(80*PI)));

    stepper1.move(steps);  // motor 1 moves forward
    stepper2.move(-steps); 

    

    // Wait until both motors reach their target positions
    while ((stepper1.distanceToGo() != 0) || (stepper2.distanceToGo() != 0)) {
      stepper1.run();
      stepper2.run();

      if (color == "color1") {
        digitalWrite(motor1_pin1, HIGH);
        digitalWrite(motor1_pin2, LOW);
        digitalWrite(motor2_pin1, LOW);
        digitalWrite(motor2_pin2, LOW);
        
      }
      else if (color == "color2") {
        digitalWrite(motor2_pin1, HIGH);
        digitalWrite(motor2_pin2, LOW);
        digitalWrite(motor1_pin1, LOW);
        digitalWrite(motor1_pin2, LOW);
      }
      else if (color == "mix") {
        digitalWrite(motor1_pin1, HIGH);
        digitalWrite(motor1_pin2, LOW);
        digitalWrite(motor2_pin1, HIGH);
        digitalWrite(motor2_pin2, LOW);
      }
      else if (color == "off") {
        digitalWrite(motor1_pin1, LOW);
        digitalWrite(motor1_pin2, LOW);
        digitalWrite(motor2_pin1, LOW);
        digitalWrite(motor2_pin2, LOW);
      }
    }
    dist2transl = 0;
    Firebase.RTDB.setInt(&fbdo, "/TRANSLATION_AMOUNT", 0);

    digitalWrite(motor1_pin1, LOW);
    digitalWrite(motor1_pin2, LOW);
    digitalWrite(motor2_pin1, LOW);
    digitalWrite(motor2_pin2, LOW);
  }

  while (dist2rot != 0) {

    if (dist2rot >0 ){
      // To rotate the car in place, we set one motor to move forward and the other to move backward
        //int steps = int((dist2rot/360)*304.8*2 * (200/(80*PI)));
        int factor = (2*PI*304.8* (200/(80*PI)))/360;
        int steps = factor*dist2rot;
        stepper1.move(steps);  // motor 1 moves forward
        stepper2.move(0); 

        // Wait until both motors reach their target positions
        while ((stepper1.distanceToGo() != 0))  {
          stepper1.run();

          if (color == "color1") {
            digitalWrite(motor1_pin1, HIGH);
            digitalWrite(motor1_pin2, LOW);
            digitalWrite(motor2_pin1, LOW);
            digitalWrite(motor2_pin2, LOW);
          }
          else if (color == "color2") {
            digitalWrite(motor2_pin1, HIGH);
            digitalWrite(motor2_pin2, LOW);
            digitalWrite(motor1_pin1, LOW);
            digitalWrite(motor1_pin2, LOW);
          }
          else if (color == "mix") {
            digitalWrite(motor1_pin1, HIGH);
            digitalWrite(motor1_pin2, LOW);
            digitalWrite(motor2_pin1, HIGH);
            digitalWrite(motor2_pin2, LOW);
          }
          else if (color == "off") {
            digitalWrite(motor1_pin1, LOW);
            digitalWrite(motor1_pin2, LOW);
            digitalWrite(motor2_pin1, LOW);
            digitalWrite(motor2_pin2, LOW);
          }
        }
        dist2rot = 0;
        Firebase.RTDB.setInt(&fbdo, "/ROTATION_AMOUNT", 0);

        digitalWrite(motor1_pin1, LOW);
        digitalWrite(motor1_pin2, LOW);
        digitalWrite(motor2_pin1, LOW);
        digitalWrite(motor2_pin2, LOW);
        
      }
    if (dist2rot < 0 ){
      // To rotate the car in place, we set one motor to move forward and the other to move backward
        //int steps = int((dist2rot/360)*304.8*2 * (200/(80*PI)));
        int factor = (2*PI*304.8* (200/(80*PI)))/360;
        int steps = factor*dist2rot;

        stepper1.move(0);  // motor 1 moves forward
        stepper2.move(steps); 

        // Wait until both motors reach their target positions
        while ((stepper2.distanceToGo() != 0))  {
          stepper2.run();

          if (color == "color1") {
            digitalWrite(motor1_pin1, HIGH);
            digitalWrite(motor1_pin2, LOW);
            digitalWrite(motor2_pin1, LOW);
            digitalWrite(motor2_pin2, LOW);
          }
          else if (color == "color2") {
            digitalWrite(motor2_pin1, HIGH);
            digitalWrite(motor2_pin2, LOW);
            digitalWrite(motor1_pin1, LOW);
            digitalWrite(motor1_pin2, LOW);
          }
          else if (color == "mix") {
            digitalWrite(motor1_pin1, HIGH);
            digitalWrite(motor1_pin2, LOW);
            digitalWrite(motor2_pin1, HIGH);
            digitalWrite(motor2_pin2, LOW);
          }
          else if (color == "off") {
            digitalWrite(motor1_pin1, LOW);
            digitalWrite(motor1_pin2, LOW);
            digitalWrite(motor2_pin1, LOW);
            digitalWrite(motor2_pin2, LOW);
          }
        }
        
        dist2rot = 0;
        Firebase.RTDB.setInt(&fbdo, "/ROTATION_AMOUNT", 0);

        digitalWrite(motor1_pin1, LOW);
        digitalWrite(motor1_pin2, LOW);
        digitalWrite(motor2_pin1, LOW);
        digitalWrite(motor2_pin2, LOW);
      }



  }

  while (circ_rad != 0) {
    // To rotate the car in place, we set one motor to move forward and the other to move backward
    int factor = (2*PI*304.8* (200/(80*PI)))/360;
    int steps = factor*dist2rot;

    stepper1.move(0);  // motor 1 moves forward
    stepper2.move(steps); 
    //int rate = (2*PI*152.4*200/(80*PI))/(2*PI*)
    stepper2.setMaxSpeed(100);
    stepper2.setAcceleration(100);

    // Wait until both motors reach their target positions
    
  }
  

}
