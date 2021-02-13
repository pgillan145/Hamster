#include <SPI.h>
#include <WiFiNINA.h>
#include <ArduinoBLE.h>

#undef OTA

#ifdef OTA
#include <ArduinoOTA.h>
#include "arduino_secrets.h" 

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
/////// Wifi Settings ///////
char ssid[] = SECRET_SSID;      // your network SSID (name)
char pass[] = SECRET_PASS;   // your network password

int status = WL_IDLE_STATUS;
#endif 

const int CALIBRATION_LED = 4;
const int LIGHTA = A2;
const int LIGHTB = A0;
const int LIGHTC = A1;

const int LEFT_MOTOR_F = 5;
const int LEFT_MOTOR_R = 3;
const int RIGHT_MOTOR_F = 6;
const int RIGHT_MOTOR_R = 9;

unsigned int speed = 128;

BLEDevice joystick;

int bright_a = 0;
int bright_b = 0;
int bright_c = 0;
void setup() {
  //Initialize serial:
  Serial.begin(9600);
  Serial.println("setup()");

  Serial.print("."); delay(1000);
  Serial.print("."); delay(1000);
  //Serial.print("."); delay(1000);
  //Serial.print("."); delay(1000);
  Serial.println(".");
#ifdef OTA
  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to Wifi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
  }

  // start the WiFi OTA library with internal (flash) based storage
  ArduinoOTA.begin(WiFi.localIP(), "arduino", "arduino", InternalStorage);

  // you're connected now, so print out the status:
  printWifiStatus();
#endif  

  BLE.begin();
  //BLE.scan();
  //BLE.scanForUuid("8C7445B1-0918-D532-6673-B6C9AFF26F0A");
  BLE.scanForName("RFJoystick");
  
  pinMode(CALIBRATION_LED, OUTPUT);
  pinMode(LEFT_MOTOR_F,    OUTPUT);
  pinMode(LEFT_MOTOR_R,    OUTPUT);
  pinMode(RIGHT_MOTOR_F,   OUTPUT);
  pinMode(RIGHT_MOTOR_R,   OUTPUT);
  pinMode(LIGHTA,          INPUT);
  pinMode(LIGHTB,          INPUT);
  pinMode(LIGHTC,          INPUT);

/*  
  // initialized the light sensors
  Serial.println("calibrating light sensors");
  analogWrite(CALIBRATION_LED, 255);
  delay(10);
  for (int i=0; i<1000; i++) {
    bright_a += analogRead(LIGHTA);
    bright_b += analogRead(LIGHTB);
    bright_c += analogRead(LIGHTC);
  }
  bright_a = bright_a/2000;
  bright_b = bright_b/2000;
  bright_c = bright_c/2000;
  analogWrite(CALIBRATION_LED, 0);
  if (bright_a < bright_b && bright_a < bright_c) {
    bright_b -= bright_a;
    bright_c -= bright_a;
    bright_a = 0;
  }
  else if (bright_b < bright_a && bright_b < bright_c) {
    bright_a -= bright_b;
    bright_c -= bright_b;
    bright_b = 0;
  }
  else {
    bright_a -= bright_c;
    bright_b -= bright_c;
    bright_c = 0;
  }

  Serial.print("bright_a:");
  Serial.println(bright_a);
  Serial.print("bright_b:");
  Serial.println(bright_b);
  Serial.print("bright_c:");
  Serial.println(bright_c);
*/
}

void loop() {
#ifdef OTA  
  // check for WiFi OTA updates
  ArduinoOTA.poll();
#endif

  int right = analogRead(LIGHTA) - bright_a;
  int forward = analogRead(LIGHTB) - bright_b;
  int left = analogRead(LIGHTC) - bright_c;
  int ble_dir = 0;
  if (joystick.connected()) {
    analogWrite(CALIBRATION_LED, 255);
    ble_dir = getDirection();
  }
  else {
    analogWrite(CALIBRATION_LED, 0);
    BLE.scanForName("RFJoystick");
    BLEDevice peripheral = BLE.available();
  
    if (peripheral) {

      Serial.print("Found ");
      Serial.print(peripheral.address());
      Serial.print(" '");
      Serial.print(peripheral.localName());
      Serial.print("' ");
      Serial.print(peripheral.advertisedServiceUuid());
      Serial.println();
      if (peripheral.localName() == "RFJoystick") {
        Serial.println("found joystick");
        BLE.stopScan();
        joyConn(peripheral);
      } else {
        //BLE.scan();
        //BLE.scanForUuid("8C7445B1-0918-D532-6673-B6C9AFF26F0A");
        //BLE.scanForName("RFJoystick");
      }
    }
  }

  if (ble_dir > 0) {
    if (ble_dir == 1) {
      Serial.println("forward");
      goForward(speed, 250);
    }
    else if (ble_dir == 3) {
      Serial.println("ble right");
      turnRight(speed, 250);
    }
    else if (ble_dir == 5) {
      Serial.println("ble back");
      goBack(speed, 250);
    }
    else if (ble_dir == 7) {
      Serial.println("ble left");
      turnLeft(speed, 250);
    }

  }
  else if (0 && (right > 200 || left > 200 || forward > 200)) {
    if (left > forward && left > right) {
      Serial.print("light left:");
      Serial.println(left);
      turnLeft(128, 250);
    }
    else if (right > forward && right > left) {
      Serial.print("light right:");
      Serial.println(right);
      turnRight(128, 250);
    }
    else {
      Serial.print("light forward:");
      Serial.println(forward);
      goForward(128, 250);
    }
  }
  delay(2);
}

void joyConn(BLEDevice peripheral) {
  if (peripheral.connect()) {
    Serial.println("Connected");
  } else {
    Serial.println("Failed to connect!");
    return;
  }
  if (peripheral.discoverService("2220")) {
    Serial.println("Service discovered");
  }
  else {
    Serial.println("Attribute discovery failed.");
    peripheral.disconnect();
    return;
  }
  BLECharacteristic simpleKeyCharacteristic = peripheral.characteristic("2221");
      
  // subscribe to the simple key characteristic
  Serial.println("Subscribing to simple key characteristic ...");
  if (!simpleKeyCharacteristic) {
    Serial.println("no simple key characteristic found!");
    peripheral.disconnect();
    return;
  } 
  else if (!simpleKeyCharacteristic.canSubscribe()) {
    Serial.println("simple key characteristic is not subscribable!");
    peripheral.disconnect();
    return;
  } 
  else if (!simpleKeyCharacteristic.subscribe()) {
    Serial.println("subscription failed!");
    peripheral.disconnect();
    return;
  } 
  else {
    Serial.println("Subscribed");
  }
  joystick = peripheral;
  return;
}

int getDirection() {
  byte value = 0;
      
  BLECharacteristic dir = joystick.characteristic("2221");

  if (dir) {
      dir.readValue(value);
      return value;
  } else {
    joystick.disconnect();
  }
  return 0;
}

void goForward (unsigned int s, long int t) {
  Serial.println("goForward()");
  analogWrite(LEFT_MOTOR_F, s);
  analogWrite(LEFT_MOTOR_R, 0);
  analogWrite(RIGHT_MOTOR_F, s);
  analogWrite(RIGHT_MOTOR_R, 0);

  //long int now = millis();
  //while (millis() < now + t) {
  //}
  delay(t);
  analogWrite(LEFT_MOTOR_F, 0);
  analogWrite(RIGHT_MOTOR_F, 0);
}

void goBack (unsigned int s, long int t) {
  Serial.println("goBack()");
  analogWrite(LEFT_MOTOR_F, 0);
  analogWrite(LEFT_MOTOR_R, s);
  analogWrite(RIGHT_MOTOR_F, 0);
  analogWrite(RIGHT_MOTOR_R, s);

  //long int now = millis();
  //while (millis() < now + t) {
  //}
  delay(t);
  analogWrite(LEFT_MOTOR_R, 0);
  analogWrite(RIGHT_MOTOR_R, 0);
}

void turnRight (unsigned int s, long int t) {
  Serial.println("turnRight()");
  analogWrite(LEFT_MOTOR_F, s);
  analogWrite(RIGHT_MOTOR_F, 0);
  analogWrite(LEFT_MOTOR_R, 0);
  analogWrite(RIGHT_MOTOR_R, s);
  //long int now = millis();
  //while (millis() < now + t) {
  //}
  delay(t);
  analogWrite(LEFT_MOTOR_F, 0);
  analogWrite(RIGHT_MOTOR_R, 0);
}

void turnLeft (unsigned int s, long int t) {
  Serial.println("turnLeft()");
  analogWrite(LEFT_MOTOR_F, 0);
  analogWrite(LEFT_MOTOR_R, s);
  analogWrite(RIGHT_MOTOR_F, s);
  analogWrite(RIGHT_MOTOR_R, 0);

  //long int now = millis();
  //while (millis() < now + t) {
  //}
  delay(t);
  analogWrite(LEFT_MOTOR_R, 0);
  analogWrite(RIGHT_MOTOR_F, 0);
}
