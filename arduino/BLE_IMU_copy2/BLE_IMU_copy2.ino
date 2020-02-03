
/*
  BLE IMU sketch on projecthub
  -----------------------------
  Arduino MKR 1010 + IMU Shield

  FOR USE WITH WEB APP DEMO AT :
  https://8bitkick.github.io/ArduinoBLE-IMU.html
  
  IMPORTANT - if required update MKR 1010 fw for bluetooth support first 
  See https://forum.arduino.cc/index.php?topic=579306.0
  
*/

#include <ArduinoBLE.h>
//#include <MKRIMU.h> XXX
#include <Arduino_LSM9DS1.h>


 // BLE Service
BLEService imuService("917649A0-D98E-11E5-9EEC-0002A5D5C51B"); // Custom UUID

// BLE Characteristic
BLECharacteristic imuCharacteristic("917649A1-D98E-11E5-9EEC-0002A5D5C51B", BLERead | BLENotify, 36);

long previousMillis = 0;  // last timechecked, in ms

//Magnetic declination in  Pilsen
#define DECLINATION +3.6

float prevEulers[3];

void setup() {
  Serial.begin(9600);    // initialize serial communication
  //while (!Serial);
  //Serial.println("Staaaart");
  //delay(1000);
  ///Serial.println("ahoj");

  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected

// begin initialization
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // Serial.print("Euler Angles sample rate = ");
  //Serial.print(IMU.eulerAnglesSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Euler Angles in degrees");
  Serial.println("X\tY\tZ");
  
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }

  // Setup bluetooth
  BLE.setLocalName("ArduinoIMU");
  BLE.setAdvertisedService(imuService); 
  imuService.addCharacteristic(imuCharacteristic);
  BLE.addService(imuService); 
  
  // start advertising
  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");
}

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(float axr, float ayr, float azr, float mxr, float myr, float mzr)
{
  
  /*
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  float eulers[3];
  
  float heading;
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(mx, my);
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;
  
  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
  Serial.print("Heading: "); Serial.println(heading, 2);
  
  // filtration: averaging with prev values
  eulers[0] = (heading   + prevEulers[0])/2.0;
  eulers[1] = (roll    + prevEulers[1])/2.0;
  eulers[2] = (pitch + prevEulers[2])/2.0;
  */
  // eulers[0] = ax;
  // eulers[1] = ay;
  // eulers[2] = az;
  float eulers[9];
  float normA = sqrt((axr * axr) + (ayr * ayr) + (azr * azr));
  
  float normM = sqrt((mxr * mxr) + (myr * myr) + (mzr * mzr));
  
  float ax = axr / normA;
  float ay = ayr / normA;
  float az = azr / normA;

  float mx = -mxr / normM;
  float my = -myr / normM;
  float mz = mzr / normM;
  
  
  // GS orto
  float mmm = (mx * ax) + (my * ay) + (mz * az);
  
  mx = mx - mmm * ax;
  my = my - mmm * ay;
  mz = mz - mmm * az;
  
  float Sb = mz;
  float Cb = sqrt(1 - Sb * Sb);
  
  float Sg = -my / Cb;
  float Cg = mx / Cb;
  
  float Sa = 0; //if -ay / Cb;
  
  float Ca = -az / Cb;
  
  if (Sg*Sg > Cg*Cg){
    Sa = (Ca * Sb *Cg - ax) / (Sg);
  }
  else {
    Sa = (-Ca * Sb * Sg - ay) / Cg;
  }
  // else if (g < -PI) g += (2 * PI);
  
  
  
  float g = atan2(Sg, Cg);
  float b = atan2(Sb, Cb);
  float a = atan2(Sa, Ca);
  
  // if (g > PI) g -= (2 * PI);
  // else if (g < -PI) g += (2 * PI);
  
  // if (b > PI) b -= (2 * PI);
  // else if (b < -PI) b += (2 * PI);
  
  // if (a > PI) a -= (2 * PI);
  // else if (a < -PI) a += (2 * PI);
  
  
  
  // eulers[0] = g;
  // eulers[1] = b;
  // eulers[2] = a;
  
  
  
  eulers[0] = a;
  eulers[1] = b;
  eulers[2] = g;
  
  eulers[3] = ax;
  eulers[4] = ay;
  eulers[5] = az;
   
  eulers[6] = mx;
  eulers[7] = my;
  eulers[8] = mz;
  
  
  
  imuCharacteristic.setValue((byte *) &eulers, 9*4); 
  prevEulers[0] = eulers[0];
  prevEulers[1] = eulers[1];
  prevEulers[2] = eulers[2];
}


// send IMU data
void sendSensorData() {
  float eulersM[3];
  float eulersA[3];
  
  // read orientation x, y and z eulers
  //IMU.readEulerAngles(eulers[0], eulers[1], eulers[2]); XXX
  IMU.readAcceleration(eulersA[0], eulersA[1], eulersA[2]);
  //IMU.readGyroscope(eulers[0], eulers[1], eulers[2]);
  IMU.readMagneticField(eulersM[0], eulersM[1], eulersM[2]);
  // IMU.readAcceleration(x, y, z);
  printAttitude(eulersA[0], eulersA[1], eulersA[2], eulersM[0], eulersM[1], eulersM[2]);
  // Send 3x eulers over bluetooth as 1x byte array 
  //imuCharacteristic.setValue((byte *) &eulersM, 12); 

} 

void loop() {
  // wait for a BLE central
  BLEDevice central = BLE.central();

  // if a BLE central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);

    // while the central is connected:
    while (central.connected()) {
      long currentMillis = millis();

      if (currentMillis - previousMillis >= 50) {
        // if (IMU.accelerationAvailable()) { // XX
        //if (IMU.gyroscopeAvailable()) {
        if (IMU.magneticFieldAvailable()) {
          
          previousMillis = currentMillis;
          sendSensorData();
        }
      }
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}
