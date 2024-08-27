#include <Wire.h>
#include "MLX90393.h"                     // https://github.com/tedyapo/arduino-MLX90393 by Theodore Yapo
#include <Q2HX711.h>                      // https://github.com/queuetue/Q2-HX711-Arduino-Library by Queuetue
#include "BluetoothSerial.h"
#include "LowPass.h"                      // Import Low Pass Filter Class
/*================================================================================================
                                         Constants
================================================================================================*/
#define SERVICE_UUID "00001101-0000-1000-8000-00805F9B34FB"
#define TCA_ADDR 0x70
#define TCA_CHANNEL_COUNT 2
#define NUM_SENSORS 4 
#define NUM_DIMENSIONS 3 
#define TOTAL_SENSORS (NUM_SENSORS * TCA_CHANNEL_COUNT)
#define LP_CUTOFF 2.0
#define LP_SAMPLE 500
#define LP_ADAPTIVE false
/*================================================================================================
                              Sensor-Distance Equation Coefficients
================================================================================================*/
#define A0 29.3885106
#define A1 -0.00714154178
#define A2 0.00000184343304
#define A3 -0.000000000261084948
#define A4 0.0000000000000136912327
/*================================================================================================
                                        Pin Addresses
================================================================================================*/
#define BUZZER_PIN 15
/*================================================================================================
                                         Variables
================================================================================================*/
String esp32_device_name = "ESP32-SpinalLog-Kiichiro";
bool wasConnected = false;                              // Flag to track the previous connection state

String mode = "default";
float InitialZValues[TCA_CHANNEL_COUNT][NUM_SENSORS];
float ZValues[TCA_CHANNEL_COUNT][NUM_SENSORS];
double initialPressure;
double pressureValue;

BluetoothSerial SerialBT;
MLX90393 mlx[TOTAL_SENSORS];
MLX90393::txyz data;

/*
  Low Pass Filter 
  Anatomy: lp(f0, fs, adaptive)
      * f0 (Cutoff Frequency): Controls the frequency at which the filter starts to tune the data. Lower means more filter.
      * fs (Sampling Frequency): The time interval between samples. Lower means how frequently the filter is applied.
      * adaptive (Adaptive Sampling Interval): Allows the filter to adjust to varying sampling intervals. For non-constant sampling.
  
  NOTE: Each sensor needs a filter instance as the it stores the previous raw and filtered data for filtering
*/

LowPass<1> lp[TOTAL_SENSORS] = {
    LowPass<1>(LP_CUTOFF, LP_SAMPLE, LP_ADAPTIVE),
    LowPass<1>(LP_CUTOFF, LP_SAMPLE, LP_ADAPTIVE),
    LowPass<1>(LP_CUTOFF, LP_SAMPLE, LP_ADAPTIVE),
    LowPass<1>(LP_CUTOFF, LP_SAMPLE, LP_ADAPTIVE),
    LowPass<1>(LP_CUTOFF, LP_SAMPLE, LP_ADAPTIVE),
    LowPass<1>(LP_CUTOFF, LP_SAMPLE, LP_ADAPTIVE),
    LowPass<1>(LP_CUTOFF, LP_SAMPLE, LP_ADAPTIVE),
    LowPass<1>(LP_CUTOFF, LP_SAMPLE, LP_ADAPTIVE)
};
/*================================================================================================
                                          Main Loop 
================================================================================================*/
void setup() {
  // Initialise Serial Connection
  Serial.begin(115200);
  Wire.begin();

  SerialBT.begin(esp32_device_name, true);  //Bluetooth device name
  SerialBT.begin(SERVICE_UUID);

  // Initialise Components
  initializePins();
  initialiseSensors();
  delay(100);

  // Set Initial Values
  setInitialZValues();
  Serial.println("System Initialized.");
}
void loop() {
  setZValues();
  
  if (SerialBT.connected()) {

    // Transmits Filtered Distance Values to Blutooth Device
    transmitDistances();

    if (!wasConnected) {
      wasConnected = true;
      BTNotification("connected");
    }

    // Recieve Bluetooth Data
    if (SerialBT.available()) {
      mode = SerialBT.readStringUntil('\n');
      Serial.println("Command Recieved: " + mode);
    }

  } else {
    if (wasConnected) {
      wasConnected = false;
      BTNotification("disconnected");
    }
  }
}
/*================================================================================================
                                        Magnetometer
================================================================================================*/
// Selects specified I2C bus on the TCA9548A multiplexer
void setMultiplexerChannel(uint8_t bus) {
  Wire.beginTransmission(TCA_ADDR);                                                   // TCA9548A address is 0x70
  Wire.write(1 << bus);                                                               // send byte to select bus
  Wire.endTransmission();
  delay(10);
}
// Initialises all MLX90393 magentometers and calibrates
void initialiseSensors() {
  for (int TCA_channel = 0; TCA_channel < TCA_CHANNEL_COUNT; TCA_channel++) {
    setMultiplexerChannel(TCA_channel);
    for (int addr = 0; addr < NUM_SENSORS; ++addr) {
      int mlxNum = TCA_channel * 4 + addr;
      mlx[mlxNum].begin(addr / 2, addr % 2);                                          // Select Sensor Addresses (0,0 to 1,1)
      mlx[mlxNum].setOverSampling(0);
      mlx[mlxNum].setDigitalFiltering(0);

      mlx[mlxNum].readData(data);            
      InitialZValues[TCA_channel][addr] = data.z;
    }
  }
}
// Read and store initial z-axis values for calibration and normalising values
void setInitialZValues() {
  for (int pass = 0; pass < 2; pass++) {                                              // The first reading gives garbage, so we need to get the 2nd reading.
    for (int TCA_channel = 0; TCA_channel < TCA_CHANNEL_COUNT; TCA_channel++) {
      setMultiplexerChannel(TCA_channel);
      for (int addr = 0; addr < NUM_SENSORS; ++addr) {
        int mlxNum = TCA_channel * 4 + addr;
        mlx[mlxNum].readData(data);            
        InitialZValues[TCA_channel][addr] = data.z;
      }
    }
  }
}
// Read and store current z-axis values 
void setZValues(){
  for (int TCA_channel = 0; TCA_channel < TCA_CHANNEL_COUNT; TCA_channel++) {
    setMultiplexerChannel(TCA_channel);
    for (int addr = 0; addr < NUM_SENSORS; ++addr) {
      int mlxNum = TCA_channel * 4 + addr;
      mlx[mlxNum].readData(data);  
      ZValues[TCA_channel][addr] = data.z - InitialZValues[TCA_channel][addr];
    }
  }
}
// Converts sensor data into distances in millimeters as a float
float calculateDistance(double sensorValue) {
  double y = A0 + A1 * sensorValue + A2 * sensorValue * sensorValue + 
             A3 * sensorValue * sensorValue * sensorValue + 
             A4 * sensorValue * sensorValue * sensorValue * sensorValue;
  // Round to the nearest float with 2 decimal places
  float roundedDistance = round(y * 100.0) / 100.0;
  return roundedDistance;
}
// Handles notifications for state changes in BT Connection  
void BTNotification(String mode){
  if (mode.equals("connected")){
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("Device connected!");
  } else if (mode.equals("disconnected")) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("Device disconnected!");
  }
}
// Transmits distance data over bluetooth
void transmitDistances() {
  float distanceValues[NUM_SENSORS * TCA_CHANNEL_COUNT];
  float val;
  String filteredValues = "";

    // Output as L1, L2 -> {[0][0], [1][0]}, {[0][1], [1][1]} etc.
    for (int addr = 0; addr < NUM_SENSORS; ++addr) {
        for (int TCA_channel = 0; TCA_channel < TCA_CHANNEL_COUNT; TCA_channel++) {
            distanceValues[TCA_channel + addr * TCA_CHANNEL_COUNT] = calculateDistance(ZValues[TCA_channel][addr]);
        } 
    }

    // Initialising and Formating String Transmission
    for (int i = 0; i < NUM_SENSORS * TCA_CHANNEL_COUNT; i++) {
      filteredValues += String(lp[i].filt(distanceValues[i]));
      // Add delimiter between all values that isn't the last one
      if (i != NUM_SENSORS * TCA_CHANNEL_COUNT - 1) { 
          filteredValues += ", ";
      }
    }
    Serial.println(filteredValues); //Debuging
    SerialBT.println(filteredValues);
}
// Assigns valves and pumps as output devices and turns them off.
void initializePins() { 
  // Buzzer Pin
  pinMode(BUZZER_PIN, OUTPUT);
}
/*================================================================================================
                                          Debugging
================================================================================================*/
// Print initial z-axis values 
void printInitialZValues() {
  for (int TCA_channel = 0; TCA_channel < TCA_CHANNEL_COUNT; TCA_channel++) {
    for (int addr = 0; addr < NUM_SENSORS; ++addr) {
      Serial.print(InitialZValues[TCA_channel][addr],0);
      if (!(TCA_channel == TCA_CHANNEL_COUNT - 1 && addr == NUM_SENSORS - 1)) {
        Serial.print(",");                                                            // Add delimiter between all values that isn't the last one
      }
    }   
  }
  Serial.println();
}
// Print current z-axis values 
void printZValues() {
  for (int TCA_channel = 0; TCA_channel < TCA_CHANNEL_COUNT; TCA_channel++) {
    for (int addr = 0; addr < NUM_SENSORS; ++addr) {
      Serial.print(ZValues[TCA_channel][addr],0);
      if (!(TCA_channel == TCA_CHANNEL_COUNT - 1 && addr == NUM_SENSORS - 1)) {
        Serial.print(",");                                                            // Add delimiter between all values that isn't the last one
      }
    }   
  }
  Serial.println();
  Serial.println("-----------------------------------------------");
}