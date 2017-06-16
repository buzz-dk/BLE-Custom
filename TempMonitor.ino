// A Bluetooth LE temperature monitor for arduino 101
// Inspired by:
// https://create.arduino.cc/projecthub/monica/getting-started-with-bluetooth-low-energy-ble-ab4c94
// A temperature sensor is attached to an analog input pin: DFRobot LM35 V2
// https://www.dfrobot.com/wiki/index.php/DFRobot_LM35_Linear_Temperature_Sensor_(SKU:DFR0023)
// This sensor has a very low output voltage. Thus some averaging and smoothing is applied.

#include <CurieBLE.h>

const unsigned char ledPin = 13;              // On-board LED
const unsigned char NUM_SAMPLES_POLLED (32);  // Number of polled samples
const unsigned char NUM_SAMPLE_SUMS (32);     // Number of sample sums!

BLEService tempService("fef431b0-51e0-11e7-9598-0800200c9a66"); // Create custom BLE Service

BLEUnsignedIntCharacteristic  tempChar("fef431b0-51e0-11e7-9598-0800200c9a66", BLERead | BLENotify); // allows remote device to get notifications

int oldTemp = 0;          // last temperature reading from analog input
long previousMillis = 0;  // last time the temperature was checked, in ms

void setup() {
  Serial.begin(9600);       // initialize serial communication - for debugging etc.
  pinMode(ledPin, OUTPUT);  // initialize the LED on pin 13 to indicate when a central is connected

  BLE.begin();              // begin initialization

  // Set a local name for the BLE device. This name will appear in advertising packets and can be used by remote devices to identify this BLE device.
  // The name can be changed but maybe be truncated based on space left in advertisement packet

  BLE.setLocalName("TempMonitor");
  BLE.setAdvertisedService(tempService);    // add the service UUID
  tempService.addCharacteristic(tempChar);  // add the temperature characteristic
  BLE.addService(tempService);              // Add the BLE service
  tempChar.setValue(oldTemp);               // initial value for this characteristic

  // Start advertising BLE. It will start continuously transmitting BLE advertising packets and will be visible to remote BLE central devices
  // until it receives a new connection

  BLE.advertise();

  Serial.println("Bluetooth device active, waiting for connections...");
  analogReadResolution(12); // This should be OK on Arduino 101
}

void loop() {
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: "); Serial.println(central.address());  // print the central's MAC address
    digitalWrite(ledPin, HIGH);  // turn on the LED to indicate the connection

    // check the temperature every 5000ms as long as the central is still connected:
    while (central.connected()) {
      long currentMillis = millis();
      if (currentMillis - previousMillis >= 5000) {
        previousMillis = currentMillis;
        updateTemperature();
      }
    }
    digitalWrite(ledPin, LOW);   // when the central disconnects, turn off the LED.
    Serial.print("Disconnected from central: "); Serial.println(central.address());
  }
}

void updateTemperature() {
  int sampleSum1 = getSamples();                  // Get the sum of a lot of samples!
  int sampleSum2 = getMovingAverage(sampleSum1);  // Get an average of the last many samples!
  Serial.print("Sample sum: "); Serial.print(sampleSum1); Serial.print(" Moving average: "); Serial.println(sampleSum2);

  // Calibration constants:
  const int CAL_X1 (6500); // The sample sum at 15.5 degrees
  const int CAL_X2 (9330); // The sample sum at 22.2 degrees
  const int CAL_Y1 (155);  // Measured 15.5 degrees (°C)
  const int CAL_Y2 (222);  // Measured 22.2 degrees (°C)

  // Transformation of sample sum average to temperature:
  // You should divide by 10 for a floating point result.
  // The line equation: y = m(x-x0) + y0
  int temp = (CAL_Y2 - CAL_Y1) * (sampleSum2 - CAL_X1) / (CAL_X2 - CAL_X1) + CAL_Y1; // The temperature (* 10) °C

  if (temp != oldTemp) {      // if the temperature has changed
    Serial.print("Temperature: "); Serial.println(temp);
    tempChar.setValue(temp);  // update the temperature characteristic
    oldTemp = temp;           // save for next comparison
  }
}

// https://en.wikipedia.org/wiki/Moving_average
uint16_t getMovingAverage(uint16_t sampleSum)
{
  static unsigned long sampleSums[NUM_SAMPLE_SUMS] = {0};
  static unsigned int sampleSumCount = 0;
  static  unsigned int sampleSumIndex = 0;
  unsigned long sampleSumSum = 0;
  unsigned char j = 0;
  sampleSums[sampleSumIndex] = sampleSum;
  if (sampleSumCount < NUM_SAMPLE_SUMS) {
    sampleSumCount ++;
  }
  sampleSumIndex ++;
  if (sampleSumIndex == NUM_SAMPLE_SUMS) {
    sampleSumIndex = 0;
  }
  for (j = 0; j < sampleSumCount; j++) {
    sampleSumSum += sampleSums[j];
  }
  return sampleSumSum / sampleSumCount;
}

uint16_t getSamples() {
  uint16_t sampleSum = 0;
  int i;
  for (i = 0; i < NUM_SAMPLES_POLLED; i++) {
    sampleSum += analogRead(A0);  // Repeatedly read the current voltage level on the A0 analog input pin. Sum up the samples.
  }
  return sampleSum; // Note: a sum is statistically equal to an average
}

