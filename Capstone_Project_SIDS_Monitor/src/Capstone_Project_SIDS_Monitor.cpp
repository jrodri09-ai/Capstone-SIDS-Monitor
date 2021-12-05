/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "/Users/jessicamacbookpro/Documents/IoT/Capstone-SIDS-Monitor/Capstone_Project_SIDS_Monitor/src/Capstone_Project_SIDS_Monitor.ino"
/*
 * Project: Capstone_Project_SIDS_Monitor
 * Description: Smart baby monitor to help prevent SIDS
 * Author: Jessica Rodriquez
 * Date: 11/29/2021
 */

void setup();
void loop();
void startDisplay();
void setupPins();
void startSpo2();
void startWifi();
void displayTime();
void findBreaths();
void sampleHeartRate();
void rockCrib();
void soundAlarm();
bool lowbreathrate();
void vibeOn();
void vibeOff();
#line 8 "/Users/jessicamacbookpro/Documents/IoT/Capstone-SIDS-Monitor/Capstone_Project_SIDS_Monitor/src/Capstone_Project_SIDS_Monitor.ino"
SYSTEM_MODE(SEMI_AUTOMATIC)

#include <Adafruit_SSD1306.h>
#include "credentials.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "math.h"
#include "IoTTimer.h"
#include "Stepper.h"
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

#define OLED_RESET D4
#define BUTTONPIN D2
#define THREADPIN A5
#define VIBEPIN D11

const int MAX_BRIGHTNESS = 255;
const int stepsPerRevolution = 2048;

static int breaths = 0;

int resistance;
int buttonpress;

String DateTime, TimeOnly;

// HeartRate Vars
uint32_t irBuffer[100];  //infrared LED sensor data
uint32_t redBuffer[100]; //red LED sensor data
int32_t bufferLength;    //data length
int32_t spo2;            //SPO2 value
int8_t validSPO2;        //indicator to show if the SPO2 calculation is valid
int32_t heartRate;       //heart rate value
int8_t validHeartRate;   //indicator to show if the heart rate calculation is valid
byte pulseLED = 11;      //Must be on PWM pin
byte readLED = D7;       //Blinks with each data read

unsigned int frequency = 396;
unsigned long duration = 1000;
unsigned long last, lastTime, lastMin, current;

// Timers
IoTTimer Timer;
// HeartRate
MAX30105 particleSensor;
// OLED
Adafruit_SSD1306 display(OLED_RESET);
// Stepper Motor
Stepper myStepper(stepsPerRevolution, A1, A3, A2, A4);
// MQTT for dash board
TCPClient TheClient;
Adafruit_MQTT_SPARK mqtt(&TheClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish mqttObjRes = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/resistance");

void setup()
{
  Serial.begin(115200); // initialize serial communication at 115200 bits per second:
  waitFor(Serial.isConnected, 3000);
  // set the speed at 60 rpm:
  setupPins();
  myStepper.setSpeed(12);
  // startWifi();
  // startDisplay();
  Timer.startTimer(15000);
  // Initialize sensor
  startSpo2();
}

void loop()
{
  // Validate connected to MQTT Broker
  // MQTT_connect();
  // displayTime();

  buttonpress = digitalRead(BUTTONPIN);
  if (buttonpress)
  {
    Serial.println("Button is pressed");
  }
  // soundAlarm();
  // findBreaths();
  // rockCrib();
  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  // sampleHeartRate();

  vibeOn();
  Serial.printf("Vibe on\n");
  delay(1000);
  vibeOff();
  Serial.printf("Vibe off\n");
}

void startDisplay()
{
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(500); // Pause for .5 seconds
  display.clearDisplay();
  display.drawPixel(10, 10, WHITE);
  display.display();
  display.clearDisplay();
}

void setupPins()
{
  pinMode(BUTTONPIN, INPUT);
  pinMode(THREADPIN, INPUT);
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);
}

void startSpo2()
{
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.printf("MAX30105 was not found. Please check wiring/power.");
    while (1)
      delay(1000);
  }

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4;  //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;        //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100;   //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;    //Options: 69, 118, 215, 411
  int adcRange = 4096;     //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0; i < bufferLength; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check();                   //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    Serial.printf("red = %i, ir = %i \n", redBuffer[i], irBuffer[i]);
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
}

void startWifi()
{
  WiFi.connect();
  while (WiFi.connecting())
  {
    Serial.printf("connecting");
  }
  Time.zone(-7);
  Particle.syncTime();
}

void displayTime()
{
  DateTime = Time.timeStr();
  TimeOnly = DateTime.substring(11, 19);
  display.printf("Time is %s\n", TimeOnly.c_str());
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(WHITE);        // Draw white text
  display.setCursor(0, 0);            // Start at top-left corner
  display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
  display.display();
}

void findBreaths()
{
  resistance = analogRead(THREADPIN);
  Serial.printf("Resistance=%i\n", resistance);
  delay(500);

  // //publish to cloud every 6 seconds
  //   if((millis()-lastTime > 6000)) {
  //     if (mqtt.Update()) {
  //       //mqttObjRes.publish();
  //       //Serial.printf("Publishing %f\n",mqttObjRes);
  //     }
  //     lastTime = millis();
  //   }

  current = analogRead(THREADPIN);
  if ((current < 2000) && (last > 2000))
  {
    breaths++;
  }
  last = current;

  if (Timer.isTimerReady())
  {

    Serial.printf("Breaths per 15 seconds=%i\n", breaths);
    breaths = 0;
    Timer.startTimer(15000);
  }
}

void sampleHeartRate()
{
  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second

  delay(100); // allow the code to pause for a moment
  //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
  for (byte i = 25; i < 100; i++)
  {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25] = irBuffer[i];
  }

  //take 25 sets of samples before calculating the heart rate.
  for (byte i = 75; i < 100; i++)
  {
    while (particleSensor.available() == false)
    {                         //do we have new data?
      particleSensor.check(); //Check the sensor for new data
    }
    digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    //send samples and calculation result to terminal program through UART
    if (((validHeartRate == 1) || (validSPO2 == 1)) && (i == 99))
    {
      Serial.printf("red = %i, ir = %i, HR = %i, HRvalid = %i, sp02 = %i, SPO2valid = %i \n", redBuffer[i], irBuffer[i], heartRate, validHeartRate, spo2, validSPO2);
    }
  }
  //After gathering 25 new samples recalculate HR and SP02
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
}

void rockCrib()
{
  Serial.println("clockwise");
  myStepper.step(-stepsPerRevolution);
}

void soundAlarm()
{
  if (lowbreathrate())
  {
    tone(2, frequency, duration);
  }
  else
  {
    noTone(2);
  }
}

bool lowbreathrate()
{
  return false;
}

void vibeOn()
{
  digitalWrite(VIBEPIN, HIGH);
}

void vibeOff()
{
  digitalWrite(VIBEPIN, LOW);
}

// Function to connect and reconnect as necessary to the MQTT server.
// void MQTT_connect()
// {
//   int8_t ret;
//   // Stop if already connected.
//   if (mqtt.connected())
//   {
//     return;
//   }
//   Serial.print("Connecting to MQTT... ");
//   while ((ret = mqtt.connect()) != 0)
//   { // connect will return 0 for connected
//     Serial.printf("%s\n", (char *)mqtt.connectErrorString(ret));
//     Serial.printf("Retrying MQTT connection in 5 seconds..\n");
//     mqtt.disconnect();
//     delay(5000); // wait 5 seconds
//   }
//   Serial.printf("MQTT Connected!\n");
// }
