/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "/Users/jessicamacbookpro/Documents/IoT/Capstone-Final/SPO2_test_code/src/SPO2_test_code.ino"
/*
 * Project SPO2_test_code
 * Description:
 * Author:
 * Date:
 */

void setup();
void loop();
void processHRandSPO2();
void sync_my_time ();
void displayPrint ();
#line 8 "/Users/jessicamacbookpro/Documents/IoT/Capstone-Final/SPO2_test_code/src/SPO2_test_code.ino"
SYSTEM_MODE(SEMI_AUTOMATIC)

#include "algorithm_by_RF.h"
#include "max30102.h"
#include "MAX30105.h"                                     // MAX3010x library
#include <Adafruit_GFX.h> 
#include  "Adafruit_SSD1306.h"                               // OLED Library

#define OLED_RESET D4
Adafruit_SSD1306 display(OLED_RESET);


// Interrupt pin
const byte oxiInt = D8;                                   // pin connected to MAX30102 INT
MAX30105 particleSensor;

uint32_t aun_ir_buffer[BUFFER_SIZE];                      //infrared LED sensor data
uint32_t aun_red_buffer[BUFFER_SIZE];  
uint32_t hr;                                              //red LED sensor data
float old_n_spo2, spo2;                                   // Previous SPO2 value
uint8_t uch_dummy,k;
bool isWristPlaced = false;

unsigned long lastDisplayTime;
unsigned long lastPublishTime;
unsigned long last;

double varBodyTempC;
float varBodyTempF;
int status;

const int MPU_ADDR = 0x68;

byte accel_x_h, accel_x_l; //X Data/var for individual bytes
int16_t accel_x; //X Data var to store the x-acceleration

byte accel_y_h, accel_y_l; //Y Data/var for individual bytes
int16_t accel_y; //Y Data/var to store the x-acceleration

byte accel_z_h, accel_z_l; //Z Data/var for individual bytes
int16_t accel_z; //Z Data/var to store the x-acceleration

float accelXG;
float accelYG;
float accelZG;  
float accelTotal;

bool fallDetected = false;
float fallValue = 0.0;
const float fallThreshold = 1.5;

const int buttonPin = D2;


void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);              // Start the OLED display
  display.clearDisplay();
  display.display();
  display.setTextSize(2);
  display.setCursor(0,0);
  display.setTextColor(WHITE);
  display.println("Place on\nwrist and\nwait...");
  display.display();

  pinMode (buttonPin, INPUT_PULLDOWN);                    //pin D2 connects to the button
  pinMode(oxiInt, INPUT);                                 //pin D10 connects to the interrupt output pin of the MAX30102
  Wire.begin();

  particleSensor.begin(Wire, I2C_SPEED_FAST);             // Use default I2C port, 400kHz speed
  particleSensor.setup();  
  
  Serial.begin(9600);
  maxim_max30102_reset();                                 //resets the MAX30102
  delay(1000);
  maxim_max30102_read_reg(REG_INTR_STATUS_1,&uch_dummy);  //Reads/clears the interrupt status register
  maxim_max30102_init();                                  //initialize the MAX30102
  old_n_spo2=0.0;


  sync_my_time();
}

//Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every ST seconds
void loop() {
  processHRandSPO2();
  displayPrint();
  if(!isWristPlaced) {  
  }
}

void processHRandSPO2(){
  long irValue = particleSensor.getIR();                  // Reading the IR value it will permit us to know if there's a finger on the sensor or not
  Serial.printf("irValue: %i\n", irValue);
  if (irValue > 50000){
    if(isWristPlaced == false) {
      isWristPlaced = true;
    }
    float n_spo2,ratio,correl;                            //SPO2 value
    int8_t ch_spo2_valid;                                 //indicator to show if the SPO2 calculation is valid
    int32_t n_heart_rate;                                 //heart rate value
    int8_t  ch_hr_valid;                                  //indicator to show if the heart rate calculation is valid
    int32_t i;
       
    //buffer length of BUFFER_SIZE stores ST seconds of samples running at FS sps
    //read BUFFER_SIZE samples, and determine the signal range
    for(i=0;i<BUFFER_SIZE;i++)
    {
      while(digitalRead(oxiInt)==1){                      //wait until the interrupt pin asserts
        //  yield();
      }
      // while (particleSensor.available() == false) {//do we have new data?
      //   particleSensor.check(); //Check the sensor for new data
      // }

      //IMPORTANT: 
      //IR and LED are swapped here for MH-ET MAX30102. Check your vendor for MAX30102
      //and use this or the commented-out function call.
      maxim_max30102_read_fifo((aun_ir_buffer+i), (aun_red_buffer+i));
      // maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));
    }
  
    //calculate heart rate and SpO2 after BUFFER_SIZE samples (ST seconds of samples) using Robert's method
    rf_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid, &ratio, &correl); 
    Serial.printf("n_spo2: %0.1f\n", n_spo2);
    Serial.printf("ch_spo2_valid: %i\n", ch_spo2_valid);
    Serial.printf("n_heart_rate: %i \n", n_heart_rate);
    Serial.printf("ch_hr_valid: %i\n", ch_hr_valid);

    spo2 = n_spo2;
  }
  else {
    isWristPlaced = false;
    // Serial.println("No wrist detected");
  }
}


void sync_my_time () {
  Time.zone(-7);
  Particle.syncTime();
  waitUntil(Particle.syncTimeDone);
}

void displayPrint () {
  String DateTime, DateOnly, TimeOnly, YearOnly;
  char currentDate[11], currentTime[9], currentYear[5];

  DateTime = Time.timeStr();

  DateOnly = DateTime.substring(0,10);
  DateOnly.toCharArray(currentDate,11);

  YearOnly = DateTime.substring(20,24);
  YearOnly.toCharArray(currentYear,5);

  TimeOnly = DateTime.substring(11,19);
  TimeOnly.toCharArray(currentTime,9);

  if (isWristPlaced == true) {
    if ((millis()-lastDisplayTime)>10000) {
      display.clearDisplay();
      display.display();

      display.setTextSize(2);
      display.setCursor(0,0);
      // display.setTextColor(BLACK, WHITE);
      // // Serial.printf("Display Date: %s %s \n", currentDate,currentYear);
      // display.printf("Date: %s %s", currentDate,currentYear);
      // // Serial.printf("Display Time: %s \n", currentTime);
      // display.printf("Time: %s\n", currentTime);

      if (hr>1 && spo2>1) {
        display.setTextColor(WHITE);
        // Serial.printf("Display Heart Rate: %i \n", hr);
        display.println();
        display.printf("HR: %i \n", hr);

        display.setTextColor(WHITE);
        // Serial.printf("Display Oxygen: %0.1f \n", spo2);
        display.printf("O2: %0.1f \n", spo2);

        display.setTextColor(WHITE);
        // Serial.printf("Display Temp: %0.1f  \n", varBodyTempF);   
        display.printf("T:  %0.1f  \n", varBodyTempF);  

        // display.setTextColor(WHITE);
        // display.printf("Accel Total: %0.2f \n", fallValue);
      }
      else {
        display.setTextColor(WHITE);
        display.println();
        display.println("Measuring Vitals...");    
      }
    
      display.display();
      lastDisplayTime = millis();
    }
  }
}

// void pushAlertButton () {
//   bool buttonState;
//   static bool lastButton;

//   buttonState = digitalRead(buttonPin);
//   if(buttonState != lastButton) {
//     if(buttonState == HIGH) {
//       if(mqtt.Update()) {
//         button.publish(buttonState);
//         Serial.printf("Alert Button Pressed \n"); 
//       } 
//     }
//   lastButton = buttonState;
//   }
// }

