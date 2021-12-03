/*
 * Project: Capstone_Project_SIDS_Monitor
 * Description: Smart baby monitor to help prevent SIDS
 * Author: Jessica Rodriquez
 * Date: 11/29/2021
 */

SYSTEM_MODE(SEMI_AUTOMATIC)

#include <Adafruit_SSD1306.h>
#include "credentials.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT.h" 
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "math.h"
#include "IoTTimer.h"
#include "Stepper.h"


String DateTime, TimeOnly;

IoTTimer Timer; 


#define OLED_RESET  D4 
#define BUTTONPIN D2
#define THREADPIN A5

int buttonpress;
int resistance;
static int breaths;
const int stepsPerRevolution =2048;


unsigned int frequency = 396;
unsigned long duration = 1000;
unsigned long last, lastTime, lastMin, current;

Adafruit_SSD1306 display(OLED_RESET);
TCPClient TheClient; 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY);
Adafruit_MQTT_Publish mqttObjRes = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/resistance");
Stepper myStepper(stepsPerRevolution, A1, A3, A2, A4);


void setup() {
  Serial.begin(9600);
// set the speed at 60 rpm:
  myStepper.setSpeed(12);
  
  Timer.startTimer(15000);

  WiFi.connect();
  while(WiFi.connecting()) {
    //Serial.printf("connecting");
  }

  Time.zone(-7);
  Particle.syncTime();

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(2000); // Pause for 2 seconds
  display.clearDisplay();
  display.drawPixel(10, 10, WHITE);
  display.display();
  display.clearDisplay();

pinMode(BUTTONPIN,INPUT);
pinMode(THREADPIN,INPUT);
}


void loop() {
   // Validate connected to MQTT Broker
  //MQTT_connect();

  DateTime = Time.timeStr();
  TimeOnly = DateTime.substring(11,19);
  display.printf("Time is %s\n",TimeOnly.c_str());
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.setTextColor(BLACK,WHITE); // Draw 'inverse' text
  display.display();
 

  buttonpress=digitalRead(BUTTONPIN);
  if (buttonpress) {
   Serial.println("Button is pressed");
  }

  if(lowbreathrate) {
    tone(2, frequency, duration);
  }
else {
  noTone(2);
}

resistance = analogRead(THREADPIN);
Serial.printf("Resistance=%i\n",resistance);
delay(500);

//publish to cloud every 6 seconds
  if((millis()-lastTime > 6000)) {
    if (mqtt.Update()) {
      //mqttObjRes.publish();
      //Serial.printf("Publishing %f\n",mqttObjRes);
    }
    lastTime = millis();
  }


  current = analogRead(THREADPIN);
  if((current<2000)&&(last>2000)) {
    breaths++;
  }
  last = current;

  if(Timer.isTimerReady()){

 
Serial.printf("Breaths per 15 seconds=%i\n",breaths);
breaths=0;
Timer.startTimer(15000);
}

Serial.println("clockwise");
  myStepper.step(-stepsPerRevolution);
}



void lowbreathrate() {
  
}



// // Function to connect and reconnect as necessary to the MQTT server.
// void MQTT_connect() {
//   int8_t ret;
 
//   // Stop if already connected.
//   if (mqtt.connected()) {
//     return;
//   }
 
//   Serial.print("Connecting to MQTT... ");
 
//   while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
//        Serial.printf("%s\n",(char *)mqtt.connectErrorString(ret));
//        Serial.printf("Retrying MQTT connection in 5 seconds..\n");
//        mqtt.disconnect();
//        delay(5000);  // wait 5 seconds
//   }
//   Serial.printf("MQTT Connected!\n");
// }
