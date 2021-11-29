/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "/Users/jessicamacbookpro/Documents/IoT/Capstone-SIDS-Monitor/Capstone_Project_SIDS_Monitor/src/Capstone_Project_SIDS_Monitor.ino"
/*
 * Project Capstone_Project_SIDS_Monitor
 * Description:
 * Author:Jessica Rodriquez
 * Date:
 */

void setup();
void loop();
#line 8 "/Users/jessicamacbookpro/Documents/IoT/Capstone-SIDS-Monitor/Capstone_Project_SIDS_Monitor/src/Capstone_Project_SIDS_Monitor.ino"
SYSTEM_MODE(SEMI_AUTOMATIC)

#include <Adafruit_SSD1306.h>

String DateTime, TimeOnly;

#define OLED_RESET  D4 
# define BUTTONPIN D2

int buttonpress;

Adafruit_SSD1306 display(OLED_RESET);

void setup() {
  Serial.begin(9600);

  Time.zone(-7);
  Particle.syncTime();

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(2000); // Pause for 2 seconds
  display.clearDisplay();
  display.drawPixel(10, 10, WHITE);
  display.display();
  //delay(2000);
  display.clearDisplay();

pinMode(BUTTONPIN,INPUT);
}


void loop() {
  DateTime = Time.timeStr();
  TimeOnly = DateTime.substring(11,19);
  display.printf("Time is %s\n",TimeOnly.c_str());
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.setTextColor(BLACK,WHITE); // Draw 'inverse' text
  display.display();
  //delay(2000);

  buttonpress=digitalRead(BUTTONPIN);
  if (buttonpress) {
   Serial.println("Button is pressed");
  }
}