#include <Adafruit_SSD1306.h>
#include <splash.h>

#include "SeeedHeartBreathRadar.h"

Adafruit_SSD1306 display;
int lastDisplayMs = 0;
int lastRequestMs = 0;
int lastPrintMs = 0;
SeeedHeartBreathRadar radar;

void setup() {
  Serial.begin(115200);
  radar.begin( &Serial1 );
  delay(1500);
  Serial.println("Readly");
}

void loop() {
  auto tnow = millis();

  radar.recvRadarBytes();
  
  if (tnow - lastPrintMs > 500) {
    Serial.print( "HR (" );
    Serial.print( SeeedHeartBreathRadar::vitalsStateValToString( (SeeedHeartBreathRadar::VitalsStateVal) radar.getHeartRateState() ) );
    Serial.print( ") " );
    Serial.print( radar.getHeartRate() );
    Serial.print( "  RR (" );
    Serial.print( SeeedHeartBreathRadar::vitalsStateValToString( (SeeedHeartBreathRadar::VitalsStateVal) radar.getRespiratoryState() ) );
    Serial.print( ") " );
    Serial.print( radar.getRespiratoryRate() );
    Serial.print( "  at " );
    Serial.print( radar.getDistance() );
    auto angles = radar.getAngles();
    Serial.print( "  angled " );
    Serial.println( angles.first );
    lastPrintMs = tnow;
  }
}

