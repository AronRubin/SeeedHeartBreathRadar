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

struct TRect {
  int x;
  int y;
  int w;
  int h;
};

TRect g_hr_rect = { 15, 0, 15, 15 };
TRect g_rr_rect = { 45, 0, 15, 15 };

void renderLabels() {
  display.setTextColor( SSD1306_WHITE, SSD1306_BLACK );
  display.setCursor( 0, 0 );
  display.setTextSize( 1 );
  display.print( "HR " );
  g_hr_rect.x = display.getCursorX();
  g_hr_rect.y = display.getCursorY();
  display.setTextSize( 2 );
  display.print( "XX " );
  g_hr_rect.w = display.getCursorX() - g_hr_rect.x;
  g_hr_rect.h = display.getCursorY() - g_hr_rect.y;
  display.setTextSize( 1 );
  display.print( "RR " );
  g_rr_rect.x = display.getCursorX();
  g_rr_rect.y = display.getCursorY();
  display.setTextSize( 2 );
  display.print( "XX " );
  g_rr_rect.w = display.getCursorX() - g_rr_rect.x;
  g_rr_rect.h = display.getCursorY() - g_rr_rect.y;
}

void renderData() {
  display.fillRect( g_hr_rect.x, g_hr_rect.y, g_hr_rect.w, g_hr_rect.h, SSD1306_BLACK );
  display.fillRect( g_rr_rect.x, g_rr_rect.y, g_rr_rect.w, g_rr_rect.h, SSD1306_BLACK );
  display.setTextColor( SSD1306_WHITE, SSD1306_BLACK );
  display.setCursor( g_hr_rect.x, g_hr_rect.y );
  display.setTextSize( 2 );
  display.print( radar.getHeartRate() );
  display.setCursor( g_rr_rect.x, g_rr_rect.y );
  display.setTextSize( 2 );
  display.print( radar.getRespiratoryRate() );
}

void setup1() {
  delay( 1500 );
  if (!display.begin( SSD1306_SWITCHCAPVCC, 0x3C )) {
    
  }

  display.clearDisplay();
  renderLabels();
  display.display();
}

void loop1() {

  auto tnow = millis();
  
  if (tnow - lastDisplayMs > 200) {
    renderData();
    display.display();
    lastDisplayMs = tnow;
  }
}
