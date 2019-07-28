#include <HM10BLE.h>

HM10BLE Ble;

void setup() {
  Ble.begin(115200);
  Serial.begin(115200);
}

void loop() {
  int roll, pitch, yaw, throttle;
  Ble.receive_user_rpyt(roll, pitch, yaw, throttle);
  
  print_rpyt(roll, pitch, yaw, throttle);
}

void print_rpyt(int roll, 
                int pitch, 
                int yaw, 
                int throttle) {
                  
  Serial.print("R:");
  Serial.print(roll);
  Serial.print('\t');
  Serial.print("P:");
  Serial.print(pitch);
  Serial.print('\t');
  Serial.print("Y:");
  Serial.print(yaw);
  Serial.print('\t');
  Serial.print("T:");
  Serial.print(throttle);
  Serial.print('\n');      
}

