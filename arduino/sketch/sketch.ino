#include <ArduinoSTL.h>
std::vector <float> voltages;
int total_cells = 4;

void voltage_sensing() {
  // sensing voltage of each cell
  int i = 0;
  for (int pin=97; pin>97-total_cells; pin--)
  {
    int i = 0;
    voltages[i++] = analogRead(pin);
  }
}

void setup() {
  // put your setup code here, to run once:
float min_temp; //Re-edit later with #define
float max_temp; //Re-edit later with #define
float max_voltage; //Re-edit later with #define
float min_voltage; //Re-edit later with #define
float current_instantaneous;
float voltage_instantaneous;
float temp_instantaneous;
int master_cutoff_pin;
int direction_current;
float cellVoltages[99];
}

void loop() {
  // put your main code here, to run repeatedly:
  voltage_sensing();
}
