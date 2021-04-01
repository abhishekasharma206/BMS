// all variables defined 
// Temperature sensing function called
// current sensing function called
// voltage sensing function called






#include <ArduinoSTL.h>

std::vector <float> voltages;
const int total_cells = 4;

//Variables for current Sensing
const int adcVoltage_pin = 91; //Pin number for current sensing
int sensetivity = 185; //As per datasheet of ACS712 for range of 5A
int adcValue = 0;
int offsetVoltage = 2500; //(mV) Offset Voltage is Vcc/2. Assuming 5V supply is given through Arduino board.
double current = 0; 
double voltage = 0;

//Variables for Temperature sensing
std::vector<float> temp_sense;
double temp;


void Temperature_sense(){
  int cell = 0;
  for(int tempPin = 90 ; tempPin > 90-total_cells ; tempPin--){   // for maximum 6 cells
  temp_sense[cell] = analogRead(tempPin);// read analog volt from sensor and save to vector temp_sense
   temp_sense[cell] *= 0.48828125;  // convert the analog volt to its temperature equivalent
                             // for LM35 IC we have to multiply temperature with 0.48828125
   cell++;
}
}
double current_sensing(){
  // senses the current of the battery pack
  adcValue = analogRead(adcVoltage_pin);
  voltage = (adcValue / 1024.0) * 5000; //converts digital value to mV
  return ((voltage - offsetVoltage)/sensetivity); //returns the current sensed
}
void voltage_sensing() {
  // sensing voltage of each cell
  int i = 0;
  for (int pin=97; pin>97-total_cells; pin--)               // for maximum 6 cells
  {
    voltages[i++] = analogRead(pin);
  }
}

void setup() {
serial.begin(9600);
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
  current = current_sensing();
Temperature_sense();
}
