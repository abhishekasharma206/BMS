
std::vector<float> temp_sense;
double temp;

void Temperature_sense(){
  int cell = 0;
  for(int tempPin=90;tempPin>90-total_cells;tempPin--){   // maximum 7 cells
  temp_sense[cell] = analogRead(tempPin);// read analog volt from sensor and save to variable temp
   temp_sense[cell] *= 0.48828125;  // convert the analog volt to its temperature equivalent
                             // for LM35 IC we have to multiply temperature with 0.48828125
   cell++;
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

}
