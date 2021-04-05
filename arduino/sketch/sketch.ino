// all variables defined 
// Temperature sensing function called
// current sensing function called
// voltage sensing function called





#include <Arduino.h>
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
  for(int tempPin = 90 ; tempPin > 90-total_cells ; tempPin--)             // for maximum 6 cells
    temp_sense.push_back(analogRead(tempPin)*0.48828125);                   // read analog volt from sensor and save to vector temp_sense
}                                                                            // convert the analog volt to its temperature equivalent  
                                                                             // for LM35 IC we have to multiply temperature with 0.48828125

   /*LM35 sensor has three terminals - Vs, Vout and GND. We will connect the sensor as follows −

Connect the +Vs to +5v on your Arduino board.
Connect Vout to Analog0 or A0 on Arduino board.
Connect GND with GND on Arduino.

The Analog to Digital Converter (ADC) converts analog values into a digital approximation based on the formula ADC Value = sample * 1024 / reference voltage (+5v).
So with a +5 volt reference, the digital approximation will be equal to input voltage * 205.   */   

void Thermal_management(){
  // opens the relay contacts if the temperature is not within the permissible limits
  bool charge;
  int pinout = 78;
  for (int i=0; i<total_cells; i++)
  {
    if (charge == TRUE)
    {
      if (temp<0 || temp>45)
      {
        digitalWrite(pinout, LOW);
      }
      else
      {
        digitalWrite(pinout, HIGH);
      }
    }
    else
    {
      if (temp<0 || temp>55)
      {
        digitalWrite(pinout, LOW);
      }
      else
      {
        digitalWrite(pinout, HIGH);
      }
    }
  }
}     
/*A 5V relay module is used                                                         
 *  The cells are connected between NC and Common pin.                                                        
 *  Ground: Connects to the ground pin on the Arduino
    5V Vcc: Connects the Arduino’s 5V pin
    Signal: Carries the trigger signal from the Arduino that activates the relay
 */ 
                         
double current_sensing(){
  // senses the current of the battery pack
  adcValue = analogRead(adcVoltage_pin);
  voltage = (adcValue / 1024.0) * 5000; //converts digital value to mV
  return ((voltage - offsetVoltage)/sensetivity); //returns the current sensed
}


void voltage_sensing() {
  // sensing voltage of each cell

  for (int pin=97; pin>97-total_cells; pin--)               // for maximum 6 cells
  {
    voltages.push_back(analogRead(pin)*5/1024);
  }
}
/* For voltage sensing voltage sensing module is used (VCC<25)
 *  It has 5 pins, VCC - positive terminal to be measured, GND - negative terminal to be measured
 *  S - Analog input to Arduino, +ve - Not Conncted, -ve pin - GND of Arduino
 */

void setup() {
Serial.begin(9600);
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  voltage_sensing();
  current = current_sensing();
  Temperature_sense();
  Thermal_management();
}
