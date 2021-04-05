    temp_sense.push_back(m);                   // read analog volt from sensor and save to vector temp_sense
}                                                                            // convert the analog volt to its temperature equivalent  
}                                                                             // for LM35 IC we have to multiply temperature with 0.48828125
   /*LM35 sensor has three terminals - Vs, Vout and GND. We will connect the sensor as follows −

Connect the +Vs to +5v on your Arduino board.
Connect Vout to Analog0 or A0 on Arduino board.
Connect GND with GND on Arduino.

The Analog to Digital Converter (ADC) converts analog values into a digital approximation based on the formula ADC Value = sample * 1024 / reference voltage (+5v).
So with a +5 volt reference, the digital approximation will be equal to input voltage * 205.   */                                                                 
                                                                         

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
    volt_measurement m(analogRead(pin)*(5/1024),pin);
    voltages.push_back(m);
  }
}

void setup() {
Serial.begin(9600);
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  voltage_sensing();
  current = current_sensing();
Temperature_sense();
}

  current = current_sensing();
Temperature_sense();
}
