//variables for master cut-off
const int relayPin = 22;


//turn on the relay
void turnOn()
{
  digitalWrite(relayPin, HIGH);//normally open
  }
// turn off the relay
void turnOff()
{
  digitalWrite(relayPin, LOW);//normally close
}


//charging-discharging
bool direction_of_flow_of_current()
{
  double current=current_sensing();
  if(current>0)
  {
    return 1;
  }
  else
  { 
    return 0;
  } 
}
