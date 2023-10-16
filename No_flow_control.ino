// This code is for recording the flow rate with no valve control.

#include <HoneywellZephyrI2C.h>
#include "math.h"
#include "Wire.h"
#include <util/delay.h>
#define valve_pin 8

#include "Adafruit_HTU21DF.h"

// Connect Vin to 3-5VDC
// Connect GND to ground
// Connect SCL to I2C clock pin (A5 on UNO)
// Connect SDA to I2C data pin (A4 on UNO)

Adafruit_HTU21DF htu = Adafruit_HTU21DF();


// construct a 50 SCCM sensor with address 0x49
ZephyrFlowRateSensor sensor(0x49, 300, ZephyrFlowRateSensor::SLPM);


const int buttonPin = 11;

bool breathing_status=LOW; // HIGH for breathing IN and LOW for breathing OUT
bool time_count_enable=HIGH;
bool valve_status=HIGH;
int time_count=0; 
int initial_time_count=0;
int initial_time_count_limit = 50;

float upper_cutoff = 30;
float lower_cutoff = -30;

float upper_cutoff_limit = 150;
float lower_cutoff_limit = -150;

float max_slope = 0; // maximum slope
float min_slope = 0; // minimum slope
float slope_iteration = 400; // system will take max/min value of perious 'avg_slope_iteration' samples
int slope_iter_count = 0; 
float alpha = 0.01;

float old_humidity_data = 0;
float exp_slope_data = 0;
float current_humidity_data=0;

float flow_value = 0;
float CO2_level[2]={0.0};



void setup() 
{
  Serial.begin(9600); // start Serial communication

  if (!htu.begin()) 
  {
    Serial.println("Couldn't find sensor!");
    while (1);
  }
  
  initialise_device();
  //initialise_interrupts();
  time_count=0;
  
}

void initialise_interrupts()
{
  noInterrupts();           // disable all interrupts

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 156;                      // compare match register 16MHz/256/2Hz 
  TCCR1A |= (1 << WGM01);           // turn ON CTC mode
  TCCR1B |= (1<<CS10)|(1<<CS12);    // 64 prescaler 
  TIMSK1 |= (1 << OCIE0A);          // enable timer compare interrupt

  interrupts();
}

void initialise_device()
{ 

  Wire.begin(); // start 2-wire communication
  //Wire.setClock(400000L); // this sensor supports fast-mode
  
  Serial.println("LABEL,TIME_STAMP,flow_rate,Himidity, valve_status, breathing_status");
  pinMode(buttonPin, INPUT);
  pinMode(valve_pin, OUTPUT);
  digitalWrite(valve_pin, LOW);
}

void loop() 
{ 
  float slope_value = check_slope();
  breathing_status = digitalRead(buttonPin);
  valve_status=HIGH;

  Serial.print("DATA,TIME,");
//  Serial.print(slope_value);  
//  Serial.print(",");
//  Serial.print(upper_cutoff);
//  Serial.print(",");
//  Serial.print(lower_cutoff);
//  Serial.print(",");
  Serial.print(check_flow());
  Serial.print(",");
  Serial.print(current_humidity_data);
  Serial.print(",");
  Serial.print(valve_status);
  Serial.print(",");
  Serial.print(breathing_status);
  Serial.println(" ");

//  if(initial_time_count >= initial_time_count_limit)
//  {    
//    if(slope_value >= upper_cutoff)
//    {
//      digitalWrite(valve_pin, HIGH);
//      valve_status = LOW;// Valve closes
//      time_count=0;
//    }
//     
//    else if(slope_value < lower_cutoff)
//    {
//      digitalWrite(valve_pin, LOW);
//      valve_status = HIGH;// Valve opens
//    } 
//  }
}

float check_flow()
{
  // the sensor returns 0 when new data is ready
  if( sensor.readSensor() == 0 )
  {
    float flow_value=0;
    float Temperature_of_gas=300;
    float pressure_of_gas=82;//1209;
    
    //flow_value=(sensor.flow() * 14.696 * Temperature_of_gas)/(273.15 * pressure_of_gas);
    flow_value=sensor.flow();
    return(flow_value);
  }
}

float check_slope()
{  
   
  old_humidity_data=exp_slope_data;

  float slope =0;
  int cnt=1;
  float temp_data[cnt];
  float total=0;
  
  for (int i=0;i<cnt;i++)
  { 
   current_humidity_data=htu.readHumidity();
   exp_slope_data =  exp(0.15*current_humidity_data);
   total=total + exp_slope_data;
  }

  total=total/cnt;
  exp_slope_data = total;
  
  time_count++;
       
  if(initial_time_count < initial_time_count_limit)
  {
   initial_time_count++;
  }   
   
  slope=(exp_slope_data - old_humidity_data)/2;
    
  if(initial_time_count < initial_time_count_limit)
  {
     Serial.println("Processing....");
  }
  else
  {
    if(slope_iter_count < slope_iteration)
    {
       if(slope > max_slope)
       {
        max_slope = slope;
       }
       
       else if(slope < min_slope)
       {
        min_slope  = slope;
       }
       
       update_threshold(max_slope, min_slope);
       slope_iter_count++;   
    }
  
    else 
    {
       max_slope = 0;
       min_slope = 0; 
       slope_iter_count = 0;     
    }
  }
  return(slope);
}

void  update_threshold(int a, int b)
{
  upper_cutoff = upper_cutoff - (alpha * (upper_cutoff    - (0.3 * a))); // 30% of the maximum slope value
  lower_cutoff = lower_cutoff - (alpha * (lower_cutoff    - (0.3 * b))); // 30% of the minimum slope value

  if (upper_cutoff <=  upper_cutoff_limit)
  {
    upper_cutoff = upper_cutoff_limit;
  }

  if (lower_cutoff >= lower_cutoff_limit)
  {
    lower_cutoff = lower_cutoff_limit;
  }
}



ISR(TIMER1_COMPA_vect)      // timer compare interrupt service routine
{ 
  time_count+=0.01;    
}
