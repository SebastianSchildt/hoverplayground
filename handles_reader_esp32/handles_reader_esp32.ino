#include <ArduinoJson.h>

#define FILTER_LEN  64

//Will only send new data if at least TIMEDELTA ms have elapses
#define TIMEDELTA 100
//timestamp of last transmissions (will roll over rafter 50 days, so no speial handling)
unsigned long last_send_ts;

const int A0Pin=34;
const int A1Pin=35;

uint32_t A0_Buffer[FILTER_LEN];
uint16_t A0_i = 0;
uint32_t A0_Raw = 0;
uint32_t A0_Filtered = 0;
uint32_t A0_Max = 0;
uint32_t A0_Min = 0xFFFFFFFF;


uint32_t A1_Buffer[FILTER_LEN];
uint16_t A1_i = 0;
uint32_t A1_Raw = 0;
uint32_t A1_Filtered = 0;
uint32_t A1_Max = 0;
uint32_t A1_Min = 0xFFFFFFFF;

uint32_t samples_since_last_transmission=0;
DynamicJsonDocument doc(255);

uint32_t rescale(uint32_t input, uint32_t min_val, uint32_t max_val, uint32_t range, uint32_t min_thread) {
  float a = 0;
  float res = 0;
  if (input < min_val) {
    input=min_val;
  }
  else if (input > max_val) {
    input=max_val;
  }

  a = (input - min_val);
  res = range * a / ((max_val - min_val));
  
  if (res < min_thread) {
    res = 0;
  }
  return res;
}

void find_max(uint32_t input, uint32_t *max_val){
 if(input > *max_val){
  *max_val = input;
 }
}

void find_min(uint32_t input, uint32_t *min_val){
 if(input < *min_val){  
  *min_val = input;
 }
}

uint32_t readADC_Avg(uint32_t ADC_Raw, uint32_t *An_Buffer, uint16_t *An_i)
{
  uint16_t i = 0;
  uint64_t Sum = 0;
  
  
  An_Buffer[*An_i] = ADC_Raw;
  *An_i = *An_i +1;

  if(*An_i == FILTER_LEN)
  {
    *An_i = 0;
  }
  for(i=0; i<FILTER_LEN; i++)
  {

    Sum += An_Buffer[i];
  }
  return (uint32_t)(Sum/FILTER_LEN);
}

void setup() {

  for(uint16_t i=0; i<FILTER_LEN; i++)
  {
    A0_Buffer[i] = analogRead(A0Pin);
    A1_Buffer[i] = analogRead(A1Pin);
  }

 last_send_ts=millis();
  Serial.begin(115200);
  while (!Serial); //wait for serial
  
}


void send() {
  doc["a0"]=rescale(A0_Filtered, 940, 3144, 1000, 10);
  doc["a1"]=rescale(A1_Filtered, 942, 3138, 1000, 10);
  doc["a0_min"] = A0_Min;
  doc["a0_max"] = A0_Max;
  doc["a1_min"] = A1_Min;
  doc["a1_max"] = A1_Max;
  doc["samples"] = samples_since_last_transmission;

  char out[255];
  serializeJson(doc, out);
  strcat(out, "\n");
  Serial.write(out);

}


void sampleInputs() {
  samples_since_last_transmission++;
  A0_Raw = analogRead(A0Pin);
  A0_Filtered = readADC_Avg(A0_Raw, A0_Buffer, &A0_i);
  
  A1_Raw = analogRead(A1Pin);
  A1_Filtered = readADC_Avg(A1_Raw, A1_Buffer, &A1_i);
  /** find min and max   **/
  find_min(A0_Filtered, &A0_Min);
  find_max(A0_Filtered, &A0_Max);
  
  find_min(A1_Filtered, &A1_Min);
  find_max(A1_Filtered, &A1_Max);  
}

void loop() {
  sampleInputs();
  if (millis() - last_send_ts > TIMEDELTA) {
    send();
    last_send_ts=millis();
    samples_since_last_transmission=0;
  }
}
