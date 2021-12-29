#include <ArduinoJson.h>

#define FILTER_LEN  15

const int A0Pin=34;
const int A1Pin=35;

uint32_t A0_Buffer[FILTER_LEN] = {0};
int A0_i = 0;
int A0_Raw = 0;
int A0_Filtered = 0;
int A0_Max = 0;
int A0_Min = 0;


uint32_t A1_Buffer[FILTER_LEN] = {0};
int A1_i = 0;
int A1_Raw = 0;
int A1_Filtered = 0;
int A1_Max = 0;
int A1_Min = 0;

DynamicJsonDocument doc(128);

float rescale(int input, int min_val, int max_val, float range, float min_thread, float max_thread) {
  float a = 0;
  float res = 0;
  if ((input - min_val) > min_thread) {
    a = (input - min_val);
  }

  if ((input- max_val) > max_thread || (max_val - input) < max_thread) {
    res= range;
    return res;
  }
  res = range * a / ((max_val - min_val));
  long val = (long) (res * 10L); 
  res = (float) val / 10.0;  
  return res;
}

void find_max(int input, int *max_val){
 if(input > *max_val){
  *max_val = input;
 }
}

void find_min(int input, int *min_val){
 if(input < *min_val){
  *min_val = input;
 }
}

int readADC_Avg(int ADC_Raw, uint32_t *An_Buffer, int *An_i)
{
  int i = 0;
  int Sum = 0;
  
  *An_i = *An_i +1;

  An_Buffer[*An_i] = ADC_Raw;
  if(*An_i == FILTER_LEN)
  {
    *An_i = 0;
  }
  for(i=0; i<FILTER_LEN; i++)
  {

    Sum += An_Buffer[i];
  }
  return (Sum/FILTER_LEN);
}

void setup() {
  doc["a0"] = 0;
  doc["a1"] = 0;
  A0_Min = analogRead(A0Pin);
  A1_Min = analogRead(A0Pin);
  
  Serial.begin(9600);
  while (!Serial);
  serializeJsonPretty(doc, Serial);
}

void loop() {

  A0_Raw = analogRead(A0Pin);
  A0_Filtered = readADC_Avg(A0_Raw, A0_Buffer, &A0_i);
  
  A1_Raw = analogRead(A1Pin);
  A1_Filtered = readADC_Avg(A1_Raw, A1_Buffer, &A1_i);

  doc["a0"]=rescale(A0_Filtered, 880, 2918, 1000, 10, 5);
  doc["a1"]=rescale(A1_Filtered, 881, 2929, 1000, 10, 5);
  
  /** find min and max 
  find_min(A0_Filtered, &A0_Min);
  find_max(A0_Filtered, &A0_Max);
  doc["a0_min"] = A0_Min;
  doc["a0_max"] = A0_Max;
  
  find_min(A1_Filtered, &A1_Min);
  find_max(A1_Filtered, &A1_Max);
  doc["a1_min"] = A1_Min;
  doc["a1_max"] = A1_Max;
  **/

  char out[128];
  serializeJson(doc, out);
  strcat(out, "\n");
  Serial.write(out);
}
