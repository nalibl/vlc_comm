#include "arduinoFFT.h"
#include "math.h"

arduinoFFT FFT = arduinoFFT(); /* FFT object*/

const sensorPin = A0;
const ledPin = 7;
const sensorValue = 0;
const sample_per = 3; // all times in us. Sampling frequency is set to 333KHz
const low_per = 100;
const high_per = 2 * low_per;
const carrier_per = (low_per + high_per)/2;
const symbol_len = low_per * 4;
const int kernel[symbol_len];

#define BUFFER_SIZE 64

void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  // turn the ledPin on
  digitalWrite(ledPin, HIGH);
  delay(500);
  // read the input on analog pin 0:
  sensorValue = analogRead(sensorPin);
  // print out the value you read:
  Serial.println(sensorValue);
  delay(500);
  digitalWrite(ledPin, LOW);
  delay(500);
  // read the input on analog pin 0:
  sensorValue = analogRead(sensorPin);
  // print out the value you read:
  Serial.println(sensorValue);
  delay(500);
}

int prepare_analog(int samples) {
  int i, sample_deriv[BUFFER_SIZE];
  
  for (i = 0; i < (BUFFER_SIZE - 1); ++i) {
    sample_deriv[i] = samples[i+1] - samples[i];
    sample_deriv[i] *= (sample_deriv[i] > 0);
  }
  return sample_dig;
}

int * convolve(int h[], int x[]){
  int nconv = symbol_len+BUFFER_SIZE-1;
  int i,j,h_start,x_start,x_end,y[nconv];
    
  for (i=0; i<nconv; i++) {
    x_start = MAX(0,i-lenH+1);
    x_end   = MIN(i+1,lenX);
    h_start = MIN(i,lenH-1);
    for(j=x_start; j<x_end; j++) {
      y[i] += h[h_start--]*x[j];
    }
  }
  return y;
}
