//This is the C code we used for counting the changes in the encoder pin’s states.
//Include libraries
#include <stdio.h>
#include <wiringPi.h>

//Constants
//Left Wheel
const int Left_Enc_A = 20;
const int Left_Enc_B = 21;

//Right Wheel
const int Right_Enc_A = 4;
const int Right_Enc_B = 5;

volatile int gLCount = 0;
volatile int gISR = 0;
volatile int gMSB = 0;
volatile int gLSB = 0;
int lastEncoded = 0;

//Interrupt functions

void updateEncoder(){
  gISR++;
  
  gMSB = digitalRead(Left_Enc_A);
  gLSB = digitalRead(Left_Enc_B);
  
  int encoded = (gMSB << 1) | gLSB;
  int sum = (lastEncoded << 2) | encoded;
  
  if(sum == 0b0010 || sum == 0b0001 || sum == 0b1000) GLCount++;
  lastEncoded = encoded;
}

//Main()
int main(){
  wiringPiSetupGpio();
  pinMode(Left_Enc_A, INPUT);
  pinMode(Left_Enc_B, INPUT);
  pinMode(Right_Enc_A, INPUT);
  pinMode(Right_Enc_B, INPUT);
  
  if(wiringPiISR(Left_Enc_A, INT_EDGE_BOTH, updateEncoder) < 0){
    printf("Unable to setup ISR\n");
  }
  
  if(wiringPiISR(Left_Enc_B, INT_EDGE_BOTH, updateEncoder) < 0){
    printf("Unable to setup ISR\n");
  }
  
  while(1){
    if(gISR>13500){
      //print something
    }
  }
  
  return 0;
}
