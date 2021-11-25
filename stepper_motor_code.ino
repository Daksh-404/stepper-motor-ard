//Motor Control Code for NEMA 23
//Written by Satvik Sharma
//Fixed parameters
//pin mapping
//max speed
//microstepping mode
//JSON IO format
//Example for JSON input
/*
{"fl": "10","fr": "10","bl": "10","br": "10","req": "0","kill": "0"}
*/
//#define debugging
//#define supernarrowpulse
#define invertenable    //Turns out the enable was inverted 
//#define turnon
#define Baudrate 115200
#define timeout 50   //this value of timeout has been optimised to get the best data rate. DO NOT INCREASE THE LENGTH OF JSON STRING SIGNIFICANTLY OR DECREASE THE BAUDRATE FROM 115200 OR YOU MAY NEED TO INCREASE THIS VALUE. 
// The maximum refreshrate frequency is roughly  1000/(timeout+10) Hz 
//On timing analaysis of this code it turns out to be about 16 hertz.
#define MaxSpeed 2
#define no_steps_per_rotation 5*200  //Due to the gear ratio of 5
#define dt  100      // Time step in microseconds
#define negate_direction -1   //Use this to flip the direction of the motors. Set it "1" for a certain direction and "-1" for the opposite direction
// Pin mapping has been fixed according to the PCB layout used
// The value passed to the arduino must be calculated using this formulae
// value passed to arduino = Round[(pi*(Radius of wheels in m))/((no_steps*(required speed in m/s)*(dt in seconds)))]   
#include<ArduinoJson.h>
DynamicJsonDocument doctransmit(256);   //The JSON doc used for tranmsitting data
DynamicJsonDocument docreceive(256);   //The JSON doc used to receiver data
int req,kill; 
int s[4];// In the order of front left , front right , back left and back right
unsigned long p[4]; // Same standard order followed
int last_time[4];
int enpins[4]={6,7,8,13};
bool increment[4];
unsigned long tcnt_val;
void setup(){
  for(int i=2;i<14;i++)
  pinMode(i,OUTPUT);
  Serial.setTimeout(timeout);
  //Setting up the enable pins inactive
  #ifdef invertenable
 digitalWrite(6,HIGH);
  digitalWrite(7,HIGH);
  digitalWrite(8,HIGH);
  digitalWrite(13,HIGH);
  #else
   digitalWrite(6,LOW);
  digitalWrite(7,LOW);
  digitalWrite(8,LOW);
  digitalWrite(13,LOW);
  
  #endif
  //Setting up the direction pins based on the negate_direction
  if(negate_direction==1)
  {
  digitalWrite(2,LOW);
  digitalWrite(3,LOW);
  digitalWrite(4,LOW);
  digitalWrite(5,LOW);
    }
  else
  {
    digitalWrite(2,HIGH);
    digitalWrite(3,HIGH);
    digitalWrite(4,HIGH);
    digitalWrite(5,HIGH);
    }
  req=0;
  for(int i=0;i<4;i++)
  {
    s[i]=0;
    p[i]=0;
    last_time[i]=0;
    increment[i]=false;
  }
  #ifdef debugging
   for(int i=0;i<4;i++)
  {
    s[i]=(i+1);
    p[i]=0;
    last_time[i]=0;
    increment[i]=false;
  }
  pinMode(A0,OUTPUT);
  pinMode(A1,OUTPUT);
   pinMode(A2,OUTPUT);
  pinMode(A3,OUTPUT);
   pinMode(A4,OUTPUT);
  pinMode(A5,OUTPUT);
  req=1;
  #endif
  Serial.begin(Baudrate);
  Serial.println("REBOOT");
  //Setting up timer 1 at a time period of dt for the frequency modulation ISR
  TCCR1A=0;
  TCCR1B=0;
  tcnt_val=65536-floor((16*dt)/1);// Because the prescalar is 1   // 60736 for 300 microsecond time period of ISR at prescalar 1
  TCNT1=tcnt_val;
  TCCR1B |= (1<<CS10);
  TIMSK1 |= (1<<TOIE1);
  #ifdef turnon
  s[0]=10;
  s[1]=10;
  s[2]=10;
  s[3]=10;
  #endif
}
// The frequency modulator ISR function
//Port bit manipulation used cause Need For Speed^TM
ISR(TIMER1_OVF_vect)
{
  #ifdef debugging
  PORTC= PORTC ^ B00000001;
  #endif
  //Updating the last_time variable to keep track of time
  last_time[0]++;
  last_time[1]++;
  last_time[2]++;
  last_time[3]++;
  //Checking if we need to toggle any pin or not
  if(last_time[0]>=s[0])
  {
    //toggle the pin
    PORTB = PORTB ^ B00000010;
    #ifdef supernarrowpulse
    PORTB = PORTB ^ B00000010;
    #endif
    //update the pos var
    if(increment[0])
    p[0]++;
    //toggle the boolean for pos variable increment
    increment[0]=!increment[0];
    //update last_time
    last_time[0]=0;
    }
    if(last_time[1]>=s[1])
  {
    //toggle the pin
    PORTB = PORTB ^ B00000100;
    #ifdef supernarrowpulse
    PORTB = PORTB ^ B00000100;
    #endif
    //update the pos var
    if(increment[1])
    p[1]++;
    //toggle the boolean for pos variable increment
    increment[1]=!increment[1];
    //update last_time
    last_time[1]=0;
    }
    if(last_time[2]>=s[2])
  {
    //toggle the pin
    PORTB = PORTB ^ B00001000;
    #ifdef supernarrowpulse
    PORTB = PORTB ^ B00001000;
    #endif
    //update the pos var
    if(increment[2])
    p[2]++;
    //toggle the boolean for pos variable increment
    increment[2]=!increment[2];
    //update last_time
    last_time[2]=0;
    }
    if(last_time[3]>=s[3])
  {
    //toggle the pin
    PORTB = PORTB ^ B00010000;
    #ifdef supernarrowpulse
    PORTB = PORTB ^ B00010000;
    #endif
    //update the pos var
    if(increment[3])
    p[3]++;
    //toggle the boolean for pos variable increment
    increment[3]=!increment[3];
    //update last_time
    last_time[3]=0;
    }
    TCNT1=tcnt_val; //Resetting TCNT to get the same interrupt frequency in the next loop too
  }
void loop() {
  
  #ifdef debugging
  PORTC= PORTC ^ B00000010;
  #endif
 if(Serial.available()>0)
  {
    #ifdef debugging
  PORTC= PORTC ^ B00000100;
  #endif
    String json = Serial.readStringUntil('\n');
    #ifdef debugging
  PORTC= PORTC ^ B00000100;
  PORTC= PORTC ^ B00001000;
  #endif
    DeserializationError err=deserializeJson(docreceive, json);
    #ifdef debugging
  PORTC= PORTC ^ B00001000;
  PORTC= PORTC ^ B00010000;
  Serial.println(json);
  #endif
    if(err)
    {
      Serial.println("Error");
      Serial.println(err.c_str());
      }
      else
      {
    s[0]=docreceive["fl"];
    s[1]=docreceive["fr"];
    s[2]=docreceive["bl"];
    s[3]=docreceive["br"];
    req=docreceive["req"];
    kill=docreceive["kill"];
    #ifdef debugging
  PORTC= PORTC ^ B00010000;
  #endif
    if(kill==1)
    {
      #ifdef invertenable
      digitalWrite(6,HIGH);
      digitalWrite(7,HIGH);
      digitalWrite(8,HIGH);
      digitalWrite(13,HIGH);
      #else
      digitalWrite(6,LOW);
      digitalWrite(7,LOW);
      digitalWrite(8,LOW);
      digitalWrite(13,LOW);
      #endif
      }
      else
      {
        #ifdef invertenable
        digitalWrite(6,LOW);
      digitalWrite(7,LOW);
      digitalWrite(8,LOW);
      digitalWrite(13,LOW);
      
      #else
      digitalWrite(6,HIGH);
      digitalWrite(7,HIGH);
      digitalWrite(8,HIGH);
      digitalWrite(13,HIGH);
      #endif
        }
    for(int i=0;i<4;i++)
    if(s[i]==0)
    { 
      #ifdef invertenable
      digitalWrite(enpins[i],HIGH);
      #else 
      digitalWrite(enpins[i],LOW);
      #endif
      }
    else 
    {
      #ifdef invertenable
      digitalWrite(enpins[i],LOW);
      #else
      digitalWrite(enpins[i],HIGH);
      #endif
      if(s[i]<0 && negate_direction==1)
    {digitalWrite(i+2,HIGH);
    s[i]=-s[i];
    }
    else if(s[i]<0)
    {digitalWrite(i+2,LOW);
    s[i]=-s[i];
    }
    else
    {
      if( negate_direction==1)
    {digitalWrite(i+2,LOW);
    
    }
    else 
    {digitalWrite(i+2,HIGH);
    }
      }
    }

    
    if(req==1)
    {
      doctransmit["pfl"]=p[0];
      doctransmit["pfr"]=p[1];
      doctransmit["pbl"]=p[2];
      doctransmit["pbr"]=p[3];
      Serial.println("");
      serializeJson(doctransmit, Serial);
      } 
      }
  }

}
