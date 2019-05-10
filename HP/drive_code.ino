#include <PID_v1.h>

const int Vout=A5;    //VOLTAGE SENSE PIN

const int PWM=5;    //FLYBACK CONVERTOR PWM

double measured_voltage;
int pwm=0;
int i=0;
int duty;

//double real_voltage;
double DUTY;
double PID_OUT;
double Error;
double ref_voltage=544;

 double Kp=1,Ki=10,Kd=0;
 PID pid(&Error,&PID_OUT,&ref_voltage,Kp,Ki,Kd,DIRECT);


void setup() {
  // put your setup code here, to run once:
  pinMode(Vout,INPUT);
  pinMode(PWM,OUTPUT);
  // 32 kHz PWM
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00); 
  TCCR0B = _BV(CS00); 
  pid.SetMode(AUTOMATIC);
  pid.SetTunings(Kp,Ki,Kd);

  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  measured_voltage=analogRead(Vout);
  Error=measured_voltage;
  pid.Compute();

  if(PID_OUT<0)
  {
    PID_OUT=0;
    Serial.print("PID is under Limit");
  }
  else if(PID_OUT>255)
  {
    PID_OUT=255;
    Serial.print("PID is over Limit");
  }


  DUTY=255-PID_OUT;
  duty=int(DUTY);


  //analogWrite(PWM,duty);
  analogWrite(PWM,100);

 if (i>5000)
 {
    Serial.print("VOLT:");
  Serial.println(measured_voltage);
  Serial.print("PID_OUT");
  Serial.println(PID_OUT);
  Serial.print("PWM:");
  Serial.println(duty);
  Serial.print("error");
  Serial.println(Error);
  i=0;
 }
i++;
}



