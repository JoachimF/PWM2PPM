/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob
*/

#include <Servo.h>

#define PWM_PIN 2
#define ESC_PIN 9
#define PWM_OUt 13
#define DEAD_TIME 50000 //50ms

int potpin = 0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin

unsigned long high_time,low_time,pwm_duty,frequency,period = 0 ;
bool fall,rise,complete_cycle = 0 ;
bool int_mutex = 0 ;
Servo ESC ;

void get_pwm_duty(void)
{
  static unsigned long rising_time,falling_time = 0 ;

  switch(digitalRead(PWM_PIN))
  {
      case 0 : falling_time = micros() ;
               high_time = falling_time - rising_time ;
               fall = 1 ;
              //Serial.print("HIGH time = ");
              //Serial.println(high_time);
              break ;
      case 1 : rising_time =  micros() ;
              low_time = rising_time - falling_time ;
              rise = 1 ;
              //Serial.print("low time = ");
              //Serial.println(low_time);
              break ; 
  }
  if(fall && rise)
  {
    period = high_time + low_time ;
    fall = rise = 0 ;
    complete_cycle = 1 ;
  }
  if(int_mutex == 0 )
  {
    pwm_duty = (low_time * 1000) /(high_time+low_time) ;
  }
}


void set_ppm() 
{
  unsigned long ppm ;
  int_mutex = 1 ; //Protection de la variable en cas d'interruption
  ppm = 1000+pwm_duty ;
  int_mutex = 0 ;
  // On évite de dépasser les limites
  if(ppm > 2000)
    ppm = 2000 ;
  if(ppm < 1000)
    ppm = 1000;
  ESC.writeMicroseconds(ppm);
}
void setup() {
  // put your setup code here, to run once:
  period = 1000000 ; //Demarrage période à 1S
  cli() ;
  pinMode(PWM_PIN, INPUT);
  pinMode(A0, INPUT);
  pinMode(PWM_OUt, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(PWM_PIN), get_pwm_duty, CHANGE);
  ESC.attach(ESC_PIN) ;
  sei() ;
  //Serial.begin(9600);
  ESC.writeMicroseconds(1000);
}

void sim_pwm()
{
    val = analogRead(A0);            // reads the value of the potentiometer (value between 0 and 1023)
    //analogWrite(PWM_OUt,map(analogRead(A0), 0, 1023, 0, 255)) ;
    if(val > 0)
      digitalWrite(PWM_OUt, LOW);
    delay(val/4);
    digitalWrite(PWM_OUt, HIGH);
    delay(255-(val/4));
}

void debug()
{
    Serial.print("Pot = ");
    Serial.print(val/4);
    Serial.print(" - Period = ");
    Serial.print(period);
    Serial.print(" - Duty = ");
    Serial.println(pwm_duty);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long last_time ;

  while(1) //Le signal PWM est inversé, on attend l'état HAUT avant de demarrer le programme, au cas ou l'ECU n'est pas allumé ou la prise branchée
  {
    //Serial.print("Wait high pwm, pot = ");
    //Serial.println(val/4);
    //sim_pwm() ;
    //delay(15);                           // waits for the servo to get there
    if(digitalRead(PWM_PIN)){ //On verifie qu'il est toujours a l'état HAUT, au cas ou ce soit un parasite
      delay(500) ; //On attend 500ms
      if(digitalRead(PWM_PIN)) //On verifie qu'il est toujours a l'état HAUT, au cas ou ce soit un parasite
      break; // On sort de la boucle
    }
  }
  last_time = micros() ;
  //Serial.print("PWM pin OK");
  while(1)  
  {
    //sim_pwm() ;
    //debug() ;
    //delay(15);  

    if(complete_cycle) // Cycle complet PWM, on génère la sortie ESC
    {
      last_time = micros() ;
      complete_cycle = 0 ;
      set_ppm() ;
    }

    if(micros() - last_time >= period + DEAD_TIME) // Le PWM est à 100% ou 0%, plus de pulse, on génère la sortie ESC à 0% ou 100%
    {
      last_time = micros() ;  //Reset du watchdog
      switch(digitalRead(PWM_PIN))
      {
      case 0 : pwm_duty = 1000 ; //plein gaz
              break ;
      case 1 : pwm_duty = 0 ; //gaz coupé
              break ; 
      }
      set_ppm() ; 
    }

  }
}

