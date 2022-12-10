/* Name: Boat_V2
 * Autor: Jesus Rojas
*/

/* Libraries*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

/**********************Variables******************/
const uint64_t pipeIn = 0xE8E8F0F0E1LL;     //Remember that this code is the same as in the transmitter
RF24 radio(9, 10);  //CSN and CE pins

const int mpuAddress = 0x68;
MPU6050 mpu(mpuAddress);

// The sizeof this struct should not exceed 32 bytes
struct Received_data {
  byte ch1;
  byte ch2;
  byte ch3;
  byte ch4;
  byte ch5;
};

struct Data_to_be_sent{
  byte boatState;
  byte batteryLevel;
};

const byte addresses [][6] = {"00001", "00002"};

Received_data received_data;
Data_to_be_sent sent_data;

Servo ESC,
      ESC2,
      RUD;

bool  SW1,
      SW2;

int x_value = 0,
    y_value = 0,
    rud_value = 0,
    ax, ay, az,
    gx, gy, gz;

int boat_state,
    oldBoat_state, 
    turnAngle,
    rudAngle,
    linearVel,
    val,
    val2,
    fr,
    fl;

long tiempo_prev;
float dt;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;

double  e,
        angle,
        batteryLevel;
        
/**********************Constant**********************/
int     TM = 10,
        JUP_LIMIT = 1956,
        JLOW_LIMIT = 1050;

/**********************Outputs**********************/
int Motor1_PWM = 6,
    Motor2_PWM = 5,
    Rudder_PWM = 3;
    
/**********************Inputs**********************/

/**************************************************/

void setup()
{
  Serial.begin(9600);
  //We reset the received values
  received_data.ch1 = 127;
  received_data.ch2 = 127;
  received_data.ch5 = 127;
  received_data.ch3 = false;
  received_data.ch4 = false;
  
  //Once again, begin and radio configuration
  radio.begin();
  radio.openWritingPipe(addresses[0]);      //Setting the address at which we will send the data
  radio.openReadingPipe(1, addresses[1]);   //Setting the address at which we will receive the data
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);  
 
  //We initialize MPU communication
  Wire.begin();
  mpu.initialize();
  Serial.println(mpu.testConnection() ? F("IMU iniciado correctamente") : F("Error al iniciar IMU"));

  //We initialize boat state in wait
  boat_state = 0;
  rudAngle = 90;

  //Attach the ESC on pin 6
  //ESC.attach(Motor2_PWM); // (pin, min pulse width, max pulse width in microseconds)
  ESC.attach(Motor1_PWM,1000,2000);
  ESC2.attach(Motor2_PWM,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
  RUD.attach(Rudder_PWM);
}

/**************************************************/
unsigned long last_Time = 0;
//We create the function that will read the data each certain time
void receive_the_data()
{
  while ( radio.available() ) {
  radio.read(&received_data, sizeof(Received_data));
  last_Time = millis(); //Here we receive the data
  }
}

/**************************************************/
void complementaryFilter(){
   dt = (millis() - tiempo_prev) / 1000.0;
   tiempo_prev = millis();
   //Calcular los ángulos con acelerometro
   float accel_ang_x = atan(ay / sqrt(pow(ax, 2) + pow(az, 2)))*(180.0 / 3.14);
   float accel_ang_y = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2)))*(180.0 / 3.14);
   //Calcular angulo de rotación con giroscopio y filtro complementario
   ang_x = 0.98*(ang_x_prev + (gx / 131)*dt) + 0.02*accel_ang_x;
   ang_y = 0.98*(ang_y_prev + (gy / 131)*dt) + 0.02*accel_ang_y;
   ang_x_prev = ang_x;
   ang_y_prev = ang_y;
}

/**************************************************/
void loop()
{   
   //We start the radio comunication
  radio.startListening();

  //Receive the radio data
  if(radio.available()){
    receive_the_data();
    x_value = map(received_data.ch1,0,255,1000,2000);
    y_value = map(received_data.ch2,0,255,1000,2000);
    rud_value = map(received_data.ch5, 0,255,1000,2000);
    SW1 = received_data.ch3;
    SW2 = received_data.ch4;
  }
  
  mpu.getRotation(&gx, &gy, &gz);
  mpu.getAcceleration(&ax, &ay, &az);
  complementaryFilter();
  
  // Mostrar resultados
  Serial.print(F("X Value:  "));
  Serial.print(x_value);
  Serial.print(F("\tY Value: "));
  Serial.print(y_value);
  Serial.print(F("\tBaterry Level: "));
  Serial.print(batteryLevel);
  Serial.print(F("\tBoat state:  "));
  Serial.print(boat_state);
  Serial.print(F("\tRudder Angle:  "));
  Serial.print(rudAngle);
  Serial.print(F("\tPWM on Motor 1: "));
  Serial.print(fr);
  Serial.print(F("\tPWM on Motor 2: "));
  Serial.println(fl);

  if (rud_value <= JLOW_LIMIT){
    rudAngle--;
  }
  if (rud_value >= JUP_LIMIT){
    rudAngle++;
  }

  if(not SW1){
    rudAngle = 90;
  }
  
  //We limit the output signal
  if (fl > 179){
    fl = 179;
  }
  if (fl < 0){
    fl = 0;
  }
  if (fr > 179){
    fr = 179;
  }
  if (fr < 0){
    fr = 0;
  }
  
  if (rudAngle > 179){
    rudAngle = 179;
  }
  if (rudAngle < 0){
    rudAngle = 0;
  }
  
  //We write on PWM pins for Motors
  val = map(fl, 0, 255, 0, 179);
  ESC.write(val);
  delay(15);
  
  val2 = map(fr, 0, 255, 0, 179);
  ESC2.write(val2);
  delay(15);

  //We delimited the rotation angle
  if(rudAngle <= 75){
    rudAngle = 75;
  }
  if(rudAngle >= 105){
    rudAngle = 105;
  }

  RUD.write(rudAngle);
    
   switch (boat_state){
    //Wait
    case 0:
      fl            = 0;
      fr            = 0;       
      turnAngle     = 0;
      oldBoat_state = 0;
      
      if (x_value >= JUP_LIMIT){
        boat_state  = 10;
      }
      if (x_value <= JLOW_LIMIT){
        boat_state  = 20;
      }

      if (y_value >= JUP_LIMIT){
        boat_state  = 30;
      }
      
      if (y_value <= JLOW_LIMIT){
        boat_state  = 40;
      }
    break;

    //Move FWD
    case 10:
    
      oldBoat_state = 10;
      
      if (x_value >= JUP_LIMIT){
        linearVel++;
      }

      fl = linearVel;
      fr = fl;
        
      if(not SW1){
      boat_state = 0;
      }

      if (y_value >= JUP_LIMIT){
        boat_state = 30;
      }
      
      if (y_value <= JLOW_LIMIT){
        boat_state = 40;
      }
    break;

    //Move REV
    case 20:
      oldBoat_state = 20;
      if(not SW1){
        boat_state = 0;
      }

      if (y_value >= JUP_LIMIT){
        boat_state = 30;
      }
      
      if (y_value <= JLOW_LIMIT){
        boat_state = 40;
      }
    break;

        //Turn Right 
    case 30:
      if (y_value >= JUP_LIMIT){
        turnAngle++;
      }

      //controllerPID();
      fr = turnAngle * 2.0 ;
      
      if(not SW2){
      turnAngle = 0;
      boat_state = oldBoat_state;
      }
    break;

    //Turn Left
    case 40:
      if (y_value <= JLOW_LIMIT){
        turnAngle--;
      }

      //controllerPID();
      fl = turnAngle * -2;

      if(not SW2){
      turnAngle = 0;
      e         = 0;
      boat_state = oldBoat_state;
      }
    break;
    
   }
   radio.stopListening();
   
   sent_data.batteryLevel = map( analogRead(A0), 0, 1024, 0, 255);
   sent_data.boatState = boat_state;
   
   radio.write(&sent_data, sizeof(sent_data));
   
}//Loop end
