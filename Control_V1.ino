/* Tranmsitter code for the Arduino Radio control with PWM output
*/

/**********************Libaries******************/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/**********************Variables******************/
const uint64_t my_radio_pipe = 0xE8E8F0F0E1LL; //Remember that this code should be the same for the receiver
const byte addresses [][6] = {"00001", "00002"};  //Setting the two addresses. One for transmitting and one for receiving

int PIN_SW = 6,
    PIN_SW2 = 5;

// The sizeof this struct should not exceed 32 bytes
struct Data_to_be_sent {
  byte ch1;
  byte ch2;
  byte ch3;
  byte ch4;
  byte ch5;
};

struct Received_data{
  byte boatState;
  byte batteryLevel;
};

Data_to_be_sent sent_data;
Received_data received_data;

RF24 radio(9, 10);

int boat_state,
    battery_level;

LiquidCrystal_I2C lcd(0x27,16,2);  //

byte emptyBattery[8] = {
  B00100,
  B11011,
  B10001,
  B10001,
  B10011,
  B10111,
  B11111,
  B11111,
};
byte fullBattery[8] = {
  B00100,
  B11011,
  B10001,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
};

/************************************************/
void setup()
{
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(addresses[1]);     //Setting the address at which we will send the data
  radio.openReadingPipe(1, addresses[0]);  //Setting the address at which we will receive the data
  
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  //radio.openWritingPipe(my_radio_pipe);  
  sent_data.ch1 = 127;
  sent_data.ch2 = 127;
  sent_data.ch5 = 127;
  
  //We configure the input pins
  pinMode(PIN_SW, INPUT_PULLUP);
  pinMode(PIN_SW2, INPUT_PULLUP);

  // Inicializar el LCD
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, emptyBattery);
  lcd.createChar(1, fullBattery);
  
}

/**********************Code**********************/
void loop()
{
  radio.stopListening();
 
  sent_data.ch1 = map( analogRead(A0), 0, 1024, 0, 255);
  sent_data.ch2 = map( analogRead(A1), 0, 1024, 0, 255);
  sent_data.ch5 = map( analogRead(A2), 0, 1024, 0, 255);
  sent_data.ch3 = digitalRead(PIN_SW);
  sent_data.ch4 = digitalRead(PIN_SW2);
  radio.write(&sent_data, sizeof(Data_to_be_sent));

  // Mostrar resultados
  Serial.print(F("X Value:  "));
  Serial.print(analogRead(A0));
  Serial.print(F("\tY Value: "));
  Serial.print(analogRead(A1));
  Serial.print(F("\tY Value: "));
  Serial.println(received_data.batteryLevel);
  
  radio.startListening();
  while(!radio.available());                         //Looking for incoming data
  radio.read(&received_data, sizeof(Received_data)); //Reading the data

  boat_state = received_data.boatState;
  battery_level = received_data.batteryLevel;
  
  // Ubicamos el cursor en la primera posición(columna:0) de la segunda línea(fila:1)
  lcd.setCursor(0,0);
  lcd.print("Ang X:");
  lcd.print("  ");
  lcd.print("Ang Z:");
  lcd.print("   ");
 
  if(boat_state == 0){
    lcd.setCursor(0, 1);
    lcd.print("Wait ");
  }
  if(boat_state == 10){
    lcd.setCursor(0, 1);
    lcd.print("Fwd  ");
  } 
    if(boat_state == 20){
    lcd.setCursor(0, 1);
    lcd.print("Rev  ");
  } 
    if(boat_state == 30){
    lcd.setCursor(0, 1);
    lcd.print("Right");
  } 
    if(boat_state == 40){
    lcd.setCursor(0, 1);
    lcd.print("Left ");
  }

   if(battery_level < 120){
       lcd.setCursor(15,0);
       lcd.write(byte(0));
   }
   else{
       lcd.setCursor(15,0);
       lcd.write(byte(1));
   }
  delay(10);
}
