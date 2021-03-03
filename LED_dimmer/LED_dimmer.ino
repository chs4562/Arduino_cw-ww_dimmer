

#include <EEPROM.h>
//#include <Encoder.h>

#define MAX_BRIGHTNESS 255
#define MAX_TEMPERATURE 100.0
#define IIR 0.4

#define BUTTON_RIGHT_INT 2
#define BUTTON_RIGHT_GPIO 7

#define BUTTON_LEFT_INT 3
#define BUTTON_LEFT_GPIO 12

 static float brightness_f = 0;
 static float temperature_f = 50;
 static float brightness_f2 = 0;
 static float temperature_f2 = 50;
 static float val_left_f = 0;
 static float val_right_f= 0;
 static uint8_t val_left;
 static uint8_t val_right;
 static uint8_t mem = LOW;
 static float factor;
 static uint32_t t = 0;

//Function Prototypes
  void calculate_values(void);  //calcualte the ouput values from the temp and brightness
  void BL_ISR(void);  //left buttin Interrupt service routine
  void BR_ISR(void);  //right button Interrupt service routine

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //pinMode(LED_BUILTIN, OUTPUT);
  //digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  pinMode(BUTTON_LEFT_INT, INPUT);
  pinMode(BUTTON_LEFT_GPIO, INPUT);
  pinMode(BUTTON_RIGHT_INT, INPUT);
  pinMode(BUTTON_RIGHT_GPIO, INPUT);

  attachInterrupt(digitalPinToInterrupt(BUTTON_LEFT_INT), BL_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_RIGHT_INT), BR_ISR, FALLING);

  //EEPROM.get(0, temperature_f);
  //EEPROM.get(4, brightness_f);
  
}

void loop() {

  //IIR LOW Pass
  temperature_f2 = temperature_f2*(1.0 - IIR) + temperature_f*IIR;
  brightness_f2 = brightness_f2*(1.0 - IIR) + brightness_f*IIR;

  //calculate output values from brightness and temperature
  calculate_values();
  
  if(digitalRead(4) == LOW && mem == HIGH){
    //EEPROM.put(0, temperature_f);
    //EEPROM.put(4, brightness_f);
    t = millis();
  }
  
  if(digitalRead(4) == HIGH && mem == LOW){
    if(millis() - t > 1000 && t != 0){
      EEPROM.put(0, temperature_f);
      EEPROM.put(4, brightness_f);      
    }
    else{
      EEPROM.get(0, temperature_f);
      EEPROM.get(4, brightness_f);
    }
    t = 0;
  }

  if(millis() - t < 1000 && t != 0){
    val_left = 10;
    val_right = 10;
  }

  mem = digitalRead(4);

  analogWrite(6, val_left);
  analogWrite(5, val_right);
  
  //Serial.print("Value_left: ");
  //Serial.println(val_left);
  //Serial.print("Value_right: ");
  //Serial.println(val_right);
}


void BL_ISR(void){
  if(digitalRead(BUTTON_LEFT_GPIO) == LOW){
    brightness_f = brightness_f - 10;
    if(brightness_f < 0){
      brightness_f = 0;
    }
  }
  else{
    brightness_f = brightness_f + 10;  
    if(brightness_f > 255){
      brightness_f = 255;  
    }
  }
}

void BR_ISR(void){
  if(digitalRead(BUTTON_RIGHT_GPIO) == LOW){
    temperature_f = temperature_f - 5;
    if(temperature_f < 0){
      temperature_f = 0;
    }
  }
  else{
    temperature_f = temperature_f + 5;  
    if(temperature_f > 100){
      temperature_f = 100;  
    }
  }
}

void calculate_values(void){
  factor = temperature_f2 / 100.0;
  val_left_f = brightness_f2 * factor * 2;
  if(val_left_f > 255){
    val_left_f = 255;
  }
  val_left = (uint8_t)val_left_f;

  val_right_f = brightness_f2 * (1.0 - factor) * 2;
  if(val_right_f > 255){
    val_right_f = 255;
  }
  if(val_right_f < 0){
    val_right_f = 0;
  }
  val_right = (uint8_t)val_right_f;
}





