#include <Wire.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(2, 3, 4, 5, 6, 7);

float frequency = 0.0;

// Ce code de Template a été écrit par Nick Gammon - Mai 2012
template <typename T> unsigned int I2C_readAnything(T& value)
{
  byte * p = (byte*) &value;
  unsigned int i;
  for (i = 0; i < sizeof value; i++)
    *p++ = Wire.read();
  return i;
}  // fin de I2C_readAnything

void setup() {
  Serial.begin(115000);
  lcd.begin(16, 1);
  Wire.begin(8);                                    // se joint au bus I2C avec l'adresse 8
  Wire.onReceive(receiveEvent);                     // définit le gestionnaire de réception
}

void loop() {
  delay(100);
}

void receiveEvent(int howMany) {

  if (howMany >= (sizeof frequency))
  {
    I2C_readAnything (frequency);
    lcd.clear();
    lcd.print(frequency);
    Serial.println(frequency);    
  } 
}
