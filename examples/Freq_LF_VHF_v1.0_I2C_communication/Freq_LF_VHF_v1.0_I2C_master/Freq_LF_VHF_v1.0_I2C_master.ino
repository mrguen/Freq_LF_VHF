#include <Wire.h>
#include <Pandauino_Freq_LF_VHF.h>

float frequency = 0.0;

// This template written by Nick Gammon - May 2012
template <typename T> unsigned int I2C_writeAnything (const T& value)
{
  Wire.write((byte *) &value, sizeof (value));
  return sizeof (value);
}  // end of I2C_writeAnything

void setup() {
  frequencyCounter.freqSetup(board_version_vhf); // use "board_version_hf" for the 5 Hz - 5 MHz board and "board_version_vhf" for the 5 Hz - 210 MHz board
  Wire.begin();    // join the I2C bus as master
}

void loop() {
  frequencyCounter.freqCount();
  frequency = frequencyCounter.readFrequency();

  if (frequency > 0.0) {
    Wire.beginTransmission(8);               // begins a transmission to slave device 8;
    I2C_writeAnything(frequency);            // send float value
    Wire.endTransmission();                  // ends transmission
  }
}
