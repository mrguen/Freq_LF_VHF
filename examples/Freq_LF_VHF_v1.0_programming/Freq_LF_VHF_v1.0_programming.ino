#include <Pandauino_Freq_LF_VHF.h>

void setup() {
  frequencyCounter.freqSetup(board_version_vhf, true, 115000); // use "board_version_hf" for the 5 Hz - 5 MHz board and "board_version_vhf" for the 5 Hz - 210 MHz board
}

void loop() {

  frequencyCounter.freqCount();

}
