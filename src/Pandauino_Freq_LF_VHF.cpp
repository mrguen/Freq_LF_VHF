/*  Pandauino_Freq_LF_VHF frequency counter library
 *  Version: 1.0
 *  By Pandauino.com / Thierry GUENNOU
 *  Frebruary 2021
 *
 *  This library is used on the board "Freq_LF_VHF v1.0"
 *
 * 	DEPENDENCIES:
 *
 *  It needs
 *
 *  ** EEPROM library included in the Arduino core
 *
 *  ** Paul Stoffregen FreqCount library see https://github.com/PaulStoffregen/FreqCount
 *
 *	** Mrguen (Pandauino) fork of Paul Stoffregen FreqMeasure library see https://github.com/mrguen/FreqMeasure
 *
 *  ** LiquidCrystal library see https://www.arduino.cc/en/Reference/LiquidCrystal
 *
 *  ** Matthias Hertel One Button library http://www.mathertel.de/Arduino/OneButtonLibrary.aspx
 *
 *  ** power & sleep libraries included in Arduino/avr core
 *
 *  This code is based on Arduino code and is provided with the same warning that:
 *  THIS SOFTWARE IS PROVIDED TO YOU "AS IS" AND WE MAKE NO EXPRESS OR IMPLIED WARRANTIES WHATSOEVER WITH RESPECT TO ITS FUNCTIONALITY, OPERABILITY, OR USE,
 *  INCLUDING, WITHOUT LIMITATION, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR INFRINGEMENT.
 *  WE EXPRESSLY DISCLAIM ANY LIABILITY WHATSOEVER FOR ANY DIRECT, INDIRECT, CONSEQUENTIAL, INCIDENTAL OR SPECIAL DAMAGES, INCLUDING, WITHOUT LIMITATION, LOST REVENUES,
 *  LOST PROFITS, LOSSES RESULTING FROM BUSINESS INTERRUPTION OR LOSS OF DATA, REGARDLESS OF THE FORM OF ACTION OR LEGAL THEORY UNDER WHICH THE LIABILITY MAY BE ASSERTED,
 *  EVEN IF ADVISED OF THE POSSIBILITY OR LIKELIHOOD OF SUCH DAMAGES.
 *
 *  Licence CC BY-NC-SA 4.0 see https://creativecommons.org/licenses/by-nc-sa/4.0/
 *  You are free to use, modifiy and distribute this software for non commercial use while keeping the original attribution to Pandauino.com
 */


/*
  This is the structure of the menu

  Frequency band
    AUTO
    5 Hz - 5 KHz
    5 KHz - 5 MHz
    5 - 20 MHz
    20 - 210 MHz

  Resolution
			ULTRA LOW
  		LOW
  		NORMAL
  		HIGH

  Calibration
  	Cal 4 MHz (press)
  	Cal 10 MHz (press)
  	Cal manual

  Display
  	Frequency
  	Period

  Store

  Retrieve

  Operation
  	Annul op.
    VFO+IF
    VFO-IF
    IF-VFO

  Sleep
  	Sleep_30s
  	Sleep_5m
  	Sleep_disabled

*/

#include <Pandauino_Freq_LF_VHF.h>

/*************************************************************************************************************************************
  INSTANCES
**************************************************************************************************************************************/

LiquidCrystal Pandauino_Freq_LF_VHF::lcd(LCDRS, LCDENABLE, LCDD4, LCDD5, LCDD6, LCDD7);
OneButton  Pandauino_Freq_LF_VHF::button(PUSHBUTTON, true);


/*************************************************************************************************************************************
  CONST
**************************************************************************************************************************************/

//*********** Hardware related CONST

// LCD control pins
const int Pandauino_Freq_LF_VHF::LCDD7 = 17;         // PC3
const int Pandauino_Freq_LF_VHF::LCDD6 = 3;          // PD3
const int Pandauino_Freq_LF_VHF::LCDD5 = 4;          // PD4
const int Pandauino_Freq_LF_VHF::LCDD4 = 6;          // PD6
const int Pandauino_Freq_LF_VHF::LCDENABLE = 7;      // PD7
const int Pandauino_Freq_LF_VHF::LCDRS = 9;          // PB1

// select1 and select2 settings determine the HF / VHF prescaling
// select1 select 2 effect
// LOW	LOW		PSC = 32
// LOW	HIGH	PSC = 4
// HIGH	x			PSC = 1
// x		x			PSC unimportant = in LF or HF mode
const byte Pandauino_Freq_LF_VHF::select1 = 15; 							// PC1 pin connected to multiplexer U4 select pin
const byte Pandauino_Freq_LF_VHF::select2 = 14; 							// PC0 pin connected to multiplexer U3 select pin

const byte Pandauino_Freq_LF_VHF::PUSHBUTTON = 2;
const byte Pandauino_Freq_LF_VHF::VccReg65EnablePin = 10;     // PB2 port to enable 6.5 Volt regulator
const byte Pandauino_Freq_LF_VHF::lcdLedPowerPin = 16;				// PC2 used to power the LCD led

const byte Pandauino_Freq_LF_VHF::coefVHF1 = 4;               // External prescaler VHF1 coef
const byte Pandauino_Freq_LF_VHF::coefVHF2 = 32;              // External prescaler VHF2 coef


//*********** Software related CONST

// All the band limits with some margin/overlap
const double  Pandauino_Freq_LF_VHF::freqLFmin 							= 1.0;
const double  Pandauino_Freq_LF_VHF::freqLFmax 							= 5100.0;
const double  Pandauino_Freq_LF_VHF::freqHFmin 							= 5000.0;
const double  Pandauino_Freq_LF_VHF::freqHFmax_hf_board 		= 5200000.0;
const double  Pandauino_Freq_LF_VHF::freqHFmax_vhf_board 		= 4200000.0;
const double  Pandauino_Freq_LF_VHF::freqVHF1min 						= 4000000.0;
const double  Pandauino_Freq_LF_VHF::freqVHF1max 						= 22500000.0;
const double  Pandauino_Freq_LF_VHF::freqVHF2min 						= 22000000.0;

// All hard messages
const char Pandauino_Freq_LF_VHF::initMessage[17] = "Initializing    ";							// Displayed at startup
const char Pandauino_Freq_LF_VHF::msgVoltageTooLow[17] = "POWER TOO LOW!  ";  			// Error message if Vcc is too low to allow normal functioning of the board
const char Pandauino_Freq_LF_VHF::msgVoltageTooHigh[17] = "POWER TOO HIGH! ";				// Error message if Vcc is too high to allow normal functioning of the board
const char Pandauino_Freq_LF_VHF::calNotPrecise[17] = "CAL NOT PRECISE ";			  		// The result of the calibration is not precise enough
const char Pandauino_Freq_LF_VHF::calibrating[17] = "Calibrating...  ";			  			// While calibrating
const char Pandauino_Freq_LF_VHF::noMeasureAvailable[17] = "No measure avail"; 			// No measure available within timeout
const char Pandauino_Freq_LF_VHF::goingStandby[17] = "Energy economy  ";						// Displayed before entering sleep mode
const char Pandauino_Freq_LF_VHF::frequencySaved[17] = "Last fr. stored!  ";				// Displayed when storing the last frequency for reference
const char Pandauino_Freq_LF_VHF::frequencyOutOfRange[17] = "F. out of range!";				// Displayed when storing the last frequency for reference

// These menu entries are indexed as the runMode enum values
const char Pandauino_Freq_LF_VHF::menuEntries[34][17] = {
"                ",
"Frequency band >",
"AUTO            ",
"LF 5 Hz - 5 KHz ",
"HF 5 KHz - 5 MHz",
"HF 5 KHz - 4 MHz",
"VH1 4 - 22 MHz  ",
"VH2 22 - 210 MHz",
"Resolution     >",
"LOW             ",
"NORMAL          ",
"HIGH            ",
"ULTRA HIGH      ",
"Calibration    >",
"Cal 4MHz (press)",
"Cal 10MHz (pres)",
"Cal manual     >",
"               >",
"Display f/p    >",
"Disp. frequency ",
"Disp. period    ",
"Store (press)   ",
"Retrieve (press)",
"Operation      >",
"Annul Op.       ",
"VFO+IF          ",
"VFO-IF          ",
"IF-VFO          ",
"Sleep          >",
"Sleep 30 s.     ",
"Sleep 5 m.      ",
"Sleep disabled  ",
"F. reset (press)",
"< Exit menu     "
};

const byte Pandauino_Freq_LF_VHF::EEPROMbaseAddress = 0;    										// Base addres where to store data in EEPROM
const byte Pandauino_Freq_LF_VHF::eepromInit = 5;          											// A number that should be present at eeAddress if the EEPROM is already programmed and not corrupted

const float Pandauino_Freq_LF_VHF::HFMeasurePeriodNormalRes = 1000.0;           // Time period for counting HF edges in milliseconds in normal resolution. USE POWER OF 10 VALUES
const unsigned int Pandauino_Freq_LF_VHF::LFTimeoutNormalRes = 3000;            // Timeout of LF measurement in milliseconds. When reached, frequency is supposed to be impossible to measure.
const unsigned int Pandauino_Freq_LF_VHF::displayTimeLap = 800;                 // Minimum period between too printings of values to the LCD screen (ms)

const float Pandauino_Freq_LF_VHF::VccDivider = 10.0 / 30.0;                   	// External resistor network divider of board VCC to ADC
const float Pandauino_Freq_LF_VHF::vref = 5.0;                                 	// ADC Reference voltage
const float Pandauino_Freq_LF_VHF::coefVcc = vref / (1024 * VccDivider);       	// Computes the multiplier to get VCC results in mV from ADC value

const unsigned int Pandauino_Freq_LF_VHF::VccTestPeriod = 60;                   // VCC is measured every VccTestPeriod in seconds

const unsigned int Pandauino_Freq_LF_VHF::vccDelayPeriod = 100;                 // Delay between vccTest when in error

const float Pandauino_Freq_LF_VHF::underVoltage = 7.5;                         	// Power under voltage threshold value in volts
const float Pandauino_Freq_LF_VHF::overVoltage = 13.0;                         	// Power over voltage ceiling value in volts
const float Pandauino_Freq_LF_VHF::hysteresisVccPerCent = 3;                   	// Power voltage hysteresis % // threshold and ceiling

const float Pandauino_Freq_LF_VHF::minCalibManValue = -10.0;										// Minimum manual calibration value in ppm
const float Pandauino_Freq_LF_VHF::maxCalibManValue = 10.0;											// Maximum manual calibration value in ppm

const unsigned int Pandauino_Freq_LF_VHF::clickticks = 400;                     // Delay in ms to detect a tick (change of button state)


/* ************************************************************************************************************************************
  VAR
**************************************************************************************************************************************/

boardType Pandauino_Freq_LF_VHF::boardVersion = board_version_vhf;							// Defines the board version, HF or VHF

runMode Pandauino_Freq_LF_VHF::editMode = display_main;   											// Defines the current state of the interface

measurementMode Pandauino_Freq_LF_VHF::mode = mode_auto;												// The mode of functionning of the frequency counter: either in auto scale or on a fixed band
measurementBand Pandauino_Freq_LF_VHF::band = band_HF; 													// The precise band used for exact computation of the frequency
measurementResolution Pandauino_Freq_LF_VHF::resolution = resolution_normal;				// Resolution level
measurementDisplayType Pandauino_Freq_LF_VHF::measurementType = measure_frequency; // Measurement type: frequency or period
byte Pandauino_Freq_LF_VHF::displayPrecision = 6;                              	// The display precision. Will be set depending on resolution parameter
float Pandauino_Freq_LF_VHF::measurementTimeCoefficient = 1.0;        					// The measure period is mutliplied by measurementTimeCoefficient to set the effective measurement time. i.e in High res = 10.0 in low res = 0.1
algorithmType Pandauino_Freq_LF_VHF::algorithm = algorithm_freqCount;						// The algorithm used to compute the frequency, depending on the current band
float Pandauino_Freq_LF_VHF::prescalerCoef = 1.0;																// The prescaler coef, depending on the configuration of the current band
float Pandauino_Freq_LF_VHF::effectiveHFMeasurePeriod;									// The effective perdiod used to compute HF/VHF values, depending on resolution
int Pandauino_Freq_LF_VHF::LFTimeout;																						// Expected maximum time to measure an LF value in normal resolution

// buffers
measurementMode Pandauino_Freq_LF_VHF::previousMode;
measurementBand Pandauino_Freq_LF_VHF::previousBand;
measurementResolution Pandauino_Freq_LF_VHF::previousResolution;
double Pandauino_Freq_LF_VHF::previousCalibration;
measurementDisplayType Pandauino_Freq_LF_VHF::previousMeasurementType;
operationType Pandauino_Freq_LF_VHF::previousOperation;
sleepMode Pandauino_Freq_LF_VHF::previousSleepSetting;

unsigned long Pandauino_Freq_LF_VHF::countLF = 0;                              	// Low frequency clock counts
unsigned long Pandauino_Freq_LF_VHF::countHF = 0;                              	// High frequency clock counts
unsigned long Pandauino_Freq_LF_VHF::measureStamp = 0;                         	// Time stamp of the last measure.
unsigned long Pandauino_Freq_LF_VHF::displayStamp = 0;                         	// Time stamp of the last displayed measurement.

double Pandauino_Freq_LF_VHF::sumDisplayFreq = 0.0;                            	// Stores the sum of frequency measurements accumulated during a time lap = displayTimeLap
long Pandauino_Freq_LF_VHF::nbAvgDisplayFreq = 0;                      	// Stores the number of frequency measurements to average during a time lap =  displayTimeLap

double Pandauino_Freq_LF_VHF::frequency = 0.0;                                 	// Computed frequency
double Pandauino_Freq_LF_VHF::frequencyTestVHF2 = 0.0;                         	// Computed frequency when testing in the VHF2 band
double Pandauino_Freq_LF_VHF::frequencyTestVHF1 = 0.0;                         	// Computed frequency when testing in the VHF1 band
double Pandauino_Freq_LF_VHF::frequencyTestHF = 0.0;														// Computed frequency when testing in the HF band
double Pandauino_Freq_LF_VHF::frequencyTest = 0.0;

operationType Pandauino_Freq_LF_VHF::operation = operation_none;								// Operation to do on the brute frequency value
double Pandauino_Freq_LF_VHF::lastValidFrequency = 0.0;                         // Last frequency that was displayed
double Pandauino_Freq_LF_VHF::refFrequency = 0.0;                              	// Reference frequency to use in the operation
double Pandauino_Freq_LF_VHF::resultFrequency = 0.0;							  						// Frequency +/- operation.

bool Pandauino_Freq_LF_VHF::outputToSerial =  false;                           	// True to print to serial port.
long Pandauino_Freq_LF_VHF::bauds = 57600;                            					// Serial monitor baud rate

float Pandauino_Freq_LF_VHF::calManValue = -9.0;																// Manual calibration value used to set the calibration value

float Pandauino_Freq_LF_VHF::underVoltageMinusHysteresis = underVoltage * ((100 - hysteresisVccPerCent) / 100); 	// Threshold voltage minus hysteresis
float Pandauino_Freq_LF_VHF::overVoltagePlusHysteresis = overVoltage * ((100 + hysteresisVccPerCent) / 100);    	// Ceiling voltage plus hysteresis
bool Pandauino_Freq_LF_VHF::voltageError = false;                              	// Flags vcc voltage error
unsigned long Pandauino_Freq_LF_VHF::lastVccMillis = 0;                        	// Time (millis) of last millis() call when Vcc was tested

unsigned long Pandauino_Freq_LF_VHF::sleepTimeout = 300000;                    	// Timeout that places the board in energy economy mode (sleep mode) (millis);
sleepMode Pandauino_Freq_LF_VHF::sleepSetting = sleep_5m;												// Sleep configuration

double Pandauino_Freq_LF_VHF::calibration = 1.0;                          			// Tweak it to precisely calibrate your board as compared to a very precise frequency reference, by the program or manually

// EEPROM Addresses
byte Pandauino_Freq_LF_VHF::addressOfMode = EEPROMbaseAddress + sizeof(eepromInit);
byte Pandauino_Freq_LF_VHF::addressOfBand = addressOfMode + sizeof(mode);
byte Pandauino_Freq_LF_VHF::addressOfResolution = addressOfBand + sizeof(band);
byte Pandauino_Freq_LF_VHF::addressOfCalibration = addressOfResolution + sizeof(resolution);
byte Pandauino_Freq_LF_VHF::addressOfMeasurementType = addressOfCalibration + sizeof(calibration);
byte Pandauino_Freq_LF_VHF::addressOfOperation = addressOfMeasurementType + sizeof(measurementType);
byte Pandauino_Freq_LF_VHF::addressOfSleepSetting = addressOfOperation + sizeof(operation);
byte Pandauino_Freq_LF_VHF::addressOfRefFrequency = addressOfSleepSetting + sizeof(sleepSetting);

//Buffers used to render lcd messages
char Pandauino_Freq_LF_VHF::text1[17] = "";
String Pandauino_Freq_LF_VHF::text = "";
char Pandauino_Freq_LF_VHF::line1[17] = "";



/* ************************************************************************************************************************************
  CONSTRUCTORS
**************************************************************************************************************************************/

Pandauino_Freq_LF_VHF::Pandauino_Freq_LF_VHF() {
  Pandauino_Freq_LF_VHF(mode_auto, band_HF, resolution_normal);
}

Pandauino_Freq_LF_VHF::Pandauino_Freq_LF_VHF(measurementMode _mode, measurementBand _band, measurementResolution _resolution) {
  mode = _mode;
  band = _band;
  resolution = _resolution;
}


/* ************************************************************************************************************************************
  PUBLIC FUNCTIONS
**************************************************************************************************************************************/

//*********************************************************************************************************
// configureComputation
// set up the board (prescaler path), compute PSC and  measurement period
// and starts the appropriate algorithm depending on measurement parameters
void Pandauino_Freq_LF_VHF::configureComputation(measurementMode _mode, measurementBand _band, measurementResolution _resolution) {
	mode = _mode;
	band = _band;
	resolution = _resolution;
	configureComputation(true);
}

//*********************************************************************************************************
// Stop computation
void Pandauino_Freq_LF_VHF::stopComputation() {

	FreqMeasure.end();
	FreqCount.end();

}

// ************************************************************************************************************************************
// freqSetup
// Called in the Arduino Setup section to initialize the board and software
void Pandauino_Freq_LF_VHF::freqSetup(boardType _boardVersion, boolean _outputToSerial = false, long _bauds = 57600) {

	boardVersion = _boardVersion;
	outputToSerial = _outputToSerial;
	bauds = _bauds;

  // Serial
  if (outputToSerial == true) {
    Serial.begin(bauds);
    while (!Serial);
    Serial.println("Init");
  }

  // activates the voltage regulator of the amplifier circuit
  pinMode(VccReg65EnablePin, OUTPUT);
  digitalWrite(VccReg65EnablePin, HIGH);

  // and the LCD
  pinMode(lcdLedPowerPin, OUTPUT);
  digitalWrite(lcdLedPowerPin, HIGH);

	// sets pins connected to mux to output mode
  pinMode(select1, OUTPUT);
  pinMode(select2, OUTPUT);

  // LCD
  // set up the LCD's number of columns and rows:
  // It's 16*1 but factory configured as 8*2 (1 line)
  lcd.begin(8, 2);
  printSixteenCharToLCD(initMessage);

  // Vcc test
  initVcc();
  VccTest();

  // sets up the button events
  button.setClickTicks(clickticks);
  button.attachClick(buttonClick);
  button.attachLongPressStart(buttonPress);

  // Loads parameters
  readAllFromEEPROM();
  setSleepTimeout();

	// sets up the prescaler and parameters for the given band and starts the algortithm
  configureComputation(true);
}


//************************************************************************************************************************************
// freqCount
// Called in the Arduino Loop section to run the measurement and manage user actions
void Pandauino_Freq_LF_VHF::freqCount() {

  // Checks machine state of menu button
  button.tick();

  if (editMode==display_main) { // i.e. not in the menu

	  // Tests VCC periodically
	  // !!! PB influence PC1
	  // A VERIFIER
	  //

	  if (timeToTestVCC()) {
//			stopComputation();
	    VccTest();
	    if (mode == mode_auto) {configureComputation(true);}
			measureStamp = millis();
	  };

    // if sleepTimeout reached places the board in power saving mode
    if ((sleepSetting != sleep_disabled) && ((millis() - displayStamp)  > sleepTimeout)) {
      standbyMode();
    };

		// ******** mode band **************************************
		if (mode == mode_band) { // The band was chosen by the user or determined in the band detection section

			if (band == band_LF) { //  LF

				// DEBUG
				//Serial.println("mode_band LF");

				frequencyTest = measureLF();

				if (frequencyTest > 0.0) {

					frequency = frequencyTest;

					// DEBUG
					// Serial.print("LF measure: ");
					// Serial.println(frequency);

					displayMeasurement();
					measureStamp = millis();
				}

				if ((millis() - measureStamp) > LFTimeout) {
	  		  printSixteenCharToLCD(const_cast<char*>(noMeasureAvailable));
	  		  measureStamp = millis();
				}

			} // LF

			else { // i.e. HF VHF

				frequencyTest = measureHF_VHF();
				if (frequencyTest > 0.0) {
					frequency = frequencyTest;
					displayMeasurement();
					measureStamp = millis();
				}

				if ((millis() - measureStamp) > (effectiveHFMeasurePeriod + 30)) {
	  		  printSixteenCharToLCD(const_cast<char*>(noMeasureAvailable));
	  		  measureStamp = millis();
				}

			} // HF VHF

		} // mode_band


  	// ******** mode auto / LF ************************************
  	else if ((mode == mode_auto) && (band == band_LF))  {

			//DEBUG
			//Serial.println("mode_auto LF");

			frequencyTest = measureLF();

			if (frequencyTest > 0.0) {

				measureStamp = millis();

			//DEBUG
			//Serial.println("frequencyTest: ");
			//Serial.println(frequencyTest);

				if (frequencyTest > freqLFmax) { 	// Frequency too high goes to HF/VHF computing
					band = band_HF;
					stopComputation();
				} else { 													// valid frequency
					frequency = frequencyTest;
					displayMeasurement();
				}
			}

			if ((millis() - measureStamp) > LFTimeout) {
				band = band_HF; 		// Goes to HF/VHF mode_auto computing
	  	  printSixteenCharToLCD(const_cast<char*>(noMeasureAvailable));
				stopComputation();
	  	  measureStamp = millis();
			}

  	// ******** mode auto in HF / VHF1 / VHF2 *********************
  	} else {

			frequencyTestVHF2 = 0.0;
			frequencyTestVHF1 = 0.0;
			frequencyTestHF = 0.0;

			// Testing VHF1 annd VHF2 band only for the VHF board version
			if (boardVersion == board_version_vhf) {

	  		// Here we determine the effective band
	  		// using freqCount in VHF2 then VHF1 then HF to check if the measure is in these ranges
	  		// If the value is in the LF range switch to LF

				// *********** Testing in the 20-210 MHz band
			  band = band_VHF2;
				configureComputation(true);
				delay(10); // to clear the prescaler buffering effect

/*
				// run once to clear the freqCount "buffer"
				frequencyTestVHF2 = 0.0;
				measureStamp = millis();

				while (frequencyTestVHF2 == 0.0) { // waits for a measurement
					frequencyTestVHF2 = measureHF_VHF();
					if ((millis() - measureStamp) > (effectiveHFMeasurePeriod + 30)) {
						break;				// Goes to next computation
					}
  				// Checks machine state of menu button so the menu can be called when in this loop
  				button.tick();
					// If the menu was called by button.tick() during these frequency tests , exits
					if (editMode != display_main) return;
				 	delay(10);
				}
*/
				// now trying to measure a valid frequency
				frequencyTestVHF2 = 0.0;
				measureStamp = millis();

				while (frequencyTestVHF2 == 0.0) { // waits for a measurement
					frequencyTestVHF2 = measureHF_VHF();
					if ((millis() - measureStamp) > (effectiveHFMeasurePeriod + 30)) {
						break;				// Goes to next computation
					}
  				button.tick();
					if (editMode != display_main) return;
				 	delay(10);
				}


				// *********** Testing in the 5-20 MHz band
			  band = band_VHF1;
				configureComputation(true);
				delay(10); // to clear the prescaler buffering effect
				frequencyTestVHF1 = 0.0;
				measureStamp = millis();

/*
				// run once to clear the freqCount "buffer"
				while (frequencyTestVHF1 == 0.0) { // waits for a measurement
					frequencyTestVHF1 = measureHF_VHF();
					if ((millis() - measureStamp) > (effectiveHFMeasurePeriod + 30)) {
						break;				// Goes to next computation
					}
  				button.tick();
					if (editMode != display_main) return;
				 	delay(10);
				}
*/
				// now trying to measure a valid frequency
				frequencyTestVHF1 = 0.0;
				measureStamp = millis();

				while (frequencyTestVHF1 == 0.0) { // waits for a measurement
					frequencyTestVHF1 = measureHF_VHF();
					if ((millis() - measureStamp) > (effectiveHFMeasurePeriod + 30)) {
						break;				// Goes to next computation
					}
  				button.tick();
					if (editMode != display_main) return;
				 	delay(10);
				}

			} // End testing VHF1/VHF2 only for VHF boards

			// *********** Testing in the 5 KHz -5 MHz band
		  band = band_HF;
			configureComputation(true);
			delay(10); // to clear the prescaler buffering effect
			frequencyTestHF = 0.0;
			measureStamp = millis();

/*
			// run once to clear the freqCount "buffer"
			while (frequencyTestHF == 0.0) { // waits for a measurement
				frequencyTestHF = measureHF_VHF();
				if ((millis() - measureStamp) > (effectiveHFMeasurePeriod + 30)) {
					break;				// Goes to next computation
				}
  			button.tick();
				if (editMode != display_main) return;
			 	delay(10);
			}
*/

			// now trying to measure a valid frequency
			frequencyTestHF = 0.0;
			measureStamp = millis();

			while (frequencyTestHF == 0.0) { // waits for a measurement
				frequencyTestHF = measureHF_VHF();
				if ((millis() - measureStamp) > (effectiveHFMeasurePeriod + 30)) {
					break;				// Goes to next computation
				}
  			button.tick();
				if (editMode != display_main) return;
			 	delay(10);
			}

			// We tested VH2, VHF1 and HF
			// We consider the highest frequency in case it was folded
			// Keep it, in case there was an aberration with for example a VHF1 freq above VHF2 freq beacuse of folding
			// DEBUG
			/*
			Serial.print("HF: ");
			Serial.println(frequencyTestHF);
			Serial.print("VHF1: ");
			Serial.println(frequencyTestVHF1);
			Serial.print("VHF2: ");
			Serial.println(frequencyTestVHF2);
			*/

			frequencyTest = frequencyTestHF;
			if (frequencyTestVHF1 > frequencyTest) { frequencyTest = frequencyTestVHF1;}
			if (frequencyTestVHF2 > frequencyTest) { frequencyTest = frequencyTestVHF2;}

			// But we use the value computed in its right corresponding band to avoid quantization error because of prescaling
			if ((boardVersion == board_version_hf) || ((freqHFmin <= frequencyTest) && (frequencyTest < freqVHF1min)))
				{ band= band_HF; frequencyTest = frequencyTestHF;}
			else if ((freqVHF1min <= frequencyTest) && (frequencyTest < freqVHF2min))
				{ band= band_VHF1;  frequencyTest = frequencyTestVHF1;}
			else
				{band = band_VHF2;  frequencyTest = frequencyTestVHF2;}

			// if frequency zero or in the LF range switch to LF computing.
			if (frequencyTest < freqLFmax) {

				band = band_LF;
				configureComputation(true);
				measureStamp = millis(); // false measureStamp to let LF measurement run

			} else {

				// displays the final value
				frequency = frequencyTest;
				// DEBUG
				//Serial.print("frequency: ");
				//Serial.println(frequency);
				displayMeasurement();

			} 	// switch to LF or not
		} 		// mode auto in HF / VHF1 / VHF2
	} 			// (editMode==display_main)

}


// ************************************************************************************************************************************
//  getFrequency / readFrequency
double Pandauino_Freq_LF_VHF::getFrequency() {
  return frequency;
}

// After reading frequency it is zeroed so the user knows when a new value is available
double Pandauino_Freq_LF_VHF::readFrequency() {
  double freq = frequency;
  frequency = 0.0;
  return freq;
}

// *********************************************************************************************************
// standbyMode()
// invoked when sleepTimeout reached or on demand
// places the board in low power consumption mode
void Pandauino_Freq_LF_VHF::standbyMode() {

  // debug
  //Serial.println(F("Enter sleep mode"));
  //delay(100);

  printSixteenCharToLCD(const_cast<char*>(goingStandby));
  delay(5000);

  // Turn off LCD display
  lcd.noDisplay();

  // Turn off power to LCD and 6.7V regulator
  digitalWrite(lcdLedPowerPin, LOW);          // lcd power cut
  digitalWrite(VccReg65EnablePin, LOW);       // 6.5V regulator disabled

  delay(1000);
  // ready to sleep
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  attachInterrupt(digitalPinToInterrupt(PUSHBUTTON), pushButtonInterrupt, LOW);    // attaches an interrupt on PUSHBUTTON pin

  // put to sleep
  sleep_mode();

  // ****************
  // wake up here
  detachInterrupt(digitalPinToInterrupt(PUSHBUTTON)); // avoid continuous firing

  // frequency re-initialized
  frequency = 0;

  digitalWrite(lcdLedPowerPin, HIGH);          // lcd power on
  digitalWrite(VccReg65EnablePin, HIGH);       // 6.5V regulator enabled

  lcd.display();

  // Let the amplifier circuit charge.
  delay(100);
  measureStamp = millis();                // false measureStamp to avoid falling into sleep again

}

// ************************************************************************************************************************************
//  Serial
void Pandauino_Freq_LF_VHF::beginSerial(long bds) {
  bauds = bds;
  Serial.begin(bauds);
  outputToSerial = true;
}

void Pandauino_Freq_LF_VHF::endSerial() {
  Serial.end();
  outputToSerial = false;
}

//*********************************************************************************************************
// Calibrate
// Used to calibrate the device against a frequency source with a voltage between 1V and 5V and a precision better than 1 ppm
void Pandauino_Freq_LF_VHF::calibrate(long calFrequency) {

  double calib;
  byte i = 0;
  countLF = 0;
  countHF = 0;

	printSixteenCharToLCD(calibrating);

  previousMode = mode;
  previousBand = band;
	previousResolution = resolution;

  determineBand(calFrequency);
  resolution = resolution_high;
  configureComputation(true);

	if (algorithm == algorithm_freqCount) {

	  // first measure is wrong so we skip it.
	  while (!FreqCount.available()) {}
	  while (!FreqCount.available()) {}

	  countHF = FreqCount.read();
	  frequency = countHF * prescalerCoef;
  	FreqCount.end();

	} else {

    while (!FreqMeasure.available()) {}

 		countLF = FreqMeasure.read();
		frequency = FreqMeasure.countToFrequency(countLF) * prescalerCoef;
  	FreqMeasure.end();
	}

  calib = calFrequency / frequency;

	// DEBUG
	// Serial.println(frequency,8);

  if ((calib < 0.99998) || (calib > 1.00002)) {

		printSixteenCharToLCD(calNotPrecise);
    delay(5000);
    return;
  }

  calibration = calib;
  EEPROM.put(addressOfCalibration, calibration);

  dtostrf(calibration, 8, 7, text1);

	text = "Cal: ";

  text.concat((String)text1);

  text.toCharArray(line1, 16);  // reconvertir en char[17]
	printSixteenCharToLCD(line1);

  delay(2000);

	// set back to original parameters. ConfigureComputation is not called. It will be when leaving the menu.
  mode = previousMode;
  band = previousBand;
	resolution = previousResolution;

}


/* ************************************************************************************************************************************
  MEASUREMENT FUNCTIONS
**************************************************************************************************************************************/

//*********************************************************************************************************
// configureComputation
// set up the board (prescaler path), compute prescaler and  measurement period
// and starts the appropriate algorithm depending on measurement parameters
void Pandauino_Freq_LF_VHF::configureComputation(bool restart = false) {

	// DEBUG
	// Serial.println("ConfigureComputation");

	algorithmType previousAlgorithm = algorithm;

	pinMode(select1, OUTPUT);
	pinMode(select2, OUTPUT);

	switch (band) {

		case band_LF:
			algorithm = algorithm_freqMeasure;
			prescalerCoef =	100;
			break;

		case band_HF:
			algorithm = algorithm_freqCount;
			prescalerCoef =	1;
			digitalWrite(select1,HIGH);
			digitalWrite(select2,HIGH); // In fact not important
			break;

		case band_VHF1:
			algorithm = algorithm_freqCount;
			prescalerCoef =	coefVHF1;
			digitalWrite(select1,LOW);
			digitalWrite(select2,HIGH);
			break;

		case band_VHF2:
			algorithm = algorithm_freqCount;
			prescalerCoef =	coefVHF2;
			digitalWrite(select1,LOW);
			digitalWrite(select2,LOW);
			break;

	}

	switch (resolution) {

		case resolution_low:
			displayPrecision = 4;
			measurementTimeCoefficient = 0.01;
			break;

		case resolution_normal:
			displayPrecision = 5;
			measurementTimeCoefficient = 0.1;
			break;

		case resolution_high:
			displayPrecision = 6;
			measurementTimeCoefficient = 1.0;
			break;

		case resolution_ultra_high:
			displayPrecision = 7;
			measurementTimeCoefficient = 10.0;
			break;
	}

	if (algorithm == algorithm_freqMeasure) {
		prescalerCoef *= measurementTimeCoefficient;
	  LFTimeout = (prescalerCoef / 10) * LFTimeoutNormalRes;
	}

	if (algorithm == algorithm_freqCount) {
		prescalerCoef /= measurementTimeCoefficient;
		effectiveHFMeasurePeriod = HFMeasurePeriodNormalRes *  measurementTimeCoefficient;
	}

  if ((algorithm != previousAlgorithm) || (restart == true)) {

		if (previousAlgorithm == algorithm_freqMeasure) 	FreqMeasure.end();
		if (previousAlgorithm == algorithm_freqCount) 	FreqCount.end();

		// Starts the appropriate measurement method
		if (algorithm == algorithm_freqMeasure) {
			// DEBUG
			// Serial.println("Starting freqMeasure");
			// Serial.println(prescalerCoef);
			FreqMeasure.begin(prescalerCoef);
		} else {
			// DEBUG
			// Serial.println("Starting freqCount");
			FreqCount.begin(effectiveHFMeasurePeriod);
		}
	} // algo != previous

}

// ************************************************************************************************************************************
// measureLF
// Measure using freqMeasure
double Pandauino_Freq_LF_VHF::measureLF() {

	double freq = 0.0;

	if (FreqMeasure.available()) {
		countLF = FreqMeasure.read();
		freq = FreqMeasure.countToFrequency(countLF) * prescalerCoef * calibration;
	}

	return freq;

}

// ************************************************************************************************************************************
// measureHF
// Measure using freqCount
double Pandauino_Freq_LF_VHF::measureHF_VHF() {

	double freq = 0.0;

	if (FreqCount.available()) {
		freq = FreqCount.read() * prescalerCoef  * calibration;
	}

	return freq;
}

/* ************************************************************************************************************************************
  EEPROM AND INIT FUNCTIONS
**************************************************************************************************************************************/

// *********************************************************************************************************
// Reads parameters stored in EEPROM
void Pandauino_Freq_LF_VHF::readAllFromEEPROM() {

  byte init = EEPROM.read(EEPROMbaseAddress);

  if (init == eepromInit) {
		EEPROM_readAnything(addressOfMode, mode);
		EEPROM_readAnything(addressOfBand, band);
		EEPROM_readAnything(addressOfResolution, resolution);
		EEPROM_readAnything(addressOfCalibration, calibration);
		EEPROM_readAnything(addressOfMeasurementType, measurementType);
		EEPROM_readAnything(addressOfOperation, operation);
		EEPROM_readAnything(addressOfSleepSetting, sleepSetting);
		EEPROM_readAnything(addressOfRefFrequency, refFrequency);
  }
  else updateAllToEEPROM();

}

//*********************************************************************************************************
// Retrieves reference frequency from EEPROM
void Pandauino_Freq_LF_VHF::readFromEEPROM_refFrequency(){
	EEPROM_readAnything(addressOfRefFrequency, refFrequency);
}


//*********************************************************************************************************
// Updates parameters stored in EEPROM
void Pandauino_Freq_LF_VHF::updateToEEPROM_init(){
	EEPROM_writeAnything(EEPROMbaseAddress, eepromInit);
}

void Pandauino_Freq_LF_VHF::updateToEEPROM_mode(){
	EEPROM_writeAnything(addressOfMode, mode);
}

void Pandauino_Freq_LF_VHF::updateToEEPROM_band(){
	EEPROM_writeAnything(addressOfBand, band);
}

void Pandauino_Freq_LF_VHF::updateToEEPROM_resolution(){
	EEPROM_writeAnything(addressOfResolution, resolution);
}

void Pandauino_Freq_LF_VHF::updateToEEPROM_calibration(){
	EEPROM_writeAnything(addressOfCalibration, calibration);
}

void Pandauino_Freq_LF_VHF::updateToEEPROM_measurementType(){
	EEPROM_writeAnything(addressOfMeasurementType, measurementType);
}

void Pandauino_Freq_LF_VHF::updateToEEPROM_operation(){
	EEPROM_writeAnything(addressOfOperation, operation);
}

void Pandauino_Freq_LF_VHF::updateToEEPROM_sleepSetting(){
	EEPROM_writeAnything(addressOfSleepSetting, sleepSetting);
}

void Pandauino_Freq_LF_VHF::updateToEEPROM_refFrequency(){
	EEPROM_writeAnything(addressOfRefFrequency, refFrequency);

}

void Pandauino_Freq_LF_VHF::updateAllToEEPROM() {

	updateToEEPROM_init();
	updateToEEPROM_mode();
	updateToEEPROM_band();
	updateToEEPROM_resolution();
	updateToEEPROM_calibration();
	updateToEEPROM_measurementType();
	updateToEEPROM_operation();
	updateToEEPROM_sleepSetting();
	updateToEEPROM_refFrequency();

}

//*********************************************************************************************************
// Software Reset
void (*Pandauino_Freq_LF_VHF::resetFunc) (void) = 0;

//*********************************************************************************************************
// setSleepTimeout
// Used to set the sleepTimeout parameter
void Pandauino_Freq_LF_VHF::setSleepTimeout() {

	if (sleepSetting == sleep_30s) { sleepTimeout = 30000; }
	else if (sleepSetting == sleep_5m) { sleepTimeout = 300000; }

}

/* ************************************************************************************************************************************
  VCC TESTING FUNCTIONS
**************************************************************************************************************************************/

//*********************************************************************************************************
void Pandauino_Freq_LF_VHF::initVcc() {

  // initializes the ADC: clock prescaler 128, Ref AVCC, input ADC7
  ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) |(1<<ADPS0) ;
  ADMUX =  (1<<REFS0) | 0x07; // Ref AVCC, input ADC7

}

//*********************************************************************************************************
// readVcc()
// gives the voltage applied through a resistor network divider to ADC0, in millivolts
// using an external reference applied to AREF pin
float Pandauino_Freq_LF_VHF::readVcc() {

  ADCSRA |= _BV(ADEN); 								// Start ADC
  delay(2); 													// Wait for Vref to settle
  ADCSRA |= _BV(ADSC); 								// Start conversion
  while (bit_is_set(ADCSRA, ADSC)); 	// measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  float result = (high << 8) | low;

  result = result * coefVcc;   // Computes Vcc (in mV);

  return result;
}


//*********************************************************************************************************
// VccTest()
// detects under or over VCC voltages
// triggers vcc error when vcc is outside of underVoltageMinusHysteresis or overVoltagePlusHysteresis values
// maintains vcc error condition as long as vcc is not between underVoltage and overVoltage values
void Pandauino_Freq_LF_VHF::VccTest() {

  float avgVcc = 0.0;
  float vcc = 0.0;

  voltageError = false;
  power_adc_enable();
  delay(1);

  do {
    vcc = readVcc();

    if (vcc < underVoltageMinusHysteresis) {
      if (!voltageError) {
  			printSixteenCharToLCD(msgVoltageTooLow);
  		}
      voltageError = true;
    } else if (vcc > overVoltagePlusHysteresis) {
      if (!voltageError) {
	    	printSixteenCharToLCD(msgVoltageTooHigh);
      }
      voltageError = true;
    }

    if ((vcc > underVoltage) and (vcc < overVoltage)) {
      voltageError = false;
    }

    delay(vccDelayPeriod);

  } while (voltageError);

  power_adc_disable();
}

//*********************************************************************************************************
// determines if enough time has passed since last Vcc test
bool Pandauino_Freq_LF_VHF::timeToTestVCC() {


  unsigned long elapsedMillis = 0;
  bool timeToTest = false;

  elapsedMillis = millis() - lastVccMillis;

  if (elapsedMillis > VccTestPeriod * 1000) {
    lastVccMillis = millis();
    timeToTest = true;
  };

  return timeToTest;
}

/* ************************************************************************************************************************************
  DISPLAY AND MANUAL CALIBRATION FUNCTIONS
**************************************************************************************************************************************/

//*********************************************************************************************************
// Needed to use 1 line LCD because it displays 2 * 8 chars on one line
void Pandauino_Freq_LF_VHF::sixteenTo8chars (char inputChar[17], char outputChar1[8], char outputChar2[8] ) {

  // cuts a char[17] into 2 * char[8]

  int i = 0;

  for (i = 0; i < 8; i++){
       outputChar1[i] = inputChar[i];
  }

  for (i = 8; i < 16; i++){
       outputChar2[i-8] = inputChar[i];
  }
}


//*********************************************************************************************************
// Used to print a line on the LCD
void Pandauino_Freq_LF_VHF::printSixteenCharToLCD (char toPrint[17]) {

  // prints a char[17] message to a 16*1 LCD screen configured as 2*8 characters on one line

   char out1[8];
   char out2[8];

   sixteenTo8chars (toPrint , out1, out2);

   lcd.clear();
   lcd.setCursor(0,0);
   lcd.print(out1);
   lcd.setCursor(0,1);
   lcd.print(out2);
}

//*********************************************************************************************************
// Called by displayMeasurement() when displaying a frequency value
// Either this value is the product of an operation or not
void Pandauino_Freq_LF_VHF::displayFrequency() {

  double displayMeasure = 0.0;
  double displayRef = 0.0;
  byte effectiveDisplayPrecision;

  String unit = "";
  byte nbOfIntDigits = 0;
  byte nbOfDecimals = 0;

  if (abs(frequency) >= 1000000.0) {					// We use the frequency and not the resultFrequency to display coherently with the actual frequency range
    displayMeasure = resultFrequency / 1000000;
    unit.concat(" MHz ");
  }
  else if (abs(frequency) >= 1000.0){
    displayMeasure = resultFrequency / 1000;
    unit.concat(" KHz ");
  }
	else {
    displayMeasure = resultFrequency;
    unit.concat(" Hz ");
	}

	if (frequency<10000.0) { effectiveDisplayPrecision = displayPrecision -1; }
	else { effectiveDisplayPrecision = displayPrecision ; }

  nbOfIntDigits = countIntDigits(displayMeasure);

  if (nbOfIntDigits == 0) {
  	nbOfDecimals = effectiveDisplayPrecision - nbOfIntDigits - 1;
  }
  else {
    nbOfDecimals = effectiveDisplayPrecision - nbOfIntDigits;
  }

  // patch whenever some float comparison turns false (foar example at 100 Khz exactly)
  if ((nbOfIntDigits + nbOfDecimals) > effectiveDisplayPrecision)  nbOfDecimals--;

  dtostrf(displayMeasure, effectiveDisplayPrecision, nbOfDecimals, text1);

  text = (String)text1;

  text.concat(unit);

	if (operation != operation_none) {
		if (operation == operation_vfo_plus) text.concat("V+I");
		if (operation == operation_vfo_minus) text.concat("V-I");
		if (operation == operation_if_minus) text.concat("I-F");
	}
	else {
		if (mode == mode_auto) text.concat("AUT");
		if ((mode == mode_band) and (band == band_LF)) text.concat("LF");
		if ((mode == mode_band) and (band == band_HF)) text.concat("HF");
		if ((mode == mode_band) and (band == band_VHF1)) text.concat("VH1");
		if ((mode == mode_band) and (band == band_VHF2)) text.concat("VH2");
	}

  text.toCharArray(line1, 17);  // reconvertir en char[17] y compris un caractère vide nécessaire
  printSixteenCharToLCD(line1);

  if (outputToSerial) Serial.println(resultFrequency,7);

}


//*********************************************************************************************************
// Called by displayMeasurement when displaying a period value
// Either this value is the product of an operation or not
void Pandauino_Freq_LF_VHF::displayPeriod() {

  double period = 1/resultFrequency;
  byte displayPrecision = 4;

  byte i = 1;
  String unit = " E-";
  double periodPow = 0.0;
  int entirePart = 0;
  double decimalPart = 0.0;

  // computation is complicated because of approximation in the comparison entirePart > 0
  // due to the resolution of float. Hence the check if the entirePart is "10"

  for (i=1; i<10; i++) {
		periodPow = period * pow(10,i);
  	entirePart = (int) (period * pow(10,i));
		decimalPart = periodPow - entirePart;
  	if (abs(entirePart) > 0) break;
  }


  if (abs(entirePart) == 10) {

    // cover the case of an error of the test (entirePart > 0) due to approximation in float operation
  	periodPow = (entirePart + decimalPart) / 10;
		i--;

  } else {

    // manually floors the decimals to avoid an error of ceiling the value to 10 due to approximation in float operation
    // there might be an error remaining for negative values
		decimalPart *= 100000;
		decimalPart = floor(decimalPart);
		decimalPart /= 100000;
		periodPow = entirePart + decimalPart;

  }

	switch (resolution) {

		case resolution_low:
	  dtostrf(periodPow, 4, 3, text1);
		break;

		case resolution_normal:
	  dtostrf(periodPow, 5, 4, text1);
		break;

		case resolution_high:
	  dtostrf(periodPow, 6, 5, text1);
		break;

		case resolution_ultra_high:
	  dtostrf(periodPow, 7, 6, text1);
		break;
	}

  text = (String)text1;

  unit.concat(i);
  unit.concat(" s ");

  text.concat(unit);
  text.toCharArray(line1, 17);
  printSixteenCharToLCD(line1);

  if (outputToSerial) Serial.println(resultFrequency,7);

}

//*********************************************************************************************************
// Tests if frequency is out of the defined band and displays en error message if needed
boolean Pandauino_Freq_LF_VHF::testFrequencyOutOfRange() {

 	boolean error = false;

 	if 	((band==band_HF) && (frequency < freqHFmin)) {
		error = true;
		text = "f < ";
   	dtostrf(freqHFmin, 8, 0, text1);
	}

 	if 	((band==band_VHF1) && (frequency < freqVHF1min)) {
		error = true;
	}

 	if 	((band==band_VHF2) && (frequency < freqVHF2min)) {
		error = true;
	}

 	if 	((band==band_LF) && (frequency > freqLFmax)) {
		error = true;
	}

 	if 	((boardVersion == board_version_hf) && (band==band_HF) && (frequency > freqHFmax_hf_board)) {
		error = true;
	}

 	if 	((boardVersion == board_version_vhf) && (band==band_HF) && (frequency > freqHFmax_vhf_board)) {
		error = true;
	}

 	if 	((band==band_VHF1) && (frequency > freqVHF1max)) {
		error = true;
	}

	if (error==true) {
		printSixteenCharToLCD(const_cast<char*>(frequencyOutOfRange));
		return true;
	} else {
		return false;
	}

}

//*********************************************************************************************************
// Main routine for displaying measurement value and parameters
void Pandauino_Freq_LF_VHF::displayMeasurement() {

  // in LF band, when trying to display measurement before the displayTimeLap is elapsed, sums the value
  // this is to avoid scintillation of the LCD and to increase averaging
  // displayTimeLap may be reduced if willing to get more frequent results (to PC as an example)

  if (band == band_LF) {

    if ((millis() - displayStamp) < displayTimeLap) {
      sumDisplayFreq = sumDisplayFreq + frequency;
      nbAvgDisplayFreq = nbAvgDisplayFreq + 1;
      return;
    }
    else { // computes the average value
      frequency = (frequency + sumDisplayFreq) / (nbAvgDisplayFreq + 1);
      sumDisplayFreq = 0.0;
      nbAvgDisplayFreq = 0;
    }
  } // band_LF

	else {	// Also to avoid flickering
	  if ((millis() - displayStamp) < displayTimeLap) {
	  	return;
	  }
	} // flickering in HF/VHF1/VHF2

	// DEBUG
	/*Serial.print("millis: ");
	Serial.println(millis());
	Serial.print("displayStamp: ");
	Serial.println(displayStamp);
	*/

	// When in mode_band checks that the frequency is within the specified band limits, otherwise the resolution might be decreased
	if (mode != mode_auto) {
		if (testFrequencyOutOfRange() == true) {
			displayStamp = millis(); // to avoid flickering of this error message
			return;
		}
	}

  switch (operation) {
		case operation_none: resultFrequency = frequency; break; // No operation applied
		case operation_vfo_plus: resultFrequency = frequency + refFrequency; break;
		case operation_vfo_minus: resultFrequency = frequency - refFrequency; break;
		case operation_if_minus: resultFrequency = refFrequency - frequency;
	}

	// Used to store the ref frequency
	lastValidFrequency = frequency;

  // display frequency or period
  if (measurementType == measure_frequency) displayFrequency();
  else displayPeriod();

  displayStamp = millis();

}

//*********************************************************************************************************
// DisplayCalManValue
// Displays a calibration value expressed in ppm
void Pandauino_Freq_LF_VHF::displayCalManValue() {

	text = "Cal: ";
	dtostrf(calManValue, 3, 1, text1);
  text.concat((String)text1);
  text.concat(" ppm");
  text.toCharArray(line1, 17);
  printSixteenCharToLCD(line1);

}
//*********************************************************************************************************
// EvaluateCalManValue
// evaluates the calManValue (used of setting) depending on calibration value (stored and use globally)
void Pandauino_Freq_LF_VHF::evaluateCalManValue() {

	int eval = (int)(2 * (calibration * 1000000.0 - 1000000.0));
	calManValue = eval/2.0;

}

/* ************************************************************************************************************************************
  MISC FUNCTIONS
**************************************************************************************************************************************/

//*********************************************************************************************************
// Counts the number of digits of the integer part
byte Pandauino_Freq_LF_VHF::countIntDigits(int num) {

  // counts the number of digits in a numbrer. Example: 123.34 --> 3
  byte count = 0;
  while (num) {
    num = num / 10;
    count++;
  }
  return count;
}

//*********************************************************************************************************
// deterline the band for a given frequency
void Pandauino_Freq_LF_VHF::determineBand(float freq) {

	// Determines the band depending on the freq
	if ((long)freq > freqVHF2min) {
		band = band_VHF2;
	} else if ((long)freq > freqVHF1min) {
		band = band_VHF1;
  }	else if ((long)freq > freqHFmin) {
		band = band_HF;
	} else {
		band = band_LF;
	}

}


/* ************************************************************************************************************************************
  INTERFACE FUNCTIONS
**************************************************************************************************************************************/

// ************************************************************************************************************************************
// Do all the actions after a user menu choice
void Pandauino_Freq_LF_VHF::actions() {

	previousMode = mode;
	previousBand = band;
	previousResolution = resolution;
	previousCalibration = calibration;
	previousMeasurementType = measurementType;
	previousOperation = operation;
	previousSleepSetting = sleepSetting;

	switch (editMode) {

		case display_freq_band_auto:
		mode = mode_auto;
		band = band_HF;
		resolution = resolution_normal;
		break;

		case display_freq_band_LF:
		mode = mode_band;
		band = band_LF;
		resolution = resolution_high;
		break;

		case display_freq_band_HF_HF_board:
		mode = mode_band;
		band = band_HF;
		resolution = resolution_high;
		break;

		case display_freq_band_HF_VHF_board:
		mode = mode_band;
		band = band_HF;
		resolution = resolution_high;
		break;

		case display_freq_band_VHF1:
		mode = mode_band;
		band = band_VHF1;
		resolution = resolution_high;
		break;

		case display_freq_band_VHF2:
		mode = mode_band;
		band = band_VHF2;
		resolution = resolution_high;
		break;

		case display_resolution_low:
		resolution = resolution_low;
		break;

		case display_resolution_normal:
		resolution = resolution_normal;
		break;

		case display_resolution_high:
		resolution = resolution_high;
		break;

		case display_resolution_ultra_high:
		resolution = resolution_ultra_high;
		break;

		case display_calibration_4M:
		calibrate(4000000);
		break;

		case display_calibration_10M:
		calibrate(10000000);
		break;

		case display_calibration_manual_set:
		calibration = (1000000.0 + calManValue) / 1000000.0;
		updateToEEPROM_calibration();
		break;

		case display_frequency:
		measurementType = measure_frequency;
		break;

		case display_period:
		measurementType = measure_period;
		break;

		case display_store:
		refFrequency = lastValidFrequency;
		updateToEEPROM_refFrequency();
		printSixteenCharToLCD(const_cast<char*>(frequencySaved));
		delay(2000);
		break;

		case display_retrieve:
		readFromEEPROM_refFrequency();
		resultFrequency = refFrequency;
		displayFrequency();
		delay(2000);
		frequency = 0.0;
		break;

		case display_operation_annul:
		operation = operation_none;
		break;

		case display_operation_vfo_plus:
		operation = operation_vfo_plus;
		break;

		case display_operation_vfo_minus:
		operation = operation_vfo_minus;
		break;

		case display_operation_if_minus:
		operation = operation_if_minus;
		break;

		case display_sleep_30s:
		sleepTimeout = 30000;
		sleepSetting = sleep_30s;
		break;

		case display_sleep_5m:
		sleepTimeout = 300000;
		sleepSetting = sleep_5m;
		break;

		case display_sleep_disabled:
		sleepSetting = sleep_disabled;
		break;

		case display_factory_reset:
		EEPROM_writeAnything(EEPROMbaseAddress, eepromInit+1); // dummy value to force update EEPROM after reset
		resetFunc();
	}

	if (mode != previousMode) updateToEEPROM_mode();
	if (band != previousBand) updateToEEPROM_band();
	if (resolution != previousResolution) updateToEEPROM_resolution();
	if (calibration != previousCalibration) updateToEEPROM_calibration();
	if (measurementType != previousMeasurementType) updateToEEPROM_measurementType();
	if (operation != previousOperation) updateToEEPROM_operation();
	if (sleepSetting != previousSleepSetting) { setSleepTimeout(); updateToEEPROM_sleepSetting();}

}


/* ************************************************************************************************************************************
  BUTTON EVENT HANDLERS
**************************************************************************************************************************************/

// ************************************************************************************************************************************
// Wake-up routine after sleep
static void Pandauino_Freq_LF_VHF::pushButtonInterrupt()
{
  displayStamp = millis(); // to reset the energy economy timeout
}

// ************************************************************************************************************************************
// Menu buton press handler
void Pandauino_Freq_LF_VHF::buttonPress() {

  bool treated = false;

	actions();

  if (editMode == display_main) {
    editMode = display_freq_band;
		stopComputation();
    printSixteenCharToLCD(menuEntries[editMode]);
		delay(500);
		return;
	// Exiting the menu
  } else if (editMode == display_exit_menu) {
    editMode = display_main;
    displayStamp = millis(); // to avoid going to sleep after a long usage of the menu
    configureComputation(true);
    printSixteenCharToLCD(menuEntries[editMode]);
		return;
	}

	// All the going to upper level moves
	if ((editMode == display_freq_band_auto) || (editMode == display_freq_band_LF) ||  (editMode == display_freq_band_HF_HF_board) || (editMode == display_freq_band_HF_VHF_board) || (editMode == display_freq_band_VHF1) || (editMode == display_freq_band_VHF2)) { editMode = display_freq_band; treated = true;}
	if ((editMode == display_resolution_low) || (editMode == display_resolution_normal) || (editMode == display_resolution_high) || (editMode == display_resolution_ultra_high)) {editMode = display_resolution; treated = true;}
	if ((editMode == display_calibration_4M) || (editMode == display_calibration_10M) || (editMode == display_calibration_manual_set)){ editMode = display_calibration; treated = true;}
	if ((editMode == display_frequency) || (editMode == display_period)){ editMode = display_fp; treated = true;}
	if ((editMode == display_operation_annul) || (editMode == display_operation_vfo_plus) ||  (editMode == display_operation_vfo_minus) || (editMode == display_operation_if_minus) ){ editMode = display_operation; treated = true;}
	if ((editMode == display_sleep_30s) || (editMode == display_sleep_5m) || (editMode == display_sleep_disabled)){ editMode = display_sleep; treated = true;}

	if (treated == true){  printSixteenCharToLCD(menuEntries[editMode]); return;}

	// Moves down the menu tree
	switch (editMode) {

		case display_freq_band:
			if (mode == mode_auto) editMode = display_freq_band_auto;
			else {
				if (band == band_LF) editMode = display_freq_band_LF;
				if ((band == band_HF) && (boardVersion == board_version_hf)) editMode = display_freq_band_HF_HF_board;
				if ((band == band_HF) && (boardVersion == board_version_vhf)) editMode = display_freq_band_HF_VHF_board;
				if (band == band_VHF1) editMode = display_freq_band_VHF1;
				if (band == band_VHF2) editMode = display_freq_band_VHF2;
			}
		break;

		case display_resolution:
		if (resolution == resolution_low) editMode = display_resolution_low;
		if (resolution == resolution_normal) editMode = display_resolution_normal;
		if (resolution == resolution_high) editMode = display_resolution_high;
		if (resolution == resolution_ultra_high) editMode = display_resolution_ultra_high;
		break;

		case display_calibration: editMode = display_calibration_4M; break;

		case display_calibration_manual:
		editMode = display_calibration_manual_set;
		evaluateCalManValue();
		displayCalManValue();
		break;

		case display_fp:
		if (measurementType == measure_frequency) editMode = display_frequency;
		if (measurementType == measure_period) editMode = display_period;
		break;

		case display_operation:
		if (operation == operation_none) editMode = display_operation_annul;
		if (operation == operation_vfo_plus) editMode = display_operation_vfo_plus;
		if (operation == operation_vfo_minus) editMode = display_operation_vfo_minus;
		if (operation == operation_if_minus) editMode = display_operation_if_minus;
		break;

		case display_sleep:
		if (sleepSetting == sleep_30s) editMode = display_sleep_30s;
		if (sleepSetting == sleep_5m) editMode = display_sleep_5m;
		if (sleepSetting == sleep_disabled) editMode = display_sleep_disabled;

	}

 	if (editMode != display_calibration_manual_set) {
 		printSixteenCharToLCD(menuEntries[editMode]);
 		delay(500);
	}

}

//*********************************************************************************************************
// Menu button click handler
void Pandauino_Freq_LF_VHF::buttonClick() {

	switch (editMode) {

		case display_freq_band:
		editMode = display_resolution;
		break;

		case display_resolution:
		editMode = display_calibration;
		break;

		case display_calibration:
		editMode = display_fp;
		break;

		case display_fp:
		editMode = display_store;
		break;

		case display_store:
		editMode = display_retrieve;
		break;

		case display_retrieve:
		editMode = display_operation;
		break;

		case display_operation:
		editMode = display_sleep;
		break;

		case display_sleep:
		editMode = display_factory_reset;
		break;

		case display_factory_reset:
		editMode = display_exit_menu;
		break;

		case display_exit_menu:
		editMode = display_freq_band;
		break;

		case display_freq_band_auto:
		editMode = display_freq_band_LF;
		break;

		case display_freq_band_LF:
		if (boardVersion == board_version_hf) editMode = display_freq_band_HF_HF_board;
		if (boardVersion == board_version_vhf) editMode = display_freq_band_HF_VHF_board;
		break;

		case display_freq_band_HF_HF_board:
		// No access to VHF1 / VHF2 band for the HF board version
		editMode = display_freq_band_auto;
		break;

		case display_freq_band_HF_VHF_board:
		editMode = display_freq_band_VHF1;
		break;

		case display_freq_band_VHF1:
		editMode = display_freq_band_VHF2;
		break;

		case display_freq_band_VHF2:
		editMode = display_freq_band_auto;
		break;

		case display_resolution_low:
		editMode = display_resolution_normal;
		break;

		case display_resolution_normal:
		// Forbids high resolution in mode_auto because it would take quite a lot of time to compute/
		// The user can use mode_band to get higher resolution.
		if (mode == mode_auto) { editMode = display_resolution_low;}
		else { editMode = display_resolution_high;}
		break;

		case display_resolution_high:
		editMode = display_resolution_ultra_high;
		break;

		case display_resolution_ultra_high:
		editMode = display_resolution_low;
		break;

		case display_calibration_4M:
		editMode = display_calibration_10M;
		break;

		case display_calibration_10M:
		editMode = display_calibration_manual;
		break;

		case display_calibration_manual:
		editMode = display_calibration_4M;
		break;

		case display_calibration_manual_set:
		calManValue +=0.5;
		if (calManValue >= maxCalibManValue) calManValue = minCalibManValue;
		displayCalManValue();
		break;

		case display_frequency:
		editMode = display_period;
		break;

		case display_period:
		editMode = display_frequency;
		break;

		case display_operation_annul:
		editMode = display_operation_vfo_plus;
		break;

		case display_operation_vfo_plus:
		editMode = display_operation_vfo_minus;
		break;

		case display_operation_vfo_minus:
		editMode = display_operation_if_minus;
		break;

		case display_operation_if_minus:
		editMode = display_operation_annul;
		break;

		case display_sleep_30s:
		editMode = display_sleep_5m;
		break;

		case display_sleep_5m:
		editMode = display_sleep_disabled;
		break;

		case display_sleep_disabled:
		editMode = display_sleep_30s;
		break;
	}

 	if (editMode != display_calibration_manual_set) {
 		printSixteenCharToLCD(menuEntries[editMode]);
	}

}

Pandauino_Freq_LF_VHF frequencyCounter;



