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

  Frequency band >
    AUTO
    5 Hz - 5 KHz
    5 KHz - 5 MHz
    4 - 22 MHz
    22 - 210 MHz

  Resolution     >
			LOW
  		NORMAL
  		HIGH
  		ULTRA HIGH

  Calibration    >
  	Cal 4 MHz (press)
  	Cal 10 MHz (press)
  	Cal manual     >
  		Cal: cal value

  Display f/p    >
  	Display frequency
  	Display period

  Store (press)

  Retrieve (press)

  Operation      >
  	Annul op.
    VFO+IF
    VFO-IF
    IF-VFO

  Sleep          >
  	Sleep 30 s.
  	Sleep 5 m.
  	Sleep disabled

  < Exit menu

*/


#ifndef Pandauino_Freq_LF_VHF_h
#define Pandauino_Freq_LF_VHF_h

#include <Arduino.h>
#include <EEPROM.h>
#include <FreqCount.h>
#include <FreqMeasure.h>
#include <LiquidCrystal.h>
#include <OneButton.h>
#include <avr/power.h>
#include <avr/sleep.h>


/* ************************************************************************************************************************************
  TEMPLATES
**************************************************************************************************************************************/

template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    int i;
    for (i = 0; i < sizeof(value); i++)
        EEPROM.update(ee++, *p++);
    return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    int i;
    for (i = 0; i < sizeof(value); i++)
        *p++ = EEPROM.read(ee++);
    return i;
}


/* ************************************************************************************************************************************
  ENUM
**************************************************************************************************************************************/
enum boardType {
	board_version_hf,
	board_version_vhf
};

enum runMode {
	display_main,

	display_freq_band,
	display_freq_band_auto,
	display_freq_band_LF,
	display_freq_band_HF_HF_board,
	display_freq_band_HF_VHF_board,
	display_freq_band_VHF1,
	display_freq_band_VHF2,

	display_resolution,
	display_resolution_low,
	display_resolution_normal,
	display_resolution_high,
	display_resolution_ultra_high,

	display_calibration,
	display_calibration_4M,
	display_calibration_10M,
	display_calibration_manual,
	display_calibration_manual_set,

	display_fp,
	display_frequency,
	display_period,

	display_store,
	display_retrieve,

	display_operation,
	display_operation_annul,
	display_operation_vfo_plus,
	display_operation_vfo_minus,
	display_operation_if_minus,

	display_sleep,
	display_sleep_30s,
	display_sleep_5m,
	display_sleep_disabled,

	display_exit_menu

};

enum measurementMode {
	mode_auto,
	mode_band
};

enum measurementBand {
	band_LF,
	band_HF,
	band_VHF1,
	band_VHF2
};

enum measurementResolution {
	resolution_low,
	resolution_normal,
	resolution_high,
	resolution_ultra_high
};

enum measurementDisplayType {
  measure_frequency,
  measure_period
};

enum operationType {
  operation_none,
  operation_vfo_plus,
  operation_vfo_minus,
  operation_if_minus
};

enum sleepMode {
  sleep_30s,
  sleep_5m,
  sleep_disabled
};


enum algorithmType {
	algorithm_freqMeasure,
	algorithm_freqCount
};

/* ************************************************************************************************************************************
  Pandauino_Freq_LF_VHF Class
**************************************************************************************************************************************/

class Pandauino_Freq_LF_VHF {

	// ******* PUBLIC PROPERTIES AND FUNCTIONS *********************************************
  public:

    Pandauino_Freq_LF_VHF();
    Pandauino_Freq_LF_VHF(measurementMode, measurementBand, measurementResolution);

  	static void configureComputation(measurementMode, measurementBand, measurementResolution);
    static void stopComputation();

    static void freqSetup(boardType, bool _outPutToSerial = false, long _bauds = 57600);
    static void freqCount();
    static double getFrequency();
    static double readFrequency();

    static void standbyMode();
    static void beginSerial(long);
    static void endSerial();
    static void calibrate(long);

 		static sleepMode sleepSetting;
   	static double calibration;


	// ******* PRIVATE PROPERTIES AND FUNCTIONS *********************************************
  private:

		// ******* FUNCTIONS
    static void configureComputation(bool restart = false);
		static double measureLF();
		static double measureHF_VHF();

		static void readAllFromEEPROM();
		static void readFromEEPROM_refFrequency();
		static void updateToEEPROM_init();
		static void updateToEEPROM_mode();
		static void updateToEEPROM_band();
		static void updateToEEPROM_resolution();
		static void updateToEEPROM_calibration();
		static void updateToEEPROM_measurementType();
		static void updateToEEPROM_operation();
		static void updateToEEPROM_sleepSetting();
		static void updateToEEPROM_refFrequency();
		static void updateAllToEEPROM();

		static void setSleepTimeout();

 		static void initVcc();
		static float readVcc();
    static void VccTest();
    static bool timeToTestVCC();

    static void sixteenTo8chars (char[], char[], char[] );
    static void printSixteenCharToLCD (char[]);
    static void displayFrequency();
    static void displayPeriod();

		static boolean testFrequencyOutOfRange();
    static void displayMeasurement();
		static void displayCalManValue();
		static void evaluateCalManValue();

		static byte countIntDigits(int);
		static void determineBand(float);

    static void pushButtonInterrupt();
    static void actions();
    static void buttonPress();
    static void buttonClick();
    static void buttonDoubleclick();


		// ******* INSTANCES
		static LiquidCrystal lcd;
    static OneButton button;

		//*********** Hardware related CONST
		static const int LCDD7;
		static const int LCDD6;
		static const int LCDD5;
		static const int LCDD4;
		static const int LCDENABLE;
		static const int LCDRS;

		static const byte select1;
		static const byte select2;

    static const byte PUSHBUTTON;
    static const byte VccReg65EnablePin;
    static const byte lcdLedPowerPin;

		static const byte coefVHF1;
		static const byte coefVHF2;

		//*********** Software related CONST
		static const double freqLFmin;
		static const double freqLFmax;
		static const double freqHFmin;
		static const double freqHFmax_hf_board;
		static const double freqHFmax_vhf_board;
		static const double freqVHF1min;
		static const double freqVHF1max;
		static const double freqVHF2min;

		static const char initMessage[17];
		static const char msgVoltageTooLow[17];
		static const char msgVoltageTooHigh[17];
		static const char calNotPrecise[17];
		static const char calibrating[17];
		static const char noMeasureAvailable[17];
		static const char goingStandby[17];
		static const char frequencySaved[17];
		static const char frequencyOutOfRange[17];
		static const char menuEntries[33][17];

  	static const byte EEPROMbaseAddress;
		static const byte eepromInit;

    static const float HFMeasurePeriodNormalRes;
    static const unsigned int LFTimeoutNormalRes ;
    static const unsigned int  displayTimeLap;

    static const float VccDivider ;
    static const float vref ;
    static const float coefVcc ;
    static const unsigned int VccTestPeriod ;
   static const unsigned int  vccDelayPeriod ;

    static const float underVoltage;
    static const float overVoltage ;
    static const float hysteresisVccPerCent ;

		static const float minCalibManValue;
		static const float maxCalibManValue;

    static const unsigned int clickticks;

		// ******* PROPERTIES

		static boardType boardVersion;
		static runMode editMode;

		static measurementMode mode;
		static measurementBand band;
		static measurementResolution resolution;
		static measurementDisplayType measurementType;
    static byte displayPrecision;
		static float measurementTimeCoefficient;
		static algorithmType algorithm;
		static float prescalerCoef;
		static float effectiveHFMeasurePeriod;
		static int LFTimeout;

		static measurementMode previousMode;
		static measurementBand previousBand;
		static measurementResolution previousResolution;
		static double previousCalibration;
		static measurementDisplayType previousMeasurementType;
		static operationType previousOperation;
		static sleepMode previousSleepSetting;

    static unsigned long countLF;
    static unsigned long countHF;
    static unsigned long measureStamp;
    static unsigned long displayStamp;

    static double sumDisplayFreq;
    static long nbAvgDisplayFreq;

    static double frequency;
    static double frequencyTestVHF2;
    static double frequencyTestVHF1;
    static double frequencyTestHF;
    static double frequencyTest;

		static  operationType operation;
    static double lastValidFrequency;
    static double refFrequency;
    static double resultFrequency;

    static bool outputToSerial;
    static long bauds ;

		static float calManValue;

    static float underVoltageMinusHysteresis ;
    static float overVoltagePlusHysteresis ;
    static bool voltageError ;
    static unsigned long lastVccMillis;

	  static unsigned long sleepTimeout;

		static byte addressOfMode;
		static byte addressOfBand;
		static byte addressOfResolution;
		static byte addressOfCalibration;
		static byte addressOfMeasurementType;
		static byte addressOfOperation;
		static byte addressOfSleepSetting;
		static byte addressOfRefFrequency;

    static char text1[17] ;
    static String text ;
    static char line1[17];

};

extern Pandauino_Freq_LF_VHF frequencyCounter;

#endif
