/*
  Semonlib.h - Library for openenergymonitor
  Created by Trystan Lea, April 27 2010
  GNU GPL
  Modified for Spark Core Feb 10 2014
 * 1) changed boolean variable type to bool
 * 2) changed timeout to unsigned int to avoid warning
 * 3) changed double to float to save RAM
 * 4) added 3 new public variables: numberOfSamples, Vwaveform[] and Iwaveform[]
 *
*/

#ifndef semonlib_h
#define semonlib_h

class EnergyMonitor
{
  public:

    void voltage(int _inPinV, float _VCAL, float _PHASECAL);
    void current(int _inPinI, float _ICAL);

    void voltageTX(float _VCAL, float _PHASECAL);
    void currentTX(int _channel, float _ICAL);

    void calcVI(int crossings, unsigned int timeout);
    float calcIrms(int NUMBER_OF_SAMPLES);
    void serialprint();

    long readVcc();
    //Useful value variables
    float realPower,
       apparentPower,
       powerFactor,
       Vrms,
       Irms;
    int numberOfSamples; // SPARK make public to check conversion rate
    char Vwaveform[128]; // SPARK try to get size up to 128 by economising on RAM
    char Iwaveform[128]; // SPARK new arrays to hold waveform use char to save RAM
  private:

    //Set Voltage and current input pins
    int inPinV;
    int inPinI;
    //Calibration coefficients
    //These need to be set in order to obtain accurate results
    float VCAL;
    float ICAL;
    float PHASECAL;

    //--------------------------------------------------------------------------------------
    // Variable declaration for emon_calc procedure
    //--------------------------------------------------------------------------------------
	unsigned int lastSampleV,sampleV;   //sample_ holds the raw analog read value, lastSample_ holds the last sample
	unsigned int lastSampleI,sampleI;   //SPARK make unsigned for bitwise operation

	float lastFilteredV,filteredV;                   //Filtered_ is the raw analog value minus the DC offset
	float lastFilteredI, filteredI;

	float phaseShiftedV;                             //Holds the calibrated phase shifted voltage.

	float sqV,sumV,sqI,sumI,instP,sumP;              //sq = squared, sum = Sum, inst = instantaneous

	unsigned int startV;                                       //Instantaneous voltage at start of sample window.

	bool lastVCross, checkVCross;                  //Used to measure number of times threshold is crossed.
	int crossCount;                                   // ''


};

#endif