/*
  Spark port of emonlib - Library for the open energy monitor
  Original Created by Trystan Lea, April 27 2010
  
  GNU GPL
  Modified to suit Spark Core 10 Feb 2014 Peter Cowley
  ***********************
  Changes made:
  1) ADC range changed from 1023 to 4095
  2) long EnergyMonitor::readVcc() deleted and calls replaced by 3300
     for my Spark actual V = 1.0004 x measured ADC reading
     averaged over 8 resistor divider values - pretty accurate.
  3) Removed references to Arduino.h and similar
  4) Changed references to zero v near 500 to zero v near 2000 (mid ADC range)
  5) Changed variable 'timeout' type to unsigned int to avoid warning
  6) Spark samples much faster so the lag between V and I readings is small
     so set the phase correction close to 1
  7) Put in 250uS delay between pairs of ADC reads to allow Arduino style phase
     correction. Each pair is now collected every 300uS
  8) crossCount is measured using filtered signal and only +ve crossings
     This gives consistent plots of the waveform.
  9) Unused functions are now deleted rather than commented out
       EnergyMonitor::voltageTX
       EnergyMonitor::currentTX
       readVcc
     
 
 NOTE more recent versions of emonlib include some of these changes
      to accommodate 12bit ADCs on newer Arduino models. 
 * ADDED - make noOfSamples and crossCount are made public for diagnostics
 *         add char arrays Vwaveform and I waveform to log waveform
 *         size of these is determined by available RAM
 *         scale to fit in 8 bit char array (V/16, I/8)
*/


#include "semonlib.h"
#include "application.h"
#include "math.h"

//--------------------------------------------------------------------------------------
// Sets the pins to be used for voltage and current sensors and the
// calibration factors which are set in setup() in the main program
// For 1v per 30a SCT-013-030 ICAL is 30
// For 9v ac power with 10:1 divider VCAL is 250
// For Spark the theoretical PHASECAL is 1.12
//--------------------------------------------------------------------------------------
void EnergyMonitor::voltage(int _inPinV, float _VCAL, float _PHASECAL)
{
   inPinV = _inPinV;
   VCAL = _VCAL;
   PHASECAL = _PHASECAL;
}

void EnergyMonitor::current(int _inPinI, float _ICAL)
{
   inPinI = _inPinI;
   ICAL = _ICAL;
}

//--------------------------------------------------------------------------------------
// emon_calc procedure
// Calculates realPower,apparentPower,powerFactor,Vrms,Irms,kwh increment
// From a sample window of the mains AC voltage and current.
// The Sample window length is defined by the number of half wavelengths or crossings we choose to measure.
// Typically call this with 20 crossings and 2000mS timeout
// SPARK replace int SUPPLYVOLTAGE = readVcc(); with = 3300;
// SPARK count +ve crossings by filteredV keep 20 for 20 cycles
// SPARK timeout of 2000 has caused Spark problems with comms so reduce to 1600
// probably not a problem with recent software - not checked as timeout
// is not reached.
//--------------------------------------------------------------------------------------
void EnergyMonitor::calcVI(int crossings, unsigned int timeout)
{
  int SUPPLYVOLTAGE = 3300;                       //Get supply voltage
  crossCount = 0;                                 //SPARK now a global variable
  numberOfSamples = 0;                            //SPARK now a global variable
  //-------------------------------------------------------------------------------------------------------------------------
  // 1) Waits for the waveform to be close to 'zero'
  // SPARK 'zero' on sin curve is 2048 on ADC
  // SPARK there is sufficient delay time in the loop for ADC to settle
  //-------------------------------------------------------------------------------------------------------------------------
  boolean st=false;                  //an indicator to exit the while loop

  unsigned long start = millis();    //millis()-start makes sure it doesnt get stuck in the loop if there is an error.
  // wait for a reading close to zero volts before updating filtered values
  while(st==false)                                       //the while loop...
  {
     startV = analogRead(inPinV);                        //using the voltage waveform
     if ((startV < 2078 ) && (startV > 2018)) st=true;   //check its within range
     if ((millis()-start)>timeout) st = true;            //with 50uS delay ADC changes 15 units per sample at 0V
  }
  //SPARK now we're close to zero start updating filtered values and wait for
  //a +ve zero crossing
  while (st ==true){
    lastSampleV=sampleV;                          //Used for digital high pass filter
    lastSampleI=sampleI;                          //Used for digital high pass filter
    
    lastFilteredV = filteredV;                    //Used for offset removal
    lastFilteredI = filteredI;                    //Used for offset removal   
    
    sampleV = analogRead(inPinV);          //Read in raw voltage signal
    sampleI = analogRead(inPinI);          //Read in raw current signal
    delayMicroseconds(250);                //SPARK this delay spaces samples to allow phase correction
    
    filteredV = 0.996*(lastFilteredV+sampleV-lastSampleV);
    filteredI = 0.996*(lastFilteredI+sampleI-lastSampleI);
    
    if((filteredV>0)&&(lastFilteredV<0)) st = false;//SPARK always start on upward transition
 }

  //-------------------------------------------------------------------------------------------------------------------------
  // 2) Main measurement loop
  // SPARK V and I are measured very close together so little or no  
  // phase correction is needed for sample lag (9v transformer is another matter)
  //------------------------------------------------------------------------------------------------------------------------- 
  start = millis(); 

  while ((crossCount < crossings) && ((millis()-start)<timeout)) 
  {
    numberOfSamples++;
    
    lastSampleV=sampleV;                          //Used for digital high pass filter
    lastSampleI=sampleI;                          //Used for digital high pass filter
    
    lastFilteredV = filteredV;                    //Used for offset removal
    lastFilteredI = filteredI;                    //Used for offset removal   
    
    //-----------------------------------------------------------------------------
    // A) Read in raw voltage and current samples
    // 
    //-----------------------------------------------------------------------------
    sampleV = analogRead(inPinV);          //Read in raw voltage signal
    sampleI = analogRead(inPinI);          //Read in raw current signal
    delayMicroseconds(250);                //SPARK this delay spaces samples to allow phase correction
    
    //-----------------------------------------------------------------------------
    // B) Apply digital high pass filters to remove 1.65V DC offset (centered on 0V).
    // SPARK grab the waveform data using [numberOfSamples%128] means that we 
    // end up with the last 128 values sampled in the arrays.
    //-----------------------------------------------------------------------------
    filteredV = 0.996*(lastFilteredV+sampleV-lastSampleV);
    filteredI = 0.996*(lastFilteredI+sampleI-lastSampleI);
    Vwaveform[numberOfSamples%128]=char((filteredV+2048)/16);//SPARK save waveform
    Iwaveform[numberOfSamples%128]=char((filteredI+1024)/8); //SPARK save waveform

    //-----------------------------------------------------------------------------
    // C) Root-mean-square method voltage
    //-----------------------------------------------------------------------------  
    sqV= filteredV * filteredV;                 //1) square voltage values
    sumV += sqV;                                //2) sum
    
    //-----------------------------------------------------------------------------
    // D) Root-mean-square method current
    //-----------------------------------------------------------------------------   
    sqI = filteredI * filteredI;                //1) square current values
    sumI += sqI;                                //2) sum 
    
    //-----------------------------------------------------------------------------
    // E) Phase calibration
    // SPARK theoretical shift is 1.12 but current clamp/transformer  
    // difference may swamp this
    //-----------------------------------------------------------------------------
    phaseShiftedV = lastFilteredV + PHASECAL * (filteredV - lastFilteredV); 
    
    //-----------------------------------------------------------------------------
    // F) Instantaneous power calc
    //-----------------------------------------------------------------------------   
    instP = phaseShiftedV * filteredI;          //Instantaneous Power
    sumP +=instP;                               //Sum  
    
    //-----------------------------------------------------------------------------
    // G) Find the number of times the voltage has crossed the initial voltage
    //    - every 2 crosses we will have sampled 1 wavelength 
    //    - so this method allows us to sample an integer number of half 
    // wavelengths which increases accuracy
    // SPARK simplify and improve accuracy by using filtered values
    //-----------------------------------------------------------------------------       
    if((filteredV>0)&&(lastFilteredV<0)) crossCount++;//SPARK always ends on upward transition
  } //closing brace for counting crossings
  
  //-------------------------------------------------------------------------------------------------------------------------
  // 3) Post loop calculations
  // SPARK replace 1024 for Arduino 10bit ADC with 4096 in voltage calculation
  //    VCAL shouldn't change much from Arduino value as SUPPLYVOLTAGE looks
  //    after the 5v to 3.3v change
  //------------------------------------------------------------------------------------------------------------------------- 
  //Calculation of the root of the mean of the voltage and current squared (rms)
  //Calibration coeficients applied. 
  
  float V_RATIO = VCAL *((SUPPLYVOLTAGE/1000.0) / 4096.0);
  Vrms = V_RATIO * sqrt(sumV / numberOfSamples); 
  
  float I_RATIO = ICAL *((SUPPLYVOLTAGE/1000.0) / 4096.0);
  Irms = I_RATIO * sqrt(sumI / numberOfSamples); 

  //Calculation power values
  realPower = V_RATIO * I_RATIO * sumP / numberOfSamples;
  apparentPower = Vrms * Irms;
  powerFactor=realPower / apparentPower;

  //Reset accumulators
  sumV = 0;
  sumI = 0;
  sumP = 0;
  }
//--------------------------------------------------------------------------------------
// SPARK replace int SUPPLYVOLTAGE = readVcc(); with = 3300;
// note that SUPPLYVOLTAGE is redefined here
// 
//--------------------------------------------------------------------------------------
// 
float EnergyMonitor::calcIrms(int NUMBER_OF_SAMPLES)
{
  
  int SUPPLYVOLTAGE = 3300;                        //SPARK delete readVcc();
  
  for (int n = 0; n < NUMBER_OF_SAMPLES; n++)
  {
    lastSampleI = sampleI;
    sampleI = analogRead(inPinI);
    delayMicroseconds(250);                //SPARK this delay spaces samples to allow phase correction
    lastFilteredI = filteredI;
    filteredI = 0.996*(lastFilteredI+sampleI-lastSampleI);

    // Root-mean-square method current
    // 1) square current values
    sqI = filteredI * filteredI;
    // 2) sum 
    sumI += sqI;
  }

  float I_RATIO = ICAL *((SUPPLYVOLTAGE/1000.0) / 4096.0);
  Irms = I_RATIO * sqrt(sumI / NUMBER_OF_SAMPLES); 

  //Reset accumulators
  sumI = 0;
//--------------------------------------------------------------------------------------       
 
  return Irms;
}

void EnergyMonitor::serialprint()
{
    Serial.print(realPower);
    Serial.print(' ');
    Serial.print(apparentPower);
    Serial.print(' ');
    Serial.print(Vrms);
    Serial.print(' ');
    Serial.print(Irms);
    Serial.print(' ');
    Serial.print(powerFactor);
    Serial.println(' ');
    delay(100); 
}