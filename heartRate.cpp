#include "heartRate.h"
#include "millis.h"

Serial dbg(USBTX, USBRX);
    int16_t IR_AC_Max = 20;
    int16_t IR_AC_Min = -20;
    
    int16_t IR_AC_Signal_Current = 0;
    int16_t IR_AC_Signal_Previous;
    int16_t IR_AC_Signal_min = 0;
    int16_t IR_AC_Signal_max = 0;
    int16_t IR_Average_Estimated;

    int16_t positiveEdge = 0;
    int16_t negativeEdge = 0;
    int32_t ir_avg_reg = 0;
    
    int16_t cbuf[32];
    uint8_t offset = 0;
    static const uint16_t FIRCoeffs[12] = {172, 321, 579, 927, 1360, 1858, 2390, 2916, 3391, 3768, 4012, 4096};
/******************************************************************************/
bool heartRate::MeasureHeartRate(MAX30105 *sensor) {
    
    long irValue;
    long startMillis = millis();
    long delta;
    long lastBeat = 0; //Time at which the last beat occurred
    long buffer[2] = {0, 0};
    uint8_t idx = 0;
    
    sensor->softReset();
    sensor->setup();
    sensor->setPulseAmplitudeIR(0x30);
    sensor->setPulseAmplitudeRed(0x30);
    sensor->clearFIFO();

    while(1) {
        irValue = sensor->getIR();
        
        if (checkForBeat(irValue) == true) {
            //We sensed a beat!
            delta = millis() - lastBeat;
            lastBeat = millis();
    
            if((delta < 1500) && (delta > 250)){
                buffer[idx] = delta;
                idx++;
            } else {
                idx = 0;
            }
    
            if(idx == 2) {
                rate = (int)(60.0/((float)((buffer[0]+buffer[1])/2.0)/1000.0));
                sensor->setPulseAmplitudeIR(0x00);
                sensor->setPulseAmplitudeRed(0x00);
                return true;
            }
        }
        if (millis() > startMillis + 10000) {
            sensor->setPulseAmplitudeIR(0x00);
            sensor->setPulseAmplitudeRed(0x00);
            return false;
        }
    }
}    
/******************************************************************************/
int heartRate::getRate() {
    return rate;
}
/******************************************************************************/
bool heartRate::checkForBeat(int32_t sample)
{
  bool beatDetected = false;

  //  Save current state
  IR_AC_Signal_Previous = IR_AC_Signal_Current;
  
  //This is good to view for debugging
  //dbg.printf("\n\nSignal_Current: %d \n\n",IR_AC_Signal_Current);
 

  //  Process next data sample
  IR_Average_Estimated = averageDCEstimator(&ir_avg_reg, sample);
  IR_AC_Signal_Current = lowPassFIRFilter(sample - IR_Average_Estimated);

  //  Detect positive zero crossing (rising edge)
  if ((IR_AC_Signal_Previous < 0) & (IR_AC_Signal_Current >= 0))
  {
  
    IR_AC_Max = IR_AC_Signal_max; //Adjust our AC max and min
    IR_AC_Min = IR_AC_Signal_min;

    positiveEdge = 1;
    negativeEdge = 0;
    IR_AC_Signal_max = 0;
    //if ((IR_AC_Max - IR_AC_Min) > 100 & (IR_AC_Max - IR_AC_Min) < 1000)
    if ((IR_AC_Max - IR_AC_Min) > 40 & (IR_AC_Max - IR_AC_Min) < 2300)
    {
      //Heart beat!!!
      beatDetected = true;
    }
  }

  //  Detect negative zero crossing (falling edge)
  if ((IR_AC_Signal_Previous > 0) & (IR_AC_Signal_Current <= 0))
  {
    positiveEdge = 0;
    negativeEdge = 1;
    IR_AC_Signal_min = 0;
  }

  //  Find Maximum value in positive cycle
  if (positiveEdge & (IR_AC_Signal_Current > IR_AC_Signal_Previous))
  {
    IR_AC_Signal_max = IR_AC_Signal_Current;
  }

  //  Find Minimum value in negative cycle
  if (negativeEdge & (IR_AC_Signal_Current < IR_AC_Signal_Previous))
  {
    IR_AC_Signal_min = IR_AC_Signal_Current;
  }
  
  return(beatDetected);
}

//  Average DC Estimator
int16_t heartRate::averageDCEstimator(int32_t *p, uint16_t x)
{
  *p += ((((long) x << 15) - *p) >> 4);
  return (*p >> 15);
}

//  Low Pass FIR Filter
int16_t heartRate::lowPassFIRFilter(int16_t din)
{  
  cbuf[offset] = din;

  int32_t z = mul16(FIRCoeffs[11], cbuf[(offset - 11) & 0x1F]);
  
  for (uint8_t i = 0 ; i < 11 ; i++)
  {
    z += mul16(FIRCoeffs[i], cbuf[(offset - i) & 0x1F] + cbuf[(offset - 22 + i) & 0x1F]);
  }

  offset++;
  offset %= 32; //Wrap condition

  return(z >> 15);
}

//  Integer multiplier
int32_t heartRate::mul16(int16_t x, int16_t y)
{
  return((long)x * (long)y);
}