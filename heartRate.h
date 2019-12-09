#include "mbed.h"
#include "MAX30105.h"

#ifndef HEARTRATE_H_
#define HEARTRATE_H_


class heartRate {
public:
    bool checkForBeat(int32_t sample);
    int16_t averageDCEstimator(int32_t *p, uint16_t x);
    int16_t lowPassFIRFilter(int16_t din);
    int32_t mul16(int16_t x, int16_t y);
    bool MeasureHeartRate(MAX30105 *sensor);
    int getRate();
private:
    int rate;    

};
 
#endif