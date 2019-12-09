#include "mbed.h"
#include "math.h"
#include "complex.h"
#include "millis.h"
#include "MAX30105.h"
#include "heartRate.h"

I2C *i2c = new I2C(I2C_SDA, I2C_SCL);
Serial pc(USBTX, USBRX);    //initializes the serial port


complex a(0.2,0.2);
#ifdef TARGET_KL25Z
PwmOut led(PTB18);  //initializes the pwm output that connects to the on board LED
DigitalIn INT(PTD1);  //pin PTD1 connects to the interrupt output pin of the MAX30102
#endif
#ifdef TARGET_K64F
DigitalIn INT(PTD1);  //pin PTD1 connects to the interrupt output pin of the MAX30102
#endif
#ifdef TARGET_MAX32600MBED
PwmOut led(LED_RED);    //initializes the pwm output that connects to the on board LED
DigitalIn INT(P2_0);  //pin P20 connects to the interrupt output pin of the MAX30102
#endif

//PwmOut led(PA_7);
DigitalIn INT(PA_5);

int main()
{
    pc.baud(115200);
    pc.format(8,SerialBase::None,1);
    //pc.printf("init...\n");
    millisStart();
    
    MAX30105 sensor(i2c);
    sensor.setup();
    sensor.clearFIFO();
    
    int i = 0, j = 0;
    long Mstart = 0;
    uint32_t max, min, samples[500];    
    Mstart = millis();
    sensor.getIR();
    
    while(true) {
        /*
        if(!print){
            if(millis() > Mstart + 1000)
                print = true;
            pc.printf("%ld , fs : %d\n", sensor.getIR(), i);
            
            i++;   
        }else{
        */
        
            samples[i] = sensor.getIR();
            
            if(samples[i] < min)
                min = samples[i];
            if(samples[i] > max)
                max = samples[i];
            i++;
            if(millis() > Mstart + 10000){
                sensor.softReset();
                break;
            }
        //}
    }
    
    while(j < i){
        pc.printf("%d;%ld\n", j, samples[i] - ((max + min) / 2));
        j++;
    }
    return 0;
}