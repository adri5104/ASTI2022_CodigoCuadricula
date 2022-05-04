#ifndef TOFSENSORS_
#define TOFSENSORS_

#include <Adafruit_VL53L0X.h>
#include "Config.h"
#include <Arduino.h>
#include "PCF8575.h"
class TofSensors
{
    private:
        Adafruit_VL53L0X* myTofSensors[3];
        int myXSHUTpins [3];
        VL53L0X_RangingMeasurementData_t myMeasurements[3];
        int distance[3];
        int* measureDistance();
        PCF8575* pinExtensor_;
        
    public:
        //TofSensors();
        TofSensors(int, int, int, PCF8575*);
        void init();
        //uint16_t getDistanceMM(int);
        void printMeasurements();
        void setID(uint8_t, uint8_t, uint8_t);
        int getDistance(int);

};
#endif