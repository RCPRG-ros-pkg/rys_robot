/*
 * hbridge.hh
 *
 *  Created on: Jun 9, 2011
 *      Author: dseredyn
 */

#ifndef HBRIDGE_HH_
#define HBRIDGE_HH_

#include "common.hpp"

class HBridge
{
public:
        HBridge();
        ~HBridge();

        void setCurrentPID(int Kp, int Ki, int Kd);
        void setSpeedPID(int Kp, int Ki, int Kd);
        void deserialize(double time, const MSG &msg);
        MSG serialize();

        void setSpeed(double vel);
        void setCurrent(double cur);

        void Run();
        void Stop();

        bool isRunning();
        bool isValid(double time);
        double getPosition();
        double getPositionDifference();
        void setMaxPower(int maxPower);
        void setAddress(int address);
        
        double currentMeasured;
        double currentGiven;
        double speedMeasured;
        double speedGiven;
        
private:
        double vel, cur;
        bool running;
        int curKp, curKi, curKd;
        int velKp, velKi, velKd;
        int posB, encB, updated;
        int oldPosB;
        unsigned char maxPower;
        int address;
};

#endif // HBRIDGE_HH_
