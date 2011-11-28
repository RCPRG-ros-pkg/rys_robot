/*
 * hbridge.cc
 *
 *  Created on: Jun 9, 2011
 *      Author: dseredyn
 */

#include <math.h>
#include "hbridge.hpp"

HBridge::HBridge() : vel(0), cur(0), running(false), curKp(0),
curKi(0), curKd(0), velKp(0), velKi(0), velKd(0), posB(0), oldPosB(0), encB(0),
updated(0), maxPower(0), address(1), posA(0), oldPosA(0), encA(0) {
}

HBridge::~HBridge(){
}

void HBridge::setCurrentPID(int Kp, int Ki, int Kd){
    curKp = Kp;
    curKi = Ki;
    curKd = Kd;
}

void HBridge::setSpeedPID(int Kp, int Ki, int Kd){
    velKp = Kp;
    velKi = Ki;
    velKd = Kd;
}

void HBridge::deserialize(double time, const MSG &msg){
/*
    msg.d[0];   // local time low byte
    msg.d[1];   // local time high byte
    msg.d[2];   // zmierzona szybkosc
    msg.d[3];   // zadana szybkosc
    msg.d[4];   // zmierzony prad
    msg.d[5];   // zadany prad
    msg.d[6];   // safe mode

    msg.d[7];   // encoder A low
    msg.d[8];   // encoder A high
    msg.d[9];   // encoder B low
    msg.d[10];  // encoder B high
    msg.d[11];  // 0
*/
// encA
    int distanceA = msg.d[7] | ((int)msg.d[8]<<8);
    if (encA > 50000 && distanceA < 10000) {            // przekrecilo sie w gore
            encA -= 0x10000;
    } else if (encA < 10000 && distanceA > 50000) {     // przekrecilo sie w dol
            encA += 0x10000;
    }
    
    if (encA - distanceA > -200 && encA - distanceA < 200)
        posA += encA - distanceA;
    encA = distanceA;
    updated = time;
    
//encB
    int distanceB = (int)msg.d[9] | ((int)msg.d[10]<<8);
    if (encB > 50000 && distanceB < 10000) {            // przekrecilo sie w gore
            encB -= 0x10000;
    } else if (encB < 10000 && distanceB > 50000) {     // przekrecilo sie w dol
            encB += 0x10000;
    }
    
    if (encB - distanceB > -100 && encB - distanceB < 100)
        posB += encB - distanceB;
    encB = distanceB;
    updated = time;
    
    currentGiven = msg.d[5];
    if (cur < 0)
        currentMeasured = -msg.d[4];
    else
        currentMeasured = msg.d[4];
        
    speedGiven = msg.d[3];
    speedMeasured = ((char)msg.d[2]);
//    speedMeasured = 
}

MSG HBridge::serialize(){
    MSG msg;

    msg.adr = address;
    msg.header = MSG_HEADER;
    msg.id = 1;

    if (running) {
        msg.d[0] = fabs(cur*255);   // current
        msg.d[8] = vel*60;          // speed
        if (cur<0)
            msg.d[1] = 0;               // direction
        else
            msg.d[1] = 1;
    } else {
        msg.d[0] = 0;       // current
        msg.d[8] = 0;       // speed
        msg.d[1] = 0;       // direction
    }

    msg.d[2] = curKp;   // P (current)
    msg.d[3] = curKi;   // I (current)
    msg.d[4] = curKd;   // D (current)

    msg.d[5] = velKp;   // P (speed)
    msg.d[6] = velKi;   // I (speed)
    msg.d[7] = velKd;   // D (speed)

    msg.d[9] = maxPower;
    msg.d[10] = 0;
    msg.d[11] = 0;

    return msg;
}

void HBridge::setSpeed(double vel){
    cur = 0;
    if (vel > 1)
        vel = 1;
    if (vel < -1)
        vel = -1;

    this->vel = vel;
}

void HBridge::setCurrent(double cur){
    vel = 0;
    if (cur > 1)
        cur = 1;
    if (cur < -1)
        cur = -1;
        
    if ((cur > -0.001 && cur < 0.001)) {
            if (this->cur < 0)
                    this->cur = -0.001;
            else
                    this->cur = 0.001;
    } else
            this->cur = cur;
}

void HBridge::Run(){
    running = true;
}

void HBridge::Stop(){
    running = false;
}

bool HBridge::isRunning() {
    return running;
}

bool HBridge::isValid(double time) {
        return (time-updated)<0.1;
}

double HBridge::getPosition() {
        return posB;
}

double HBridge::getPositionA() {
        return posA;
}

double HBridge::getPositionDifference() {
        double result = posB - oldPosB;
        oldPosB = posB;
        return result;
}

double HBridge::getPositionDifferenceA() {
        double result = posA - oldPosA;
        oldPosA = posA;
        return result;
}

void HBridge::setMaxPower(int maxPower) {
    if (maxPower<0)
        maxPower = 0;
    if (maxPower>255)
        maxPower = 255;
    this->maxPower = maxPower;
}

void HBridge::setAddress(int address) {
    this->address = address;
}
