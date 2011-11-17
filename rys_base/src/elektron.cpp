/*
 * elektron.cpp
 *
 *  Created on: Sep 5, 2009
 *      Author: konradb3
 */

#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <ros/console.h>
#include <cstring>
#include <iostream>
#include <errno.h>

#include "elektron.hpp"

using namespace std;

double ang_nor_rad(double rad) {
        static double TWO_PI = 2.0 * M_PI;
        for (;;) {
                if (rad >= M_PI)
                        rad -= TWO_PI;
                else if (rad <= -M_PI)
                        rad += TWO_PI;
                else
                return (rad);
        }
}

Protonek::Protonek(const std::string& port, const Protonek::Parameters &parL,
                    const Protonek::Parameters &parR,  int baud)
                    : isBalancing(false), speedRegulator(true), ldif(0), rdif(0),
                    state(STOPPING) {

        bridgeL.setCurrentPID(parL.currentKp, parL.currentKi, parL.currentKd);
        bridgeL.setSpeedPID(parL.speedKp, parL.speedKi, parL.speedKd);
        bridgeL.setMaxPower(parL.maxCurrent);
        bridgeL.setAddress(2);

        bridgeR.setCurrentPID(parR.currentKp, parR.currentKi, parR.currentKd);
        bridgeR.setSpeedPID(parR.speedKp, parR.speedKi, parR.speedKd);
        bridgeR.setMaxPower(parR.maxCurrent);
        bridgeR.setAddress(1);

        connected = false;

        xpos = 0;
        ypos = 0;
        apos = 0;

        robot_axle_length = AXLE_LENGTH;
        m_per_tick = M_PI * WHEEL_DIAM / ENC_TICKS;
        enc_ticks = ENC_TICKS;

        fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd==-1) {
            printf("nie mozna otworzyc portu: %s errno:%d\n",port.c_str(), (int)errno);
            throw;
        }

        tcgetattr(fd, &oldtio);

        // set up new settings
        struct termios newtio;
        memset(&newtio, 0, sizeof(newtio));
        newtio.c_cflag = CBAUD | CS8 | CLOCAL | CREAD | CSTOPB;
        newtio.c_iflag = IGNPAR;
        newtio.c_oflag = 0;
        newtio.c_lflag = 0;
        if (cfsetispeed(&newtio, baud) < 0 || cfsetospeed(&newtio, baud) < 0) {
            fprintf(stderr, "Failed to set serial baud rate: %d\n", baud);
            tcsetattr(fd, TCSANOW, &oldtio);
            close(fd);
            fd = -1;
        }
        // activate new settings
        tcflush(fd, TCIFLUSH);
        tcsetattr(fd, TCSANOW, &newtio);

        connected = true;
}

Protonek::~Protonek() {
        // restore old port settings
        if (fd > 0)
        tcsetattr(fd, TCSANOW, &oldtio);
        close(fd);
}

void Protonek::enableSpeedRegulator() {
        speedRegulator = true;
}

void Protonek::disableSpeedRegulator() {
        speedRegulator = false;
}

void Protonek::runMotors()
{
        bridgeL.Run();
        bridgeR.Run();
}

void Protonek::stopMotors()
{
        bridgeL.Stop();
        bridgeR.Stop();
}

double Protonek::getInterval()
{
        static bool validTime=false;

        static struct timeval tv_old;

        struct timeval tv;
        gettimeofday(&tv,0);

        double result=0;
        if (validTime)
        {
                result = ((double)tv.tv_sec - (double)tv_old.tv_sec) + ((double)tv.tv_usec - (double)tv_old.tv_usec)/1000000.0;
        } else
        {
                validTime=true;
        }
        tv_old = tv;
        return result;
}
/*
void Protonek::Balance2(double time, bool reset) {

        const double Kp = 0.020 * 1.7;//2;
        const double Ki = 0.0050 * 0.0;
        const double Kd = 0.1 * 0.0;
        double A = Kp + Ki + Kd;
        double B = -(Kp + 2 * Kd);
        double C =  Kd;
        static double sERRprev2 = 0;
        static double sERRprev = 0;
        static double sOUTprev = 0;
        static double vel = 0;

        if (!orientation.isValid(time)) {
//                ROS_DEBUG("no gyro/acc data!");
//                state = STOPPING;
//                return;
        }
        
        if (reset) {
            sERRprev2 = 0;
            sERRprev = 0;
            sOUTprev = 0;
            vel = 0;
        }

        double addedAngle = joystick.speedLinear;

        double addedAngleMul = (bridgeL.speedMeasured-bridgeR.speedMeasured)/500;
        if (addedAngleMul > 1)
                addedAngleMul = 1;
        if (addedAngleMul < 0)
                addedAngleMul = 0;

//        addedAngle *= 1.0 - addedAngleMul;

        // regulator PID
        double sERRt = orientation.pitchGyro-(balanceAngle + 8*(addedAngle));

//        printf("%lf\n",sERRt);
//        if (sERRt<0)
//            sERRt = -sERRt*sERRt / 20;
//        else
//            sERRt = sERRt*sERRt / 20;

        double sOUTt = sOUTprev + ((sERRt * A + sERRprev * B + sERRprev2 * C)/2);

        if (sOUTt<0.8 && sOUTt>-0.8)    // ograniczenie calkowania
        {
                sERRprev2 = sERRprev;
                sERRprev = sERRt;
                sOUTprev = sOUTt;
        }

        if (sOUTt<-0.8)
            sOUTt = -0.8;
        if (sOUTt>0.8)
            sOUTt = 0.8;

//        ROS_DEBUG("%lf   %lf", sOUTt, sERRt);

        double reaction = sOUTt;

        if (reaction<-0.8)
                reaction = -0.8;
        if (reaction>0.8)
                reaction = 0.8;

        double output = reaction;//0.9*output + 0.1*reaction;


        vel -= output*0.020*1.7;    // 0.025

        if (vel>0.7)
            vel = 0.7;

        if (vel<-0.7)
            vel = -0.7;

//        if ((orientation.getPitch()-90)>25)
//            vel = 0;

        double vel2 = vel - output;

        double turn =  joystick.speedAngular*0.2;

        if (speedRegulator)
        {
                bridgeR.setSpeed(-vel2 - turn);
                bridgeL.setSpeed(vel2 - turn);
        } else {
//                bridgeR.setSpeed(0);
//                bridgeL.setSpeed(0);
                bridgeL.setCurrent(vel2*0.7 + turn);
                bridgeR.setCurrent(-vel2*0.7 + turn);
        }

        //s.rcur = -(vel2 + turn);
        //s.lcur = (vel2 - turn);
}
*/

void Protonek::setupPIDangularVelocity(double Kp, double Ki, double Kd) {
        avKp = Kp;
        avKi = Ki;
        avKd = Kd;
}

void Protonek::setupPIDlinearVelocity(double Kp, double Ki, double Kd) {
        lvKp = Kp;
        lvKi = Ki;
        lvKd = Kd;
}

void Protonek::setupPIDangle(double Kp, double Ki, double Kd) {
        aKp = Kp;
        aKi = Ki;
        aKd = Kd;
}

double Protonek::angularVelocityPID(double thvel) {
        double A = avKp + avKi + avKd;
        double B = -(avKp + 2 * avKd);
        double C =  avKd;
        static double sERRprev2 = 0;
        static double sERRprev = 0;
        static double sOUTprev = 0;
        static double sOUTt = 0;

        double mean = 0;
        
        if (!getMeanAngularVelocity(mean))
                return sOUTt;

                        
//        mean*=10000;
        ROS_DEBUG("                    ang: %lf   %lf",thvel, mean);

        // regulator PID
        double sERRt = mean - thvel;

        sOUTt = sOUTprev + ((sERRt * A + sERRprev * B + sERRprev2 * C)/2);

        if (sOUTt<0.9 && sOUTt>-0.9)    // ograniczenie calkowania
        {
                sERRprev2 = sERRprev;
                sERRprev = sERRt;
                sOUTprev = sOUTt;
        }

//        ROS_DEBUG("sERR:%lf    sOUT:%lf", sERRt, sOUTt);


//        ROS_DEBUG("%lf    %lf", bridgeL.speedMeasured, bridgeR.speedMeasured);
        return sOUTt;
}

double Protonek::linearVelocityPID(double xvel) {
        double A = lvKp + lvKi + lvKd;
        double B = -(lvKp + 2 * lvKd);
        double C =  lvKd;
        static double sERRprev2 = 0;
        static double sERRprev = 0;
        static double sOUTprev = 0;
        static double sOUTt = 0;

        double mean = 0;
        
        if (!getMeanLinearVelocity(mean))
                return sOUTt;

//        mean*=10000; 
//        ROS_DEBUG("%lf",mean);
        ROS_DEBUG("lin: %lf   %lf",xvel, mean);

        // regulator PID
        double sERRt = mean - xvel;

        sOUTt = sOUTprev + ((sERRt * A + sERRprev * B + sERRprev2 * C)/2);

        if (sOUTt<0.8 && sOUTt>-0.8)    // ograniczenie calkowania
        {
                sERRprev2 = sERRprev;
                sERRprev = sERRt;
                sOUTprev = sOUTt;
        }

//        ROS_DEBUG("sERR:%lf    sOUT:%lf", sERRt, sOUTt);


//        ROS_DEBUG("%lf    %lf", bridgeL.speedMeasured, bridgeR.speedMeasured);
        return sOUTt;
}

void Protonek::Balance3(double time, bool reset) {

        double A = aKp + aKi + aKd;
        double B = -(aKp + 2 * aKd);
        double C =  aKd;
        static double sERRprev2 = 0;
        static double sERRprev = 0;
        static double sOUTprev = 0;
        static double vel = 0;

        if (reset) {
            sERRprev2 = 0;
            sERRprev = 0;
            sOUTprev = 0;
            vel = 0;
        }

//        double addedAngle = linearVelocityPID(joystick.speedLinear*0.001);
//        double turn = angularVelocityPID(joystick.speedAngular*0.1);

        double turn = -angularVelocityPID(joystick.speedAngular*0.05);
        double addedAngle = linearVelocityPID(joystick.speedLinear*0.07);

//        lvel = l + t;
//        rvel = l - t;

//        ROS_DEBUG("a:%lf   t:%lf", 8.0*addedAngle, turn);
        // regulator PID
        double sERRt = orientation.pitchGyro-(balanceAngle + 6*(addedAngle));

        double ffadd = sin(3.1415*sERRt/180.0)*(51.0/255.0);
        
        double sOUTt = sOUTprev + ((sERRt * A + sERRprev * B + sERRprev2 * C)/2);

        if (sOUTt<0.8 && sOUTt>-0.8)    // ograniczenie calkowania
        {
                sERRprev2 = sERRprev;
                sERRprev = sERRt;
                
                sOUTprev = sOUTt;
        }

        vel = -sOUTt-ffadd;

        if (vel>0.5)
            vel = 0.5;

        if (vel<-0.5)
            vel = -0.5;

        bridgeL.setCurrent(vel - turn);
        bridgeR.setCurrent(-vel - turn);
}
// current L: 35 (przy 90st.)
// current R: 35 (przy 90st.)

void Protonek::Send() {
        MSG msg;
        static int device = 1;
        if (device==1)
                device = 2;
        else if (device==2)
                device = 3;
        else
                device = 1;

//device = 3;
        if (device==1)
        {
            msg = bridgeR.serialize();
        } else
        if (device==2)
        {
            msg = bridgeL.serialize();
        } else
        if (device==3)
        {
            msg.adr = 3;
            msg.header = MSG_HEADER;
            msg.id = 1;
        }

        msg.crc = Crc::Crc16Byte((uint8_t*)&msg,sizeof(MSG)-2,0);
        write(fd, &msg, sizeof(MSG));
        tcdrain(fd);
}

void Protonek::Receive(double time) {
        static MSG answer;
        unsigned char *ans_buf = (unsigned char*)&answer;
        static int ans_i=0;
        bool trans_OK=false;

        int bytesRead;
        unsigned char buffer[500];
        unsigned char buf=0;

        bytesRead = read(fd,buffer,500);

        static bool receivedCorrectData = false;

        for (int i_buf=0; i_buf<bytesRead; ++i_buf)
        {
                buf = buffer[i_buf];
                for (int i=0;i<sizeof(MSG)-1;++i)
                        ans_buf[i] = ans_buf[i+1];
                ans_buf[sizeof(MSG)-1] = buf;
                ++ans_i;

                uint16_t crc = Crc::Crc16(answer);

// ramka danych ze sterownika silnikow
// answer.d[0]		local time low byte
// answer.d[1]		local time high byte
// answer.d[2]		zmierzona szybkosc (enc B)
// answer.d[3]		zadana szybkosc
// answer.d[4]		zmierzony prad
// answer.d[5]		zadany prad
// answer.d[6]		safe mode
// answer.d[7]		odom (A) low byte
// answer.d[8]		odom (A) high byte
// answer.d[9]		odom (B) low byte
// answer.d[10]		odom (B) high byte
// answer.d[11]		0


                if (crc==0 && answer.header==MSG_HEADER && answer.adr==1)       // crc
                {
                        int mocZadana = answer.d[5];
                        int szybkoscZadana = answer.d[3];
                        //printf("ok\n");
                        //float speedRatio = (float)((char)answer.d[2])/(float)((char)answer.d[3]);
                        //rspeed = (signed char)(answer.d[2]);// | (answer.d[3]<<8));

                        //rspeed = 100.0f/rspeed;

                        int distance = answer.d[9] | (answer.d[10]<<8);

                        bridgeL.deserialize(time, answer);
                        //position_left.update(distance,time);
//ROS_DEBUG("adr = 1");
    ldif = bridgeL.getPositionDifference()*3.1415*WHEEL_DIAM/ENC_TICKS;

                        //static float smoothSpeed = 0;
                        //smoothSpeed = smoothSpeed*0.9 + speed*0.1;
                        if (0)
                                ROS_DEBUG("%d %dms %d %d %d %.3f / %.3f %d moc: %.3f / %.3f dist: %d",
                                        (int)answer.adr,
                                        (int)0,//(position_left.getInterval()*1000),
                                        ans_i,
                                        (int)answer.d[0],
                                        (int)answer.d[1],
                                        0,//rspeed,
                                        (float)szybkoscZadana,
                                        (int)answer.d[4],
                                        (float)answer.d[5]/23.5,
                                        (float)mocZadana/23.5,
                                        distance);
                        trans_OK = true;
                        receivedCorrectData = true;

                        ans_i = 0;		// czekamy na nastepne bajty
                } else
                if (crc==0 && answer.header==MSG_HEADER && answer.adr==2)	// crc
                {
                        int mocZadana = answer.d[5];
                        int szybkoscZadana = answer.d[3];
                        //printf("ok\n");
                        //float speedRatio = (float)((char)answer.d[2])/(float)((char)answer.d[3]);
                        
                        //lspeed = (signed char)(answer.d[2]);// | (answer.d[3]<<8));

                        //lspeed = 100.0f/lspeed;

                        int distance = answer.d[9] | (answer.d[10]<<8);

                        bridgeR.deserialize(time, answer);
                        //position_right.update(distance,time);
//ROS_DEBUG("adr = 2");
    rdif = -bridgeR.getPositionDifference()*3.1415*WHEEL_DIAM/ENC_TICKS;

                        //static float smoothSpeed = 0;
                        //smoothSpeed = smoothSpeed*0.9 + speed*0.1;
                        if (0)
                                ROS_DEBUG("%d %dms %d %d %d %.3f / %.3f %d\tmoc: %.3f / %.3f dist: %d",
                                        (int)answer.adr,
                                        (int)0,//(position_right.getInterval()*1000),
                                        ans_i,
                                        (int)answer.d[0],
                                        (int)answer.d[1],
                                        0,//lspeed,
                                        (float)szybkoscZadana,
                                        (int)answer.d[4],
                                        (float)answer.d[5]/23.5,
                                        (float)mocZadana/23.5,
                                        distance);
                        trans_OK = true;
                        receivedCorrectData = true;

                        ans_i = 0;		// czekamy na nastepne bajty
                } else
                if (crc==0 && answer.header==MSG_HEADER && answer.adr==3)
                {
                        static double accX=100, accY=100, accZ=100;
                        accX = (answer.d[2] + (answer.d[3]<<8))*0.4 + 0.6*accX;
                        accY = (answer.d[4] + (answer.d[5]<<8))*0.4 + 0.6*accY;
                        accZ = (answer.d[6] + (answer.d[7]<<8))*0.4 + 0.6*accZ;

                        orientation.update(accX, accY, accZ, ((int)answer.d[10]-117), time);

//ROS_DEBUG("adr = 3");

//                      printf("%d   %d   %d   %d\n", (int)answer.d[8], (int)answer.d[9], (int)answer.d[10], (int)answer.d[11]);

                        trans_OK = true;
                        receivedCorrectData = true;

                        ans_i = 0;		// czekamy na nastepne bajty
                } else
                {
                        //ans_i = sizeof(MSG)-1;		// cos sie pomieszalo - moze zgubiono 1 bajt
                }
        }
}

void Protonek::stateTeleop() {

        double lvel, rvel;

        lvel = -joystick.speedLinear - joystick.speedAngular;
        rvel = -joystick.speedLinear + joystick.speedAngular;

        lvel = lvel>1.0?1.0:(lvel<-1.0?-1.0:lvel);
        rvel = rvel>1.0?1.0:(rvel<-1.0?-1.0:rvel);

//*
//        double t = angularVelocityPID(joystick.speedAngular*0.05);
//        double l = -linearVelocityPID(joystick.speedLinear*0.07);

//        lvel = l + t;
//        rvel = l - t;
        
/*/        
        double turn = (joystick.speedAngular);
        double linear = (joystick.speedLinear);
        lvel = linear + turn;
        rvel = linear - turn;

        double angVel = 0;
        double linVel = 0;
        getMeanAngularVelocity(angVel);
        getMeanLinearVelocity(linVel);
        ROS_DEBUG("%lf        %lf", linVel, angVel);
//*/

//        ROS_DEBUG("%lf  %lf        %lf  %lf", joystick.speedLinear, linear, joystick.speedAngular, turn);
        if (joystick.buttonStop) {
                state = STOPPING;
                lvel = 0;
                rvel = 0;
        } else {
                if (!joystick.buttonTurbo) {
                        lvel *= 0.5;
                        rvel *= 0.5;
                }
                
                if (joystick.buttonGetUp) {
                        state = GETTINGUP;
                } else
                if (joystick.buttonTrick) {
                }
                
                if (fabs(orientation.getPitch()-balanceAngle)<35) {
                        state = BALANCING;
                }


        }
        
//        ROS_DEBUG("TELEOP %lf %lf", lvel, rvel);

        if (speedRegulator) {
                bridgeL.setSpeed(lvel);
                bridgeR.setSpeed(-rvel);
        } else {
                bridgeL.setCurrent(lvel*0.7);
                bridgeR.setCurrent(-rvel*0.7);
        }

}

void Protonek::stateBalancing(double time, double interval) {
//        ROS_DEBUG("BALANCING");

        static double verticalTime = 0.0;
        
        if (verticalTime < 0.5) {
                Balance3(time,true);      // reset regulatorow balansowania
//                throw;
        }
        else {
                Balance3(time,false);
        }
        verticalTime += interval;

        if (fabs(orientation.getPitch()-balanceAngle)>35 || joystick.buttonStop) {
                verticalTime = 0.0;
                state = STOPPING;
        }
}

void Protonek::stateStopping(double interval) {
//        ROS_DEBUG("STOPPING");

        static double stoppingTime = 0;
        
        bridgeL.setSpeed(0);
        bridgeR.setSpeed(0);
        bridgeL.setCurrent(0);
        bridgeR.setCurrent(0);

        stopMotors();
                
        stoppingTime += interval;
        if (stoppingTime > 0.5) {
                stoppingTime = 0.0;
                state = TELEOP;
                runMotors();
        }
}

void Protonek::stateGettingUp() {
//        ROS_DEBUG("GETTINGUP");
}

void Protonek::update() {
        static int error = 0;

        // okreslenie jaki czas minal od ostatniego wywolania metody Update
        static double elapsed = 0;
        double interval = getInterval();        // interwal
        elapsed += interval;            // czas co jaki sprawdzamy czy nastapila przerwa w transmisji
        static double time = 0;
        time += interval;               // czas skumulowany

        if (elapsed>0.02) {

//        ROS_DEBUG("%lf  %lf  %lf  %lf", bridgeL.getPositionA(), bridgeR.getPositionA(),bridgeL.getPosition(), bridgeR.getPosition());

        static double lpos=0, rpos=0;

//        lpos += ldif;
//        rpos += rdif;
//        ROS_DEBUG("%lf   %lf", lpos, rpos);

//ROS_DEBUG("%lf   %lf", ldif, rdif);
        if (elapsed > 0.8) {
                ROS_DEBUG("too long: %lf", elapsed);
                //throw;
        }

        if (!orientation.isValid(time)) {
                ROS_DEBUG("no gyro/acc data!");
        }
                switch (state) {
                case TELEOP:
                        stateTeleop();
                        break;
                case BALANCING:
                        stateBalancing(time,interval);
                        break;
                case STOPPING:
                        stateStopping(interval);
                        break;
                case GETTINGUP:
                        stateGettingUp();
                        break;
                }

                Send();
                elapsed = 0;
                
        }

        Receive(time);

}

void Protonek::SetPower(double lcur, double rcur)
{
        if (lcur>1)
                lcur = 1;
        else if (lcur<-1)
                lcur = -1;

        if (rcur>1)
                rcur = 1;
        else if (rcur<-1)
                rcur = -1;
}

void Protonek::trick1()
{
        trick.start();
}

void Protonek::trick2()
{
}

double Protonek::getPitch() {
    return orientation.getPitch();
}

double Protonek::getDPitch() {
    return orientation.getDPitch(0);

}

void Protonek::getVelocity(double &xvel, double &thvel) {
//        double lvel = bridgeL.speedMeasured*WHEEL_DIAM/ENC_TICKS;
//        double rvel = bridgeR.speedMeasured*WHEEL_DIAM/ENC_TICKS;
        double lvel = ldif;//*WHEEL_DIAM/ENC_TICKS;
        double rvel = rdif;//*WHEEL_DIAM/ENC_TICKS;

        xvel = (lvel + rvel) * 0.5;
        thvel = (lvel - rvel) / AXLE_LENGTH;
}

double ll = 0, rr = 0;
void Protonek::updateOdometry() {

//    ldif = bridgeL.getPositionDifference()*3.1415*WHEEL_DIAM/ENC_TICKS;
//    rdif = bridgeR.getPositionDifference()*3.1415*WHEEL_DIAM/ENC_TICKS;

    if (ldif > 1 || ldif < -1 || rdif > 1 || rdif < -1)
    {
//            ROS_DEBUG("%lf  %lf", ldif, rdif);    
            throw;
    }
    ll += ldif;    
    rr += rdif;    
//    ROS_DEBUG("%d  %d", (int)(ll*100), (int)(rr*100));
    apos += (ldif+rdif)/(AXLE_LENGTH);
//    ROS_DEBUG("%lf", apos);
    
    double dist = (ldif-rdif)/2;
    
    xpos += dist * cos(apos);
    ypos += dist * sin(apos);
}

bool Protonek::getMeanAngularVelocity(double &mean) {
        double xvel, thvel;
        
        getVelocity(xvel, thvel);

        const int count = 4;
        static double m[count];
        static int index = 0;

        m[index] = thvel;
        index = (index+1)%count;

        if (index == 0) {        
                mean = 0;
                for (int i=0; i<count; ++i)
                        mean += m[i];
                mean /= count;
                return true;
        }

        return false;
}

bool Protonek::getMeanLinearVelocity(double &mean) {
        double xvel, thvel;
        
        getVelocity(xvel, thvel);
//        ROS_DEBUG("%lf   %lf", xvel, thvel);
        const int count = 4;
        static double m[count];
        static int index = 0;

        m[index] = xvel;
        index = (index+1)%count;
        if (index == 0) {        
                mean = 0;
                for (int i=0; i<count; ++i)
                        mean += m[i];

                mean /= count;
                return true;
        }                
        return false;
}

void Protonek::getOdometry(double &x, double &y, double &a) {
        x = xpos;
        y = ypos;
        a = apos;
}

void Protonek::setOdometry(double x, double y, double a) {
        xpos = x;
        ypos = y;
        apos = a;
}

void Protonek::getImu(double &accX, double &accY, double &accZ, double &omegaY, double &pitch, double &pitch2) {

        accX = orientation.accX;
        accY = orientation.accY;
        accZ = orientation.accZ;
        omegaY = orientation.omegaY;
        pitch = orientation.getPitch();
        pitch2 = orientation.pitchGyro;
}

void Protonek::getRawOdometry(double &linc, double &rinc) {
}

bool Protonek::isConnected() {
        return connected;
}

Protonek::Orientation::Orientation() : accX(100), accY(100), accZ(100),
        pitch(0), oldPitch(0), updated(-100), updated2(-101), pitchGyro(0) {
}

void Protonek::Orientation::update(int accX, int accY, int accZ, int g, double time) {

/*    double acceleration = sqrt((accY-311)*(accY-311) + (accZ-316)*(accZ-316))/145.0;

    pitch = (180.0*atan2((accY-311),(accZ-316))/M_PI) + (double)g*0.1;

    pitch2 = (180.0*atan2((accY-311)*acceleration,(accZ-316)/acceleration)/M_PI);//*0.5 + pitch2*0.5;

    dpitch = pitch2 + (pitch2-pitch)*1 - oldPitch;
    oldPitch = pitch2 + (pitch2-pitch)*1;

    this->accX = accX;
    this->accY = accY;
    this->accZ = accZ;
    omegaY = g;

    updated2 = updated;
    updated = time;
*/

    double acceleration = sqrt((accY-311)*(accY-311) + (accZ-316)*(accZ-316))/145.0;

    pitch = (180.0*atan2((accY-311),(accZ-316))/M_PI) + (double)g*0.1;

    pitch2 = (180.0*atan2((accY-311)*acceleration,(accZ-316)/acceleration)/M_PI);


    this->accX = accX;
    this->accY = accY;
    this->accZ = accZ;
    omegaY = g;

    pitchGyro += ((double)omegaY+3.0)*0.08;
    
    pitchGyro += (pitch - pitchGyro)*0.1;

//ROS_DEBUG("%lf    %lf",pitch,pitchGyro);
    dpitch = pitchGyro - oldPitch;
    oldPitch = pitchGyro;
    
    updated2 = updated;
    updated = time;

}

bool Protonek::Orientation::isValid(double time) {
        return (time-updated)<0.2;
}

double Protonek::Orientation::getPitch() {
    return pitch;
}

double Protonek::Orientation::getDPitch(double time) {
        return dpitch;
}

double Protonek::Orientation::getInterval() {
        return updated-updated2;
}

Protonek::Trick::Trick() : active(false) {
}

void Protonek::Trick::start() {
        if (active)
                return;

        active = true;
        angleAccum = 0;
}

void Protonek::Trick::getVelocity(double& lvel, double& rvel) {
        if (!active) {
                lvel = 0;
                rvel = 0;
        } else {
                lvel = 0.2;
                rvel = -0.2;
        }
}

void Protonek::Trick::setAngle(double th) {
        if (!active)
                return;

        if (angleAccum == 0) {
                angleOld = th;
                angleAccum = 0.001;
        } else {
/*                if (angleOld > th + 3) {
                        angleOld -= 3.1415 * 2.0;
                }
                
                if (angleOld < th - 3) {
                        angleOld += 3.1415 * 2.0;
                }
*/
//                ROS_DEBUG("%lf     %lf", th, angleAccum);
                angleAccum += (th-angleOld);
                angleOld = th;
        }
        
        if (angleAccum > 3.14 || angleAccum < -3.14)
                active = false;
}

bool Protonek::Trick::isActive() {
        return active;
}

