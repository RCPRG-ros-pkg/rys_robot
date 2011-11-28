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

Protonek::Protonek(const std::string& port0, const std::string& port1, const Protonek::Parameters &parL,
                    const Protonek::Parameters &parR,  int baud)
                    : isBalancing(false), speedRegulator(true), ldif(0), rdif(0),
                    state(STOPPING), destAngle(0), ldifOdom(0), rdifOdom(0) {

        std::string devRs485Name;

        struct termios oldtio0, oldtio1;
        int f0 = OpenImuDevice(port0, oldtio0);
        int f1 = OpenImuDevice(port1, oldtio1);
        
        if (f0 == -1 || f1 == -1) {
                ROS_DEBUG("nie mozna otworzyc urzadzenia");
                throw;
        }
        
        int bytesRead0=0, bytesRead1=1;
        unsigned char buffer[500];

/*        do {
                bytesRead0 += read(f0,buffer,500);
                bytesRead1 += read(f1,buffer,500);
        } while (bytesRead0 > 100 || bytesRead1 > 100);
*/
        if (bytesRead0 > bytesRead1) {
                devRs485Name = port1;
                tcsetattr(f1, TCSANOW, &oldtio1);
                close(f1);
                oldtioImu = oldtio0;
                fdImu = f0;
        } else {
                devRs485Name = port0;
                tcsetattr(f0, TCSANOW, &oldtio0);
                close(f0);
                oldtioImu = oldtio1;
                fdImu = f1;
        }

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

        fdRs485 = open(devRs485Name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fdRs485==-1) {
            printf("nie mozna otworzyc portu: %s errno:%d\n",devRs485Name.c_str(), (int)errno);
            throw;
        }

        tcgetattr(fdRs485, &oldtioRs485);

        // set up new settings
        struct termios newtio;
        memset(&newtio, 0, sizeof(newtio));
        newtio.c_cflag = CBAUD | CS8 | CLOCAL | CREAD | CSTOPB;
        newtio.c_iflag = IGNPAR;
        newtio.c_oflag = 0;
        newtio.c_lflag = 0;
        if (cfsetispeed(&newtio, baud) < 0 || cfsetospeed(&newtio, baud) < 0) {
            fprintf(stderr, "Failed to set serial baud rate: %d\n", baud);
            tcsetattr(fdRs485, TCSANOW, &oldtioRs485);
            close(fdRs485);
            fdRs485 = -1;
        }
        // activate new settings
        tcflush(fdRs485, TCIFLUSH);
        tcsetattr(fdRs485, TCSANOW, &newtio);

        connected = true;
}

int Protonek::OpenImuDevice(const std::string& port, struct termios &oldtio)
{
        int f = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (f==-1) {
            printf("nie mozna otworzyc portu: %s errno:%d\n",port.c_str(), (int)errno);
            throw;
        }

        tcgetattr(f, &oldtio);

        // set up new settings
        struct termios newtio;
        memset(&newtio, 0, sizeof(newtio));
        newtio.c_cflag = CBAUD | CS8 | CLOCAL | CREAD | CSTOPB;
        newtio.c_iflag = IGNPAR;
        newtio.c_oflag = 0;
        newtio.c_lflag = 0;
        if (cfsetispeed(&newtio, B57600) < 0 || cfsetospeed(&newtio, B57600) < 0) {
            fprintf(stderr, "Failed to set serial baud rate: %d\n", B57600);
            tcsetattr(f, TCSANOW, &oldtio);
            close(f);
            f = -1;
            return f;
        }
        // activate new settings
        tcflush(f, TCIFLUSH);
        tcsetattr(f, TCSANOW, &newtio);

        return f;
}

Protonek::~Protonek() {
        // restore old port settings
        if (fdRs485 > 0) {
                tcsetattr(fdRs485, TCSANOW, &oldtioRs485);
                close(fdRs485);
        }
        if (fdImu > 0) {
                tcsetattr(fdImu, TCSANOW, &oldtioImu);
                close(fdImu);
        }
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

double Protonek::angularVelocityPID(double thvel, bool reset) {
        double A = avKp + avKi + avKd;
        double A_no_Ki = aKp + aKd;
        double B = -(avKp + 2 * avKd);
        double C =  avKd;
        static double sERRprev2 = 0;
        static double sERRprev = 0;
        static double sOUTprev = 0;
        static double sOUTt = 0;

        double mean = 0;

        if (reset) {
            sERRprev2 = 0;
            sERRprev = 0;
            sOUTprev = 0;
            return 0;
        }
        
        if (!getMeanAngularVelocity(mean))
                return sOUTt;

        // regulator PID
        double sERRt = mean - thvel;

        sOUTt = sOUTprev + ((sERRt * A + sERRprev * B + sERRprev2 * C)/2);

        if (sOUTt > 0.8)
                sOUTt = 0.8;
        else if (sOUTt < -0.8)
                sOUTt = -0.8;

        sERRprev2 = sERRprev;
        sERRprev = sERRt;
        sOUTprev = sOUTt;

        return sOUTt;
}

double Protonek::linearVelocityPID(double xvel, bool reset) {
        double A = lvKp + lvKi + lvKd;
        double A_no_Ki = aKp + aKd;
        double B = -(lvKp + 2 * lvKd);
        double C =  lvKd;
        static double sERRprev2 = 0;
        static double sERRprev = 0;
        static double sOUTprev = 0;
        static double sOUTt = 0;

        double mean = 0;

        if (reset) {
            sERRprev2 = 0;
            sERRprev = 0;
            sOUTprev = 0;
            return 0;
        }

        if (!getMeanLinearVelocity(mean))
                return sOUTt;

        // regulator PID
        double sERRt = mean - xvel;

        sOUTt = sOUTprev + ((sERRt * A + sERRprev * B + sERRprev2 * C)/2);

        if (sOUTt > 6.8)
                sOUTt = 6.8;
        else if (sOUTt < -6.8)
                sOUTt = -6.8;

        sERRprev2 = sERRprev;
        sERRprev = sERRt;
        sOUTprev = sOUTt;

        return sOUTt;
}

void Protonek::Balance3(bool reset) {
        double A = aKp + aKi + aKd;
        double A_no_Ki = aKp + aKd;
        double B = -(aKp + 2 * aKd);
        double C =  aKd;
        static double sERRprev2 = 0;
        static double sERRprev = 0;
        static double sOUTprev = 0;
        static double vel = 0;

        double turn = -angularVelocityPID(joystick.speedAngular*0.02, reset);
        double addedAngle = linearVelocityPID(joystick.speedLinear*0.015, reset);
//        double addedAngle = joystick.speedLinear*4;

        if (reset) {
            sERRprev2 = 0;
            sERRprev = 0;
            sOUTprev = 0;
            vel = 0;
            return;
        }

        destAngle = balanceAngle + 8*(addedAngle);
        
        // regulator PID
        double sERRt = orientation.pitchGyro-destAngle;

        // feed forward
        double ffadd = sin(3.1415*(orientation.pitchGyro-balanceAngle)/180.0)*(51.0/255.0);
        
        double sOUTt = sOUTprev + sERRt * A + sERRprev * B + sERRprev2 * C;

        if (sOUTt > 0.8)
                sOUTt = 0.8;
        else if (sOUTt < -0.8)
                sOUTt = -0.8;
        
        sERRprev2 = sERRprev;
        sERRprev = sERRt;
        sOUTprev = sOUTt;

        vel = -sOUTt-ffadd;
        
        bridgeL.setCurrent(vel - turn);
        bridgeR.setCurrent(-vel - turn);
}

void Protonek::Send() {
        MSG msg;
        static int device = 1;
        if (device==1)
                device = 2;
        else
                device = 1;

        if (device==1)
            msg = bridgeR.serialize();
        else if (device==2)
            msg = bridgeL.serialize();

        msg.crc = Crc::Crc16Byte((uint8_t*)&msg,sizeof(MSG)-2,0);
        write(fdRs485, &msg, sizeof(MSG));
        tcdrain(fdRs485);
}

void Protonek::Receive(double time) {
        static MSG answer;
        unsigned char *ans_buf = (unsigned char*)&answer;
        static int ans_i=0;
        bool trans_OK=false;

        int bytesRead;
        unsigned char buffer[500];
        unsigned char buf=0;

        bytesRead = read(fdRs485,buffer,500);

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


                if (crc==0 && answer.header==MSG_HEADER && answer.adr==1)
                {
                        int mocZadana = answer.d[5];
                        int szybkoscZadana = answer.d[3];

                        int distance = answer.d[9] | (answer.d[10]<<8);

                        bridgeL.deserialize(time, answer);
                        ldif = bridgeL.getPositionDifference()*3.1415*WHEEL_DIAM/ENC_TICKS;
                        ldifOdom += ldif;
                        
                        trans_OK = true;
                        receivedCorrectData = true;

                        ans_i = 0;              // czekamy na nastepne bajty
                } else
                if (crc==0 && answer.header==MSG_HEADER && answer.adr==2)
                {
                        int mocZadana = answer.d[5];
                        int szybkoscZadana = answer.d[3];

                        int distance = answer.d[9] | (answer.d[10]<<8);

                        bridgeR.deserialize(time, answer);
                        rdif = -bridgeR.getPositionDifference()*3.1415*WHEEL_DIAM/ENC_TICKS;
                        rdifOdom += rdif;

                        trans_OK = true;
                        receivedCorrectData = true;

                        ans_i = 0;              // czekamy na nastepne bajty
                }
        }
}

void Protonek::ReceiveImu(double time) {
        int bytesRead;
        unsigned char buffer[500];
        unsigned char buf=0;
        static char line[300];
        static int lineIndex = 0;

        bytesRead = read(fdImu,buffer,500);
        if (bytesRead < 0) {
                ROS_DEBUG("connection error");
                throw;
        }

        for (int i_buf=0; i_buf<bytesRead; ++i_buf) {
                buf = buffer[i_buf];
                
                if (buf == 0x0A || buf == 0x0D) {
                        if (lineIndex > 0) {
                                line[lineIndex] = 0;
                                int n[6];
                                for (int i=0; i<lineIndex; ++i) {
                                        if (line[i] == '$' || line[i] == '#' || line[i] == ',')
                                                line[i] = ' ';
                                }
                                                                        
                                sscanf(line, "%d%d%d%d%d%d", &n[0], &n[1], &n[2], &n[3], &n[4], &n[5]);
                                lineIndex = 0;
                                
                                orientation.updateFromImu(n[0], n[1], n[2], n[4], n[5], time);
                        }
                } else {
                        line[lineIndex++] = buf;
                }
        }
}


void Protonek::stateTeleop() {
        ROS_DEBUG("teleop");

        double lvel, rvel;

        lvel = -joystick.speedLinear - joystick.speedAngular;
        rvel = -joystick.speedLinear + joystick.speedAngular;

        lvel = lvel>1.0?1.0:(lvel<-1.0?-1.0:lvel);
        rvel = rvel>1.0?1.0:(rvel<-1.0?-1.0:rvel);

        double meanVel = 0;
        getMeanLinearVelocity(meanVel);

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
                        getUp();
                } else
                if (joystick.buttonTrick) {
                }
                
                if (fabs(orientation.pitchGyro-balanceAngle)<10) {
                        state = BALANCING;
                        Balance3(true);
                }
        }
        
        if (speedRegulator) {
                bridgeL.setSpeed(lvel);
                bridgeR.setSpeed(-rvel);
        } else {
                bridgeL.setCurrent(lvel*0.7);
                bridgeR.setCurrent(-rvel*0.7);
        }

}

void Protonek::stateBalancing(double time, double interval) {
        ROS_DEBUG("balancing");

        Balance3(false);

        if (fabs(orientation.pitchGyro-balanceAngle)>55 || joystick.buttonStop) {
                state = STOPPING;
        }
}

void Protonek::stateStopping(double interval) {
        ROS_DEBUG("STOPPING");

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

        double lcur = 0, rcur = 0;
        static FilteredDouble m(8,0);
        double omega = m.SetGet(orientation.omegaZ);
                
        if (joystick.buttonStop) {
                state = STOPPING;
                lcur = 0;
                rcur = 0;
        } else {
                if (fabs(orientation.pitchGyro-balanceAngle)<10) {
                        state = BALANCING;
                        Balance3(true);
                } else {
                        if (gettingUpData.ticks < 70) {
                                lcur = rcur = gettingUpData.cur;
                        } else if (gettingUpData.ticks < 200) {
                                lcur = rcur = -gettingUpData.cur;
                                gettingUpData.cur = (orientation.getPitch()-balanceAngle)/90.0;
                                if (gettingUpData.cur > 0.4)
                                        gettingUpData.cur = 0.4;
                                else if (gettingUpData.cur < -0.4)
                                        gettingUpData.cur = -0.4;
                        } else
                                state = TELEOP;
                }
        }
        
        bridgeL.setCurrent(lcur);
        bridgeR.setCurrent(-rcur);

        ++gettingUpData.ticks;
}

void Protonek::getUp() {
        if (state != TELEOP)
                return;         // wstawanie tylko z trybu TELEOP

        if (orientation.pitchGyro > 60 && orientation.pitchGyro < 120)
                gettingUpData.cur = 0.4;
        else if (orientation.pitchGyro < -60 && orientation.pitchGyro > -120)
                gettingUpData.cur = -0.4;
        else
                return;         // jakis dziwny kat - nie wstajemy
                
        gettingUpData.ticks = 0;
        state = GETTINGUP;
}

void Protonek::update() {
        static int error = 0;

        // okreslenie jaki czas minal od ostatniego wywolania metody Update
        static double elapsed = 0;
        double interval = getInterval();        // interwal
        elapsed += interval;            // czas co jaki sprawdzamy czy nastapila przerwa w transmisji
        static double time = 0;
        time += interval;               // czas skumulowany

//        if (elapsed>0.010) {

                static double lpos=0, rpos=0;

                if (elapsed > 0.8) {
                        ROS_DEBUG("too long: %lf", elapsed);
                        //throw;
                }

                if (!orientation.isValid(time)) {
                        ROS_DEBUG("no gyro/acc data!");
                        state = STOPPING;
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
                
//        }

        Receive(time);
        ReceiveImu(time);
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
        double lvel = ldif;
        double rvel = rdif;

        xvel = (lvel + rvel) * 0.5;
        thvel = (lvel - rvel) / AXLE_LENGTH;
}

double ll = 0, rr = 0;
void Protonek::updateOdometry() {

    ll += ldifOdom;
    rr += rdifOdom;
    apos += (ldifOdom+rdifOdom)/(AXLE_LENGTH);
    
    double dist = (ldifOdom-rdifOdom)/2;
    
    xpos += dist * cos(apos);
    ypos += dist * sin(apos);
    
    ldifOdom = 0;
    rdifOdom = 0;
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

        static double accum = 0.0;
        accum += GetHorizontalAcceleration()*0.000003;
        accum += (xvel - accum)*0.05;

        mean = accum;

        return true;
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

void Protonek::getImu(double &accX, double &accY, double &accZ, double &omegaZ,
                        double &pitch, double &pitch2, double &destPitch) {

        accX = orientation.accX;
        accY = orientation.accY;
        accZ = orientation.accZ;
        omegaZ = orientation.omegaZ;
        pitch = orientation.getPitch();
        pitch2 = orientation.pitchGyro;
        destPitch = this->destAngle;
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

    double acceleration = sqrt((accY-311)*(accY-311) + (accZ-316)*(accZ-316))/145.0;

    pitch = (180.0*atan2((accY-311),(accZ-316))/M_PI);

    this->accX = accX;
    this->accY = accY;
    this->accZ = accZ;
    
    omegaZ = g;

    pitchGyro += ((double)omegaZ+3.0)*0.1;
    if (pitchGyro > 180 + pitch)
            pitchGyro -= 360;
    if (pitchGyro < -180 + pitch)
            pitchGyro += 360;
    
    pitchGyro += (pitch - pitchGyro)*0.1;
    if (pitchGyro > 180 + pitch)
            pitchGyro -= 360;
    if (pitchGyro < -180 + pitch)
            pitchGyro += 360;

    dpitch = pitchGyro - oldPitch;
    if (dpitch > 180)
            dpitch -= 360;
    if (dpitch < -180)
            dpitch += 360;
    oldPitch = pitchGyro;
    
    updated2 = updated;
    updated = time;

}

void Protonek::Orientation::updateFromImu(int accX, int accY, int accZ, int omegaY, int omegaZ, double time) {

    static FilteredDouble m(4,0);
    pitch = m.SetGet((180.0*atan2(accX,accY)/M_PI));

    this->accX = accX;
    this->accY = accY;
    this->accZ = accZ;

    this->omegaY = omegaY;
    this->omegaZ = omegaZ;

    pitchGyro += ((double)this->omegaZ+10.0)*0.001;
    if (pitchGyro > 180 + pitch)
            pitchGyro -= 360;
    if (pitchGyro < -180 + pitch)
            pitchGyro += 360;
    
    pitchGyro += (pitch - pitchGyro)*0.03;
    if (pitchGyro > 180 + pitch)
            pitchGyro -= 360;
    if (pitchGyro < -180 + pitch)
            pitchGyro += 360;

    dpitch = pitchGyro - oldPitch;
    if (dpitch > 180)
            dpitch -= 360;
    if (dpitch < -180)
            dpitch += 360;
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

double Protonek::GetHorizontalAcceleration()
{
        static FilteredDouble m(256,0);
        
        double alpha = -M_PI*(orientation.pitchGyro)/180.0;
        return m.SetGet(orientation.accX * cos(alpha) + orientation.accY * sin(alpha));
}

Protonek::FilteredDouble::FilteredDouble(int count, double init) {
        this->count = count;
        
        if (this->count > 0)
                m = new double[this->count];
        else
                m = 0;
                
        index = 0;
        for (int i=0; i<this->count; ++i)
                m[i] = init;
}

Protonek::FilteredDouble::~FilteredDouble() {
        if (m != 0)
                delete[] m;
}

double Protonek::FilteredDouble::SetGet(double d) {
        m[index] = d;
        index = (index+1)%count;

        double mean = 0;        
        for (int i=0; i<count; ++i)
                mean += m[i];
                
        return mean/count;
}
