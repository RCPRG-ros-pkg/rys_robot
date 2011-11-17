/*
 * elektron.hpp
 *
 *  Created on: Sep 5, 2009
 *      Author: konradb3
 */

#ifndef ELEKTRON_HPP_
#define ELEKTRON_HPP_

#include <stdint.h>
#include <termios.h>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string>
#include <fstream>

// +
#include "common.hpp"
#include "hbridge.hpp"
// /+

// baudrate
#define BAUD B115200
// port
#define PORT = "/dev/ttyUSB0"

// wheel diameter in SI units [m]
#define WHEEL_DIAM 0.5
// axle length in SI units [m]
#define AXLE_LENGTH 0.45

// regulator rate in SI units [Hz]
#define REGULATOR_RATE 100

// number of encoder ticks per single wheel rotation
#define ENC_TICKS 1056

class Protonek {
public:
        class Parameters {
        public:
                int currentKp, currentKi, currentKd;
                int speedKp, speedKi, speedKd;
                int maxCurrent;
        };
        
        Protonek(const std::string& port, const Parameters &parL, const Parameters &parR, int baud = BAUD);
        ~Protonek();

        void update();

        void getVelocity(double &lvel, double &rvel);

        void SetPower(double lvel, double rvel);

        void setupPIDmotorA(int Kp, int Ki, int Kd);
        void setupPIDmotorB(int Kp, int Ki, int Kd);

        void setupPIDangle(double Kp, double Ki, double Kd);
        void setupPIDangularVelocity(double Kp, double Ki, double Kd);
        void setupPIDlinearVelocity(double Kp, double Ki, double Kd);

        void AddPortName(const char* port);

        double getPitch();
        double getDPitch();
        double getOdomL();
        double getOdomR();

        void runMotors();
        void stopMotors();
        
        void updateOdometry();
        void getRawOdometry(double &linc, double &rinc);
        void getOdometry(double &x, double &y, double &a);
        void setOdometry(double x, double y, double a);
        void getImu(double &accX, double &accY, double &accZ, double &omegaY, double &pitch, double &pitch2);
        
        bool isConnected();

        double m_per_tick;
        double robot_axle_length;
        double enc_ticks;

        void trick1();
        void trick2();

        void enableSpeedRegulator();
        void disableSpeedRegulator();
        
        HBridge bridgeL, bridgeR;

        double balanceAngle;
        
        class Joystick {
        public:
                Joystick() : buttonStop(false), buttonTurbo(false), buttonGetUp(false),
                            buttonTrick(false), speedAngular(0.0), speedLinear(0.0) {}
                            
                double speedLinear;
                double speedAngular;
                bool buttonStop;
                bool buttonTurbo;
                bool buttonGetUp;
                bool buttonTrick;
        } joystick;
        
private:
        // state
        enum STATE { TELEOP, BALANCING, STOPPING, GETTINGUP };
        STATE state;

        void stateTeleop();
        void stateBalancing(double time, double interval);
        void stateStopping(double interval);
        void stateGettingUp();

        bool getMeanAngularVelocity(double &mean);
        bool getMeanLinearVelocity(double &mean);

        double angularVelocityPID(double thvel);
        double linearVelocityPID(double xvel);

        // serial port descriptor
        int fd;
        struct termios oldtio;
        bool connected;

        double ldif, rdif;
        
        // PID angle
        double aKp, aKi, aKd;
        
        // PID angular velocity
        double avKp, avKi, avKd;

        // PID linear velocity
        double lvKp, lvKi, lvKd;

        double xpos;
        double ypos;
        double apos;

        bool odom_initialized;

        std::ofstream of;

        void Balance3(double time, bool reset);
        void Send();
        void Receive(double time);

        double getInterval();

        bool isBalancing;

        bool speedRegulator;
                
        class Orientation {
        public:
                Orientation();
                void update(int accX, int accY, int accZ, int gyro, double time);
                bool isValid(double time);
                double getPitch();
                double getDPitch(double time);
                double getInterval();

                int accX,accY,accZ;	// pomiar przyspieszenia
                int omegaY;
                double pitchGyro;
        private:
                double updated;		// czas uaktualnienia
                double updated2;	// poprzedni czas uaktualnienia
                double pitch, oldPitch;	// kat i szybkosc katowa wzdluz osi kol
                double pitch2;
                double dpitch;
                
        } orientation;

        class Trick {
        public:
                Trick();
                
                void getVelocity(double& lvel, double& rvel);
                void setAngle(double th);
                bool isActive();
                void start();
                
        private:
                bool active;
                double angleAccum;
                double angleOld;
        } trick;
};

#endif /* ELEKTRON_HPP_ */
