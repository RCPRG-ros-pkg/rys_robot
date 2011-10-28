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
#define WHEEL_DIAM 0.1
// axle length in SI units [m]
#define AXLE_LENGTH 0.355

// regulator rate in SI units [Hz]
#define REGULATOR_RATE 100

// maximum velocity, in internal units
#define MAX_VEL 550
// number of encoder ticks per single wheel rotation
#define ENC_TICKS 4000

class Protonek {
public:
	Protonek(const std::string& port, int baud = BAUD);
	~Protonek();

	void update();

	void setVelocity(double lvel, double rvel);
	void getVelocity(double &lvel, double &rvel);

// +
        void SetPower(double lvel, double rvel);

        void setupPIDmotorA(int Kp, int Ki, int Kd);
        void setupPIDmotorB(int Kp, int Ki, int Kd);
        
        void AddPortName(const char* port);

        double getPitch();
        double getDPitch();
        double getOdomL();
        double getOdomR();

        void Run();
        void Stop();
        
// /+

	void updateOdometry();
	void getRawOdometry(double &linc, double &rinc);
	void getOdometry(double &x, double &y, double &a);
	void setOdometry(double x, double y, double a);

	bool isConnected();

	double m_per_tick;
	double robot_axle_length;
	double enc_ticks;

	void dump();

	void setParams(double ls, double rs);

	void trick1();
	void trick2();
	
private:
	// serial port descriptor
	int fd;
	struct termios oldtio;
	bool connected;

	double llpos;
	double lrpos;

	double xpos;
	double ypos;
	double apos;

	bool odom_initialized;

	int32_t old_lpos, old_rpos;

	std::ofstream of;

	bool _dump;

	double lin_scale;
	double rot_scale;

// +
        void Balance2(double time, bool reset);
        void Send();
        void Receive(double time);

        double getInterval();

        bool running;

        double rspeed, lspeed;

        bool isBalancing;

        HBridge bridgeL, bridgeR;
        double rvel, lvel;

        class Orientation {
        public:
                Orientation();
                void update(int accX, int accY, int accZ, int gyro, double time);
                bool isValid(double time);
                double getPitch();
                double getDPitch(double time);
                double getInterval();
        private:
                int accX,accY,accZ;	// pomiar przyspieszenia
                int gyro;
                double updated;		// czas uaktualnienia
                double updated2;	// poprzedni czas uaktualnienia
                double pitch, oldPitch;	// kat i szybkosc katowa wzdluz osi kol
                double pitch2;
                double dpitch;
        } orientation;

        class Position {
        public:
                Position();
                void update(int newPosition, double time);
                bool isValid(double time);
                double getPosition();
                double getDPosition(double time);
                double getInterval();
        private:
                int enc;
                double updated;		// czas uaktualnienia
                double updated2;	// poprzedni czas uaktualnienia
                double pos, oldPos;
                double dpos;
        } position_left, position_right;
// /+
};

#endif /* ELEKTRON_HPP_ */
