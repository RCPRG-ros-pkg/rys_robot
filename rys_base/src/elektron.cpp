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

Protonek::Protonek(const std::string& port, int baud) : rspeed(0), lspeed(0), isBalancing(false), rvel(0), lvel(0)
{
// +

    bridgeL.setCurrentPID(0,2,0);
    bridgeL.setSpeedPID(8,0,0);
    bridgeL.setMaxPower(150);
    bridgeL.setAddress(2);

    bridgeR.setCurrentPID(0,2,0);
    bridgeR.setSpeedPID(8,0,0);
    bridgeR.setMaxPower(150);
    bridgeR.setAddress(1);
// /+

    connected = false;

	llpos = 0;
	lrpos = 0;

	xpos = 0;
	ypos = 0;
	apos = 0;

//	setvel.start = 'x';
//	setvel.cmd = 'a';
//	setvel.lvel = 0;
//	setvel.rvel = 0;

	odom_initialized = false;

	robot_axle_length = AXLE_LENGTH;
	m_per_tick = M_PI * WHEEL_DIAM / ENC_TICKS;
	enc_ticks = ENC_TICKS;

	lin_scale = 1.0;
	rot_scale = 1.0;

// +
    fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd==-1) {
        printf("nie mozna otworzyc portu: %s\n",port.c_str());
        switch (errno) {
            case EACCES:		printf("The requested access to the file is not allowed, or search permission is denied for one of the directories in the path prefix of pathname, or the file did not exist yet and write access to the parent directory is not allowed. (See also path_resolution(2).) \n"); break;
            case EEXIST:		printf("pathname already exists and O_CREAT and O_EXCL were used. \n"); break;
            case EFAULT:		printf("pathname points outside your accessible address space. \n"); break;
            case EISDIR:		printf("pathname refers to a directory and the access requested involved writing (that is, O_WRONLY or O_RDWR is set). \n"); break;
	    case ELOOP:			printf("Too many symbolic links were encountered in resolving pathname, or O_NOFOLLOW was specified but pathname was a symbolic link. \n"); break;
            case EMFILE:		printf("The process already has the maximum number of files open. \n"); break;
            case ENAMETOOLONG:	printf("pathname was too long. \n"); break;
            case ENFILE:		printf("The system limit on the total number of open files has been reached. \n");break;
            case ENODEV:		printf("pathname refers to a device special file and no corresponding device exists. (This is a Linux kernel bug; in this situation ENXIO must be returned.) \n");break;
            case ENOENT:		printf("O_CREAT is not set and the named file does not exist. Or, a directory component in pathname does not exist or is a dangling symbolic link. \n");break;
            case ENOMEM:		printf("Insufficient kernel memory was available. \n");break;
            case ENOSPC:		printf("pathname was to be created but the device containing pathname has no room for the new file. \n");break;
            case ENOTDIR:		printf("A component used as a directory in pathname is not, in fact, a directory, or O_DIRECTORY was specified and pathname was not a directory. \n");break;
            case ENXIO:			printf("O_NONBLOCK | O_WRONLY is set, the named file is a FIFO and no process has the file open for reading. Or, the file is a device special file and no corresponding device exists. \n");break;
            case EOVERFLOW:		printf("pathname refers to a regular file, too large to be opened; see O_LARGEFILE above. \n");break;
            case EPERM:			printf("The O_NOATIME flag was specified, but the effective user ID of the caller did not match the owner of the file and the caller was not privileged (CAP_FOWNER). \n");break;
            case EROFS:			printf("pathname refers to a file on a read-only filesystem and write access was requested. \n");break;
            case ETXTBSY:		printf("pathname refers to an executable image which is currently being executed and write access was requested. \n");break;
            case EWOULDBLOCK:	printf("The O_NONBLOCK flag was specified, and an incompatible lease was held on the file (see fcntl(2)).\n"); break;
            default:			printf("Unknown error.\n");
        }
        printf("\n");
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

// /+

/*
	fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd >= 0) {
		tcgetattr(fd, &oldtio);

		// set up new settings
		struct termios newtio;
		memset(&newtio, 0, sizeof(newtio));
		newtio.c_cflag = CBAUD | CS8 | CLOCAL | CREAD | CSTOPB;
		newtio.c_iflag = INPCK; //IGNPAR;
		newtio.c_oflag = 0;
		newtio.c_lflag = 0;
		if (cfsetispeed(&newtio, baud) < 0 || cfsetospeed(&newtio, baud) < 0) {
			fprintf(stderr, "Failed to set serial baud rate: %d\n", baud);
			tcsetattr(fd, TCSANOW, &oldtio);
			close(fd);
			fd = -1;
			return;
		}
		// activate new settings
		tcflush(fd, TCIFLUSH);
		tcsetattr(fd, TCSANOW, &newtio);
		connected = true;
	}
	*/
connected = true;
	_dump = false;
}

Protonek::~Protonek() {
    // restore old port settings
    if (fd > 0)
	tcsetattr(fd, TCSANOW, &oldtio);
    close(fd);
}

void Protonek::setParams(double ls, double rs) {
	lin_scale = ls;
	rot_scale = rs;
}

void Protonek::dump() {
	_dump = true;
	of.open("/tmp/odom_dump.txt");
	of << "lpos;lindex;rpos;rindex;lvel;rvel;apos;xpos;ypos\n";
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

void Protonek::Balance2(double time, bool reset) {

//        s.lcur = s.rcur = 0;
//        s.lvel = s.rvel = 0;

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

        if (reset) {
            sERRprev2 = 0;
            sERRprev = 0;
            sOUTprev = 0;
            vel = 0;
        }

        double addedAngle = 0;//steering.rvel+steering.lvel;

        addedAngle *= (1-0.5*(lspeed+rspeed)/2);

        // regulator PID
        double sERRt = orientation.getPitch()-(89.5 + 8*(addedAngle));

//        printf("%lf\n",sERRt);
//        if (sERRt<0)
//            sERRt = -sERRt*sERRt / 20;
//        else
//            sERRt = sERRt*sERRt / 20;

        double sOUTt = sOUTprev + ((sERRt * A + sERRprev * B + sERRprev2 * C)/2);
        if (sOUTt<0.8 && sOUTt>-0.8)	// ograniczenie calkowania
        {
                sERRprev2 = sERRprev;
                sERRprev = sERRt;
                sOUTprev = sOUTt;
        }

        if (sOUTt<-0.8)
            sOUTt = -0.8;
        if (sOUTt>0.8)
            sOUTt = 0.8;

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

        //double turn =  (steering.rvel - steering.lvel)*0.2;

        bridgeR.setSpeed(-vel2);// + turn);
        bridgeL.setSpeed(vel2);// - turn);

        //s.rcur = -(vel2 + turn);
        //s.lcur = (vel2 - turn);
}

void Protonek::Send() {
        MSG msg;
        static int device = 1;
        if (device==1)
                device = 2;
        else if (device==2)
                device = 3;
        else
                device = 1;

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

//ROS_DEBUG("                  bytes read: %d\n", bytesRead);
        for (int i_buf=0; i_buf<bytesRead; ++i_buf)
        {
                buf = buffer[i_buf];
                for (int i=0;i<sizeof(MSG)-1;++i)
                        ans_buf[i] = ans_buf[i+1];
                ans_buf[sizeof(MSG)-1] = buf;
                ++ans_i;

                //if (ans_i == sizeof(MSG))
                {
                    uint16_t crc = Crc::Crc16(answer);

                        if (crc==0 && answer.header==MSG_HEADER && answer.adr==1)	// crc
                        {
                                int mocZadana = answer.d[5];
                                int szybkoscZadana = answer.d[3];
                                //printf("ok\n");
                                //float speedRatio = (float)((char)answer.d[2])/(float)((char)answer.d[3]);
                                rspeed = (signed char)(answer.d[2]);// | (answer.d[3]<<8));

                                //rspeed = 100.0f/rspeed;

                                int distance = answer.d[9] | (answer.d[10]<<8);

                                position_left.update(distance,time);

                                //static float smoothSpeed = 0;
                                //smoothSpeed = smoothSpeed*0.9 + speed*0.1;
                                if (1)
                                        ROS_DEBUG("%d %dms %d %d %d %.3f / %.3f %d moc: %.3f / %.3f dist: %d",
                                                (int)answer.adr,
                                                (int)(position_left.getInterval()*1000),
                                                ans_i,
                                                (int)answer.d[0],
                                                (int)answer.d[1],
                                                rspeed,
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
                                lspeed = (signed char)(answer.d[2]);// | (answer.d[3]<<8));

                                //lspeed = 100.0f/lspeed;

                                int distance = answer.d[9] | (answer.d[10]<<8);

                                position_right.update(distance,time);

                                //static float smoothSpeed = 0;
                                //smoothSpeed = smoothSpeed*0.9 + speed*0.1;
                                if (1)
                                        ROS_DEBUG("%d %dms %d %d %d %.3f / %.3f %d\tmoc: %.3f / %.3f dist: %d",
                                                (int)answer.adr,
                                                (int)(position_right.getInterval()*1000),
                                                ans_i,
                                                (int)answer.d[0],
                                                (int)answer.d[1],
                                                lspeed,
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


//                                printf("%d   %d   %d   %d\n", (int)answer.d[8], (int)answer.d[9], (int)answer.d[10], (int)answer.d[11]);

                                trans_OK = true;
                                receivedCorrectData = true;

                                ans_i = 0;		// czekamy na nastepne bajty
                        } else
                        {
                                //ans_i = sizeof(MSG)-1;		// cos sie pomieszalo - moze zgubiono 1 bajt
                        }
                }
        }
}

void Protonek::update() {
//	unsigned int ret = 0;
//	tcflush(fd, TCIFLUSH);
//	write(fd, &setvel, sizeof(setvel));
//	ROS_DEBUG("%d %d\n", setvel.lvel, setvel.rvel);
//	while (ret < sizeof(getdata))
//		ret += read(fd, ((char*) &getdata) + ret, sizeof(getdata) - ret);

//printf("%lf\n",(lspeed+rspeed)/2);
        static int error = 0;
        static float velocityA=0, velocityB=0;

        unsigned int ret = 0;

        // okreslenie jaki czas minal od ostatniego wywolania metody Update
        static double elapsed = 0;
        double interval = getInterval();	// interwal
        elapsed += interval;				// czas co jaki sprawdzamy czy nastapila przerwa w transmisji
        static double time = 0;
        time += interval;					// czas skumulowany

//        getJoystickState();

bridgeL.Run();
bridgeR.Run();

        if (elapsed>0.01)
        {
    	    ROS_DEBUG("%f %f", (float)lvel, (float)rvel);
                static bool manual = false;
                static double verticalTime = 0;

               // printf("%d\n",(int)isBalancing);

                if (1) {//fabs(orientation.getPitch()-90.0)>35) {
                        isBalancing = false;

                        verticalTime = 0;
                        bridgeL.setSpeed(lvel);
                        bridgeR.setSpeed(-rvel);
                        bridgeL.setSpeedPID(8,0,0);
                        bridgeR.setSpeedPID(8,0,0);

                        Send();
                } else {
                    isBalancing = true;

                    if (verticalTime < 0.5) {
                            Balance2(time,true);      // reset regulatorow balansowania
                        }
                        else {
                            Balance2(time,false);
                        }
                        bridgeL.setSpeedPID(8,0,0);
                        bridgeR.setSpeedPID(8,0,0);
                        verticalTime += interval;
                        Send();
                    }
                elapsed = 0;
        }

        Receive(time);

}
/*
void Protonek::setVelocity(double lvel, double rvel) {
	setvel.lvel = (int16_t)(lvel * (1 / m_per_tick) * 0.1); // Convert SI units to internal units
	setvel.rvel = (int16_t)(rvel * (1 / m_per_tick) * 0.1);

	if (setvel.rvel > MAX_VEL)
		setvel.rvel = MAX_VEL;
	else if (setvel.rvel < -MAX_VEL)
		setvel.rvel = -MAX_VEL;

	if (setvel.lvel > MAX_VEL)
		setvel.lvel = MAX_VEL;
	else if (setvel.lvel < -MAX_VEL)
		setvel.lvel = -MAX_VEL;
}
*/
void Protonek::setVelocity(double lvel, double rvel)
{
        if (lvel>1)
                lvel = 1;
        else if (lvel<-1)
                lvel = -1;
        this->lvel = lvel;

        if (rvel>1)
                rvel = 1;
        else if (rvel<-1)
                rvel = -1;
        this->rvel = rvel;
}

void Protonek::SetPower(double lcur, double rcur)
{
        if (lcur>1)
                lcur = 1;
        else if (lcur<-1)
                lcur = -1;
//	steering.lcur = lcur;

        if (rcur>1)
                rcur = 1;
        else if (rcur<-1)
                rcur = -1;
//	steering.rcur = rcur;
//	steering.lvel = steering.rvel = 0;
}


double Protonek::getPitch() {
    return orientation.getPitch();
}

double Protonek::getDPitch() {
    return orientation.getDPitch(0);
}

void Protonek::getVelocity(double &xvel, double &thvel) {
	static int maxl = 0, maxr = 0;
	double lvel = 0;//(double) (getdata.lvel) * m_per_tick * 10;
	double rvel = 0;//(double) (getdata.rvel) * m_per_tick * 10;

//	if (getdata.lvel > maxl) maxl = getdata.lvel;
//	if (getdata.rvel > maxr) maxr = getdata.rvel;

	//std::cout << maxl << " " << maxr << "\n";
	xvel = (lvel + rvel) * 0.5;
	thvel = (lvel - rvel) / AXLE_LENGTH;
}

void Protonek::updateOdometry() {

	//std::cout << "lpos: " << getdata.lpos << ", rpos: " << getdata.rpos << " lindex: " << getdata.lindex << " rindex: " << getdata.rindex << "\n";

	//std::cout << "vel: " << getdata.lvel << " " << getdata.rvel << "\n";
/*
	double lpos = getdata.lpos + enc_ticks * getdata.lindex;
	double rpos = getdata.rpos + enc_ticks * getdata.rindex;



	double linc = (double) (llpos - lpos) * m_per_tick * lin_scale;
	double rinc = (double) (lrpos - rpos) * m_per_tick * lin_scale;
	//std::cout << "linc: " << (llpos - lpos) << ", rinc: " << (lrpos - rpos) << "\n";

	llpos = lpos;
	lrpos = rpos;
	if (odom_initialized == true) {
		apos -= (linc - rinc) / robot_axle_length * rot_scale;
		apos = ang_nor_rad(apos);
		double dist = (rinc + linc) / 2;

		xpos += dist * cos(apos);
		ypos += dist * sin(apos);

		if (_dump) {
			of << getdata.lpos << ";" << getdata.lindex << ";" << getdata.rpos << ";" << getdata.rindex << ";" 
				<< getdata.lvel << ";" << getdata.rvel << ";"
				<< apos << ";" << xpos << ";" << ypos << "\n";				
		}
	} else
		odom_initialized = true;
*/
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

void Protonek::getRawOdometry(double &linc, double &rinc) {
	int lpos = 0;//getdata.lpos + getdata.lindex * enc_ticks;
	int rpos = 0;//getdata.rpos + getdata.rindex * enc_ticks;

	linc = (lpos - llpos) * m_per_tick;
	rinc = (rpos - lrpos) * m_per_tick;

	llpos = lpos;
	lrpos = rpos;
}

bool Protonek::isConnected() {
	return connected;
}

Protonek::Orientation::Orientation() : accX(100), accY(100), accZ(100),
        pitch(0), oldPitch(0), updated(-100), updated2(-101) {
}

void Protonek::Orientation::update(int accX, int accY, int accZ, int g, double time) {

    printf("accY: %d    accZ: %d    g: %d\n",accY,accZ,g);
    double acceleration = sqrt((accY-311)*(accY-311) + (accZ-316)*(accZ-316))/145.0;

    pitch = (180.0*atan2((accY-311),(accZ-316))/M_PI) + (double)g*0.1;

    pitch2 = (180.0*atan2((accY-311)*acceleration,(accZ-316)/acceleration)/M_PI);//*0.5 + pitch2*0.5;

    dpitch = pitch2 + (pitch2-pitch)*1 - oldPitch;
    oldPitch = pitch2 + (pitch2-pitch)*1;

    this->accX = accX;
    this->accY = accY;
    this->accZ = accZ;
    gyro = g;

    updated2 = updated;
    updated = time;
}

bool Protonek::Orientation::isValid(double time) {
        return (time-updated)<0.1;
}

double Protonek::Orientation::getPitch() {
    return pitch;
    //return pitch2 + (pitch2-pitch)*2;
}

double Protonek::Orientation::getDPitch(double time) {
        return dpitch;
}

double Protonek::Orientation::getInterval() {
        return updated-updated2;
}

Protonek::Position::Position() : enc(0), pos(0), oldPos(0), dpos(0), updated(-100), updated2(-101) {
}

void Protonek::Position::update(int newPosition, double time) {
        if (enc>50000 && newPosition<10000) {	// przekrecilo sie w gore
                enc-=0x10000;
        } else if (enc<10000 && newPosition>50000) {	// przekrecilo sie w dol
                enc+=0x10000;
        }
        pos += enc - newPosition;
        enc = newPosition;
        updated2 = updated;
        updated = time;
}

bool Protonek::Position::isValid(double time) {
        return (time-updated)<0.1;
}

double Protonek::Position::getPosition() {
        return pos;
}

double Protonek::Position::getDPosition(double time) {
        dpos = pos - oldPos;
        oldPos = pos;
        return dpos;
}

double Protonek::Position::getInterval() {
        return updated-updated2;
}


/*
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <string.h>
#include <errno.h>
#include "protonek.hh"

using namespace std;

void Protonek::getJoystickState() {
    // read the joystick state
    read(joy.joy_fd, &joy.js, sizeof(struct js_event));

    // see what to do with the event
    switch (joy.js.type & ~JS_EVENT_INIT)
    {
            case JS_EVENT_AXIS:
                    joy.axis   [ joy.js.number ] = joy.js.value;
                    break;
            case JS_EVENT_BUTTON:
                    joy.button [ joy.js.number ] = joy.js.value;
                    break;
    }

    // os Y - przod/tyl
    // os Z - lewo/prawo
    double forward_speed = ((double)joy.axis[1]/32768.0);
    double rotation_speed = -((double)joy.axis[2]/32768.0)/2.0;

    // przycisk TURBO
    if (joy.button[7]==0) {
        forward_speed /= 2;
        rotation_speed /= 2;
    }

    // przycisk STOP
    if (joy.button[6]==0) {
        bridgeL.Run();
        bridgeR.Run();
    } else {
        bridgeL.Stop();
        bridgeR.Stop();
    }

    double command_rvel = forward_speed + rotation_speed;
    double command_lvel = forward_speed - rotation_speed;

    if (command_rvel>1)
                        command_rvel = 1;
    if (command_rvel<-1)
                        command_rvel = -1;

    if (command_lvel>1)
                        command_lvel = 1;
    if (command_lvel<-1)
                        command_lvel = -1;

    // print the results
//                printf( "X: %6d  Y: %6d  ", joy.axis[0], joy.axis[1] );

//        if( joy.num_of_axis > 2 )
//            printf("Z: %6d  ", joy.axis[2] );

//        if( joy.num_of_axis > 3 )
//            printf("R: %6d  ", joy.axis[3] );

//        for(int x=0 ; x<joy.num_of_buttons ; ++x )
//            printf("B%d: %d  ", x, joy.button[x] );
//        printf("\n");


    // B6 - stop
    // B7 - turbo
    //printf("%lf    %lf\n",command_lvel,command_rvel);
        //dev->SetPower(command_lvel, -command_rvel);
    SetVelocity(command_lvel, command_rvel);
}

void Protonek::Update()
{
//printf("%lf\n",(lspeed+rspeed)/2);
        static int error = 0;
        static float velocityA=0, velocityB=0;

        unsigned int ret = 0;

        // okreslenie jaki czas minal od ostatniego wywolania metody Update
        static double elapsed = 0;
        double interval = getInterval();	// interwal
        elapsed += interval;				// czas co jaki sprawdzamy czy nastapila przerwa w transmisji
        static double time = 0;
        time += interval;					// czas skumulowany

        getJoystickState();



        if (elapsed>0.01)
        {
                static bool manual = false;
                static double verticalTime = 0;

               // printf("%d\n",(int)isBalancing);

                if (fabs(orientation.getPitch()-90.0)>35) {
                        isBalancing = false;

                        verticalTime = 0;
                        bridgeL.setSpeed(lvel);
                        bridgeR.setSpeed(-rvel);
                        bridgeL.setSpeedPID(8,0,0);
                        bridgeR.setSpeedPID(8,0,0);

                        Send();
                } else {
                    isBalancing = true;

                    if (verticalTime < 0.5) {
                            Balance2(time,true);      // reset regulatorow balansowania
                        }
                        else {
                            Balance2(time,false);
                        }
                        bridgeL.setSpeedPID(8,0,0);
                        bridgeR.setSpeedPID(8,0,0);
                        verticalTime += interval;
                        Send();
                    }
                elapsed = 0;
        }

        Receive(time);
}


void Protonek::SetVelocity(double lvel, double rvel)
{
        if (lvel>1)
                lvel = 1;
        else if (lvel<-1)
                lvel = -1;
        this->lvel = lvel;

        if (rvel>1)
                rvel = 1;
        else if (rvel<-1)
                rvel = -1;
        this->rvel = rvel;
}

void Protonek::SetPower(double lcur, double rcur)
{
        if (lcur>1)
                lcur = 1;
        else if (lcur<-1)
                lcur = -1;
//	steering.lcur = lcur;

        if (rcur>1)
                rcur = 1;
        else if (rcur<-1)
                rcur = -1;
//	steering.rcur = rcur;
//	steering.lvel = steering.rvel = 0;
}


double Protonek::getPitch() {
    return orientation.getPitch();
}

double Protonek::getDPitch() {
    return orientation.getDPitch(0);
}

double Protonek::getOdomL() {
    return -123;
}

double Protonek::getOdomR() {
    return -123;
}


 */
