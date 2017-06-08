#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>

#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>

#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <unistd.h>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#define BAUDRATE B115200
#define _POSIX_SOURCE 1

#define FALSE 0
#define TRUE 1

#define NavDataFieldAt 6

typedef union _covL4 {
  long uLong;
  unsigned char bytes[4];
} covL4;
covL4 uLongitude, uLatitude, uHeightAE, uHeightASL;

typedef union _covU4 {
  unsigned long uLong;
  unsigned char bytes[4];
} covU4;
covU4 uHoriAccu, uVertAccu;

int timetodie = 0;
void sigproc(int param)
{ 
  signal(SIGINT, sigproc);
  timetodie = 1;
  printf("Got ctrl-c \n");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "receive_ublox");
  ros::NodeHandle nh;

  int fd, nRead, msgcount = 0;
  struct termios oldtio,newtio;
  char buf[1024];
  struct timeval tv;
  struct timeval old;

  signal(SIGINT, sigproc);

  std::string modeDevice;
  nh.param("ublox_port", modeDevice, std::string("/dev/serial/by-path/platform-3f980000.usb-usb-0:1.5:1.0"));
  fd = open(modeDevice.c_str(), O_RDWR);
  if (fd <0) {
    printf("\nCannot find GPS device at %s. \n\n", modeDevice.c_str());
    return(0); 
  }
  
  printf("\nGPS device opened.\n\n");
  tcgetattr(fd, &oldtio);
  bzero(&newtio, sizeof(newtio));
  
  newtio.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
  newtio.c_iflag &= ~(BRKINT | ICRNL | ISTRIP | IXON);
  newtio.c_cflag &= ~(CSIZE | PARENB);
  newtio.c_cflag |= CS8;
  newtio.c_oflag &= ~(OPOST);
  newtio.c_cc[VMIN] = 255;
  newtio.c_cc[VTIME] = 0;
  
  cfsetispeed(&newtio, BAUDRATE);
  cfsetospeed(&newtio, BAUDRATE); 
  
  tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &newtio);

  ros::Publisher gpspub = nh.advertise<sensor_msgs::NavSatFix> ("/gps/data", 1);
  sensor_msgs::NavSatFix gpsdata;
  gpsdata.header.frame_id = "gps";
  gpsdata.status.status = 0;
  gpsdata.status.service = 1;
  gpsdata.position_covariance_type = 1;

  int k = 0, head1 = 0, head2 = 0, classId = 0, pkgId = 0, navDataId = 0;
  char HEAD1 = 0xB5, HEAD2 = 0x62, CLASSID = 0x01, PKGID = 0x02;
  int toNavData = 0, bCount = 0;
  double longitude, latitude, heightAE, heightASL, horiAccu, vertAccu;

  while (!timetodie) {
    nRead = read(fd, buf, 255);
    if (nRead < 1) {
	printf("\nWarning read returned %d.\n\n", nRead);
	continue;
    } 

    k = 0;
HD: while (head1 != 1 && k < nRead) {
      if (buf[k++] == HEAD1) {
	head1 = 1;		
      }
    }

    if (head1 == 1 && head2 != 1 && k < nRead) {
      if (buf[k++] == HEAD2) {
	head2 = 1;		
      } else { 
	head1 = 0;
	goto HD;
      }
    }

    if (head1 == 1 && head2 == 1 && classId != 1 && k < nRead) {
      if (buf[k++] == CLASSID) {
	classId = 1;		
      } else {
	head1 = head2 = 0;
	goto HD;
      }
    }
     
    if (head1 == 1 && head2 == 1 && classId == 1 && pkgId != 1 && k < nRead) {
      if (buf[k++] == PKGID) {
	pkgId = 1;	
	toNavData = NavDataFieldAt;
      } else {
	head1 = head2 = classId = 0;
	goto HD;
      }
    }
     
    while (head1 == 1 && head2 == 1 && classId == 1 && pkgId == 1 
	   && navDataId != 1 && k < nRead) {
      while (k < nRead && toNavData > 0) {
	k++; toNavData--;
      }

      if (toNavData == 0) {
	navDataId = 1;
      } else {
	goto CNT;
      }
    }

    if (head1 == 1 && head2 == 1 && classId == 1 && pkgId == 1 
	&& navDataId == 1 && bCount != 24 && k < nRead) {
      if (bCount < 4) {
        do {
	  uLongitude.bytes[bCount++] = buf[k++];
        } while (k < nRead && bCount < 4);
      } 
  
      if (bCount >= 4 && bCount < 8) {
        do {
	  uLatitude.bytes[(bCount++) - 4] = buf[k++];
        } while (k < nRead && bCount < 8);
      } 

      if (bCount >= 8 && bCount < 12) {
        do {
	  uHeightAE.bytes[(bCount++) - 8] = buf[k++];
        } while (k < nRead && bCount < 12);
      }

      if (bCount >= 12 && bCount < 16) {
        do {
	  uHeightASL.bytes[(bCount++) - 12] = buf[k++];
        } while (k < nRead && bCount < 16);
      }

      if (bCount >= 16 && bCount < 20) {
        do {
	  uHoriAccu.bytes[(bCount++) - 16] = buf[k++];
        } while (k < nRead && bCount < 20);
      } 

      if (bCount >= 20) {
        do {
	  uVertAccu.bytes[(bCount++) - 20] = buf[k++];
        } while (k < nRead && bCount < 24);
      }

      if (bCount == 24) {
	longitude = (double)uLongitude.uLong / 10000000.0;
	latitude = (double)uLatitude.uLong / 10000000.0;
	heightAE = (double)uHeightAE.uLong / 1000.0;
	heightASL = (double)uHeightASL.uLong / 1000.0;
	horiAccu = (double)uHoriAccu.uLong / 1000.0;
	vertAccu = (double)uVertAccu.uLong / 1000.0;

        gpsdata.header.stamp = ros::Time::now();
        gpsdata.longitude = longitude;
        gpsdata.latitude = latitude;
        gpsdata.altitude = heightAE;

        gpsdata.position_covariance[0] = horiAccu * horiAccu / 2;
        gpsdata.position_covariance[4] = horiAccu * horiAccu / 2;
        gpsdata.position_covariance[8] = vertAccu * vertAccu;

        gpspub.publish(gpsdata);

	head1 = head2 = classId = pkgId = navDataId = bCount = 0;
	goto HD;
      }
    }

CNT:gettimeofday(&tv, NULL);
    msgcount++;
    
    if (!(msgcount % 100)) {
      old = tv;
      fflush(0);
    }
  }

  tcsetattr(fd, TCSANOW, &oldtio);
  return(0);
}

