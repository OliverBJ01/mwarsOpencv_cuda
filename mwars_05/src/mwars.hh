/*
 * mwars02.hh
 *
 *  Created on: 29/05/2014
 *      Author: bernard
 */

#ifndef MWARS_HH_
#define MWARS_HH_

/*
 * MOG2_kalman_01.h
 *
 *  Created on: 22/05/2014
 *      Author: bernard
 */
/* ---------------------------------------  tracks ---------------------------------------------------
 * 					              ++------+----------++
 * tracks is a STL map container: || CvID | CvTrack* ||
 * 					              ++------+----------++
 * struct CvTrack  {
    CvID id; ///< Track identification number.
    CvLabel label; 			///< Label assigned to the blob related to this track.
    unsigned int minx; 		///< X min.
    unsigned int maxx; 		///< X max.
    unsigned int miny; 		///< Y min.
    unsigned int maxy; 		///< y max.
    CvPoint2D64f centroid; 	///< Centroid.
    unsigned int lifetime; 	///< Indicates how much frames the object has been in scene.
    unsigned int active; 	///< Indicates number of frames that has been active from last inactive period.
    unsigned int inactive; 	///< Indicates number of frames that has been missing.
  };
    typedef std::map<CvID, CvTrack *> CvTracks;    // map container
    CvTracks tracks;
    ----------------------------------------------------------------------------------------------------
 */
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgcodecs.hpp>

#include "../cvblob/cvblob.h"

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <fstream>
#include <iostream>
#include <errno.h>
//#include <string>

#include <signal.h>
#include <sys/time.h>
#include <time.h>

#include <pthread.h>
#include <unistd.h>			// for usleep()

using namespace cvb;		// for cvBlobs
using namespace std;		// std library
using namespace cv;			// opencv

// xml file with the camera calibration matrices previously saved by CameraCalibration App
#define CAMCALFILE "/home/bernard/ws/mwarsOpencv/cameraCalibrate/out_camera_data.xml"

// IP cam stream addresses
//#define IPCAM_STREAMADDRESS "http://root:Isabel00@10.1.1.5/mjpg/video.mjpg"	//Axis IP Camera @ Port 80
#define IPCAM_STREAMADDRESS "http://admin:Isabel00@10.1.1.5/video.cgi?.mjpg"  	// D-Link IP camera @ Port 80

#define LOGFILE "tracklog_01.txt"

// ------- logitech cam -----------
// --------------------------------
// a. logitech camera frame size in pixels
#define CAMERA_X_MIN 0
#define CAMERA_X_MAX 640    //1280
#define CAMERA_Y_MIN 0
#define CAMERA_Y_MAX 480    //960

#define draw_cross1( frame, center, color, d )                                 		\
   line( frame, cvPoint( center.x - d, center.y - d ),                \
                cvPoint( center.x + d, center.y + d ), color, 1, CV_AA, 0);\
   line( frame, cvPoint( center.x + d, center.y - d ),                \
                cvPoint( center.x - d, center.y + d ), color, 1, CV_AA, 0 )

// old version
//#define draw_cross( iplFrame, center, color, d )                                 		\
//   cvLine( &iplFrame, cvPoint( center.x - d, center.y - d ),                \
//                cvPoint( center.x + d, center.y + d ), color, 1, CV_AA, 0);\
//   cvLine( &iplFrame, cvPoint( center.x + d, center.y - d ),                \
//                cvPoint( center.x - d, center.y + d ), color, 1, CV_AA, 0 )

void displayCalibrationLines(IplImage  & frame, int frameWidth, int frameHeight);

void printKalmanArrays (cv::Mat & measurementNoiseCov, cv::Mat & errorCovPost, cv::Mat & errorCovPre, cv::Mat & processNoiseCov);
void tbarFunction(int tbPosn);
void buttonFunction(int state, void *a);

//int readCameraParams(const string& cameraFileName, Mat& cameraMatrix, Mat& distCoeffs );
int 	getCameraCalibrationData( Mat &cameraMatrix, Mat &distCoeffs);

int cameraCapture( VideoCapture &cap, string camType, int usbDeviceNo, string ipCamStreamAddress,
			int &frameWidth, int &frameHeight);
//int openLogFile(ofstream &logfile, string logFileName);
int openLogFile( ofstream &logfile, const char * logFileName);
// UDP soket routines
void sendMssg(uint x, uint y, bool laserOn) ;
int initSocket(void );
void showCalRectangle(int state, void*);
void disableEngagement(int state, void*);
int initLaserEnableTimer(void) ;
//static void signalrmHandler(int sig);

void *laserThread(void *arg);





#endif /* MWARS_HH_ */
