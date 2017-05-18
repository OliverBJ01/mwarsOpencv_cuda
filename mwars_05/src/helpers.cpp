/*
 * helpers.cpp
 *
 *  Created on: 30 Dec 2015
 *      Author: bernard
 */


#include "mwars.hh"

extern bool gshowCalRegion;	// set by button callback onButton() to display cal region
extern bool gLaserEnable;
extern bool gdisableEngagement;
extern uint xx, yy;											//co-ords send to scanner


//-----------------------------------------------------------------------------------------------
//  void laserThread(void *arg)
//
//  1. initLaserEnableTimer() raises a signal @ 3Hz rate which sets flag gLaserEnable
//  2. when an engageable track is detected, flag gEngage is set
//  3. when gLaserEnable&&gEngage true, laserThread() is forked
//  4. laserThread() turns off gEngage, turns on laser, waits, turns off laser
//  5. starts over at step 1.
// -----------------------------------------------------------------------------------------------
void *laserThread(void *arg)  {

	 //  printf("pthread:  %d %d  \n", xx,  yy);
	 gLaserEnable = false;		// turned off until raised again by initLaserEnableTimer()	 	 	 	 	 	 	 	 	 	 	 	//
	 sendMssg(xx, yy, true);
	 usleep (500000);				// in uS    i.e. 333mS
	 sendMssg(xx, yy, false);	// because xx, yy are global, this  turns
	 	 	 	 	 	 	 	 	 	 	 	 	// off laser at current scanner position
	 pthread_exit (NULL);
}



//-----------------------------------------------------------------------------------------------
//  static void signalrmHandler(int sig)
//
//   invoked by signal SIGALRM raised by timer in initLaserEnableTimer()
// -----------------------------------------------------------------------------------------------
static void signalrmHandler(int sig) {
	// cout << "signal received "  << endl;
	gLaserEnable = true;
}

// -------------------------------------------------------------------------------------------------------------------
//  int initLaserEnableTimer(void)
//
//  runs at low frequency; raises signal SIGALRM which invokes signal handler signalrmHandler()
//  which sets flag gLaserEnable
//-------------------------------------------------------------------------------------------------------------------
int initLaserEnableTimer(void)  {

	struct itimerval itv;			// interval timer structure
	struct sigaction sa;			// signal structure

	sigemptyset (&sa.sa_mask);
	sa.sa_flags = 0;
	sa.sa_handler = signalrmHandler;

	// SIGALARM generated on expiry of real-time timer
	if (sigaction (SIGALRM, &sa, NULL) == -1) {
			cerr << "unable to create signal handler: " << strerror(errno) << endl;
			return 1;
	}

	itv.it_interval.tv_sec = 0;					// interval period
	//	itv.it_interval.tv_usec = 33000;	// 33mS   30Hz
	//	itv.it_interval.tv_usec = 66000;	// 66mS   15Hz
    //	itv.it_interval.tv_usec = 20000;	// 20mS   50Hz
	//itv.it_interval.tv_usec = 50000;		// 50mS   20Hz
	//itv.it_interval.tv_usec = 500000;	// 500mS   2Hz
	itv.it_interval.tv_usec = 333000;		// 333 mS   3Hz
	//itv.it_interval.tv_usec = 200000;		// 200mS  5Hz

	itv.it_value.tv_sec = 1;						// initial timeout period
	itv.it_value.tv_usec = 0;

	if (setitimer (ITIMER_REAL, &itv, 0) == -1) {
		cerr << "unable to create timer: " << strerror(errno) << endl;
			return 1;
	}
	return 0;
}





//-----------------------------------------------------------------------------------------------
//  void showCalRectangle(int state, void*)
//
//   button bar callback that sets flag to display calibration rectangle
// -----------------------------------------------------------------------------------------------
void showCalRectangle(int state, void*) {
	if (state) {
		gshowCalRegion = true;}
	else {
		gshowCalRegion = false; }
	 cout << "click " << state  << gshowCalRegion << endl;
}

//-----------------------------------------------------------------------------------------------
//  void disableEngagement(int state, void*)
//
//   button bar callback that sets flag to disable engagement
// -----------------------------------------------------------------------------------------------
void disableEngagement(int state, void*) {
	if (state) {
		gdisableEngagement = true;}
	else {
		gdisableEngagement = false; }
	 cout << "click " << state  << gshowCalRegion << endl;
}



// -------------------------------------------------------------------------------------------------------------------
//  int getCameraCalibrationData(string camCalFile, Mat &cameraMatrix, Mat &distCoeffs))
//-------------------------------------------------------------------------------------------------------------------
void displayCalibrationLines(IplImage  & IplImage, int frameWidth, int frameHeight)   {

	// draw cross at boresight
	//   draw_cross1 (cvarrToMat(& IplImage), CvPoint( (frameWidth)/2, (frameHeight)/2), CV_RGB(255,0,0), 20);			// red cross

		// vert and horz lines through centrepoint
	   line (cvarrToMat(& IplImage), Point(0, frameHeight/2), Point(frameWidth, frameHeight/2), Scalar(0,0,255));
	   line (cvarrToMat (& IplImage), Point(frameWidth/2, 0), Point(frameWidth/2, frameHeight), Scalar(0,0,255));

	   //draw frame at frame extent
	   rectangle(cvarrToMat(& IplImage), Point(0, 0), Point(frameWidth-1, frameHeight-1), Scalar(0,0,255));

	   // draw frame at extents/2
	   rectangle(cvarrToMat(& IplImage), Point(frameWidth/4, frameHeight/4),
			   	   Point(frameWidth*3/4-1, frameHeight*3/4-1), Scalar(0,0,255));
}


// -------------------------------------------------------------------------------------------------------------------
//  int getCameraCalibrationData(string camCalFile, Mat &cameraMatrix, Mat &distCoeffs))
//
// undistort points process:
// Previously have run the camera calibration routine which saves camera instrinics into .xml file
// 1. read camera intrinsics .xml file saved by camera calibration app
// 1.1 cameraMatrix: (double) 3 x 3 with focalX/Y and principalX/Y are in the (3x3) camera matrix in
//			fX 0 pX
//			0 fY pY
//   		0 0 1
// 1.2 distCoeffs:  (double) 5 x 1 array
// 2.  undistortPoints( src, dst, cameraMatrix, distCoeffs )
// 		The undistorted points are relative to the center of the camera lens relative to the sensor
// 		(the principal point) and normalized relative to the focal distance in x and y:
//			 i.e. undistortPoints() moves the points so that the centre of the image is 0,0 and scales everything so that the width and height are both 1 unit.
//			 if my image is 640*480 the coordinate locations are : (320+640*x_value,240+480*y_value);
//			(refer: https://forum.openframeworks.cc/t/undistortpoints-opencv/4192/3)
//	 3.  convert back to 0,0 at top left corner with:
//					undistorted.x = (x* focalX + principalX);  and undistorted.y = (y* focalY + principalY);
//					i.e.
//			   		uint xx = (dst.at<Vec2d>(0,0) [0] * cameraMatrix.at<double>(0, 0) + cameraMatrix.at<double>(0, 2));
//			   		uint yy = (dst.at<Vec2d>(0,0) [1] * cameraMatrix.at<double>(1, 1) + cameraMatrix.at<double>(1, 2));
//-------------------------------------------------------------------------------------------------------------------
int getCameraCalibrationData(Mat &cameraMatrix, Mat &distCoeffs) {

	FileStorage fs (CAMCALFILE, FileStorage::READ);
	if (fs.isOpened()) {
		fs ["camera_matrix"] >> cameraMatrix;
		fs ["distortion_coefficients"] >> distCoeffs;
		fs.release();
		cout << "Read camera calibration file: " << CAMCALFILE << endl;
		//cout << "camera matrix: " << cameraMatrix << endl;
		//cout << "distortion coefficients: " << distCoeffs << endl;
		return 0;
	}
	else  {
		cerr << "unable to open camera calibration file: " << strerror(errno) << endl;
		return 1;
	}
}


//=============================================
// int cameraCapture( VideoCapture &cap, string camType, int usbDeviceNo, string ipCamStreamAddress)
//
//  establishes camera capture from USB or IP cameras
// 	camType: "USB" or "IP"
// 	for USB cam: usbDeviceNo: generally 0, or 1 with 2 cameras
// 	for IP cam: 	ipCamStreamAddress: stream address
//			e.g.  "http://root:Isabel00@10.1.1.69/mjpg/video.mjpg"	//Axis IP Camera @ Port 80
//										root  PW: usual
//					 "http://admin:Isabel00@10.1.1.5/video.cgi?.mjpg"  	// D-Link IP camera @ Port 80
//=============================================
int cameraCapture( VideoCapture &cap, string camType, int usbDeviceNo, string ipCamStreamAddress,
			int &frameWidth, int &frameHeight)   {

	// IP camera capture
	if (camType == "IP") {
		cap.open(ipCamStreamAddress);

		if (!cap.isOpened())       {
			cerr << "Cannnot initialise IP camera:" << strerror(errno) << endl;
			return -1;
		}
	}
	// USB camera capture
    else  if (camType == "USB")  {
		cap.open(usbDeviceNo);

		if(!cap.isOpened()) {
			cerr << "Cannnot initialise USB camera: " << usbDeviceNo  << strerror(errno) << endl;
			return 1;
		}
    }
	// bad camera type
    else  {
        cerr << "bad camera parameter; " << camType << "use USB or IP"  << endl;
        return 1;
    }

	// falls through if OK
	cout << "camera initialised" << endl;

	frameWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
	frameHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	cout << "frame width: " << frameWidth << " frame height: " << frameHeight <<endl;

	return 0;
    }
//VideoCapture cap(videoStreamAddress);		// IP Cam
//int process(VideoCapture& cap);


//=============================================
//
// int openLogFile(ofstream &logfile, string logFileName)
//
//  log file for whatever
//=============================================
int openLogFile(ofstream &logfile, const char * logFileName)  {

	logfile.open (logFileName);
	if (logfile.is_open()) {
		  cout << "Opened logfile: " << logFileName << endl;
		  return 0;
	  }
	  else {
		  cerr << "Cannnot open log file: " << logFileName  << strerror(errno) << endl;
		  return 1;
	  }
}



// -------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------
void helpKalman(void) {
    printf("\n");
}

