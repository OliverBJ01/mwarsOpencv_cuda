
/*
 *  mwars_03    11/01/2016
 *
 *  CTRL P to present calibrate button to present X at boresight and extent frames for laser alignment
 *
 *  Camera calibration uses camera intrinics file saved by camera calibration routine
 *  at samples/cpp/tutorial_code/calib3d/camera_calibration/
 *
 *
//  1. initLaserEnableTimer() raises a signal @ 3Hz rate which sets flag gLaserEnable
//  2. when an engageable track is detected, flag gEngage is set
//  3. when gLaserEnable&&gEngage true, laserThread() is forked
//  4. laserThread() turns off gEngage, turns on laser, waits, turns off laser
//  5. starts over at step 1.
 *
 *
 */


#include "mwars.hh"

bool gLaserEnable = false;			// set locally from ggLaserEnable
bool gEngage = false;					// set when target is to be engaged, results in laser turn on command
bool gshowCalRegion = false;		// set by button callback onButton() to display cal region
bool gdisableEngagement;			// set by button bar to disable any engagement
uint xx, yy;											//co-ords send to scanner



//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
//  int main()
// -----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------

int main(int argc, char** argv)
{

	pthread_t lt;
	int s;

	Mat cameraMatrix, distCoeffs;     // camera calibration matrices
	Mat frame;						// the live image  (3 channels)
    Mat bgMask;   					// foreground image  ( 3 channels)
    Mat fgMask;   					// foreground mask  ( 1 channel)
    Mat backGnd; 					//background image  (3 channels)
    IplImage  iplFrame;			// iplImage copy of (Mat) frame for cvBlobs use
    IplImage * iplbgMask;		// ptr to iplImage copy of (Mat)  for cvBlobs use
    IplImage * imgLabel;		// cvBlob's cvLabel's output image matrix
	CvTracks tracks;				//cvBlob's map of tracks

   VideoCapture cap;			// opencv video capture class
   ofstream logfile;				// logfile handle

   // camera frame size read during videocapture
   int frameWidth, frameHeight;

    // ----------------  Kalman stuff  ----------------------------------
    //standard Kalman filter (in G. Welch' and G. Bishop's notation):
    //  x(k)=A*x(k-1)+B*u(k)+w(k)  p(w)~N(0,Q)
    //  z(k)=H*x(k)+v(k),   p(v)~N(0,R)
    //
    // Newton: x = x0 + v.t + 1/2.a.t^2
    // y(t) = y +y'(t) + 1/2.y''(t)

    // x = state vector = {X, Y, Vx, Vy, Ax, Ay}
    // x_pred (predicted state) = x x F (transition Matrix) + Noise Covariance

    /*

    KalmanFilter KF(6, 					// No. of state vector dimensions
                    						2,					// No. of measurement vector dimensions
                    						0);				// No. control vector dimensions
    KF.transitionMatrix = *(Mat_<float>(6,6) << 1, 0, 1, 0,  0, 0,
                                                0, 1, 0, 1,  0, 0,
                                                0, 0, 1, 0,  1, 0,
                                                0, 0, 0, 1,  0, 0,
                                                0, 0, 0, 0, .5, 0,
                                                0, 0, 0, 0, 0, .5);
    KF.measurementMatrix = *(Mat_<float>(2,6) << 1, 0, 0, 0, 0, 0,
                                                0, 1, 0, 0, 0, 0);
    Mat measurement = Mat::zeros( 2, 1, CV_32FC1 );			// Measurement(2,1,CV_32FC1): {X, Y} for kalman

    // Noise
    // 1. Process Noise Covariance is not updated by the filter. Set by kalman constructor to identity matrix.
    //    zero process noise ??
    // 2. Measurement Noise Covariance is not updated by filter. Set by kalman constructor to identity matrix.
    // 	  If initial position is unknown, the measurement noise cov matrix should be initialised with large
    //    values on diagonal so the filter will prefer info from first measurement over information in the model
    // 3. errorCovPre and errorCovPost are updated by the filter. Initialised by kalman constructor to all zeros

// THESE NEED TUNING..........
    // setIdentity(KF.errorCovPost, Scalar::all(1));						// kalman example value
    // setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));	// kalman example value
    setIdentity(KF.measurementNoiseCov, Scalar::all(5));		// what is 'large' ? MATLAB uses 1000, but causes lag here*************************************************************************
    setIdentity(KF.processNoiseCov, Scalar::all(1));					// something small

*/


    // ---------  timer for controlling and measuring frame rate ---------------------
    constexpr float frameRate  {28};  				// required frame rate
    constexpr float framePeriod {1000/frameRate};	// required frame period in mS
    double tickCount;											// tick count per loop
    double fps, sleepTime, execTime;
    double tickFreq = getTickFrequency();		// gives ticks per second  (10^9)
    unsigned long nFrames = 0;							// frame counter, No. frames

   // unsigned long xCount = 0;

 //  ---------------------------- MOG2 Stuff -------------------------------------
    // BackgroundSubtractorMOG2's initialisation params/(100, 3, 0.3, 5);
  //  const int nmixtures =3;
    //const bool bShadowDetection = false;
 //   const int history = 5;

    // ------------------------  cvBlob parameters -------------------------------------
    //cvFilterbyArea() params
    int minBlobArea = 50;				//adjusted by slider
    int maxBlobArea = 3000;			//adjusted by slider

    //  cvUpdateTracks()    tracking params
    double distance = 200.;			// Max distance to determine when a track and a blob match.
    unsigned int inactive = 5;		// Max number of frames a track can be inactive.
    unsigned int active = 2;			// If a track becomes inactive but it has been active  // default = 0
    													// less than Active frames, the track will be deleted.


    // ---------------------- morphology parameters --------------------------------
 	int threshold_value = 120.;		// init slider value, adjusted by slider

    // create track bar window and track bars to adjust parameters
    namedWindow ("TrackBarWnd", WINDOW_NORMAL);

   // morph thresholding trackbar
   createTrackbar( "morphTHold", "TrackBarWnd", &threshold_value, 255, nullptr);

   // trackbar sets min/max blob area for cvFilterByArea()
   createTrackbar( "minblobarea", "TrackBarWnd",  &minBlobArea, 300, nullptr);
   createTrackbar( "maxblobarea", "TrackBarWnd", &maxBlobArea, 2000, nullptr);

   createButton("Show Cal Rectangle", showCalRectangle, NULL, QT_CHECKBOX, 0);
   createButton("Disable Engagement", disableEngagement, NULL, QT_CHECKBOX, 0);




   // ********************** STARTS HERE ***********************************
   if (openLogFile (logfile, LOGFILE))
	   return 1;
   if (getCameraCalibrationData(cameraMatrix, distCoeffs) )	 	// load camera intrinsics files
	   return 1;
   if ( initSocket())	    											// socket: UDP packets to scanner Console app
	   return 1;
   if (initLaserEnableTimer())
	   return 1;

   tickCount = (double)getTickCount();								// start timer for frame rate timing

   if (cameraCapture (cap, "IP", 0, IPCAM_STREAMADDRESS, frameWidth, frameHeight ))    // establish videocapture with USB or IP camera
	   return 1;
   cap >> frame;

   imgLabel = cvCreateImage(frame.size(), IPL_DEPTH_LABEL, 1);		// for cvLabel later

   Ptr<BackgroundSubtractorMOG2> bg_model = createBackgroundSubtractorMOG2();
   bg_model->setVarThreshold(10);


   // ********************** MAIN PROCESSING LOOP ***********************************
    for(;;)      {

    	cap >> frame;
        if( frame.empty() )   break;

        Mat gekkoHunt = frame.clone();

        //update the MOG2 model
        bg_model->apply(frame, bgMask);				// background Mask = 1 channel grey scale

        //namedWindow("fgMask", WINDOW_NORMAL);
        //imshow("fgMask", bgMask);

        //bg_model->getBackgroundImage(backGnd);		// backGnd = 3 channels
        //namedWindow("background frame", WINDOW_NORMAL);
        //imshow("background frame", backGnd );


        // this is not used; not sure why I want foreground in colour. Info to to categorise the target?
        // use/ fgMask to extract 3 channel bgMask
        //bg_model->apply(frame, fgMask);				// foreground Mask = 1 channel
        //bgMask = Scalar::all(0);
        //frame.copyTo(bgMask, fgMask);			// only masked entries copied from frame-> bgMask
        //namedWindow("bgMaskWnd", WINDOW_NORMAL);
        //imshow("bgMaskWnd", bgMask);

		//*************************** Colour Reduction ************************************
		// cvBlob works with monochrome
		//cvtColor(bgMask, bgMask, COLOR_BGR2GRAY);		// COLOR_BGR2GRAY
		  //  ----      OR   ------  (doesn't work) to allow colour selection
		 /*  // split image into the colour planes
		vector<Mat> planes;
		split( bgMask, planes );
		Mat b = planes[0] ,  g = planes[1], r = planes[2];
		// Accumulate separate planes, combine and threshold
		Mat s = Mat::zeros( b.size(), CV_32F);
		accumulate( b, s );
		accumulate( g, s );
		accumulate( r, s );
		// Truncate values above 100 and rescale into dst
		threshold( s, s, 100, 100, THRESH_TRUNC );
		s.convertTo(bgMask, b.type());
		*/

        // threshold the greys in bgMask, this removes them but slider works strange?
        threshold(bgMask, bgMask, (double)threshold_value, 255, CV_THRESH_BINARY); // 0: CV_THRESH_BINARY

        // *************************** **Morphology  **************************************
        //change morphological close value (25) to another value to get a solid blob
        Mat element(5, 5, CV_8U, Scalar(1));
        morphologyEx(bgMask, bgMask, MORPH_OPEN, element);	// dilation then erosion
        morphologyEx(bgMask, bgMask, MORPH_CLOSE, element);  // erosion then dilation

       // namedWindow("frame", WINDOW_NORMAL);
       // imshow("frame", frame);
       namedWindow("morphed bgMask", WINDOW_NORMAL);
       imshow("morphed bgMask", bgMask);			//morphed foreground mask

        // ********************** cvBlobs: Connected Components Section  **********************
       CvBlobs blobs;							// map of blobs, needs to stay in the loop

       iplbgMask = new IplImage(bgMask);		// ptr to iplImage copy of bgMask for cvBlobs use

       // labels the connected parts (blobs).
       // Returns: a) imgLabel image of labels, b) an stl map of blobs
       cvLabel(iplbgMask, imgLabel, blobs);
       //namedWindow("imgLabel", WINDOW_NORMAL);
       //imshow("imgLabel", cvarrToMat(imgLabel));

       // filter blobs by size
       cvFilterByArea(blobs, (unsigned int)minBlobArea, (unsigned int)maxBlobArea);


       //void cvFilterBlobsByColour(CvBlobs &blobs, IplImage const *imgLabel, Mat &frame);

        // filter blobs by colour, reject red
       //cvFilterBlobsByColour(blobs, imgLabel, frame);

       iplFrame = frame;						// iplImage shallow copy of (Mat) frame for cvBlobs use
       cvRenderBlobs(imgLabel, blobs, &iplFrame, &iplFrame, CV_BLOB_RENDER_BOUNDING_BOX);

        //unused cvBlob functions to consider: : cvsetregionof interest(), cvrenderblob() ,colour()
        //namedWindow("Blobs_Connected", CV_GUI_EXPANDED);
        //imshow("Blobs_Connected", cvarrToMat(&iplFrame));



        // ************************* cvBlobs: Tracking Section  ******************************
        // cvUpdateTracks()
        //	brief Updates list of tracks based on current blobs.
        // 		param: List of blobs - input
        // 		param: List of tracks - updated
        // 		param: Max distance to determine when a track and a blob match.
        // 		param: Max number of frames a track can be inactive.
        // 		param: If a track becomes inactive but it has been active less than thActive frames, the track will be deleted.
       cvUpdateTracks(blobs, tracks, distance, inactive, active);

       // namedWindow ( "Blobs Tracking Wnd",  CV_GUI_EXPANDED);
      //  imshow("Blobs Tracking Wnd", cvarrToMat(&iplFrame));

      // line( cvarrToMat(&iplFrame),  CvPoint(10, 20), CvPoint(40, 50), CV_RGB(255,0,0), 10, 2 );

	   // if gshowCalRegion flag set, set calibration lines on image
       // these are used by scanner server to align camera and scanner, and scale camera pixels to
       // scanner units
	   if (gshowCalRegion)
		   	   displayCalibrationLines( iplFrame, frameWidth, frameHeight);

       // ------------------------------ Path 1: don't engage when there are between 1 and 3 tracks ----------------------------
       if ((tracks.size() == 0)||(tracks.size() > 3)  ) {
			gEngage = false;
			gLaserEnable = false;
			//printf(" Path 1: %d %d %d \n", gEngage, gLaserEnable, laserOnCount);
		}

       else   {
		   //-----------------------------Path 2: possibly engage when ther are 1 to 3 tracks ----------------------------------
			//cvRenderTracks()															//CvTracks is pair: {CvID, CvTrack}
			//	brief Prints tracks information.
			// 		param: List of tracks input
			// 		param: Input image (depth=IPL_DEPTH_8U and num. channels=3).
			// 		param: Output image (depth=IPL_DEPTH_8U and num. channels=3).
			// 		param: Render mode. By default is CV_TRACK_RENDER_ID|CV_TRACK_RENDER_BOUNDING_BOX.
			// 		param: OpenCV font for print on the image.
			// 				|CV_TRACK_RENDER_TO_STD to print track stats
			cvRenderTracks(tracks, &iplFrame, &iplFrame, CV_TRACK_RENDER_ID|CV_TRACK_RENDER_BOUNDING_BOX);

			//  Select the track with longest lifetime
			// searches each key/value pair in tracks (STL map) and returns iterator pointing to longest lifetime track
			CvTracks::const_iterator iter, iterMax;
			unsigned int maxLifetime = 0;
			for ( iter = tracks.begin(); iter != tracks.end(); ++iter) {  // scan each track and select track with longest lifetime
					if ((iter->second->lifetime) > maxLifetime) {			// Note iter = tracks.end() at end of this loop, and if used to
							maxLifetime = iter->second->lifetime;			// access an element causes error-less crash
							iterMax = iter;														// iterMax points to longest duration track
					}
			}
			// obtain centroid and lifetime of longest lifetime  track
			CvPoint2D64f trackCentroid = iterMax->second->centroid;
			//unsigned int lifetime = iterMax->second->lifetime;
			//cout << "Lifetime: " << lifetime  <<" Centroid:  "  << (int)trackCentroid.x << ", " << (int)trackCentroid.y  <<endl;

			// display track as white cross
			draw_cross1(cvarrToMat(&iplFrame), cvPoint( (int) trackCentroid.x, (int) trackCentroid.y), CV_RGB(255,255,255), 10);

			// ********************************* engagement  *********************************
			// using the longest lifetime track
			#define MIN_TRACK_LIFETIME_TO_ENGAGE 15


			if (maxLifetime < MIN_TRACK_LIFETIME_TO_ENGAGE) {
					   draw_cross1(cvarrToMat(&iplFrame),  cvPoint( (int) trackCentroid.x, (int) trackCentroid.y), CV_RGB(0,255,0), 10);
					//	draw_cross(cvPoint( (int)measurementCoord.x, (int)measurementCoord.y), CV_RGB(0,255,0), 7);	// green cross to rawImage
					  gEngage = false;																	// laser off
					  //cout << "laser "  << gEngage  << endl;
					   cvCircle( &iplFrame,cvPoint(20, 20), 6, CV_RGB(0, 255, 0), 4, CV_AA, 0);
			}
			else {
					   draw_cross1(cvarrToMat(&iplFrame),  cvPoint( (int) trackCentroid.x, (int) trackCentroid.y), CV_RGB(255,0,0), 10);;	// red cross
					   //draw_cross(cvPoint( (int)measurementCoord.x, (int)measurementCoord.y), CV_RGB(255,0,0), 7);	// red cross

					  gEngage = true;																		// laser on
					  //cout << "laser "  << gEngage  << endl;
					  cvCircle( &iplFrame,cvPoint(20, 20), 6, CV_RGB(255,0,0), 4, CV_AA, 0);

					 // Point r_min = Point (iterMax->second->minx, iterMax->second->miny );
					 // Point r_max = Point (iterMax->second->maxx, iterMax->second->maxy);
					  //cvRectangle (&iplFrame,  r_min, r_max,  CV_RGB(0,255,0), 3, 8 );

					  namedWindow ("gekkoWindow", WINDOW_NORMAL);

					  // 10 pixel border, but exceeding frame is bad
					  int x_min = iterMax->second->minx -10;
					  if (x_min < 0)
						  x_min = x_min + 10;
					  int y_min =  iterMax->second->miny  - 10;
					  if (y_min < 0)
						  y_min = y_min + 10;
					  int width = iterMax->second->maxx -  iterMax->second->minx + 10;
					  if ( width > CAMERA_X_MAX)
						  width = width - 10;
					  int height = iterMax->second->maxy -  iterMax->second->miny + 10;
					  if ( height > CAMERA_Y_MAX)
						  height = height - 10;
					  Mat roi = gekkoHunt (Rect (x_min,   y_min, width,  height));

					  imshow ("gekkoWindow", roi);

			}

			// ************************** Correct Camera Distortion  *********************************
			// uses camera intrinics file save by camera calibration routine
			Mat dst;
			//trackCentroid.x  = 111.1;  	trackCentroid.y = 222.2;

			// convert Point trackCentroid into Mat for undistortPoints()
			Mat_<Vec2d> src (1, 1, (Vec2d(trackCentroid.x, trackCentroid.y)));
			//Mat_<Point2d> src (1, 1, (Point2f (111.1, 222.2)));
			//cout << "src " << src.at<Vec2d>(0,0) [0] << endl;  cout << "src " << src.at<Vec2d>(0,0) [1] << endl;

			undistortPoints( src, dst, cameraMatrix, distCoeffs );
			//cout << "dst " << dst.at<Vec2d>(0,0) [0] << endl;  cout << "dst " << dst.at<Vec2d>(0,0) [1] << endl;

			// translate undistorted points back to Mat co-ordinates
		   xx = (dst.at<Vec2d>(0,0) [0] * cameraMatrix.at<double>(0, 0) + cameraMatrix.at<double>(0, 2));
		   yy = (dst.at<Vec2d>(0,0) [1] * cameraMatrix.at<double>(1, 1) + cameraMatrix.at<double>(1, 2));
		   //cout << "xx " << xx << endl;  cout << "yy " << yy << endl;

		   // ************************** send to scanner   *********************************
		   if (gLaserEnable && gEngage && !gdisableEngagement)  	{		// ENGAGED & ENABLED; laser ON
				   //printf("Path 2:  %d %d  \n", xx,  yy);

				   s = pthread_create (&lt, NULL, laserThread,   (void *)",,");		// turn laser on for period
				   if ( s )  {
						cerr << "Cannnot create laser pthread"  << strerror(errno) << endl;
						return 1;
					}
			 }
		   else
				 sendMssg(xx, yy, false);																	// track target only

       }   // ***************** Path1 and Path 2 close out here           ***************
       	   // thread holding laser on continues until it times out


       namedWindow ( "BlobsTrackingWnd",  WINDOW_NORMAL);
	   imshow("BlobsTrackingWnd", cvarrToMat(&iplFrame));

            //logFile << "Track#: " << longestTrackID << " Lifetime: " << maxLifetime << " Target: " << target.at<float>(0) << " " << target.at<float>(1) <<  endl;
            //co ut << "Track#: " << longestTrackID << " Lifetime: " << maxLifetime << " Target: " << target.at<float>(0) << " " << target.at<float>(1);
            //cout << " fps: " << fps;
            //cout  << " blob min: " << minBlobArea << " blob max: " << maxBlobArea;
            //cout << endl;




            // ********************************* Kalman tracking  *********************************
            // on event {					// kalman tuning proposal...............
            //    setIdentity(KF.measurementNoiseCov, Scalar::all(nn));
            //	  setIdentity(KF.processNoiseCov, Scalar::all(nn));
            // }
            // predict()
            //	brief:  does kalman prediction, mucks around with numerous matrices
      //      Mat prediction = KF.predict();

            //measurement = corTarget.reshape(1, 2);						// from 1x2 -> 2x1

            // correct()
            //	brief:  does kalman correction
            // Measurement(2,1,CV_32FC1): {X, Y} for kalman
            // 		param: x & y points in measurement *(Mat_<float>(2,6)  // Measurement: {X, Y} for kalman
            //KF.correct(measurement);
            // get measurement coordinates from prediction matrix
            //Point2f measurementCoord = Point2f(prediction.at<float>(0), prediction.at<float>(1));
           //  if (0) printKalmanArrays( KF.measurementNoiseCov, KF.errorCovPost,  KF.errorCovPre, KF.processNoiseCov); // 1: on, 0: off
//            imshow("cvBlobs - tracking", &rawImage);
        // test trackbar and buttons
        //cout << "value1 " <<value1 <<endl;



//endMainLoop:
        // --------------------- frame rate control ---------------------------------
        // calculates (required frame rate period - loop execution time) and sleeps for that
        // period to ensure stable frame rate for kalman calculations
        execTime = ((double)getTickCount() - tickCount)*1000./tickFreq; // execution time for loop in mS
        sleepTime = framePeriod-execTime;						// required frame period - execution time
      //  if (sleepTime >= .1)									// to within .1 mS
      //      usleep(sleepTime * 1000);							// usleep wants it in uS
       fps = tickFreq / ((double)getTickCount()-tickCount);			// calculate the frame rate
       tickCount = (double)getTickCount();								//re-set tick counter
     //   cout << "fps " << fps << endl;
       // cout << "fps " << fps << " execTime " << execTime << endl;
       //logFile << "fps " << fps << " execTime " << execTime << endl;


        // --------------------- User input ---------------------
        char k = (char)waitKey(1);     // param is mS delay wait for key event
        if( k == 27 ) break;
        if( k == ' ' )  {   printf("Space bar pressed\n");  break;  }


     ++nFrames;                      					//count every frame.


    }	//  for(;;) ****************************   MAIN PROCESSING LOOP  ***********************************
    destroyAllWindows();
    logfile.close();

    return (EXIT_SUCCESS);
}	//main()


/*
//-------------------------------------------------------------------------
// 	cvBlobMeanColor() returns cvScalar with count of red  green blue pixels
// 	red: colour.val[0] green: colour.val[1]  blue: colour.val[2]

// ------------------------------------------------------------------------

void cvFilterBlobsByColour(CvBlobs &blobs, IplImage const *imgLabel, Mat &frame)  {

	CvBlobs::iterator it=blobs.begin();
	while(it!=blobs.end())
	{
		CvBlob *blob=(*it).second;			// blob is a pointer
		//CvBlob *blob = it->second;

		//CvScalar cvBlobMeanColor(CvBlob const *blob, IplImage const *imgLabel, IplImage const *img);

	    IplImage iplFrame = frame;	   // iplImage shallow copy of Mat frame
	    CvScalar colour = cvBlobMeanColor( blob, imgLabel, &iplFrame);

		  /// \param blob Blob.
		  /// \param imgLabel Image of labels.
		  /// \param img Original image.
		  /// \return Average color:

	    double area = it->second->area;
	    double r = colour.val[0]/(colour.val[0]+colour.val[1]+colour.val[2]);
	    double g = colour.val[1]/(colour.val[0]+colour.val[1]+colour.val[2]);
	    double b = colour.val[2]/(colour.val[0]+colour.val[1]+colour.val[2]);

		cout << "rgb: " << r << g << b << " area " << area <<endl;

		// if red pixels predominateit->second
	    //  cout << "b " << (unsigned int)colour.val[0] <<" " << (unsigned int)colour.val[1]
		//			<<" " <<  (unsigned int)colour.val[2] << endl;

		if (colour.val[0] > 2 * (colour.val[1]+colour.val[2]))
		{
			cvReleaseBlob(blob);			// deletes the blob pointed to by blob

		    cout  << "released blob *************"  << endl;

			CvBlobs::iterator tmp=it;		// save iterator
			++it;
			blobs.erase(tmp);				// erases the map element
		}
			else
			++it;
	  }
	// i inc it in both arms, could do it here
}
*/



/*-------------------------------------------
  prints out changes to the trackbar setting as they occur

-------------------------------------------*/
/*
void tbarFunction(int posn) {
    printf("trackbar function - pos %d \n", posn);
}
void buttonFunction(int state, void * a) {
    printf("button function - state %d \n", state);
}*/



/*
void printKalmanArrays (cv::Mat & measurementNoiseCov, cv::Mat & errorCovPost, cv::Mat & errorCovPre, cv::Mat & processNoiseCov)  {
//	printf("\nmeasurement Noise Covariance\n%f %f \n%f %f \n",  measurementNoiseCov.at<float>(0,0), measurementNoiseCov.at<float>(0,1),
//			measurementNoiseCov.at<float>(1,0), measurementNoiseCov.at<float>(1,1));
    printf("\nerror Cov Pre:\n");
    for (int i=0; i<6; i++) {
        for (int j=0; j<6; j++)
            printf("%f ", errorCovPre.at<float>(i, j));
        printf("\n");
    }
    printf("error Cov Post:\n");
    for (int i=0; i<6; i++) {
        for (int j=0; j<6; j++)
            printf("%f ", errorCovPost.at<float>(i, j));
        printf("\n");
    }
//	printf("\nprocess Noise Covariance:\n");
//	for (int i=0; i<6; i++) {
//		for (int j=0; j<6; j++)
////			printf("%f ", processNoiseCov.at<float>(i, j));
//		printf("\n");
//	}

}*/

