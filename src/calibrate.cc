/* Camera Calibration - Vaibhav Bajpai */

#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>
#include <vector>
#include <cstdlib>

#include <stdio.h>

#define HEIGHT 4
#define WIDTH 6

using namespace std;

class CalibrateCamera{
	
	private:
	
	int width, height, numberOfCorners, cvFindCornerCount, numberOfImages, numberOfSuccessfulImages;	
	CvPoint2D32f* corners; 
	IplImage *img, *dest;
	string filename;	
	vector<string> imageList;	
	CvMat* objectPointsOld, *imagePointsOld, *pointCountsOld;
	CvMat* objectPointsNew, *imagePointsNew, *pointCountsNew;
	CvMat* cameraMatrix, *distCoeffs, *rvecs, *tvecs;
	
	void liveCapture(){
		
		cout << endl << endl << "initializing camera ...";
		
		/* initialize capture source */
		cvNamedWindow("Webcam", CV_WINDOW_AUTOSIZE);
		CvCapture* capture = cvCaptureFromCAM(0); 
		
		cout << "[done]" << endl << endl << "looking for chessboard corners ...";		
		do{		
			/* grab and copy the frame to buffer */
			img = cvQueryFrame(capture);
			dest = cvCloneImage(img);
			
			/* search and draw chessboard corners */
			int ifFound = cvFindChessboardCorners(img, cvSize(width,height), corners, &cvFindCornerCount);	
			cvDrawChessboardCorners(img, cvSize(width, height), corners, cvFindCornerCount, ifFound);
			cvShowImage("Webcam", img);
			cvWaitKey(10);		
			
			if(cvFindCornerCount == numberOfCorners){
				
				cout << endl << endl << "capturing ...";
				
				/* save the image from buffer to a file */
				time_t rawtime;
				struct tm * timeinfo;
				
				/* generate unique filename using current time */
				time ( &rawtime );
				timeinfo = localtime ( &rawtime );
				string filename = string(asctime(timeinfo)).substr(11, 8);				
				char iter[1];
				sprintf(iter, "%d", numberOfSuccessfulImages);
				filename.append(iter);
				filename.append(".jpg");
				if (!cvSaveImage(filename.c_str(), dest)) {
					cout << "Could not save image" << endl;
					exit(1);
				}			
				
				/* push the filename to a vector */
				imageList.push_back(filename);	
				
				cout << "[done]" << endl << "saving as " << filename << "..."<< "[done]";					
				numberOfSuccessfulImages++;
			}
			
		}while(numberOfSuccessfulImages < 10);
		
		/* release capture source */
		numberOfSuccessfulImages = 0;
		cvDestroyWindow("Webcam");
		cvReleaseCapture(&capture);
	}
	
	void allocateMatrixObjects(){

		objectPointsOld = cvCreateMat(numberOfCorners * numberOfImages, 3, CV_32FC1 );
		imagePointsOld = cvCreateMat(numberOfCorners * numberOfImages, 2, CV_32FC1 );
		pointCountsOld = cvCreateMat(numberOfImages, 1, CV_32SC1 );		
		cameraMatrix = cvCreateMat( 3, 3, CV_32FC1 );
		distCoeffs = cvCreateMat( 5, 1, CV_32FC1 );	
	}
	
	void findChessboardCorner(){	
		
		for (int imageIndex=0; imageIndex<numberOfImages; imageIndex++) {
			
			/* load the image from filename */
			filename = imageList[imageIndex];						
			img = cvLoadImage(filename.c_str(), CV_LOAD_IMAGE_UNCHANGED);	
			
			/* search chessboard corners */
			int ifFound = cvFindChessboardCorners(img, cvSize(width, height), corners, &cvFindCornerCount);	
			
			if(ifFound)
				cout << "[filename:"<< filename << "]" << "[corners:" << cvFindCornerCount << "]"<< endl;
			else
				cout << "[filename:"<< filename << "]" << "[corners: 0]" << endl;
			
			/* fill structures */		
			if(cvFindCornerCount == numberOfCorners){				
				CV_MAT_ELEM(*pointCountsOld, int, numberOfSuccessfulImages, 0) = numberOfCorners;				
				int step = numberOfSuccessfulImages*numberOfCorners;
				for( int i=step, j=0; j < numberOfCorners; ++i, ++j ){				
					CV_MAT_ELEM( *imagePointsOld, float, i, 0 ) = corners[j].x;
					CV_MAT_ELEM( *imagePointsOld, float, i, 1 ) = corners[j].y;
					CV_MAT_ELEM( *objectPointsOld, float, i, 0 ) = j/width;
					CV_MAT_ELEM( *objectPointsOld, float, i, 1 ) = j%width;
					CV_MAT_ELEM( *objectPointsOld, float, i, 2 ) = 0.0f;
				}	
				numberOfSuccessfulImages++;
			}
		}	
	}
	
	void calibrateCamera(){
		
		if (numberOfSuccessfulImages != 0){
			
			/* reallocate the matrix objects */
			objectPointsNew = cvCreateMat(numberOfSuccessfulImages*numberOfCorners, 3, CV_32FC1 );
			imagePointsNew = cvCreateMat(numberOfSuccessfulImages*numberOfCorners, 2, CV_32FC1 );
			pointCountsNew = cvCreateMat(numberOfSuccessfulImages, 1, CV_32SC1 );				
			reAllocateMatrixObjects();	
			
			/* calibrate the camera */
			rvecs = cvCreateMat(numberOfSuccessfulImages, 3, CV_32FC1);
			tvecs = cvCreateMat(numberOfSuccessfulImages, 3, CV_32FC1);	
			cout << endl << endl << "running cvCalibrateCamera2() ..." << endl << endl;
			cvCalibrateCamera2(objectPointsNew, imagePointsNew, pointCountsNew, cvGetSize(img), cameraMatrix, distCoeffs, rvecs, tvecs);
			
			/* print intrinsic | extrinsic parameters */
			printParameters();		
			
			/* undistort image */
			img = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
			dest = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
			cvUndistort2(img, dest, cameraMatrix, distCoeffs);
			cvDrawChessboardCorners(dest, cvSize(WIDTH,HEIGHT), corners, 
									numberOfCorners, cvFindChessboardCorners(dest, cvSize(WIDTH,HEIGHT), corners, &cvFindCornerCount));
			
			/* show the undistort image */
			cvNamedWindow("Undistorted Image", CV_WINDOW_AUTOSIZE); 
			cvShowImage("Undistorted Image",dest);
			cout << endl << endl << "press enter to save the image...";
			cin.get();
			cvDestroyWindow("Undistorted Image");
			
			/* save the undistort image */
			string filename = "undistorted_grab.jpg";
			cvSaveImage(filename.c_str(), dest);
			cout << endl <<endl << "saving undistorted copy as..." << filename << " [done]" << endl << endl;		
		}else{	
			cout << endl << endl << "cannot calibrate the camera" << endl << endl;
		}
	}
	
	void reAllocateMatrixObjects(){
		
		// transfer to new matrix
		for( int i = 0; i < numberOfSuccessfulImages*numberOfCorners; ++i ){		
			CV_MAT_ELEM( *imagePointsNew, float, i, 0) = cvmGet(imagePointsOld, i, 0);
			CV_MAT_ELEM( *imagePointsNew, float, i, 1) = cvmGet(imagePointsOld, i, 1);
			CV_MAT_ELEM( *objectPointsNew, float, i, 0) = cvmGet(objectPointsOld, i, 0);
			CV_MAT_ELEM( *objectPointsNew, float, i, 1) = cvmGet(objectPointsOld, i, 1);
			CV_MAT_ELEM( *objectPointsNew, float, i, 2) = cvmGet(objectPointsOld, i, 2);
		}	
		for( int i=0; i < numberOfSuccessfulImages; ++i ){
			CV_MAT_ELEM(*pointCountsNew, int, i, 0) = CV_MAT_ELEM(*pointCountsOld, int, i, 0); 
		}
		
		/* release old matrix objects */
		cvReleaseMat(&imagePointsOld);
		cvReleaseMat(&objectPointsOld);
		cvReleaseMat(&pointCountsOld);
	}
	
	void printParameters(){
				
		// print distortion matrix
		if (distCoeffs){
			cout << "radial-distoration k1: " << cvmGet(distCoeffs, 0, 0) << endl;
			cout << "radial-distoration k2: " << cvmGet(distCoeffs, 1, 0) << endl << endl;
			cout << "tangential-distoration p1: " << cvmGet(distCoeffs, 2, 0) << endl;
			cout << "tangential-distoration p2: " << cvmGet(distCoeffs, 3, 0) << endl << endl;			
		}
		
		// print camera matrix
		if (cameraMatrix){			
			cout << "optical-center cx: " << cvmGet(cameraMatrix, 0, 2) << endl;
			cout << "optical-center cy: " << cvmGet(cameraMatrix, 1, 2) << endl << endl;
			cout << "focal-length fx: " << cvmGet(cameraMatrix, 0, 0) << endl;
			cout << "focal-length fy: " << cvmGet(cameraMatrix, 1, 1) << endl << endl;			
		}
		
		// print translation matrix
		if (tvecs){
			cout << "translation matrix - " << endl << endl;
			for (int j = 0; j < numberOfSuccessfulImages; j++){
				for (int i = 0; i < 3; i++){
					if(i!=0)
						cout << setw(20);		
					cout << cvmGet(tvecs, j, i);
				}
				cout << endl;
			}		
		}
		
		// print rotation matrix
		if (rvecs){
			cout << endl << endl << "rotation matrix - " << endl << endl;
			for (int j = 0; j < numberOfSuccessfulImages; j++){
				for (int i = 0; i < 3; i++){
					if(i!=0)
						cout << setw(20);			
					cout << cvmGet(rvecs, j, i);	
				}
				cout << endl;
			}		
		}	
	}

	public:
	
	CalibrateCamera(int width, int height){
		this->width = width;
		this->height = height;
		numberOfCorners = width * height;
		corners = new CvPoint2D32f[numberOfCorners];
	}
	
	void calibrateFromImages(vector<string> imageList){
		
		/* allocate matrix objects */
		this->imageList = imageList;
		numberOfImages = imageList.size();
		allocateMatrixObjects();
		
		/* findChessboardCorners */
		cout << endl << endl << "running cvFindChessboardCorners() ..." << endl << endl;
		findChessboardCorner();
		
		/* calibrate camera */
		calibrateCamera();	
	}	
	
	void calibrateFromLiveCapture(){
		
		/* live capture images */
		liveCapture();
		
		/* allocate matrix objects */
		numberOfImages = imageList.size();
		allocateMatrixObjects();
		
		/* findChessboardCorners */
		cout << endl << endl << "running cvFindChessboardCorners() ..." << endl << endl;
		findChessboardCorner();
		
		/* calibrate camera */
		calibrateCamera();
	}
};

int main(int argc, char *argv[]){
	
	CalibrateCamera *camera = new CalibrateCamera(WIDTH, HEIGHT);
	
	if (argc <= 1){
		camera->calibrateFromLiveCapture();		
	}
	else{
		
		/* push filenames to a vector */
		vector<string> imageList;
		for(int i=1; i<argc; i++){
			string filename = argv[i];
			imageList.push_back(filename);			
		}			
		
		camera->calibrateFromImages(imageList);		
	}
}

