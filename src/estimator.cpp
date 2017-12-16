// estimator2.cpp : Definiert den Einstiegspunkt für die Konsolenanwendung.

#include <flycapture/FlyCapture2.h>
#include <iostream>
#include <string>
#include <sstream>
#include <chrono>
#include <list>
#include <iterator>
#include <ostream>

#include <cstdlib>
#include <pthread.h>
#include <unistd.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <math.h>
// #include <Eigen/Dense>

#include "estimator.h"
#include "transformation.h"

// FlyCapture2 and cv have conflicting namespaces
// Since cv is short, we're going to use FlyCapture2 namespace here
using namespace FlyCapture2;
using namespace std;
using namespace chrono;
// using namespace Eigen;

// Globals
Error error;
// Image size settings and more. Set in main()
Format7ImageSettings f7is;
Property framerate;
Property shutter;
// Embedded Image info, f.e. timestamp, framecounter
EmbeddedImageInfo embeddedInfo;
// Camera
static Camera cam;
// Blob detection settings
static cv::SimpleBlobDetector::Params params;

static BusManager busMgr;
static PGRGuid guid;

void PrintError(Error error)
{
	error.PrintErrorTrace();
}

/*
// untested! Maybe use opencv kalman filter instead?
static void kalman(const Vector2f z, Vector4f& x, Matrix4f& P, const float dt)
{
	// Kalman filtering constants
	const float e = 0.8f; // Damping constant in 1/s
	const float q = 0.0001f; // process covariance coeff in 1/s^2
	const float r = 0.006f; // measurement covariance coeff in 1/s^2
	// Kalman filter matrices
	Matrix4f A = Matrix4f::Identity();
	A.topRightCorner(2, 2) = dt*Matrix2f::Identity();
	A.bottomLeftCorner(2, 2) -= e*dt*Matrix2f::Identity();
	Matrix<float, 2, 4> H = MatrixXf::Zero(2,4);
	H.topLeftCorner(2, 2) = Matrix2f::Identity();
	Matrix4f Q = Matrix4f::Identity()*q*dt*dt;
	Matrix2f R = Matrix2f::Identity()*r*dt*dt;

	Vector4f x_estimate = A*x;
	Matrix4f P_estimate = A*P*A.transpose() + Q;

	Matrix2f temp = H*P_estimate.transpose()*H.transpose() + R;
	Matrix<float, 4, 2> K = P_estimate*H.transpose()*temp.inverse();
	x = x_estimate + K*(z - H*x_estimate);
	P = (Matrix4f::Identity() - K*H)*P_estimate;
}*/

/* Finds the nearest keypoint between begin and end to the x,y position
 * If no point is closer to x,y than max_distance (default infinity), end is returned
 * If a keypoint is the closest to x,y and its distance smaller than max_distance,
 * its iterator is returned. 
 * If a keypoint between begin and end is to be ignored, set ommit to its pointer.
 */
static const list<cv::KeyPoint>::iterator nearestKeypoint(const float x, const float y, \
	list<cv::KeyPoint>::iterator begin, const list<cv::KeyPoint>::iterator end, \
	const float max_distance = INFINITY, const cv::KeyPoint* ommit = NULL)
{
	list<cv::KeyPoint>::iterator candidate = end;
	float min_distance = max_distance;
	for (list<cv::KeyPoint>::iterator it = begin; it != end; it++) {
		if (&(*it) == ommit) continue;
		float distance = pow(x - it->pt.x, 2.0f) + pow(y - it->pt.y, 2);
		if (distance < min_distance) {
			min_distance = distance;
			candidate = it;
		}
		
	}

	return candidate;
}

static float signum(const float x)
{
	return  (float)(0.0f < x) - (float)(0.0f > x);
}

/* This function finds the chuck. The chuck is to be marked with two blobs, having the same y position.
 * The mean blob center y_position must be equal to the chuck center y_position. 
 * x_position_deviance, spacing between blobs, tolerance etc. is set in transformation.h
 * 
 * Side effects: If chuck is found:
 * chuckstate is set, with chuckstate.valid = true
 * keypoints of chuck blobs are removed from keypoints
 *
 * If no chuck is found:
 * chuckstate.valid is set to false
 */
static void findChuck(list<cv::KeyPoint>& keypoints, chuckState& chuckstate)
{
	if (chuckstate.valid == true) {
		// Find the blob closest to where they were last time
		// Then iterate over the keypoints to find the second blob
		const list<cv::KeyPoint>::iterator candidate = nearestKeypoint((chuckstate.x - chuck_x_offset) / height_correction_factor, \
			chuckstate.y / height_correction_factor, keypoints.begin(), keypoints.end());
		 
		// Try to see if there is a keypoint to the left or right of the candidate
		list<cv::KeyPoint>::iterator candidate2 = nearestKeypoint(candidate->pt.x, candidate->pt.y + \
			signum(chuckstate.y/height_correction_factor - candidate->pt.y) * chuck_blobs_apparent_y_distance, \
			keypoints.begin(), keypoints.end(), r_tolerance, &(*candidate));
		if (candidate2 == keypoints.end()) { // None found on the side we thought it on using the signum function
			// Look the other way
			candidate2 = nearestKeypoint(candidate->pt.x, candidate->pt.y - \
				signum(chuckstate.y / height_correction_factor - candidate->pt.y) * chuck_blobs_apparent_y_distance, \
				keypoints.begin(), keypoints.end(), r_tolerance, &(*candidate));
		}
		if (candidate2 != keypoints.end()) {
			// Found the chuck
			cv::KeyPoint chuckKeyPoint = blob_to_chuck(height_t(*candidate), height_t(*candidate2));
			chuckstate.x = chuckKeyPoint.pt.x;
			chuckstate.y = chuckKeyPoint.pt.y;
			// Set to erase keypoints
			keypoints.erase(candidate);
			keypoints.erase(candidate2);
		}
		else {
			chuckstate.valid = false;
		}
	}
	else {
		// Scroll through all keypoints and find a suitable candidate
		for (std::list<cv::KeyPoint>::iterator candidate = keypoints.begin(); candidate != keypoints.end(); candidate++) {
			// Try to see if there is a keypoint to the left or right of the candidate within reasonable distance set in transformation.h
			list<cv::KeyPoint>::iterator candidate2 = nearestKeypoint(candidate->pt.x, candidate->pt.y + chuck_blobs_apparent_y_distance, \
				keypoints.begin(), keypoints.end(), r_tolerance, &(*candidate));
			if (candidate2 == keypoints.end()) { // None found to the left, look to the right
				candidate2 = nearestKeypoint(candidate->pt.x, candidate->pt.y - chuck_blobs_apparent_y_distance, \
					keypoints.begin(), keypoints.end(), r_tolerance, &(*candidate));
			}
			if (candidate2 != keypoints.end()) {
				// Found the chuck
				cv::KeyPoint chuckKeyPoint = blob_to_chuck(height_t(*candidate), height_t(*candidate2));
				chuckstate.x = chuckKeyPoint.pt.x;
				chuckstate.y = chuckKeyPoint.pt.y;
				// set to erase keypoints
				keypoints.erase(candidate);
				keypoints.erase(candidate2);
				chuckstate.valid = true;
				break;
			}
		}
	}
}

/* Estimator thread to be called with beginthreadex()
* The argument must be an estimatorState* var
* Semiperiodically updates estimatorState to estimated state
* Can be terminated by setting ((estimatorState*)(data))->estimatorOn = false
*/
void* estimator(void* data)
{
	// Configuration for estimator
	const bool calculatePos = true;
	const bool printPos = true;
	const bool printKeypoints = false;
	const bool printTimeStamp = false;
	const bool showImage = false;
	const bool kalmanFilter = false; // Not implemented!

	// Point Grey Variables
	Image image = Image();
	Image image_copy = Image(); // We always need to copy image, since original is volatile
	unsigned char *array;
	TimeStamp ts;
	ImageMetadata imdata; // Framecounter

	// OpenCV Variables
	cv::Mat mat; // Where the raw camera photo is stored
	cv::Mat minuend; // The subtract mask
	bool useMinuend = false; // Is set to true when trying to load the subtract mask 
	vector<cv::KeyPoint> _keypoints;
	cv::Size size = cv::Size(f7is.width, f7is.height);

	// Estimator extern variable, which is used to communicate with the player
	// For more information, see estimator.h
	estimatorState* state = (estimatorState*)(data);
	state->estimatorOn = true; // If this variable is false, the estimator is halted
	// More estimator variables
	puckState p = puckState();
	puckState p_old = puckState();
	float dt; // Time difference between this frame and last frame
	float framePeriod = 1.0f / framerate.absValue;
	unsigned int lastFrame = 0;
	unsigned int dFrame; // Difference of frames (proportional to dt)
	size_t nrOfKeypoints; // How many blobs were detected
	unsigned int findingCounter = 0; // Used in the state_machine to find the puck

	// State machine
	puckSM sm = PUCK_LOST;
	chuckState chuckstate = chuckState();

	// Loading minuend
	minuend = cv::imread(MINUEND_PATH, CV_LOAD_IMAGE_GRAYSCALE);
	if (minuend.size() == size) 
	{
		useMinuend = true;
	}
	else {
		cerr << "Not using minuend!" << endl;
	}

	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

	while (state->estimatorOn) {
		//auto start = high_resolution_clock::now();
		// Get image
		error = cam.RetrieveBuffer(&image);
		if (error != PGRERROR_OK) break;
	
		if (printTimeStamp) {
			ts = image.GetTimeStamp();
			imdata = image.GetMetadata();
			cout << ts.seconds << " " << ts.microSeconds << " " << imdata.embeddedFrameCounter << endl;
		}

		// Copy image, so it won't be destroyed
		error = image_copy.DeepCopy(&image);
		if (error != PGRERROR_OK) break;
		// Get data from image
		array = image_copy.GetData();
		// mat still uses array internally, so do not delete it
		mat = cv::Mat(size, CV_8U, array);

		if (calculatePos) {
			if (useMinuend) detector->detect(mat - minuend, _keypoints);
			else detector->detect(mat, _keypoints);


			list<cv::KeyPoint> keypoints(_keypoints.begin(), _keypoints.end());

			// Lense correction elliptic -> cartesian
			//lense_correction(&keypoints);
			// Convert keypoints into metric coordinate system
			linear_t(&keypoints, f7is.width, f7is.height);
			// Check amount of keypoints
			nrOfKeypoints = keypoints.size();

			if (printKeypoints) {
				cout << "-----" << endl;
				for (list<cv::KeyPoint>::iterator it = keypoints.begin(); it != keypoints.end(); it++) {
					cout << "(X, Y) = (" << it->pt.x << ", " << it->pt.y << ")" << endl;
				}
				cout << "*****" << endl;
			}

			// Find delta_t between this image and last image
			imdata = image.GetMetadata();
			dFrame = imdata.embeddedFrameCounter - lastFrame;
			lastFrame = imdata.embeddedFrameCounter;
			dt = framePeriod * dFrame;

			// First, we filter out the chuck. There are two blobs on the chuck.
			// If we find a pair of blobs, we take it out of keypoints
			if (nrOfKeypoints >= 2) {
				if (chuckstate.valid == false) findChuck(keypoints, chuckstate);
				else {
					findChuck(keypoints, chuckstate);
					// If after knowing the chuckpos we loose it, scan all keypoints for the chuck
					if (chuckstate.valid == false) findChuck(keypoints, chuckstate);
				}
			}
			else chuckstate.valid = false;
			nrOfKeypoints = keypoints.size();

			// At this point, we ought to have only one keypoint left
			// If there are more keypoints and sm != PUCK_NORMAL or PUCK_INSTABLE,
			// we will not be able to find the puck, or even shortly (1 frame) track 
			// the wrong blob
			if (nrOfKeypoints > 0 && sm == PUCK_LOST) {
				sm = PUCK_FINDING;
				findingCounter = 0;
			}
			else if (nrOfKeypoints == 0) sm = PUCK_LOST;

			switch (sm) {
			case PUCK_LOST: {
				// We try to extrapolate position from old data
				if (p.valid) {
					p_old = p;
					p.x += dt * p_old.v_x;
					p.y += dt * p_old.v_y;

					// Check if p is reasonable, table size in transformation.h
					// If the position is unreasonable, set p.valid false
					if (abs(p.x) > size_x / 2.0f || abs(p.y) > size_y / 2.0f) p.valid = false;
				}				
				break;
			}
			case PUCK_FINDING: {
				// If there is more than one keypoint, we can't distinct between the blobs
				if (nrOfKeypoints == 1) {
					cv::KeyPoint kp = keypoints.front();
					if (findingCounter == 0) {
						findingCounter++;
						p.x = kp.pt.x;
						p.y = kp.pt.y;
						p.v_x = 0.0f;
						p.v_y = 0.0f;
						p.valid = true;
					}
					else if (findingCounter == 1) {
						p_old = p;
						p.x = kp.pt.x;
						p.y = kp.pt.y;
						p.v_x = (p.x - p_old.x) / dt;
						p.v_y = (p.y - p_old.y) / dt;
						sm = PUCK_NORMAL;
					}

				}
				else sm = PUCK_LOST;
				break;
			}
			case PUCK_INSTABLE: // Unused atm
			case PUCK_NORMAL: {
				p_old = p;
				// Estimate of position
				p.x = p_old.x + p_old.v_x*dt;
				p.y = p_old.y + p_old.v_y*dt;
				// Find right keypoint
				const list<cv::KeyPoint>::iterator candidate = nearestKeypoint(p.x, p.y, keypoints.begin(), keypoints.end());
				// Use the keypoint for estimate
				// This is where the Kalman filter should go, right now it is unfiltered
				if (kalmanFilter) {
					; // To implement
				}
				else {
					p.x = candidate->pt.x;
					p.y = candidate->pt.y;
					p.v_x = (p.x - p_old.x) / dt;
					p.v_y = (p.y - p_old.y) / dt;
				}
				break;
			}
			default: {
				cout << "Unknown state of state-machine. Exiting" << endl;
				state->estimatorOn = false;
			}
			}

			// Make new data available to player
			state->p = p;
			state->state = sm;
			state->c = chuckstate;
			state->frameCounter = lastFrame;

			if (printPos && p.valid) {
				cout << p.x << ";" << p.y << ";" << p.v_x << ";" << p.v_y << ";" << dt;
				if (sm == PUCK_LOST) cout << "; LOST";
				cout << endl;
			}
			if (printPos && chuckstate.valid) {
				cout << chuckstate.x << ";" << chuckstate.y << endl;
			}
		}

		/* Here, four images are shown: 
		 * 1. The image taken by the camera
		 * 2. How it looks thresholded
		 * 3. The image - subtractMask
		 * 4. The above thresholded (this is what the blobDetect algorithm actually does)
		 */
		if (showImage) {
			cv::Mat thresholded = mat.clone();
			uchar* ptr = thresholded.ptr(0);
			cv::Mat difference = mat - minuend;
			cv::Mat dthresholded = difference.clone();
			uchar* dptr = dthresholded.ptr(0);

			for (unsigned int i = 0; i < f7is.width*f7is.height; i++,ptr++,dptr++) {
				*ptr = (*ptr < (uchar)params.minThreshold) ? 0 : 255;
				*dptr = (*dptr < (uchar)params.minThreshold) ? 0 : 255;
			}

			cv::namedWindow("image", CV_WINDOW_KEEPRATIO);
			cv::imshow("image", mat);
			cv::namedWindow("thresholded", CV_WINDOW_KEEPRATIO);
			cv::imshow("thresholded", thresholded);
			cv::namedWindow("difference", CV_WINDOW_KEEPRATIO);
			cv::imshow("difference", difference);
			cv::namedWindow("dthresholded", CV_WINDOW_KEEPRATIO);
			cv::imshow("dthresholded", dthresholded);
			cv::waitKey(0);
			thresholded.~Mat();
			dthresholded.~Mat();
		}

		// Free memory
		mat.~Mat();
	}

	return NULL;
}

/* This function generates a subtract maks to filter out some distortions rudimentarily
 * It saves the subtract mask as a file (name and format tbd) in the same folder as the .exe
 */
static void generateSubtractMask()
{
	Image image = Image();
	Image image_copy = Image();
	cv::Mat mat;
	cv::Size cv_size = cv::Size(f7is.width, f7is.height);

	unsigned char *array; // Image raw data
	unsigned int size = f7is.height * f7is.width;
	const int amount = 1024; // Amount of pictures taken
	unsigned int *sum = new unsigned int[size]; // Sum in unsigned int to avoid overflow
	unsigned char *endsum = new unsigned char[size]; // Endsum to keep compatibility in unsigned char
	unsigned int i,j; // Counter variables
	unsigned int *s; // Counter variable for sum
	unsigned char *e; // Counter variable for endsum or array

	// Set sum = 0
	for (i = 0, s = sum; i < size; i++, s++) {
		*s = 0;
	}

	for (i = 0; i < amount; i++) {
		error = cam.RetrieveBuffer(&image);
		if (error != PGRERROR_OK) break;
		// Copy it, so it won't be destroyed
		error = image_copy.DeepCopy(&image);
		if (error != PGRERROR_OK) break;
		// Get data from image
		array = image_copy.GetData();
		
		// Add image to sum
		for (j = 0, s = sum, e = array; j < size; j++, e++, s++) {
			*s += *e;
		}

	}

	// Divide by amount of pictures taken and save to endsum
	for (i = 0, s = sum, e = endsum; i < size; i++, s++, e++) {
		*e = (unsigned char)(*s / amount);
	}
	free(sum);
	
	// Save endsum
	mat = cv::Mat(cv_size, CV_8U, endsum);
	cv::namedWindow("image", CV_WINDOW_KEEPRATIO);
	cv::imshow("image", mat);
	cv::waitKey(0);

	cv::imwrite(MINUEND_PATH, mat);

	free(endsum);

}

/* Must be called before using console or estimator.
* Only call this once
* Camera configuration is set in this function, as well as the blob detect config
*/
FlyCapture2::Error init() 
{
	Error err;
	// do once, so we can break at an error
	do {
		// Connect camera
		err = busMgr.GetCameraFromIndex(0, &guid);
		if (err != PGRERROR_OK) break;
		err = cam.Connect(&guid);
		if (err != PGRERROR_OK) break;
		// Camera connected

		// Configure camera
		f7is.height = 160;
		f7is.width = 320;
		f7is.offsetY = 40;
		f7is.offsetX = 0;
		f7is.mode = MODE_1; // Activate pixel binning
		f7is.pixelFormat = PIXEL_FORMAT_MONO8;
		err = cam.SetFormat7Configuration(&f7is, 100.0f);
		if (err != PGRERROR_OK) break;

		framerate.type = FRAME_RATE;
		framerate.absControl = true;
		framerate.absValue = 445.0f;
		framerate.autoManualMode = false;
		framerate.onOff = true;
		framerate.onePush = false;
		framerate.present = true;
		err = cam.SetProperty(&framerate);
		if (err != PGRERROR_OK) break;

		shutter.type = SHUTTER;
		shutter.absControl = true;
		shutter.absValue = 1;
		shutter.autoManualMode = false;
		shutter.onOff = true;
		shutter.onePush = false;
		shutter.present = true;
		err = cam.SetProperty(&shutter);
		if (err != PGRERROR_OK) break;
		err = cam.GetProperty(&shutter);

		// Set timestamp and framecounter to be delivered from the camera
		err = cam.GetEmbeddedImageInfo(&embeddedInfo);
		if (err != PGRERROR_OK) break;
		embeddedInfo.timestamp.onOff = true;
		embeddedInfo.frameCounter.onOff = true;
		err = cam.SetEmbeddedImageInfo(&embeddedInfo);
		if (err != PGRERROR_OK) break;

		// Parameters for blob detection of opencv
		params.minThreshold = 50;
		params.maxThreshold = 255;
		params.filterByArea = false;
		params.filterByCircularity = false;
		params.filterByColor = false;
		params.filterByConvexity = false;
		params.filterByInertia = false;
		params.minDistBetweenBlobs = 1.5f;

		// Start the camera
		cam.StartCapture();
	} while (false);

	return err;
}

ostream &operator<<(ostream &os, const puckState &p)
{
	os << p.x << ';' << p.y << ';' << p.v_x << ';' << p.v_y << endl;
	return os;
}

ostream &operator<<(ostream &os, const chuckState &c)
{
	os << c.x << ';' << c.y << endl;
	return os;
}

int main()
{
	/*Estimator estimatorObj;
	try {
		estimatorObj.startEstimator();
	} 
	catch (Error err) {
		PrintError(err);
		return -1;
	}

	unsigned int f = 0;
	for (int i = 0; i < 100; i++) {
		while (f == estimatorObj.getFrameCounter()) {
			usleep(1);
		}
		f = estimatorObj.getFrameCounter();
		puckState p = estimatorObj.getPuckState();
		chuckState c = estimatorObj.getChuckState();
		cout << p;
		cout << c;
		cout << f << endl << endl;
	}

	try {
		estimatorObj.stopEstimator();
	}
	catch (Error err) {
		PrintError(err);
		return -1;
	}*/
	
	Estimator obj;
	obj.startConsole();

	cin.ignore();
	return 0;
}

Estimator::Estimator()
{
	err = init();
}

puckState Estimator::getPuckState()
{
	if (err != PGRERROR_OK) throw err;
	return state.p;
}

chuckState Estimator::getChuckState()
{
	if (err != PGRERROR_OK) throw err;
	return state.c;
}

puckSM Estimator::getPuckEstimatorState()
{
	if (err != PGRERROR_OK) throw err;
	return state.state;
}

unsigned int Estimator::getFrameCounter()
{
	if (err != PGRERROR_OK) throw err;
	return state.frameCounter;
}

bool Estimator::estimatorIsOn()
{
	return state.estimatorOn;
}

bool Estimator::startEstimator()
{
	if (err != PGRERROR_OK) throw err;
	if (state.estimatorOn) {
		cerr << "Estimator already on..." << endl;
		return false; // Estimator already on
	}
	cerr << "Starting estimator..." << endl;
	// Create the thread
	int thread_error = pthread_create(&thread, NULL, estimator, &state);
	if (thread_error) {
		cerr << "Error: Unable to create thread with error code: " << thread_error << endl;
		return false;
	}	
	return true;
}

void Estimator::stopEstimator()
{
	if (state.estimatorOn == false) {
		cerr << "Estimator already off..." << endl;
		return; // Estimator already off
	}
	cerr << "Stopping estimator..." << endl;
	state.estimatorOn = false;
	//int terminate_error = pthread_timedjoin_np(thread, NULL, timeout);
	pthread_join(thread,NULL);
}

/* A console to debug or configure the estimator.
 * exit: exits the console, if the estimator isn't running
 * startEstimator: starts the estimator
 * stopEstimator: stops the estimator
 * subtract: Generates a subtract mask which is henceforth subtracted from every image,
 *           before being passed to the blobDetector. Generates a file in the same folder
 *           as the executable
 * printState: Prints the position and velocity of the puck
 * edit threshold XXX: Sets the minimum brightness of a blob to be detected between 0 and 255
 */
void Estimator::startConsole()
{
	bool stop = false;
	string str = string();

	while (!stop) {
		getline(cin,str);
		
		if (str.compare("exit") == 0) {
			if (estimatorIsOn()) {
				cerr << "stopEstimator first!" << endl;
			}
			else {
				stop = true;
			}
		}
		else if (str.compare("startEstimator") ==  0) {
			try { startEstimator(); }
			catch (FlyCapture2::Error error) {
				error.PrintErrorTrace();
				return;
			}
		}
		else if (str.compare("stopEstimator") == 0) {
			stopEstimator();
		}
		else if (str.compare("subtract") == 0) {
			if (estimatorIsOn()) {
				cerr << "Stop Estimator first!" << endl;
			}
			else {
				cerr << "Generating subtract mask" << endl;
				generateSubtractMask();
			}
			
		}
		else if (str.compare("printState") == 0) {
			puckState p = getPuckState();
			cerr << "(x, y) = (" << p.x << ", " << p.y << ")\n";
			cerr << "(v_x, v_y) = (" << p.v_x << ", " << p.v_y << ")\n";
		}
		else if (str.find("edit ", 0) != string::npos) {
			if (estimatorIsOn()) {
				cerr << "Stop Estimator first!" << endl;
				continue;
			}
			str.erase(0, 5);
			if (str.find("threshold ", 0) != string::npos) {
				str.erase(0, 10);
				params.minThreshold = stof(str);
				cerr << "Threshold is now " << params.minThreshold << endl;
			}
		}
		else {
			cerr << "Unknown command" << endl;
		}

	}
}
