#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <list>

#include "transformation.h"

/* In this file, the transformation into metric, cartesian coordinates from camera coord. is defined
*  The table sizes are used to get the metric approximately correct.
*
*
*/

using namespace std;

const cv::Size camMatSize = cv::Size(3,3);
const cv::Size distMatSize = cv::Size(1,5);

// width and height refer to amount of pixels in the image
void linear_t(list<cv::KeyPoint> *keypoints, const float width, const float height)
{
	for (std::list<cv::KeyPoint>::iterator it = keypoints->begin(); it != keypoints->end(); it++) {
		// Shift center
		it->pt.x -= width / 2.0f;
		it->pt.y -= height / 2.0f;
		// Adjust to meters. Using table size of 150x75cm. 
		// Using rough estimate (actual size doesn't matter much)
		it->pt.x *= size_x / width;
		it->pt.y *= size_y / height;
		// Fine correction
		it->pt.x += -0.0f;
	}
}

void lense_correction(list<cv::KeyPoint> *keypoints)
{
	float camMatrix[9] = {
		585.1198,0,271.877,
		0,585.491365,167.047,
		0,0,1
	};
	float dist[5] = { -0.351424,-0.07300474,0,0,0.26193 };

	cv::Mat camMat = cv::Mat(camMatSize, CV_32F, camMatrix);
	cv::Mat distMat = cv::Mat(distMatSize, CV_32F, dist);
	vector<CV_32FC2> inputpoints(keypoints->size());
	vector<CV_32FC2> outputpoints(keypoints->size());


	int i = 0;
	for (list<cv::KeyPoint>::iterator it = keypoints->begin(); it != keypoints->end(); it++) {
		inputpoints[i] = Scalar(it->pt.x,it->pt.y);
		++i;
	}
	cv::undistortPoints(inputpoints, outputpoints, camMat, distMat);
	
	i=0;
	for (list<cv::KeyPoint>::iterator it = keypoints->begin(); it != keypoints->end(); it++) {
		it->pt.x = outputpoints[i];
		it->pt.y = outputpoints[i];
	}
}

cv::KeyPoint height_t(cv::KeyPoint kp)
{
	kp.pt.x *= height_correction_factor;
	kp.pt.y *= height_correction_factor;
	return kp;
}

cv::KeyPoint blob_to_chuck(cv::KeyPoint p1, cv::KeyPoint p2) 
{
	// Average blobs
	p1.pt.x += p2.pt.x;
	p1.pt.y += p2.pt.y;
	p1.pt.x /= 2;
	p1.pt.y /= 2;

	// Correct x_offset
	p1.pt.x += chuck_x_offset;

	return p1;
}

