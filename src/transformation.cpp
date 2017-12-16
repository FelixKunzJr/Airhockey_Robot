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
	// not tested (should work...)
	float camMatrix[3][3] = {
		{ 585.1198,0,271.877 },
		{ 0,585.491365,167.047 },
		{0,0,1}
	};
	float dist[5] = { -0.351424,-0.07300474,0,0,0.26193 };
	float points[2];

	cv::Mat camMat = cv::Mat(3, 3, CV_32F, camMatrix);
	cv::Mat distMat = cv::Mat(1, 5, CV_32F, dist);
	cv::Mat pointMat;


	for (list<cv::KeyPoint>::iterator it = keypoints->begin(); it != keypoints->end(); it++) {
		points[0] = it->pt.x;
		points[1] = it->pt.y;
		pointMat = cv::Mat(1, 2, CV_32F, points);
		cv::undistortPoints(pointMat, pointMat, camMat, distMat);
		it->pt.x = pointMat.at<uchar>(0,0);
		it->pt.y = pointMat.at<uchar>(0,1);
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

