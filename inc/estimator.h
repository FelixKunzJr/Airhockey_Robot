#pragma once

// Defines where the minuend is stored
#define MINUEND_PATH "./subtract.png"

#include <flycapture/FlyCapture2.h>
#include <iostream>
#include <ostream>

#include <cstdlib>
#include <pthread.h>

typedef enum {
	PUCK_NORMAL, // Sure of the puck position and velocity
	PUCK_INSTABLE, // Unsure of puck velocity (only with calman filtering)
	PUCK_LOST, // No sign of puck
	PUCK_FINDING // Regaining puck position (Only one frame detected)
} puckSM;

//typedef enum

typedef struct _puckState {
	float x;
	float y;
	float v_x;
	float v_y;
	// Vector4f x;
	bool valid;
	_puckState() : x(0.0f), y(0.0f), v_x(0.0f), v_y(0.0f), valid(false) {}
	_puckState(const float _x, const float _y, const float _v_x, const float _v_y) :
		x(_x), y(_y), v_x(_v_x), v_y(_v_y), valid(true) {}
}puckState;

typedef struct _chuckState {
	float x;
	float y;
	bool valid;
	_chuckState() : valid(false) {}
	_chuckState(const float _x, const float _y) : x(_x), y(_y), valid(true) {}
} chuckState;

// This is a struct of all information to be communicated between the player and the estimator
// Does not have to be used by the player directly!
typedef struct _estimatorState {
	puckState p; // The position and velocity
	chuckState c; // The position of the chuck
	bool estimatorOn; // Used to communicate from the player to the estimator
	puckSM state; // Used to elaborate on how sure the estimator is about the puck state
	unsigned int frameCounter;

	_estimatorState() : estimatorOn(false), state(PUCK_LOST), frameCounter(0) {}
} estimatorState;

//unsigned int __stdcall estimator(void* data);
//FlyCapture2::Error init();
//void console(estimatorState* state);

/* Interface object to control estimator
 * All functions except Estimator() and stopEstimator can throw exceptions of type FlyCapture2::Error
 */
class Estimator {
	estimatorState state;
	FlyCapture2::Error err;
	pthread_t thread;
	//const struct timespec timeout = {5,0} // 5 seconds max_join timeout
	

public:
	Estimator();

	puckState getPuckState();
	chuckState getChuckState();
	puckSM getPuckEstimatorState();
	unsigned int getFrameCounter();
	bool estimatorIsOn();

	bool startEstimator();
	void stopEstimator();
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
	void startConsole();
};
