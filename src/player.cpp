#include <iostream>
#include <cmath>
#include <vector>

#include "player.h"
#include "estimator.h"
#include <flycapture/FlyCapture2.h>

using namespace std;

void player()
{
	// Start estimator
	Estimator estimator;
	try { estimator.startEstimator(); }
	catch (FlyCapture2::Error error) {
		error.PrintErrorTrace();
		return;
	}

	// Starting player
	const unsigned int maxtime = 1000; // in ms
	const unsigned int densityperms = 1; // in 1/ms (how exact do we want to calculate the stuff....)
	playerSM sm = NO_DANGER;
	vector<float> pos_x (maxtime*densityperms);
	vector<float> pos_y (maxtime*densityperms);
	vector<float> dist(maxtime*densityperms);
	float impact_x;
	float impact_y;

	while (true) {
		puckState p = estimator.getPuckState();
		pos_x[0] = p.x; // current position of puck
		pos_y[0] = p.y;
		float vx = p.v_x; // currtne speed of puck
		float vy = p.v_y;
		float currx = -0.5; // current position of player
		float curry = 0;

		// calculating the function of position, depending on time of puck
		for (int i = 1; i <= maxtime*densityperms; i++) {
			// bouncing off the wall!
			if (pos_x[i] > 0.75) {
				vx = -vx;
			}
			if (pos_y[i] > 0.375) {
				vy = -vy;
			}
			// the next step (vx in m/s -> m/ms -> need to devide by 1000)
			pos_x[i] = pos_x[i - 1] + vx / (1000 * densityperms);
			pos_y[i] = pos_y[i - 1] + vy / (1000 * densityperms);

			dist[i] = sqrt(pow(abs(currx - pos_x[i]), 2) + pow(abs(curry - pos_y[i]), 2));
		}

		if (vx < 0) {
			sm = BLOCK;
			for (int i = 1; i <= 30; i++){
				if (dist[i] < 0.005) {
					sm = SHOOT;
					break;
				}
			}
		}
		else {
			sm = NO_DANGER;
		}
		switch (sm) {
		case NO_DANGER: {
			// player.move(default/middle)
			break;
		} 
		case BLOCK: {
			// calculating closes pint to current playerpos


			int timebest;
			float distbest = 2;

			for (int i = 1; i <= maxtime*densityperms; i++) {
				if (pos_x[i] < -0.5 && distbest < 2)
					break;
				if (distbest > dist[i] && abs(pos_y[i]) < 0.3 && pos_x[i] < -0.2) {
					distbest = dist[i];
					timebest = i;
				}
			}
			impact_x = pos_x[timebest];
			impact_y = pos_y[timebest];
			// player.move(impact_x, impact_y);
			break;
		} 
		case SHOOT: {
			//player.move(impact_x, impact_y);
			break;
		}
		}
	}


	// Stop estimator
	estimator.stopEstimator();

}
