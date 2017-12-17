#include <iostream>
#include <cmath>
#include <vector>

#include "player.h"
#include "estimator.h"
#include "motor.h"

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

	// static variables/ dimensions
	const float desk_bound_x = 1.4;
	const float desk_bound_y = 0.65;
	const float line_of_impact_x = -0.25;
	const float time_before_shoot = 100;
	const float m_backoff = 0.05;
	const float m_default_x = -0.45;
	const float m_default_y = 0;

	const float m_bound_x_l = -0.13;
	const float m_bound_x_h = -0.54;
	const float m_bound_y_l = -0.28;
	const float m_bound_y_h = 0.28;

	const unsigned int maxtime = 500; // in ms
	const unsigned int densityperms = 1; // in 1/ms (how exact do we want to calculate the stuff....)

	// check estimator variables
	float prev_framecounter = 0;

	// player variables
	playerSM sm = NO_DANGER;
	vector<float> p_x(maxtime*densityperms); // position of te puck in time (steps and length defined above)
	vector<float> p_y(maxtime*densityperms);
	vector<float> dist(maxtime*densityperms);
	float p_vx; // current speed of puck
	float p_vy;
	float c_x; // current position of chuck
	float c_y;
	int nr_bounce;
	bool error;
	int t_impact;


	while (true) {

		// Get the information from the estimator
		unsigned int frameCounter = estimator.getFrameCounter();
		puckState p = estimator.getPuckState();
		chuckState cs = estimator.getChuckState();

		// check if the estimator has a valid puck and chuck state 
		// and we are not looking at the same frame twice
		if (p.valid && cs.valid && !(frameCounter == estimator.getFrameCounter())){

			p_x[0] = p.x; // current position of puck
			p_y[0] = p.y;
			float p_vx = p.v_x; // current speed of puck
			float p_vy = p.v_y;
			float c_x = cs.x; // current position of chuck
			float c_y = cs.y;
			nr_bounce = 0;
			error = false;

			cout << "p_x: " << p_x[0] << "   p_y: " << p_y[0] << "   v_x: " << p_vx << "   v_y: " << p_vy << "\n";
			// calculating the function of position, depending on time of puck
			for (int i = 1; i <= maxtime*densityperms && !error; i++) {
				// bouncing off the wall!
				if (abs(p_x[i-1]) > desk_bound_x) {
					p_vx = -p_vx;
					++nr_bounce;
					cout << "bouncing x\n";
				}
				if (abs(p_y[i-1]) > desk_bound_y) {
					p_vy = -p_vy;
					++nr_bounce;
					cout << "bouncing y\n";
				}
				// the next step (vx in m/s -> m/ms -> need to devide by 1000)
				p_x[i] = p_x[i - 1] + p_vx / (1000 * densityperms);
				p_y[i] = p_y[i - 1] + p_vy / (1000 * densityperms);
				
				// logic to determine, which case to take:
				sm = NO_DANGER; //default
				if (nr_bounce > 2) {
					error = true;
				}
				if (p_x[i] < line_of_impact_x){ //when approaching our side
					error = true;
					t_impact = i;
					if (i < time_before_shoot*densityperms){
						sm = SHOOT;
					}else{
						sm = BLOCK;
					}
				}
			}

			// case handling:
			switch (sm) {
			case NO_DANGER: {
				motorSetPosition(m_default_x, m_default_y,cs);
				cout << "NO DANGER\n";
				break;
			} 
			case BLOCK: {
				motorSetPosition(p_x[t_impact]-m_backoff, p_y[t_impact],cs);
				cout << "BLOCK\n" << "impact_x: " << p_x[t_impact]-m_backoff << "    impcat_y: " << p_y[t_impact];
				break;
			} 
			case SHOOT: {
				motorSetSpeed(5,0);
				cout << "SHOOT\n";
				break;
			}
			}
		}
		
	}


	// Stop estimator
	estimator.stopEstimator();

}
