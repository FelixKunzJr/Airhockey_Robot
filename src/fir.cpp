#include "fir.h"

puckState fir(dequeue<puckState> puckStates, const float dt) {
	// Use at most 5 elements
	if (puckStates.size() > 5) puckStates.pop_back();
	
	unsigned int amount_valid = 0;
	for (puckState p : puckStates) {
		if (p.valid) amount_valid++;
		else break;
	}
	
	puckState retState = puckStates.front();
	
	if (2 <= amount_valid && amount_valid < 5) { // Difference
		retState.v_x = (puckStates.at(0).v_x - puckStates.at(1).v_x)/dt;
		retState.v_y = (puckStates.at(0).v_y - puckStates.at(1).v_y)/dt;
	}
	else if (amount == 5) { // FIR
		retState.v_x = (  0.2f*puckStates.at(0).v_x \
										+ 0.1f*puckStates.at(1).v_x \
										- 0.1f*puckStates.at(3).v_x \
										- 0.2f*puckStates.at(4).v_x);
		retState.v_y = (  0.2f*puckStates.at(0).v_y \
										+ 0.1f*puckStates.at(1).v_y \
										- 0.1f*puckStates.at(3).v_y \
										- 0.2f*puckStates.at(4).v_y);
	}
	
	return retState;
}
