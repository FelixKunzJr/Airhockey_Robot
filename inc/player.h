#pragma once

typedef enum {
	NO_DANGER, // No need to do anything/ puck is moving away
	BLOCK, // Unsure of puck velocity
	SHOOT, // Puck is close -> shoot
} playerSM;
