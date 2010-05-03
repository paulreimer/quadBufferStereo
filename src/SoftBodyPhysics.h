#pragma once

#include "PhysicsEngine.h"

class SoftBodyPhysics
: public PhysicsEngine
{
public:
	void setup();
	void update();
	void draw();
};
