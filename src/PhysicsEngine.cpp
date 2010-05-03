#include "PhysicsEngine.h"

#include "ofMain.h"
///create 125 (5x5x5) dynamic object
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_Z 5

//maximum number of objects (and allow user to shoot additional boxes)
#define MAX_PROXIES (ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z + 1024)

///scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1.
#define START_POS_X -5
#define START_POS_Y -5
#define START_POS_Z -3

///scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1.

//--------------------------------------------------------------
PhysicsEngine::PhysicsEngine()
:
m_dynamicsWorld(0),
m_pickConstraint(0),
m_cameraDistance(15.0),
m_ele(20.),
m_azi(0.),
m_cameraPosition(0.,0.,0.),
m_cameraTargetPosition(0.,0.,0.),
m_scaleBottom(0.5f),
m_scaleFactor(2.),
m_cameraUp(0,1,0),
m_forwardAxis(2),	
m_sundirection(btVector3(1,-2,1)*1000),
oldPickingDist(0.),
pickedBody(0),
mousePickClamping(30.),
m_drag(false)
{
	m_drawer = new GL_Drawer();
	m_drawer->enableTexture(true);
	
	ofAddListener(ofEvents.setup,			this, &PhysicsEngine::_setup);
	ofAddListener(ofEvents.update,			this, &PhysicsEngine::_update);

	ofAddListener(ofEvents.keyPressed,		this, &PhysicsEngine::_keyPressed);
	ofAddListener(ofEvents.keyReleased,		this, &PhysicsEngine::_keyReleased);

	ofAddListener(ofEvents.mousePressed,	this, &PhysicsEngine::_mousePressed);
	ofAddListener(ofEvents.mouseMoved,		this, &PhysicsEngine::_mouseMoved);
	ofAddListener(ofEvents.mouseDragged,	this, &PhysicsEngine::_mouseDragged);
	ofAddListener(ofEvents.mouseReleased,	this, &PhysicsEngine::_mouseReleased);
}

//--------------------------------------------------------------
PhysicsEngine::~PhysicsEngine()
{
	destroy();
}

//--------------------------------------------------------------
void
PhysicsEngine::_setup(ofEventArgs &e)
{ setup(); }	

void
PhysicsEngine::setup()
{
	m_cameraDistance = btScalar(SCALING*50.);
	
	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//m_collisionConfiguration->setConvexConvexMultipointIterations();
	
	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
	
	m_broadphase = new btDbvtBroadphase();
	
	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;
	
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	
	m_dynamicsWorld->setGravity(btVector3(0,-10,0));
	
	///create a few basic rigid bodies
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
	//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
	
	m_collisionShapes.push_back(groundShape);
	
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-50,0));
	
	//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
	{
		btScalar mass(0.);
		
		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);
		
		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);
		
		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);
		
		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}
	
	
	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance
		
		btCollisionShape* colShape = new btBoxShape(btVector3(SCALING*1,SCALING*1,SCALING*1));
		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		m_collisionShapes.push_back(colShape);
		
		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();
		
		btScalar	mass(1.f);
		
		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);
		
		btVector3 localInertia(0,0,0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);
		
		float start_x = START_POS_X - ARRAY_SIZE_X/2;
		float start_y = START_POS_Y;
		float start_z = START_POS_Z - ARRAY_SIZE_Z/2;
		
		for (int k=0;k<ARRAY_SIZE_Y;k++)
		{
			for (int i=0;i<ARRAY_SIZE_X;i++)
			{
				for(int j = 0;j<ARRAY_SIZE_Z;j++)
				{
					startTransform.setOrigin(SCALING*btVector3(
															   btScalar(2.0*i + start_x),
															   btScalar(20+2.0*k + start_y),
															   btScalar(2.0*j + start_z)));
					
					
					//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
					btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
					btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
					btRigidBody* body = new btRigidBody(rbInfo);
					
					body->setActivationState(ISLAND_SLEEPING);
					
					m_dynamicsWorld->addRigidBody(body);
					body->setActivationState(ISLAND_SLEEPING);
				}
			}
		}
	}
	
//	m_dynamicsWorld->setDrawFlags(fDrawFlags::Nodes);
}

//--------------------------------------------------------------
void
PhysicsEngine::_exit(ofEventArgs &e)
{ destroy(); }	

void
PhysicsEngine::destroy()
{
	//cleanup in the reverse order of creation/initialization
	
	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
			delete body->getMotionState();
		
		while(m_dynamicsWorld->getNumConstraints())
		{
			btTypedConstraint*	pc=m_dynamicsWorld->getConstraint(0);
			m_dynamicsWorld->removeConstraint(pc);
			delete pc;
		}

		if (body)
			m_dynamicsWorld->removeRigidBody(body);
		else
			m_dynamicsWorld->removeCollisionObject(obj);
		
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}
	
	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
	
	delete m_dynamicsWorld;
	delete m_solver;
	delete m_broadphase;
	delete m_dispatcher;
	delete m_collisionConfiguration;
}

//--------------------------------------------------------------
void
PhysicsEngine::_update(ofEventArgs &e)
{ update(); }	

void
PhysicsEngine::update()
{
	//simple dynamics world doesn't handle fixed-time-stepping
#ifdef USE_BT_CLOCK
	btScalar dt = (btScalar)m_clock.getTimeMicroseconds();
	m_clock.reset();
	float ms = dt;
#else
	float ms = btScalar(16666.);
#endif

	///step the simulation
	if (m_dynamicsWorld)
		m_dynamicsWorld->stepSimulation(ms / 1000000.);		
}
	
//--------------------------------------------------------------
void
PhysicsEngine::_draw(ofEventArgs &e)
{ draw(); }	

void
PhysicsEngine::draw()
{
	GLfloat light_ambient[] = { btScalar(0.2), btScalar(0.2), btScalar(0.2), btScalar(1.0) };
	GLfloat light_diffuse[] = { btScalar(1.0), btScalar(1.0), btScalar(1.0), btScalar(1.0) };
	GLfloat light_specular[] = { btScalar(1.0), btScalar(1.0), btScalar(1.0), btScalar(1.0 )};
	/*	light_position is NOT default value	*/
	GLfloat light_position0[] = { btScalar(1.0), btScalar(10.0), btScalar(1.0), btScalar(0.0 )};
	GLfloat light_position1[] = { btScalar(-1.0), btScalar(-10.0), btScalar(-1.0), btScalar(0.0) };
	
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position0);
	
	glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT1, GL_POSITION, light_position1);
	
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	
	
	glShadeModel(GL_SMOOTH);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	
	glClearColor(btScalar(0.7),btScalar(0.7),btScalar(0.7),btScalar(0));
	
	//  glEnable(GL_CULL_FACE);
	//  glCullFace(GL_BACK);
	
	//toed-in stereo
/*
	float w			= ofGetWidth();
	float h			= ofGetHeight();

	double fovy		= 45;									//field of view in y-axis
	double aspect	= w/h;									//screen aspect ratio
	double nearZ	= 3.0;									//near clipping plane
	double farZ		= 30.0;									//far clipping plane
	double screenZ	= 10.0;									//screen projection plane
*/
	float depthZ	= -10.0;								//depth of the object drawing
	double IOD		= 0.5;									//intraocular distance
	
	glDrawBuffer(GL_BACK);									//draw into both back buffers
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);		//clear color and depth buffers

	glDrawBuffer(GL_BACK_LEFT);								//draw into back left buffer
	updateCamera(CAMERA_LEFT, IOD);

	glPushMatrix();
	{
		glTranslatef(0.0, 0.0, depthZ);						//translate to screenplane
		render();
	}
	glPopMatrix();

	glDrawBuffer(GL_BACK_RIGHT);							//draw into back right buffer
	updateCamera(CAMERA_RIGHT, IOD);

	glPushMatrix();
	{
		glTranslatef(0.0, 0.0, depthZ);						//translate to screenplane
		render();
	}
	glPopMatrix();
}

//--------------------------------------------------------------
void
PhysicsEngine::render()
{
	if (m_dynamicsWorld)
	{
		glDisable(GL_CULL_FACE);
		
		const btScalar scl=(btScalar)0.1;
		
		btScalar	m[16];
		btMatrix3x3	rot;rot.setIdentity();
		const int	numObjects=m_dynamicsWorld->getNumCollisionObjects();
		btVector3 wireColor(1,0,0);
		for(int i=0;i<numObjects;i++)
		{
			btCollisionObject*	colObj=m_dynamicsWorld->getCollisionObjectArray()[i];
			btRigidBody*		body=btRigidBody::upcast(colObj);
			if(body&&body->getMotionState())
			{
				btDefaultMotionState* myMotionState = (btDefaultMotionState*)body->getMotionState();
				myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);
				rot=myMotionState->m_graphicsWorldTrans.getBasis();
			}
			else
			{
				colObj->getWorldTransform().getOpenGLMatrix(m);
				rot=colObj->getWorldTransform().getBasis();
			}
			btVector3 wireColor(1.,1.0f,0.5f); //wants deactivation
			if(i&1)
				wireColor=btVector3(0.,0.0f,1.);
			///color differently for active, sleeping, wantsdeactivation states
			if (colObj->getActivationState() == 1) //active
				if (i & 1)
					wireColor += btVector3 (1.,0.,0.);
				else
					wireColor += btVector3 (.5f,0.,0.);
			
			if(colObj->getActivationState()==2) //ISLAND_SLEEPING
				if(i&1)
					wireColor += btVector3 (0.,1., 0.);
				else
					wireColor += btVector3 (0.,0.5f,0.);
			
			btVector3 aabbMin,aabbMax;
			m_dynamicsWorld->getBroadphase()->getBroadphaseAabb(aabbMin,aabbMax);
			
			aabbMin-=btVector3(BT_LARGE_FLOAT,BT_LARGE_FLOAT,BT_LARGE_FLOAT);
			aabbMax+=btVector3(BT_LARGE_FLOAT,BT_LARGE_FLOAT,BT_LARGE_FLOAT);
			
			m_drawer->drawOpenGL(m,colObj->getCollisionShape(),wireColor,false,aabbMin,aabbMax);
		}
	}	
}

//--------------------------------------------------------------
void
PhysicsEngine::updateCamera(int camera, float IOD)
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	btScalar rele = m_ele * btScalar(0.01745329251994329547);// rads per deg
	btScalar razi = m_azi * btScalar(0.01745329251994329547);// rads per deg

	btQuaternion rot(m_cameraUp,razi);
	
	btVector3 eyePos(0,0,0);
	eyePos[m_forwardAxis] = -m_cameraDistance;
	
	btVector3 forward(eyePos[0],eyePos[1],eyePos[2]);
	if (forward.length2() < SIMD_EPSILON)
		forward.setValue(1.,0.,0.);

	btVector3 right = m_cameraUp.cross(forward);
	btQuaternion roll(right,-rele);
	
	eyePos = btMatrix3x3(rot) * btMatrix3x3(roll) * eyePos;
	
	m_cameraPosition[0] = eyePos.getX();
	m_cameraPosition[1] = eyePos.getY();
	m_cameraPosition[2] = eyePos.getZ();
	m_cameraPosition += m_cameraTargetPosition;
	
	btScalar aspect;
	btVector3 extents;
	
	if (ofGetWidth() > ofGetHeight()) 
	{
		aspect = ofGetWidth() / (btScalar)ofGetHeight();
		extents.setValue(aspect * 1.0f, 1.0f,0);
	}
	else {
		aspect = ofGetHeight() / (btScalar)ofGetWidth();
		extents.setValue(1.0f, aspect*1.,0);
	}

	if (ofGetWidth() > ofGetHeight()) 
		glFrustum (-aspect, aspect, -1.0, 1.0, 1.0, 10000.0);
	else
		glFrustum (-1.0, 1.0, -aspect, aspect, 1.0, 10000.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	
	float cameraOffset = m_cameraPosition[0];
	cameraOffset += (camera==CAMERA_LEFT)? -IOD/2 : IOD/2;

	gluLookAt(cameraOffset, m_cameraPosition[1], m_cameraPosition[2], 
			  m_cameraTargetPosition[0], m_cameraTargetPosition[1], m_cameraTargetPosition[2], 
			  m_cameraUp.getX(),m_cameraUp.getY(),m_cameraUp.getZ());
}

//--------------------------------------------------------------
btVector3
PhysicsEngine::getRayTo(int x,int y)
{
	float top = 1.;
	float bottom = -1.;
	float nearPlane = 1.;
	float tanFov = (top-bottom)*0.5f / nearPlane;
	float fov = btScalar(2.0) * btAtan(tanFov);
	
	btVector3 rayFrom = m_cameraPosition;
	btVector3 rayForward = (m_cameraTargetPosition-m_cameraPosition);
	rayForward.normalize();
	float farPlane = 10000.;
	rayForward*= farPlane;
	
	btVector3 rightOffset;
	btVector3 vertical = m_cameraUp;
	
	btVector3 hor;
	hor = rayForward.cross(vertical);
	hor.normalize();
	vertical = hor.cross(rayForward);
	vertical.normalize();
	
	float tanfov = tanf(0.5f*fov);

	hor *= 2. * farPlane * tanfov;
	vertical *= 2. * farPlane * tanfov;
	
	btScalar aspect;
	
	if (ofGetWidth() > ofGetHeight()) 
	{
		aspect = ofGetWidth() / (btScalar)ofGetHeight();
		hor*=aspect;
	}
	else {
		aspect = ofGetHeight() / (btScalar)ofGetWidth();
		vertical*=aspect;
	}

	btVector3 rayToCenter = rayFrom + rayForward;
	btVector3 dHor = hor * 1./float(ofGetWidth());
	btVector3 dVert = vertical * 1./float(ofGetHeight());

	btVector3 rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
	rayTo += btScalar(x) * dHor;
	rayTo -= btScalar(y) * dVert;
	return rayTo;
}

//--------------------------------------------------------------
void
PhysicsEngine::_keyPressed(ofKeyEventArgs &e)
{ keyPressed(e.key); }	

void
PhysicsEngine::keyPressed(int key)
{
	const float STEPSIZE = 5;
	
	switch (key)
	{
		case OF_KEY_LEFT:
			m_azi -= STEPSIZE;
			if (m_azi < 0)
				m_azi += 360;
			break;
		case OF_KEY_RIGHT:
			m_azi += STEPSIZE;
			if (m_azi >= 360)
				m_azi -= 360;
			break;
		case OF_KEY_UP:
			m_ele += STEPSIZE;
			if (m_ele >= 360)
				m_ele -= 360;
			break;
		case OF_KEY_DOWN:
			m_ele -= STEPSIZE;
			if (m_ele < 0)
				m_ele += 360;
			break;
		case '+':
			m_cameraDistance -= btScalar(0.4);
			if (m_cameraDistance < btScalar(0.1))
				m_cameraDistance = btScalar(0.1);
			break;
		case '-':
			m_cameraDistance += btScalar(0.4);
			break;
	}
	updateCamera(); 
}

//--------------------------------------------------------------
void
PhysicsEngine::_keyReleased(ofKeyEventArgs &e)
{ keyReleased(e.key); }	

void
PhysicsEngine::keyReleased(int key)
{}

//--------------------------------------------------------------
void
PhysicsEngine::_mouseMoved(ofMouseEventArgs &e)
{ mouseMoved(e.x, e.y); }	

void
PhysicsEngine::mouseMoved(int x, int y)
{}

//--------------------------------------------------------------
void
PhysicsEngine::_mouseDragged(ofMouseEventArgs &e)
{ mouseDragged(e.x, e.y, e.button); }	

void
PhysicsEngine::mouseDragged(int x, int y, int button)
{
	if (m_pickConstraint)
	{
		//move the constraint pivot
		btPoint2PointConstraint* p2p = static_cast<btPoint2PointConstraint*>(m_pickConstraint);
		if (p2p)
		{
			//keep it at the same picking distance
			btVector3 newRayTo = getRayTo(x,y);
			btVector3 rayFrom;
			btVector3 oldPivotInB = p2p->getPivotInB();

			rayFrom = m_cameraPosition;
			btVector3 dir = newRayTo-rayFrom;
			dir.normalize();
			dir *= oldPickingDist;
			
			btVector3 newPivotB = rayFrom + dir;
			
			p2p->setPivotB(newPivotB);
		}
	}
}

//--------------------------------------------------------------
void
PhysicsEngine::_mousePressed(ofMouseEventArgs &e)
{ mousePressed(e.x, e.y, e.button); }	

void
PhysicsEngine::mousePressed(int x, int y, int button)
{
	//add a point to point constraint for picking
	if (m_dynamicsWorld)
	{
		btVector3 rayTo = getRayTo(x,y);
		btVector3 rayFrom = m_cameraPosition;
		
		btCollisionWorld::ClosestRayResultCallback rayCallback(rayFrom,rayTo);
		m_dynamicsWorld->rayTest(rayFrom,rayTo,rayCallback);
		if (rayCallback.hasHit())
		{
			btRigidBody* body = btRigidBody::upcast(rayCallback.m_collisionObject);
			if (body)
			{
				//other exclusions?
				if (!(body->isStaticObject() || body->isKinematicObject()))
				{
					pickedBody = body;
					pickedBody->setActivationState(DISABLE_DEACTIVATION);
					
					
					btVector3 pickPos = rayCallback.m_hitPointWorld;
//					printf("pickPos=%f,%f,%f\n",pickPos.getX(),pickPos.getY(),pickPos.getZ());
					
					btVector3 localPivot = body->getCenterOfMassTransform().inverse() * pickPos;
					
					btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*body,localPivot);
					p2p->m_setting.m_impulseClamp = mousePickClamping;
					
					m_dynamicsWorld->addConstraint(p2p);
					m_pickConstraint = p2p;
					
					//save mouse position for dragging
					oldPickingDist  = (pickPos-rayFrom).length();
					
					//very weak constraint for picking
					p2p->m_setting.m_tau = 0.1;
				}
			}
		}
	}
}

//--------------------------------------------------------------
void
PhysicsEngine::_mouseReleased(ofMouseEventArgs &e)
{ mouseReleased(e.x, e.y, e.button); }	

void
PhysicsEngine::mouseReleased(int x, int y, int button)
{
	m_drag=false;
//	m_results.fraction=1.;
	
	if (m_pickConstraint && m_dynamicsWorld)
	{
		m_dynamicsWorld->removeConstraint(m_pickConstraint);
		delete m_pickConstraint;
		//printf("removed constraint %i",gPickingConstraintId);
		m_pickConstraint = 0;
		pickedBody->forceActivationState(ACTIVE_TAG);
		pickedBody->setDeactivationTime( 0. );
		pickedBody = 0;
	}
}

/*
 //--------------------------------------------------------------
void PhysicsEngine::touchDown(float x, float y, int touchId, ofxMultiTouchCustomData *data)
{}

//--------------------------------------------------------------
void PhysicsEngine::touchMoved(float x, float y, int touchId, ofxMultiTouchCustomData *data)
{}

//--------------------------------------------------------------
void PhysicsEngine::touchUp(float x, float y, int touchId, ofxMultiTouchCustomData *data)
{}

//--------------------------------------------------------------
void PhysicsEngine::touchDoubleTap(float x, float y, int touchId, ofxMultiTouchCustomData *data)
{}
*/
