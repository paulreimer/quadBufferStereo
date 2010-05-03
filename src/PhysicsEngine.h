#pragma once

///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "GL_Drawer.h"

#include "ofEvents.h"

class PhysicsEngine
{
public:
	PhysicsEngine();
	~PhysicsEngine();

	void _setup			(ofEventArgs &e);
	void _update		(ofEventArgs &e);
	void _draw			(ofEventArgs &e);
	void _exit			(ofEventArgs &e);
	
	void setup();
	void update();
	void draw();
	void destroy();

	void _keyPressed	(ofKeyEventArgs &e);
	void _keyReleased	(ofKeyEventArgs &e);
	void _mouseMoved	(ofMouseEventArgs &e);
	void _mouseDragged	(ofMouseEventArgs &e);
	void _mousePressed	(ofMouseEventArgs &e);
	void _mouseReleased	(ofMouseEventArgs &e);

	void keyPressed		(int key);
	void keyReleased	(int key);
	void mouseMoved		(int x, int y);
	void mouseDragged	(int x, int y, int button);
	void mousePressed	(int x, int y, int button);
	void mouseReleased	(int x, int y, int button);
/*
	void touchDown(float x, float y, int touchId, ofxMultiTouchCustomData *data = NULL);
	void touchMoved(float x, float y, int touchId, ofxMultiTouchCustomData *data = NULL);
	void touchUp(float x, float y, int touchId, ofxMultiTouchCustomData *data = NULL);
	void touchDoubleTap(float x, float y, int touchId, ofxMultiTouchCustomData *data = NULL);
*/
	btVector3				m_cameraPosition;
	btVector3				m_cameraTargetPosition;//look at
	bool					m_drag;
	int						m_lastmousepos[2];
	
	btVector3 getRayTo(int x,int y);
protected:
	void render();
	
	enum cameraTypes {
		CAMERA_LEFT,
		CAMERA_RIGHT,
	};

	void updateCamera(int camera=CAMERA_LEFT, float IOD=0.);

private:
	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;
	btBroadphaseInterface*					m_broadphase;
	btCollisionDispatcher*					m_dispatcher;
	btConstraintSolver*						m_solver;	
	btDefaultCollisionConfiguration*		m_collisionConfiguration;
	
	btDiscreteDynamicsWorld*				m_dynamicsWorld;

#ifdef USE_BT_CLOCK
	btClock					m_clock;
#endif //USE_BT_CLOCK
	
	///constraint for mouse picking
	btTypedConstraint*		m_pickConstraint;

	float					m_cameraDistance;
	
	float					m_ele;
	float					m_azi;
	
protected:	
	float					m_scaleBottom;
	float					m_scaleFactor;
	btVector3				m_cameraUp;
	int						m_forwardAxis;
	
	GL_Drawer*				m_drawer;

	btVector3				m_sundirection;	

	float					oldPickingDist;
	btRigidBody*			pickedBody;
	btScalar				mousePickClamping;
};
