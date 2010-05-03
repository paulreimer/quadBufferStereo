#pragma once
#define USE_GUI

#include "ofMain.h"
#include "MagicMouseListener.h"
#include "PhysicsEngine.h"

class testApp : public ofBaseApp, public ofxMultiTouchListener
{
public:
	void setup();
	void update();
	void draw();
	
	void keyPressed  (int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y );
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	
	void touchDown(float x, float y, int touchId, ofxMultiTouchCustomData *data = NULL);
	void touchMoved(float x, float y, int touchId, ofxMultiTouchCustomData *data = NULL);
	void touchUp(float x, float y, int touchId, ofxMultiTouchCustomData *data = NULL);
	void touchDoubleTap(float x, float y, int touchId, ofxMultiTouchCustomData *data = NULL);
	
	MagicMouseListener magicMouseListener;
	PhysicsEngine physics;

	ofTrueTypeFont font;

	map<int, pair<ofRectangle,float> > touches;
};
