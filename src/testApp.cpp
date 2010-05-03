#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup()
{
	ofBackground(0, 0, 0);
//	ofSetBackgroundAuto(true);
	ofDisableSetupScreen();
	
	font.loadFont("fonts/FontinSans.ttf",6);
	//	font_sm.loadFont("fonts/FontinSans.ttf",2);
	
	ofxMultiTouch.addListener(this);
	magicMouseListener.setup();
}

//--------------------------------------------------------------
void testApp::update()
{}

//--------------------------------------------------------------
void testApp::draw()
{
	physics.draw();
/*
	ofSetupScreen();

	ofFill();
	ofSetColor(0x00aa00);
	
	map<int, pair<ofRectangle,float> >::iterator touch_it;
	for (touch_it = touches.begin(); touch_it != touches.end(); touch_it++)
	{
		ofRectangle& rect	= touch_it->second.first;
		float& angle		= touch_it->second.second;
		
		ofPushMatrix();
		ofTranslate(rect.x, rect.y, 0.0);
		ofRotate(angle, 0., 0., 1.);
		ofTranslate(-rect.width*2, -rect.height*2, 0.0);
		
		ofEllipse(0, 0, rect.width*4, rect.height*4);
		
		ofPopMatrix();
	}
*/
}

//--------------------------------------------------------------
void testApp::keyPressed(int key)
{
	//	switch (key){
	//	}	
}

//--------------------------------------------------------------
void testApp::keyReleased(int key)
{}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y )
{}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}

//--------------------------------------------------------------
void testApp::touchDown(float x, float y, int touchId, ofxMultiTouchCustomData *data)
{
	MagicMouseTouchData* touchData = (MagicMouseTouchData*)(data);
	ofRectangle ellipsoid(x*ofGetWidth(),y*ofGetHeight(),
						  touchData->majorAxis, touchData->minorAxis);
	touches[touchId] = make_pair(ellipsoid, touchData->angle);
}

//--------------------------------------------------------------
void testApp::touchMoved(float x, float y, int touchId, ofxMultiTouchCustomData *data)
{
	MagicMouseTouchData* touchData = (MagicMouseTouchData*)(data);
	ofRectangle ellipsoid(x*ofGetWidth(),y*ofGetHeight(),
						  touchData->majorAxis, touchData->minorAxis);
	touches[touchId] = make_pair(ellipsoid, touchData->angle);
}

//--------------------------------------------------------------
void testApp::touchUp(float x, float y, int touchId, ofxMultiTouchCustomData *data)
{
	touches.erase(touchId);
}

//--------------------------------------------------------------
void testApp::touchDoubleTap(float x, float y, int touchId, ofxMultiTouchCustomData *data)
{}
