#pragma once

#define cmath

#include "ofMain.h"
#include <vector>

class testApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		//float getDirection(vector<int> photoResistVals);
		float getDirection();
		void fillOffsetPointsVector(float offset, ofPoint beginPoint);
		ofPoint rotateDesignatorPointAroundBodyJointPoint(float rotationAmount, ofPoint rotationAxisPoint, ofPoint designator);
		vector<float> getAnglesForPoint(ofPoint bodyPoint, float xPoint, float yPoint, float zPoint);
		vector<float> lawOfCosinesAngles(float lengthOne, float lengthTwo, float lengthThree);

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

		ofVec3f whereToGo;
		bool inLight;

		ofLight myLight;
		
		float secondXZangle;
		float XYangle;
		float XZangle;
		float yRotation;

		float theta;

		ofSerial serial;

		ofVec3f bodyJointPoint;

		float yAngleAdjustment;
		float xAngleAdjustment;

		float servoOneAngle;
		float servoTwoAngle;
		float servoThreeAngle;

		float distanceToKnee;
		float distanceToFoot;
		float distanceFromHip;

		float overallLength2D;

		bool walking;

		vector<float> offsetVals;
		vector<ofPoint> offsetPoints;
		int numLegsWalking;

		vector<vector<float>> legAngles;

		vector<ofPoint> newDesignators;
		vector<ofPoint> designators;

		int numLegs;

		float amplitude;
		float changeInAngle;

		bool dataReceived;
};
