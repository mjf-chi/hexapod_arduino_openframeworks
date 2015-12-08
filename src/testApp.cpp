#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup(){
	//everything with "of" in front of it is for the visuals of the simulation
	//ofSetBackgroundAuto(false);
	ofBackground(0);
	ofEnableAlphaBlending();
	ofEnableLighting();
	ofSetSmoothLighting(true);

	serial.setup("COM4", 9600); //allowing for communication with the arduino

	myLight.setPosition(400, 20, 0);
	myLight.setAmbientColor(ofFloatColor(ofColor(255, 20, 0)));
	myLight.enable();

	inLight = false;

	secondXZangle = 0;
	XYangle = 1.5707; //this keeps the angle parrallel to the y-axis
	XZangle = 0; //updating the periodic motion angle
	yRotation = 0; //this rotates the period around the y-axis, changing direction

	bodyJointPoint = ofVec3f(15, 59.5, 0); //the point where the leg connects to the body

	distanceFromHip = 44.58; //distance from servo1 rotatingpoint to servo2 point, servo1 is connected to the body, servo2 holds onto the second part of the leg
	distanceToKnee = 44.02; //distance from servo2 rotating point to servo3 rotating point
	distanceToFoot = 71.08; //distance from servo3 rotating point to the ground

	walking = true; //stops and starts the motion

	xAngleAdjustment = 0;
	yAngleAdjustment = 0;

	numLegs = 6;
	amplitude = 10;
	changeInAngle = 0.025;

	float offsetBegin = 0;

	for(int i = 0; i< numLegs; i++)
	{
		offsetVals.push_back(ofDegToRad(offsetBegin));

		offsetBegin += 180;
	}

	dataReceived = false;
}

//--------------------------------------------------------------
void testApp::update(){
	
}

//--------------------------------------------------------------
void testApp::draw(){
	//all this ofStuff is just for visualizing the data

	ofPushMatrix();
	ofTranslate(ofGetWidth()/2, ofGetHeight()/2, 0);
	ofRotateZ(180);

		//need to setup the ability to offset these values
		//set to zero there are negative points
		ofPoint initialPoint(0, 0, 0);

		for(int i = 0; i < numLegs; i++)
		{
			fillOffsetPointsVector(offsetVals[i], initialPoint);
		}

		int theBuf = 0;

		if(!dataReceived)
		{
			if(serial.available() > 0)
			{
				theBuf = serial.readByte();
				dataReceived = true;

				cout << "val = " << theBuf << endl;
			}

			else
			{
				walking = false;
			}
		}

		if(serial.available() > 0)
		{
			cout << "val = " << serial.readByte();
		}
		//float directionVal = getDirection();

		//cout << "zPointTwo = " << zPointTwo << endl; 
		//cout << "xPointTwo = " << xPointTwo << endl;
		//vector<int> photoVals;
		
		//while(serial.available > 0)
		//{
		//buffer += serial.readByte();
		//}
		//vector<bytes> values = ofSplitString(buffer, ",");
		//for(int i = 0l i< values.size(); i++)
		//{
		//photoVals.push_back(values[i]);
		//}

		//yRotation = getDirection(photoVals);

		/*
		ofFill();

		ofPushStyle();
		ofSetColor(0, 0, 0, 1);
		ofRect(-200, -200, 500, ofGetWidth()*2, ofGetHeight()*2);
		ofPopStyle();

		ofSetColor(255);
		ofSetLineWidth(4);
		ofLine(intialPoint.x, intialPoint.y, intialPoint.z, xPointTwo, yPointTwo, zPointTwo);
		ofSphere(xPointTwo, yPointTwo, zPointTwo, 2);
		*/

		//if(inLight)
		//{
		//	walking = false;
		//}
		//else
		//{
		//	walking = true;
		//}

		if(dataReceived)
		{
			if(theBuf > 0)
			{
				walking = true;
			}

			else
			{
				walking = false;
			}
		}

		if(walking) //this boolean  lets me turn the rotation on or off
			{
				XZangle += changeInAngle;
			}

			designators.push_back(initialPoint);

			for(int i = 0; i < 5; i++)
			{
				//designators.push_back(rotateDesignatorPointAroundBodyJointPoint(radtodegDR, initialPoint, initialPoint)); 
				//this rotates the designator original point around the body point
				designators.push_back(initialPoint);
			}

			float designatorRotationAmount = 0;
	
			for(int i = 0; i < designators.size(); i++)
			{
				designatorRotationAmount += 30;
				float radtodegDR = ofDegToRad(designatorRotationAmount);
	
				newDesignators.push_back(ofPoint(designators[i].x + offsetPoints[i].x, designators[i].y + offsetPoints[i].y, designators[i].z + offsetPoints[i].z));
				newDesignators[i] = rotateDesignatorPointAroundBodyJointPoint(radtodegDR, initialPoint, newDesignators[i]);
			}
			
			for(int i = 0; i < newDesignators.size(); i++)
			{
				ofSphere(newDesignators[i], 2);
			}

			//yRotation = 0;
			ofSphere(bodyJointPoint, 15);

		ofPopMatrix();

		//INVERSE KINEMATICS ALGORITHMS

		//cout << "BEGIN IK ALGORITHMS" << endl;

		for(int i = 0; i< newDesignators.size(); i++)
		{
			vector<float> gotAngles = getAnglesForPoint(bodyJointPoint, newDesignators[i].x , newDesignators[i].y, newDesignators[i].z);
			legAngles.push_back(gotAngles);
		}
	
	//	cout << "FILL LEG ANGLES COMPLETE" << endl;
	
		int serialSent = 0;

		vector<unsigned char> sendArduino;

		for(int i = 0; i<legAngles.size(); i++)
		{
			for(int j = 0; j < legAngles[i].size(); j++)
			{
				if(serialSent < 18)
				{
					sendArduino.push_back(unsigned char(int(legAngles[i][j])));
					//serial.writeByte(unsigned char(int(legAngles[i][j])));
					serialSent++;
				}
			}
		}

		legAngles.clear();
		designators.clear();
		newDesignators.clear();
		offsetPoints.clear();

		if(walking)
		{
			for(int i = 0; i < offsetVals.size(); i++)
			{
				offsetVals[i] += changeInAngle;
			}
		}
		
		if(dataReceived)
		{
			serial.writeBytes(&sendArduino[0], 18);
			serial.writeByte(255);

			dataReceived = false;
		}

		//COME BACK HERE TO RESET 
		/*
			if(walking) //this boolean  lets me turn the rotation on or off
			{
				XZangle += changeInAngle;
			}

			designators.push_back(initialPoint);

			for(int i = 0; i < 5; i++)
			{
				//designators.push_back(rotateDesignatorPointAroundBodyJointPoint(radtodegDR, initialPoint, initialPoint)); 
				//this rotates the designator original point around the body point
				designators.push_back(initialPoint);
			}

			float designatorRotationAmount = 0;
	
			for(int i = 0; i < designators.size(); i++)
			{
				designatorRotationAmount += 30;
				float radtodegDR = ofDegToRad(designatorRotationAmount);
	
				newDesignators.push_back(ofPoint(designators[i].x + offsetPoints[i].x, designators[i].y + offsetPoints[i].y, designators[i].z + offsetPoints[i].z));
				newDesignators[i] = rotateDesignatorPointAroundBodyJointPoint(radtodegDR, initialPoint, newDesignators[i]);
			}
			
			for(int i = 0; i < newDesignators.size(); i++)
			{
				ofSphere(newDesignators[i], 2);
			}

			//yRotation = 0;
			ofSphere(bodyJointPoint, 15);

		ofPopMatrix();

		//INVERSE KINEMATICS ALGORITHMS

		//cout << "BEGIN IK ALGORITHMS" << endl;

		for(int i = 0; i< newDesignators.size(); i++)
		{
			vector<float> gotAngles = getAnglesForPoint(bodyJointPoint, newDesignators[i].x , newDesignators[i].y, newDesignators[i].z);
			legAngles.push_back(gotAngles);
		}
	
	//	cout << "FILL LEG ANGLES COMPLETE" << endl;
	
		int serialSent = 0;

		vector<unsigned char> sendArduino;

		for(int i = 0; i<legAngles.size(); i++)
		{
			for(int j = 0; j < legAngles[i].size(); j++)
			{
				if(serialSent < 18)
				{
					sendArduino.push_back(unsigned char(int(legAngles[i][j])));
					//serial.writeByte(unsigned char(int(legAngles[i][j])));
					serialSent++;
				}
			}
		}

		legAngles.clear();
		designators.clear();
		newDesignators.clear();
		offsetPoints.clear();

		if(walking)
		{
			for(int i = 0; i < offsetVals.size(); i++)
			{
				offsetVals[i] += changeInAngle;
			}
		}

		serial.writeBytes(&sendArduino[0], 18);
		serial.writeByte(255);
	}
	*/
	//RESET HERE TOO
}

void testApp::fillOffsetPointsVector(float offset, ofPoint beginPoint)
{
		float xPointTwo = beginPoint.x + cos(XYangle)*sin(offset)*amplitude; //this is the desired point xLocation
		float yPointTwo = beginPoint.y + sin(offset)*sin(XYangle)*amplitude; //yLocation
		float zPointTwo = beginPoint.z + cos(offset)*amplitude; //zLocation
		
		float newXPoint = xPointTwo; //these values help me adjust the secondary points based on the yRotation
		float newZPoint = zPointTwo;
		
		zPointTwo = zPointTwo*cos(yRotation) - xPointTwo*sin(yRotation);
		xPointTwo = newZPoint*sin(yRotation) + xPointTwo*cos(yRotation);
		
		offsetPoints.push_back(ofPoint(xPointTwo, yPointTwo, zPointTwo));
}
;
ofPoint testApp::rotateDesignatorPointAroundBodyJointPoint(float rotationAmount, ofPoint rotationAxisPoint, ofPoint designator)
{
	ofPoint newDesignator = designator;

	newDesignator.x -= rotationAxisPoint.x;

	float newZpoint = newDesignator.z;

	newDesignator.z = newDesignator.z *cos(rotationAmount) - newDesignator.x*sin(rotationAmount);
	newDesignator.x = newZpoint * sin(rotationAmount) + newDesignator.x*cos(rotationAmount);

	newDesignator.x += rotationAxisPoint.x;

	return newDesignator;
}

vector<float> testApp::getAnglesForPoint(ofPoint bodyJoint, float xPoint, float yPoint, float zPoint)
{
	float bodyJointServo = atan2(double(abs(zPoint - bodyJoint.z)), double(abs(xPoint - bodyJoint.x))); 

	//actualX is the projection of X onto the 2D plane
	float actualX = (xPoint - bodyJoint.x) * cos(bodyJointServo);

	//this is the length from the rotating point of the second servo to the desired point on the 2D plane that x is projected onto
	float lengthFromCOXA = sqrt(pow(bodyJoint.y, 2) + (pow((actualX - distanceFromHip), 2)));

	//servo2's angle = alpha
	//alpha is split in two in order for easier deciphering of each individual angle
	//the line from servo2's rotation point to the desired point in space divides the plane(desired point, servo3 rotation 
	//point, servo2 rotation point, into two triangles
	float alpha1 = acos(((bodyJoint.y - yPoint)/lengthFromCOXA));

	//law of cosines to find alpha2
	float alpha2 = acos((pow(distanceToFoot, 2)-pow(distanceToKnee, 2) - pow(lengthFromCOXA, 2))/(-2*(distanceToKnee)*(lengthFromCOXA)));

	float totalHipJointAngle = alpha1 + alpha2;

	float beta = acos((pow(lengthFromCOXA, 2) - pow(distanceToFoot, 2) - pow(distanceToKnee, 2))/(-2*(distanceToFoot)*(distanceToKnee)));

	float kneeJointServoAngle = beta;
	//cout << xPointTwo << " , " << yPointTwo << " , " << zPointTwo << endl;

	bodyJointServo = ofRadToDeg(bodyJointServo);
	totalHipJointAngle = ofRadToDeg(totalHipJointAngle); //for the servo data
	kneeJointServoAngle = ofRadToDeg(kneeJointServoAngle);

	//totalHipJointAngle += 90;

	if(zPoint < 0)
	{
		bodyJointServo *= -1;
	}

	bodyJointServo += 90;

	ofPoint kneePoint;

	kneePoint.z = bodyJoint.z + sin(ofDegToRad(bodyJointServo)) * distanceToKnee;
	kneePoint.y = bodyJoint.y + cos(ofDegToRad(totalHipJointAngle)) * distanceToKnee;
	kneePoint.x = bodyJoint.x + sin(ofDegToRad(totalHipJointAngle)) * distanceToKnee;
	//(xPointTwo - bodyJointPoint.x) * cos(bodyJointServo);

	ofPoint footPoint;

	footPoint.z = kneePoint.z + sin(ofDegToRad(bodyJointServo)) * distanceToFoot;
	footPoint.y = kneePoint.y + cos(ofDegToRad(kneeJointServoAngle)) * distanceToFoot;
	footPoint.x = kneePoint.x + sin(ofDegToRad(kneeJointServoAngle) )* distanceToFoot;

	ofPushMatrix();
	ofTranslate(ofGetWidth()/2, ofGetHeight()/2, 0);
	ofRotateZ(180);
	ofPushStyle();
	ofFill();
	ofSetColor(255);
	ofSetLineWidth(2);
	ofSphere(kneePoint, 2);
	ofLine(bodyJoint, kneePoint);
	ofLine(kneePoint, footPoint);
	ofPopStyle();
	ofPopMatrix();

	vector<float> anglesToReturn;
	anglesToReturn.push_back(bodyJointServo);
	anglesToReturn.push_back(totalHipJointAngle);
	anglesToReturn.push_back(kneeJointServoAngle);

	return anglesToReturn;
}

float testApp::getDirection()
{
	vector<unsigned char> getArduino;

	//cout << "running getDirection" << endl;

	if(serial.available())
	{
	int photoValString = serial.readBytes(&getArduino[0], 3);

	cout << photoValString << endl;
	}
	/*
	if any of the of the photoResistVals >= someValue then inLight = true, stop walking
	always finding the ideal direction to walk
	just a matter of turning the walk on or off
	*/
	float directionAngle;

	directionAngle = 0;

	dataReceived = true;

	return directionAngle;
}

//--------------------------------------------------------------
void testApp::keyPressed(int key){
	if(key == 'a')
	{
		yRotation += 0.05;
	}

	if(key == 's')
	{
		yRotation -= 0.05;
	}

	if(key == 'd')
	{
		walking = !walking;
	}

	if(key == 'i')
	{
		bodyJointPoint.x += 0.1;
		cout << "x: " << bodyJointPoint.x << endl;
	}
	
	if(key == 'o')
	{
		bodyJointPoint.x -= 0.1;
		cout << "x: " << bodyJointPoint.x << endl;
	}

	if(key == 'h')
	{
		bodyJointPoint.y += 0.1;
		cout << "y: " << bodyJointPoint.y << endl;
	}

	if(key == 'n')
	{
		bodyJointPoint.y -= 0.1;
		cout << "y: " << bodyJointPoint.y << endl;
	}

	if(key == 'j')
	{
		bodyJointPoint.z += 0.1;
		cout << "z: " << bodyJointPoint.z << endl;
	}

	if(key == 'k')
	{
		bodyJointPoint.z -= 0.1;
		cout << "z: " << bodyJointPoint.z << endl;
	}
}

//--------------------------------------------------------------
void testApp::keyReleased(int key){

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo){ 

}
