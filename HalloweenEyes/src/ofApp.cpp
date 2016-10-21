#include "ofApp.h"
#include <math.h>

//--------------------------------------------------------------
void ofApp::setup() {
	//ofSetLogLevel(OF_LOG_VERBOSE);
	ofSetLogLevel(OF_LOG_NOTICE);

	//Set up serial communication to arduino
	serial.listDevices();
	vector <ofSerialDeviceInfo> deviceList = serial.getDeviceList();

	int baud = 57600;
	serial.setup("COM8", baud);

	// enable depth->video image calibration
	kinect.setRegistration(true);

	//start up kinect
	kinect.init(true,false);
	kinect.open();

	if (kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}

	//Grab some memory for frames
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);

	//set up openFrameworks target framerate
	ofSetFrameRate(30);

	// reset tilt
	angle = 0;
	kinect.setCameraTiltAngle(angle);

	//setup 3d render
	camViewport.set(RENDER_X_OFFSET, RENDER_Y_OFFSET, RENDER_WIDTH, RENDER_HEIGHT);
	cam.setPosition(0.0, 1000, 0.0);
	cam.rotate(-90.0, 1.0, 0.0, 0.0);

	//Default location for target box
	targetBox.set(50);
	targetBox.setPosition(0, 0, 0);
	leftEyeParentBox.set(10);
	rightEyeParentBox.set(10);

	//Set target offsets and location scale
	targetBoxLocOffset.set(0.0, 0.0, 10.0);
	targetBoxLocScale.set(1.0, 1.0, 1.0);

	//Set up Eye physical -> virtual offsets
	leftEyeDefaultLoc.set(-100.0, 0.0, 100.0);
	rightEyeDefaultLoc.set(100.0, 0.0, 100.0);
	leftEyeLocOffset.set(0.0, 0.0, 0.0);
	rightEyeLocOffset.set(0.0, 0.0, 0.0);

	//Set up left and right eyes
	eyeUpVector.set(0.0, 1.0, 0.0);
	leftEyeCone.set(25, 25, 5, 2);
	rightEyeCone.set(25, 25, 5, 2);
	leftEyeParentBox.setPosition(leftEyeDefaultLoc);
	rightEyeParentBox.setPosition(rightEyeDefaultLoc);
	leftEyeCone.setParent(leftEyeParentBox);
	rightEyeCone.setParent(rightEyeParentBox);
	leftEyeCone.rotate(90, 1.0, 0, 0);
	rightEyeCone.rotate(90, 1.0, 0, 0);

	//Set up GUI
	eyePanel.setup();
	eyePanel.add(leftEyeHorizLocEyeSlider.setup("LEH Slider", 0, -500.0, 500.0));
	eyePanel.add(rightEyeHorizLocEyeSlider.setup("REH Slider", 0, -500.0, 500.0));

	targetPanel.setup();
	targetPanel.add(targetLocScaleXSlider.setup("Target Scale X", targetBoxLocScale.x, 0.1, 10));
	targetPanel.add(targetLocScaleYSlider.setup("Target Scale Y", targetBoxLocScale.y, 0.1, 10));
	targetPanel.add(targetLocScaleZSlider.setup("Target Scale Z", targetBoxLocScale.z, 0.1, 10));
	targetPanel.add(targetLocOffsetXSlider.setup("Taget LocOffset X", targetBoxLocOffset.x, -10, 20));
	targetPanel.add(targetLocOffsetYSlider.setup("Taget LocOffset Y", targetBoxLocOffset.y, -10, 20));
	targetPanel.add(targetLocOffsetZSlider.setup("Taget LocOffset Z", targetBoxLocOffset.z, 0, 20));

}

//--------------------------------------------------------------
void ofApp::update() {
	ofBackground(100, 100, 100);

	kinect.update();
	ofxCvBlob blob;
	bool newBlob = false;
	int brightestPixelBrightness = -1;
	int brightestPixelIndex = -1;

	// there is a new frame and we are connected
	if (kinect.isFrameNew()) {

		//Grab Eye Locations Slider Values
		leftEyeLocOffset.set(leftEyeHorizLocEyeSlider, leftEyeLocOffset.y, leftEyeLocOffset.z);
		rightEyeLocOffset.set(rightEyeHorizLocEyeSlider, rightEyeLocOffset.y, rightEyeLocOffset.z);

		//Grab Target location/scale slider values
		targetBoxLocOffset.set(targetLocOffsetXSlider, targetLocOffsetYSlider, targetLocOffsetZSlider);
		targetBoxLocScale.set(targetLocScaleXSlider, targetLocScaleYSlider, targetLocOffsetZSlider);

		//Set up virtual eye locations
		leftEyeParentBox.setPosition(leftEyeDefaultLoc + leftEyeLocOffset);
		rightEyeParentBox.setPosition(rightEyeDefaultLoc + rightEyeLocOffset);

		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);

		//get pointer to pixels
		unsigned char * pix = grayImage.getPixels();

		//calc number of linear pixels
		int numPixels = grayImage.getWidth() * grayImage.getHeight();

		//Search for closest/brightest pixel
		for (int p = 0; p < numPixels; p++) {
			if (pix[p] > brightestPixelBrightness) {
				brightestPixelBrightness = pix[p];
				brightestPixelIndex = p;
			}
		}

		frontThreshold = brightestPixelBrightness;
		backThreshold = frontThreshold - threshDepth;

		for (int i = 0; i < numPixels; i++) {
			if (pix[i] < frontThreshold && pix[i] > backThreshold) {
				pix[i] = 255;
			}
			else {
				pix[i] = 0;
			}
		}
	
		// update the cv images
		grayImage.flagImageChanged();

		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		newBlob = false;
		contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height) / 2, 1, false);
		if (contourFinder.nBlobs > 0) {
			newBlob = true;
			blob = contourFinder.blobs[0];
		}

		//Update target box location
		int targetBoxX = ((128 - mapInt((int)blob.centroid.x, 0, kinect.width, 0, 254)) * targetBoxLocScale.x) + targetBoxLocOffset.x;
		int targetBoxY = ((128 - mapInt((int)blob.centroid.y, 0, kinect.width, 0, 254)) * targetBoxLocScale.y) + targetBoxLocOffset.y;
		int targetBoxZ = (-1 * (255 - frontThreshold) * targetBoxLocScale.z) + targetBoxLocOffset.z;

		//ofLogNotice("Mapped Z: ") << targetBoxZ;

		//Calculate Lookat for preview draw
		targetBox.setPosition(targetBoxX, targetBoxY, targetBoxZ);

		leftEyeParentBox.lookAt(targetBox, eyeUpVector);
		rightEyeParentBox.lookAt(targetBox, eyeUpVector);

	

		//send new info to arduino
		if (serial.isInitialized() && newBlob) {

			//Calculate horizontal / vertial servo positions
			//left eye
			ofVec3f leftEyeOrientation = leftEyeParentBox.getOrientationEuler();
			unsigned char leftEyeHorizontal = (mapInt(leftEyeOrientation.y, -90.0, 90.0, 0, 254) + LH_SERVO_OFFSET);
			unsigned char leftEyeVertical = mapInt(leftEyeOrientation.x, -90.0, 90.0, 0, 254) + LV_SERVO_OFFSET;

			//right eye
			ofVec3f rightEyeOrientation = rightEyeParentBox.getOrientationEuler();
			unsigned char rightEyeHorizontal = mapInt(rightEyeOrientation.y, -90.0, 90.0, 0, 254) + RH_SERVO_OFFSET;
			unsigned char rightEyeVertical = mapInt(rightEyeOrientation.x, -90.0, 90.0, 0, 254) + RV_SERVO_OFFSET;

			ofLogNotice("Horiz Rotation: ") << leftEyeOrientation.y;
			ofLogNotice("Left Eye: ") << (unsigned int)leftEyeHorizontal << "," << (unsigned int)leftEyeVertical;
			//ofLogNotice("Right Eye: ") << (unsigned int)rightEyeHorizontal << "," << (unsigned int)rightEyeVertical;


			//We will send 1 initByte of 255 and 4 bytes, 0-254 for each servo (5 bytes total)
			//InitByte
			unsigned char initByte = 255;
			//ofLogNotice() << blob.centroid.x;
		
			//Byte0: left eye x
			unsigned char byte0 = leftEyeHorizontal;
		
			//Byte1: left eye y
			unsigned char byte1 = leftEyeVertical;

			//Byte2: right eye x
			unsigned char byte2 = rightEyeHorizontal;
		
			//Byte3: right eye y
			unsigned char byte3 = rightEyeVertical;

			//Write bytes
			unsigned char buf[5] = { initByte, byte0, byte1, byte2, byte3};
			serial.writeBytes(&buf[0], 5);
			//ofLogNotice("ofApp") << (int)byte0;

			//Log out info from Arduino
			static string str;
			stringstream ss;
			char ch;
			int readLimit = 1000;
			bool anyBytes = false;
			while ((ch = serial.readByte()) > 0 && readLimit-- > 0) {
				ss << ch;
				anyBytes = true;
			}

			if (anyBytes) {
				str += ss.str();
				ofLog(OF_LOG_NOTICE, str);
				anyBytes = false;
			}
		}
	}

}

//--------------------------------------------------------------
void ofApp::draw() {
	ofSetColor(255, 255, 255);

	// draw from the live kinect
	kinect.drawDepth(10, 10, 400, 300);
	//kinect.draw(420, 10, 400, 300);

	grayImage.draw(10, 320, 400, 300);
	contourFinder.draw(10, 320, 400, 300);

	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;

	if (kinect.hasAccelControl()) {
		reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
			<< ofToString(kinect.getMksAccel().y, 2) << " / "
			<< ofToString(kinect.getMksAccel().z, 2) << endl;
	}
	else {
		reportStream << "Note: this is a newer Xbox Kinect or Kinect For Windows device," << endl
			<< "motor / led / accel controls are not currently supported" << endl << endl;
	}

	reportStream << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
		<< "threshDepth: " << threshDepth << endl
		<< "set near threshold " << frontThreshold << " (press: + -)" << endl
		<< "set far threshold " << backThreshold << endl 
		<< " num blobs found " << contourFinder.nBlobs
		<< ", fps: " << ofGetFrameRate() << endl
		<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;

	if (kinect.hasCamTiltControl()) {
		reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
			<< "press 1-5 & 0 to change the led mode" << endl;
	}

	ofDrawBitmapString(reportStream.str(), 20, 652);

	//Draw 3D render
	cam.begin(camViewport);
	ofSetColor(255, 0, 0, 255);
	targetBox.draw();
	leftEyeCone.draw();
	rightEyeCone.draw();
	cam.end();

	//Draw GUI
	eyePanel.draw();
	targetPanel.draw();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
	switch (key) {
	case ' ':
		bThreshWithOpenCV = !bThreshWithOpenCV;
		break;

	case '=':
	case '+':
		threshDepth++;
		threshDepth > 255 ? threshDepth = 255 : threshDepth = threshDepth;
		break;

	case '-':
		threshDepth--;
		threshDepth < 0  ? threshDepth = 0 : threshDepth = threshDepth;
		break;

	case 'w':
		kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
		break;

	case 'o':
		kinect.setCameraTiltAngle(angle); // go back to prev tilt
		kinect.open();
		break;

	case 'c':
		kinect.setCameraTiltAngle(0); // zero the tilt
		kinect.close();
		break;

	case '1':
		kinect.setLed(ofxKinect::LED_GREEN);
		break;

	case '2':
		kinect.setLed(ofxKinect::LED_YELLOW);
		break;

	case '3':
		kinect.setLed(ofxKinect::LED_RED);
		break;

	case '4':
		kinect.setLed(ofxKinect::LED_BLINK_GREEN);
		break;

	case '5':
		kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
		break;

	case '0':
		kinect.setLed(ofxKinect::LED_OFF);
		break;

	case OF_KEY_UP:
		angle++;
		if (angle>30) angle = 30;
		kinect.setCameraTiltAngle(angle);
		break;

	case OF_KEY_DOWN:
		angle--;
		if (angle<-30) angle = -30;
		kinect.setCameraTiltAngle(angle);
		break;
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}

//--------------------------------------------------------------
void ofApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();

}

//--------------------------------------------------------------
int ofApp::mapInt(int x, int in_min, int in_max, int out_min, int out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
