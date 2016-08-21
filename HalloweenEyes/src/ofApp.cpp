#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);

	// enable depth->video image calibration
	kinect.setRegistration(true);

	//start up kinect
	kinect.init(true);
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
	ofSetFrameRate(60);

	// reset tilt
	angle = 0;
	kinect.setCameraTiltAngle(angle);

}

//--------------------------------------------------------------
void ofApp::update() {
	ofBackground(100, 100, 100);

	kinect.update();

	// there is a new frame and we are connected
	if (kinect.isFrameNew()) {

		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);

		//get pointer to pixels
		unsigned char * pix = grayImage.getPixels();

		//calc number of linear pixels
		int numPixels = grayImage.getWidth() * grayImage.getHeight();

		//Search for closest/brightest pixel
		int brightestPixelBrightness = 0;
		for (int p = 0; p < numPixels; p++) {
			if (pix[p] > brightestPixelBrightness) {
				brightestPixelBrightness = pix[p];
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
		contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height) / 2, 1, false);
	}
}

//--------------------------------------------------------------
void ofApp::draw() {
	ofSetColor(255, 255, 255);

	// draw from the live kinect
	kinect.drawDepth(10, 10, 400, 300);
	kinect.draw(420, 10, 400, 300);

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