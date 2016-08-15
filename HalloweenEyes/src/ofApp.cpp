#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	ofSetLogLevel(OF_LOG_VERBOSE);

	// enable depth->video image calibration
	kinect.setRegistration(true);

	kinect.init();
								//kinect.init(true);  // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)

	kinect.open();		// opens first available kinect
						//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
						//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #

	// print the intrinsic IR sensor values
	if (kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}

	grayImage.allocate(kinect.width, kinect.height);

	//set up depth layers
	depthLayers.push_back(DepthLayer(190, 255, kinect.width, kinect.height, 10, (kinect.width * kinect.height)/2, 1));
	
	//Target Frame rate
	ofSetFrameRate(60);

	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
}

//--------------------------------------------------------------
void ofApp::update(){
	ofBackground(100, 100, 100);

	kinect.update();

	// there is a new frame and we are connected
	if (kinect.isFrameNew()) {

		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
		//depthLayers[0].update(grayImage.getPixels());
		depthLayers[0].update(grayImage);
		//for (int i = 0; i < depthLayers.size(); i++) {
		//	depthLayers[i].update(grayImage.getPixels());
		//}
	}

}

//--------------------------------------------------------------
void ofApp::draw(){
	ofSetColor(255, 255, 255);

	
	// draw from the live kinect
	kinect.drawDepth(10, 10, 400, 300);
	kinect.draw(420, 300, 400, 300);

	depthLayers[0].drawDepth(10, 300, 400, 300);
	depthLayers[0].drawContours(420, 10, 400, 300);
	//for (int i = 0; i < depthLayers.size(); i++)
	//{
	//	depthLayers[i].drawDepth(10, 300, 400, 300);
	//	depthLayers[i].drawContours(420, 10, 400, 300);
	//}


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

	reportStream << "FPS: " << ofGetFrameRate() << endl
		<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;

	if (kinect.hasCamTiltControl()) {
		reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
			<< "press 1-5 & 0 to change the led mode" << endl;
	}

	ofDrawBitmapString(reportStream.str(), 20, 652);

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	switch (key) {

	case ' ':
		depthLayers[0].toggleFindContours();
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
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}

//--------------------------------------------------------------
void ofApp::exit() {
	ofLogNotice() << "exit";
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	//system("pause");
}