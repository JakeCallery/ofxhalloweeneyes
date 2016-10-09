#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"

class ofApp : public ofBaseApp {

public:
	void setup();
	void update();
	void draw();
	void exit();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	const int RENDER_X_OFFSET = 420;
	const int RENDER_Y_OFFSET = 10;
	const int RENDER_WIDTH = 400;
	const int RENDER_HEIGHT = 300;


	int mapInt(int x, int in_min, int in_max, int out_min, int out_max);

	ofxKinect kinect;

	ofxCvColorImage colorImg;

	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image

	ofxCvContourFinder contourFinder;

	bool bThreshWithOpenCV;

	int baseNearThreshold;
	int baseFarThreshold;
	int threshDepth = 15;
	int frontThreshold;
	int backThreshold;

	int angle;

	ofBoxPrimitive targetBox;
	ofLight light;
	ofCamera cam;
	ofRectangle camViewport;
	ofSerial serial;

	
};
