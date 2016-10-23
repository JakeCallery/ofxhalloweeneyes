#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxGui.h"

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

	const int LH_SERVO_OFFSET = 0;
	const int LV_SERVO_OFFSET = 0;
	const int RH_SERVO_OFFSET = 0;
	const int RV_SERVO_OFFSET = 0;

	int mapInt(int x, int in_min, int in_max, int out_min, int out_max);

	ofxKinect kinect;

	ofxCvColorImage colorImg;

	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image

	ofxCvContourFinder contourFinder;

	ofxPanel leftEyePanel;
	ofxPanel rightEyePanel;
	ofxPanel targetPanel;

	//Left Eye Sliders
	ofxFloatSlider leftEyeHorizLocSlider;
	ofxFloatSlider leftEyeRotXOffsetSlider;
	ofxFloatSlider leftEyeRotYOffsetSlider;
	
	//Right Eye Sliders
	ofxFloatSlider rightEyeHorizLocSlider;
	ofxFloatSlider rightEyeRotXOffsetSlider;
	ofxFloatSlider rightEyeRotYOffsetSlider;

	//Target Sliders
	ofxFloatSlider targetLocScaleXSlider;
	ofxFloatSlider targetLocScaleYSlider;
	ofxFloatSlider targetLocScaleZSlider;
	ofxFloatSlider targetLocOffsetXSlider;
	ofxFloatSlider targetLocOffsetYSlider;
	ofxFloatSlider targetLocOffsetZSlider;

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
	ofBoxPrimitive leftEyeParentBox;
	ofBoxPrimitive rightEyeParentBox;
	ofConePrimitive leftEyeCone;
	ofConePrimitive rightEyeCone;
	ofVec3f eyeUpVector;

	int targetXOffset = 0;
	int targetYOffset = 0;
	int targetZOffset = 0;

	float targetXScale = 1.0;
	float targetYScale = 1.0;
	float targetZScale = 1.0;

	ofVec3f leftEyeDefaultLoc;
	ofVec3f rightEyeDefaultLoc;

	ofVec3f leftEyeLocOffset;
	ofVec3f rightEyeLocOffset;

	ofVec3f targetBoxLocScale;
	ofVec3f targetBoxLocOffset;

	ofSerial serial;

	
};
