#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxHomography.h"
#include "LineEquation.h"

#define USE_LIVE_CAM        // uncomment this to use a live camera
                                // otherwise, we'll use a movie file

#define LIVE_CAM_MAX 4

#include "ofxOsc.h"

// send host (aka ip address)
//#define HOST "localhost"
#define HOST "192.168.1.1"

/// send port
#define PORT 9998


class ofApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
	void keyPressed(int key);
    void mousePressed(int x, int y, int button);
    void bubble_sort(ofPoint pos[], int n);
    void checkPos(ofPoint pos[]);
    ofPoint getCenterOfGravityPoint(vector<ofPoint> pos);
	
    
    ofTrueTypeFont font;
    ofxOscSender sender;
    
	float threshold;
    
    #ifdef USE_LIVE_CAM
        ofVideoGrabber  cam[LIVE_CAM_MAX];
    #else
        ofVideoPlayer   movie;
    #endif
    
	ofxCv::ContourFinder contourFinder;
	bool showLabels;
    bool showPos;
    
    int w;
    int h;
    int camFix_X;
    int camFix_Y;
    
    vector<ofPoint> xyPosOsc;
    vector<ofPoint> vertexPoint;
    int Gx = 0;
    int Gy = 0;
    ofPoint lastGxy;
    float distance = 0;
    int vertexDistance = 130; // 130, 200
    
    LineEquation line;
    
    
    ofPixels cam1cam2_Pixels;
    ofFbo fbo;
    
    ofPoint originalCorners[4];
    ofPoint distortedCorners[4];
    int oCornerIndex = 0;
    int dCornerIndex = 0;
    ofMatrix4x4 homography;
};
