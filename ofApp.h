#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "fft.h"

#define BUFFER_SIZE 256
#define NUM_WINDOWS 80

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
        void exit();
        void drawPointCloud();
		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
    
        void audioReceived     (float * input, int bufferSize, int nChannels);
    
        ofxKinect kinect;
    
        #ifdef USE_TWO_KINECTS
            ofxKinect kinect2;
        #endif
    
        ofxCvColorImage colorImg;
    
        ofxCvGrayscaleImage grayImage;
        ofxCvGrayscaleImage grayThreshNear;
        ofxCvGrayscaleImage grayThreshFar;
    
        ofxCvContourFinder contourFinder;
    
        bool bThreshWithOpenCV;
        bool bDrawPointCloud;
    
        int nearThreshold;
        int farThreshold;
    
        int angle;
    
        ofEasyCam easyCam;
    
        vector <float> left;
        vector <float> right;
        vector <float> volHistory;
    
        int     bufferCounter;
        int     drawCounter;
    
        float smoothedVol;
        float scaledVol;
    
        ofSoundStream soundStream;
    
        ofVec3f center;
    ofVec3f speeds2;
        vector <ofVec2f> speeds;
    
        ofVbo vbo;
    
        int mode;
    
        float * fftSmoothed;
    
        ofSoundPlayer         beat;
        ofSoundPlayer         beat2;
        ofSoundPlayer         beat3;
        ofSoundPlayer         beat4;
    
        int nBandsToGet;
        int nBandsToGet1;
        int nBandsToGet2;
        int nBandsToGet3;
        int nBandsToGet4;
    
        int time;
    
        int timer;
    
        ofQuaternion curRot;
    
        ofLight pointLight;
        ofLight pointLight2;
        ofLight pointLight3;
        ofLight pointLight4;
        ofLight pointLight5;
        ofLight pointLight6;

        ofEasyCam cam;
    
    public:
        float * rise;
        fft        myfft;
    
        float magnitude[BUFFER_SIZE];
        float phase[BUFFER_SIZE];
        float power[BUFFER_SIZE];
    
        float freq[NUM_WINDOWS][BUFFER_SIZE/2];
        float freq_phase[NUM_WINDOWS][BUFFER_SIZE/2];

    //this holds all of our points
    vector<ofVec3f> points;
    //this keeps track of the center of all the points

    void addPoint(float x, float y) {
        points.push_back(ofVec2f(x, y));
        speeds.push_back(ofVec2f(ofRandom(-1, 1), ofRandom(-1, 1)));
    }
    
    int ofVec;
    
    int vortex;
};
