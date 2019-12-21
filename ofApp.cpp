#include "ofApp.h"

//--------------------------------------------------------------

void ofApp::audioReceived     (float * input, int bufferSize, int nChannels){
    // samples are "interleaved"
    for (int i = 0; i < bufferSize; i++){
        rise[i] = input[i*2];
    }
    bufferCounter++;
}

//--------------------------------------------------------------

void ofApp::setup(){

    ofSetSmoothLighting(true);
    pointLight.setDiffuseColor( ofFloatColor(0, 50, 255) );
    pointLight.setSpecularColor( ofFloatColor(0, 50, 255));
    pointLight2.setDiffuseColor( ofFloatColor(0, 50, 50) );
    pointLight2.setSpecularColor( ofFloatColor(0, 50, 255));
    pointLight3.setDiffuseColor( ofFloatColor(0, 50, 255) );
    pointLight3.setSpecularColor( ofFloatColor(0, 50, 50));
    pointLight4.setDiffuseColor( ofFloatColor(0, 50, 255) );
    pointLight4.setSpecularColor( ofFloatColor(0, 50, 255));
    
    
    soundStream.printDeviceList();
    
    soundStream.setDeviceID(4);
    rise = new float[BUFFER_SIZE];
    
    int bufferSize = 256;
    ofToggleFullscreen();    left.assign(bufferSize, 0.0);
    right.assign(bufferSize, 0.0);
    volHistory.assign(400, 0.0);
    
    bufferCounter    = 0;
    drawCounter        = 0;
    smoothedVol     = 0.0;
    scaledVol        = 0.0;
    
    soundStream.setup(this, 0, 2, 44100, bufferSize, 4);
    
    beat.load("sounds/brothel.mp3");
    
    fftSmoothed = new float[8192];
    for (int i = 0; i < 8192; i++){
        fftSmoothed[i] = 0;
    }
    
    for (int i1 = 0; i1 < 8192; i1++){
        fftSmoothed[i1] = 50;
    }
    for (int i2 = 0; i2 < 8192; i2++){
        fftSmoothed[i2] = 100;
    }
    for (int i3 = 0; i3 < 8192; i3++){
        fftSmoothed[i3] = 150;
    }
    for (int i4 = 0; i4 < 8192; i4++){
        fftSmoothed[i4] = 200;
    }
    
    bool musicvideo;
    
    nBandsToGet = 1;
    nBandsToGet1 = 11;
    nBandsToGet2 =  25;
    nBandsToGet3 =  35;
    nBandsToGet4 =  2;
    
    ofSetLogLevel(OF_LOG_VERBOSE);
    
    kinect.setRegistration(true);
    kinect.init();
    
    kinect.open();
    
    if(kinect.isConnected()) {
        ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
        ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
        ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
        ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
    }
    
    #ifdef USE_TWO_KINECTS
        kinect2.init();
        kinect2.open();
    #endif
    
    colorImg.allocate(kinect.width, kinect.height);
    grayImage.allocate(kinect.width, kinect.height);
    grayThreshNear.allocate(kinect.width, kinect.height);
    grayThreshFar.allocate(kinect.width, kinect.height);
    
    nearThreshold = -2000;
    farThreshold = 1000;
    bThreshWithOpenCV = true;
    
    ofSetFrameRate(60);

    angle = 0;
    kinect.setCameraTiltAngle(angle);

    bDrawPointCloud = true;
    
    glShadeModel (GL_SMOOTH);
    
    ofSetBackgroundColor(0);
    
    ofSetFullscreen(true);
    
    ofHideCursor() ;
}

//--------------------------------------------------------------
void ofApp::update(){

    scaledVol = ofMap(smoothedVol, 0.0, 0.17, 0.0, 1.0, true);

    time = ofGetElapsedTimef() * 100;
    
    ofSoundUpdate();
    
    kinect.update();
    
    float * val = ofSoundGetSpectrum(nBandsToGet);
    for (int i = 0;i < nBandsToGet; i++){
        fftSmoothed[i] *= 0.96f;
        if (fftSmoothed[i] < val[i]) fftSmoothed[i] = val[i];
    }
    
    float * val1 = ofSoundGetSpectrum(nBandsToGet1);
    for (int i = 0;i < nBandsToGet1; i++){
        fftSmoothed[i] *= 0.96f;
        if (fftSmoothed[i] < val[i]) fftSmoothed[i] = val1[i];
        
    }
    
    float * val2 = ofSoundGetSpectrum(nBandsToGet2);
    for (int i = 0;i < nBandsToGet2; i++){
        fftSmoothed[i] *= 0.96f;
        if (fftSmoothed[i] < val[i]) fftSmoothed[i] = val2[i];
    }
    
    float * val3 = ofSoundGetSpectrum(nBandsToGet3);
    for (int i = 0;i < nBandsToGet3; i++){
        fftSmoothed[i] *= 0.96f;
        if (fftSmoothed[i] < val[i]) fftSmoothed[i] = val3[i];
    }
    
    float * val4 = ofSoundGetSpectrum(nBandsToGet4);
    for (int i = 0;i < nBandsToGet4; i++){
        fftSmoothed[i] *= 0.96f;
        if (fftSmoothed[i] < val[i]) fftSmoothed[i] = val4[i];
    }
    
    #ifdef USE_TWO_KINECTS
        kinect2.update();
    #endif
    
}

//--------------------------------------------------------------
void ofApp::draw(){
    
    pointLight.setPosition(0, 0, -2000);
    pointLight2.setPosition(0, 0, 1000);
    pointLight3.setPosition(1000, 0, 0);
    pointLight4.setPosition(-1000, 0, 0);
    
    float rot = 0.8 * ofGetFrameNum();
    
    cam.setFov(60);
    cam.setNearClip(1);
    cam.setFarClip(2000);
    cam.setPosition(0, 0, 1000);
    cam.begin();
    
//    cam.setFov(60);
//    cam.setNearClip(1);
//    cam.setFarClip(2000);
//    cam.setPosition(0, 0, 2750 - count);
//
//    timer++;
//    timer2 = 12;
//
//    count = timer * timer2;
//
//    if( count >= 5500)
//    {
//        timer = 0;
//    }
//
//    cam.begin();
    
    ofTranslate(0, 0, -100);
    ofRotateDeg(ofGetElapsedTimef()*5, 0, 1, 0);
    drawPointCloud();

    cam.end();
    
}

void ofApp::drawPointCloud() {
    
    for (int i1 = 0;i1 < nBandsToGet; i1++){
        for (int i2 = 10;i2 < nBandsToGet1; i2++){
            for (int i3 = 24;i3 < nBandsToGet2; i3++){
                for (int i4 = 1;i4 < nBandsToGet4; i4++){
                    
//                    if (rise[i1] >= 0.1) {
                        addPoint(0, 0);
                    
//                    }
                    
                }
                
                ofVec3f sumOfAllPoints(0,0,0);
                for(unsigned int i = 0; i < points.size(); i++){
                    points[i].z = 300;
                    ofSetColor(255, 255, 111);
                    
                    points[i].z -= rise[i1] * 800;
                    
                    sumOfAllPoints += points[i];
                    
                    center = sumOfAllPoints / points.size();
                    
                }
                
                ////
                
                for (unsigned int i=0; i<points.size(); i++) {
                    
                    //DOWN BY VOLUME
                    
                    speeds[i].y += rise[i2] * 400;
                    
                    //UP BY MID
                    
                    speeds[i].y -= rise[i1] * 400;
                    
                    //RIGHT BY TREBLE
                    
                    speeds[i].x -= rise[i2] * 400;
                    
                    //LEFT BY SNARE
                    
                    speeds[i].x += rise[i3] * 400;
                    
                    //------
                    
                    points[i]   += speeds[i];
                    
                    speeds[i]   *= 0.09;
                    
                    ofVec2f mouseVec = ofVec2f(0,
                                               0)
                    - points[i];

                }
                
                
                ofMesh mesh1;
                mesh1.setMode(OF_PRIMITIVE_POINTS);
                for(unsigned int i = 1; i < points.size(); i++){
                    
                    glPointSize(20);
                    
                    ofVec3f thisPoint = points[i-1];
                    ofVec3f nextPoint = points[i];
                    
                    ofVec3f direction = (nextPoint - thisPoint);
                    
                    float distance = direction.length();
                    
                    ofVec3f unitDirection = direction.getNormalized();
                    
                    ofVec3f toTheLeft = unitDirection.getRotated(-90, ofVec3f(0,0,1));
                    ofVec3f toTheRight = unitDirection.getRotated(90, ofVec3f(0,0,1));

                    float thickness = (20);
                    
                    ofVec3f leftPoint = thisPoint+toTheLeft*thickness;
                    ofVec3f rightPoint = thisPoint+toTheRight*thickness;
                    
                    mesh1.addVertex(ofVec3f(leftPoint.x, leftPoint.y, leftPoint.z));
                    mesh1.addVertex(ofVec3f(rightPoint.x, rightPoint.y, rightPoint.z));
                    
                }

            // - - - - - -
                
                    int w = 640;
                    int h = 480;
                    ofMesh mesh;
                    
                    mesh.setMode(OF_PRIMITIVE_TRIANGLES);
                    
                    int step = 2;
                for(int y = 0; y < h; y += step + rise[i2] * 50)
                    
                {
                    for(int x = 0; x < w; x += step + rise[i3] * 50)
                            if(kinect.getDistanceAt(x, y) > 0) {
                                mesh.addColor(kinect.getColorAt(x,y));
                                mesh.addVertex(kinect.getWorldCoordinateAt(x, y));

                                
                            }
                        }
                    }
                    
                    glPointSize(0
//                                + (rise[i1]*100)
                                + (rise[i2]*20)
                                + (rise[i3]*500)
                                );
                    
                    ofPushMatrix();
                    
                    ofScale(1,-1,-1);
                    
                    ofTranslate(0, 0, -400);
                    
                    glEnable(GL_DEPTH_TEST);
                    mesh.drawVertices();
                mesh1.draw();
                    glDisable(GL_DEPTH_TEST);
                    ofPopMatrix();
                
                    ofDrawBitmapString("Audio: SAMPLE LOOPS from Looperman.com", 100, 100);
//                    ofDrawBitmapString("Vocals: Virtual Riot", 100, 200);
//                    ofDrawBitmapString("Vocals: Virus Syndicate & Virtual Riot & Dion Timmer", 100, 300);

                }
                
            }
        }
    
    ofEnableLighting();
    pointLight.enable();
    pointLight2.enable();
    pointLight3.enable();
    
}

void ofApp::exit() {
    kinect.setCameraTiltAngle(0); // zero the tilt on exit
    kinect.close();
    
    #ifdef USE_TWO_KINECTS
        kinect2.close();
    #endif
    
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    switch (key) {
        case '1':
            kinect.setLed(ofxKinect::LED_YELLOW);
//            beat.play();
            points.clear();
            break;
    case OF_KEY_UP:
        angle++;
        if(angle>30) angle=30;
        kinect.setCameraTiltAngle(angle);
        break;
    
    case OF_KEY_DOWN:
        angle--;
        if(angle<-30) angle=-30;
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
