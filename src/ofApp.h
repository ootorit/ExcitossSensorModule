#pragma once

#include "ofMain.h"
#include "ofxOsc.h"
#include "ofxKinect.h"
#include "ofxOpenCv.h"
#include "ofMain.h"
#include "ofxGui.h"
#include "ofxXmlSettings.h"

#define HOST "192.168.11.10"
#define PORT 12345
#define RECEIVE_PORT 12346

class ofApp : public ofBaseApp{
    
public:
    void setup();
    void update();
    void draw();
    
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    
    ofxPanel panel;
    ofParameter<bool> idolRed;
    ofParameter<bool> idolBlue;
    ofParameter<bool> audRed;
    ofParameter<bool> audBlue;
    ofxSlider <int> minDistance;
    ofxSlider <int> maxDistance;
    ofxSlider <int> farThreshold;
    ofxSlider <int> nearThreshold;
    ofxSlider <int> minDetectSize;
    ofxSlider <int> maxDetectSize;
    ofxSlider <int> maxDetectNumber;
    ofxSlider <float> lowPassValue;
    ofxSlider <int> blurValue;
    ofxButton resetButton;
    
    float rateOfthresholdToDistance;
    void onValueChanged(int& num);
    void onMinDistanceChanged(int& num);
    void onMaxDistanceChanged(int& num);
    void onNearThresholdChanged(int& num);
    void onFarThresholdChanged(int& num);
    void onAudRedChanged(bool& num);
    void onAudBlueChanged(bool& num);
    void onIdolChanged(bool& num);
    void onLowPassValueChanged(float& num);
    void resetButtonPressed();
    
    ofxOscSender sender;
    ofxOscReceiver receiver;
    ofxKinect kinect;
    ofxCvColorImage colorImg;
    
    ofxCvGrayscaleImage grayImage; // grayscale depth image
    ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
    ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
    
    ofxCvContourFinder contourFinder;

    int nearDepth;
    int farDepth;
    
    float startTime;
    float nowTime;
    float clock;
    float clockTime;
    string filePath;
    
    int rhysmR;
    int rhysmG;
    int rhysmB;
    
    int rhysmFlag[100];
    int tmpFlag;
    int centerX[100];
    int centerY[100];
    string roleStr;
    string center;
    
    //検知された手が新しいものか検知用
    std::vector<ofPtr<ofRectangle> > preRectangles;
    std::vector<ofPtr<ofRectangle> > preRightRectangles;
    bool detectRectangleCollision( const ofRectangle& firstRectangle , const ofRectangle& secondRectangle);
    
    bool bThreshWithOpenCV;
    
};
