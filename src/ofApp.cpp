#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofSetLogLevel(OF_LOG_VERBOSE);
    
    // enable depth->video image calibration
    kinect.setRegistration(true);
    kinect.init();
    
    kinect.open(0);		// opens first available kinect
    //kinect.open(2);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
    //kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
    
    colorImg.allocate(kinect.width, kinect.height);
    grayImage.allocate(kinect.width, kinect.height);
    grayThreshNear.allocate(kinect.width, kinect.height);
    grayThreshFar.allocate(kinect.width, kinect.height);
    
    bThreshWithOpenCV = true;
    
    ofSetFrameRate(60);
    
    // open an outgoing connection to HOST:PORT
    sender.setup(HOST, PORT);
    receiver.setup(RECEIVE_PORT);
    
    ofxGuiSetDefaultWidth(300);
    panel.setup("kinect settings", "settings.xml", 540, 100);
    panel.add(minDistance.setup("minDistance",500,500,12000));
    panel.add(maxDistance.setup("maxDistance",6000,500,12000));
    panel.add(nearThreshold.setup("nearThreshold",128,0,255));
    panel.add(farThreshold.setup("farThreshold",128,0,255));
    panel.add(minDetectSize.setup("minDetectSize",2,0,4000));
    panel.add(maxDetectSize.setup("maxDetectSize",1000,0,100000));
    panel.add(maxDetectNumber.setup("maxDetectNumber",10,0,300));
    panel.add(lowPassValue.setup("lowPassValue",0,0,1));
    panel.add(blurValue.setup("blurValue",0,0,40));
    panel.add(idolBlue.set("idolBlue",  false));
    panel.add(idolRed.set("idolRed",  false));
    panel.add(audBlue.set("audBlue", false));
    panel.add(audRed.set("audRed", false));
    panel.add(resetButton.setup("reset"));
    panel.loadFromFile("settings.xml");
    
    resetButton.addListener(this,&ofApp::resetButtonPressed);
    lowPassValue.addListener(this, & ofApp::onLowPassValueChanged);
    minDistance.addListener(this, &ofApp::onMinDistanceChanged );
    maxDistance.addListener(this, &ofApp::onMaxDistanceChanged );
    idolBlue.addListener(this, &ofApp::onIdolChanged );
    idolRed.addListener(this, &ofApp::onIdolChanged );
    audRed.addListener(this, &ofApp::onAudRedChanged );
    audBlue.addListener(this, &ofApp::onAudBlueChanged );
    minDetectSize.addListener(this, &ofApp::onValueChanged );
    maxDetectSize.addListener(this, &ofApp::onValueChanged );
    maxDetectNumber.addListener(this, &ofApp::onValueChanged );
    blurValue.addListener(this, &ofApp::onValueChanged );
    kinect.setDepthClipping(minDistance,maxDistance);
    
}

//--------------------------------------------------------------
void ofApp::update(){
    
    if (idolBlue) {
        roleStr = "ib";
    }else if(idolRed){
        roleStr = "ir";
    }else if(audRed){
        roleStr = "r";
        filePath = "blueRythm.csv";
    }else if(audBlue){
        roleStr = "b";
        filePath = "redRythm.csv";
    }else{
        roleStr = "error";
    }
    
    while(receiver.hasWaitingMessages()){
        // get the next message
        ofxOscMessage m;
        receiver.getNextMessage(&m);
        
        // check for mouse moved message
        if(m.getAddress() == "/start"){
            startTime = ofGetElapsedTimeMillis();
        }
    }
    
    ofFile file(filePath);
    ofBuffer buffer(file);
    if(!file.exists()){
        ofLogError("The file " + filePath + " is missing");
    }
    while (!buffer.isLastLine()) {
        string line = buffer.getNextLine();
        clockTime = ofToFloat(line);
        if(clockTime <= clock+0.3 && clockTime >= clock-0.3 ){
            tmpFlag = 1;
            rhysmR = 255;
            rhysmG = 0;
            rhysmB = 0;
            break;
        }else{
            tmpFlag = 0;
            rhysmR = 255;
            rhysmG = 255;
            rhysmB = 255;
        }
        //Save MorseCodeSymbols for later
        //morseCodeSymbols.push_back(symbol);
    }
    //csvから時間を取得し比較
    
    nowTime = ofGetElapsedTimeMillis();
    clock = (nowTime - startTime)/1000;
    //開始時からの経過時間を取得
    
    ofBackground(100, 100, 100);
    kinect.update();
    
    // there is a new frame and we are connected
    if(kinect.isFrameNew()) {
        
        // load grayscale depth image from the kinect source
        grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
        grayImage.blur(blurValue);
        // we do two thresholds - one for the far plane and one for the near plane
        // we then do a cvAnd to get the pixels which are a union of the two thresholds
        if(bThreshWithOpenCV) {
            grayThreshNear = grayImage;
            grayThreshFar = grayImage;
            grayThreshNear.threshold(nearThreshold, true);
            grayThreshFar.threshold(farThreshold);
            cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
        } else {
            
            // or we do it ourselves - show people how they can work with the pixels
            unsigned char * pix = grayImage.getPixels();
            
            int numPixels = grayImage.getWidth() * grayImage.getHeight();
            for(int i = 0; i < numPixels; i++) {
                if(pix[i] < nearThreshold && pix[i] > farThreshold) {
                    pix[i] = 255;
                } else {
                    pix[i] = 0;
                }
            }
        }
        
        // update the cv images
        grayImage.flagImageChanged();
        
        // find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
        // also, find holes is set to true so we will get interior contours as well....
        contourFinder.findContours(grayImage, minDetectSize, maxDetectSize, maxDetectNumber, false);
        
        for( int i=0; i<contourFinder.nBlobs; i++ ) {
            bool isDraw = false;
            //一つ前の時間に検知された長方形との衝突判定
            
            for ( std::vector<ofPtr <ofRectangle> >::iterator it = preRectangles.begin() ; it != preRectangles.end() ; it ++ ){
                if ( detectRectangleCollision(contourFinder.blobs[i].boundingRect, **it )){
                    isDraw = true;
                    
                }
            }
            if( !isDraw ){
                
                //add center cross and center
                centerX[i] = contourFinder.blobs[i].centroid.x;
                centerY[i] = contourFinder.blobs[i].centroid.y;
                ofxOscMessage m;
                rhysmFlag[i]=tmpFlag;
                m.setAddress("/kinect");
                m.addIntArg(centerX[i]);
                m.addIntArg(centerY[i]);
                m.addIntArg(rhysmFlag[i]);
                m.addStringArg(roleStr);
                sender.sendMessage(m);
            }
        }
        preRectangles.clear();
        for( int k = 0 ; k < contourFinder.nBlobs ;  k ++ ){
            preRectangles.push_back(ofPtr<ofRectangle>( new ofRectangle(contourFinder.blobs[k].boundingRect)));
        }
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    
    
    ofSetColor(255, 255, 255);
    
    // draw from the live kinect
    kinect.draw(420, 40, 400, 300);
    grayImage.draw(10, 40, 400, 300);
    contourFinder.draw(10, 40, 400, 300);
    
    // display instructions
    string buf;
    buf = "HOST: " + string(HOST) + "  PORT: " + ofToString(PORT);
    ofDrawBitmapString(buf, 20, 25);
    
    panel.draw();
    panel.setPosition(10, 360);
    ofDrawBitmapString("Time: " + ofToString(clock), 320, 25);
    
    ofSetColor(rhysmR,rhysmG,rhysmB);
    ofRect(480,10,20,20);
    
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    switch(key){
        case ' ': //rhism counter
        {
            startTime = ofGetElapsedTimeMillis();
            break;
        }
        case 'd': //rhism counter
        {
            //add center cross and center
            ofxOscMessage m;
            m.setAddress("/kinect");
            m.addIntArg(10);
            m.addIntArg(10);
            m.addIntArg(tmpFlag);
            m.addStringArg(roleStr);
            sender.sendMessage(m);
        }
    }
}

//--------------------------------------------------------------
bool ofApp::detectRectangleCollision( const ofRectangle& firstRectangle,const ofRectangle& secondRectangle){
    //なぜかrectangle.x が不定のデータが入ってくることがあるため緊急的な措置
    if ( firstRectangle.getWidth() > 0 && secondRectangle.getWidth() > 0 && firstRectangle.getHeight() > 0 && secondRectangle.getWidth() > 0) {
        if ( firstRectangle.getWidth() < 1000000 && secondRectangle.getWidth() < 1000000 && firstRectangle.getHeight()  < 1000000 && secondRectangle.getWidth() < 1000000) {
            if (  ( secondRectangle.getPosition().x <  firstRectangle.getPosition().x+ firstRectangle.getWidth() ) &&  ( firstRectangle.getPosition().x < secondRectangle.getPosition().x + secondRectangle.getWidth() )){
                if ( ( secondRectangle.getPosition().y < firstRectangle.getPosition().y + firstRectangle.getHeight() ) && (firstRectangle.getPosition().y < secondRectangle.getPosition().y + secondRectangle.getHeight() ) ){
                    
                    return true;
                }
            }
        }
    }
    
    return false;
}

//--------------------------------------------------------------
void ofApp::resetButtonPressed(){
    kinect.close();
    kinect.open();
    startTime = ofGetElapsedTimeMillis();
}

//--------------------------------------------------------------
void ofApp::onLowPassValueChanged(float& num){
    ofxKinect::lowPassRate = lowPassValue;
    std::cout  << ofxKinect::lowPassRate  << std::endl;
    panel.saveToFile("settings.xml");
}

//--------------------------------------------------------------
void ofApp::onValueChanged(int& num){
    panel.saveToFile("settings.xml");
}

//--------------------------------------------------------------
void ofApp::onAudRedChanged(bool& num){

    panel.saveToFile("settings.xml");
    
    //    nearThreshold =
}

//--------------------------------------------------------------
void ofApp::onAudBlueChanged(bool& num){
    
    panel.saveToFile("settings.xml");
    
    //    nearThreshold =
}

//--------------------------------------------------------------
void ofApp::onIdolChanged(bool& num){
    
    panel.saveToFile("settings.xml");
    
    //    nearThreshold =
}

//--------------------------------------------------------------
void ofApp::onMinDistanceChanged(int& num){
    
    maxDistance.setMin(minDistance);
    panel.saveToFile("settings.xml");
    kinect.setDepthClipping(minDistance,maxDistance);
    rateOfthresholdToDistance = ( maxDistance  - minDistance )  / 255.0;
    
    //    nearThreshold =
}

//--------------------------------------------------------------
void ofApp::onMaxDistanceChanged(int& num){
    minDistance.setMax(maxDistance);
    if(maxDistance <= minDistance ) maxDistance = minDistance;
    panel.saveToFile("settings.xml");
    kinect.setDepthClipping(minDistance,maxDistance);
    rateOfthresholdToDistance = ( maxDistance  - minDistance )  / 255.0;
    
    //    farThreshold =
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
void ofApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 
    
}
