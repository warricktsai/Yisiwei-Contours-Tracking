#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

void ofApp::setup() {
    ofSetWindowTitle("yisiwei Tracking System");
    ofSetFrameRate(60);
    ofSetVerticalSync(true);
	ofBackground(0);
    
    // open an outgoing connection to HOST:PORT
    sender.setup(HOST, PORT);

    w = 640;
    h = 480;
    camFix_X = -91;   //-127
    camFix_Y = -20;   // -31
    
#ifdef USE_LIVE_CAM
    ofSetWindowShape(w*2+camFix_X, h*2+camFix_Y);
    
    cam[0].listDevices();
    
    int camIdList[LIVE_CAM_MAX] = {0, 2, 3, 4};
    
    for (int i=0; i<LIVE_CAM_MAX; i++) {
        cam[i].setDeviceID(camIdList[i]);
        cam[i].setVerbose(true);
        cam[i].setup(w,h);
        cam[i].setDesiredFrameRate(60);
    }
#else
    movie.load("12points.mov");
    movie.play();
    movie.setLoopState(OF_LOOP_NORMAL);
    ofSetWindowShape(movie.getWidth(), movie.getHeight());
#endif
        
    contourFinder.setMinAreaRadius(1); //1
    contourFinder.setMaxAreaRadius(100);
    contourFinder.setThreshold(12); //12
    // wait for half a second before forgetting something
    contourFinder.getTracker().setPersistence(300);
    // an object can move up to 32 pixels per frame
    contourFinder.getTracker().setMaximumDistance(600);

	showLabels = true;
    showPos = true;
    
//    originalCorners[0].set(11, 11);
//    originalCorners[1].set(852, 5);
//    originalCorners[2].set(860, 611);
//    originalCorners[3].set(11, 615);
    
    originalCorners[0].set(27+fixP1.x, 40+fixP1.y);
    originalCorners[1].set(1156+fixP2.x, 25+fixP2.y);
    originalCorners[2].set(1160+fixP3.x, 892+fixP3.y);
    originalCorners[3].set(23+fixP4.x, 884+fixP4.y);
    
    distortedCorners[0].set(0, 0);
    distortedCorners[1].set(w*2+camFix_X, 0);
    distortedCorners[2].set(w*2+camFix_X, h*2+camFix_Y);
    distortedCorners[3].set(0, h*2+camFix_Y);
}

void ofApp::update() {
    
    bool bNewFrame[LIVE_CAM_MAX] = { false, false};
    
#ifdef USE_LIVE_CAM
    for (int i=0; i<LIVE_CAM_MAX; i++) {
        cam[i].update();
        bNewFrame[i] = cam[i].isFrameNew();
    }
#else
    movie.update();
    bNewFrame[0] = bNewFrame[1] = movie.isFrameNew();
#endif

    
    if (bNewFrame[0] && bNewFrame[1]){
#ifdef USE_LIVE_CAM
//        fbo.allocate(h*4,w, GL_RGBA);
        fbo.allocate(w*2,h*2, GL_RGBA);
        
        fbo.begin();
            ofSetColor(100,100,100);
            ofDrawRectangle(0, 0, ofGetWidth(), ofGetHeight());
            ofSetColor(50);
            ofDrawRectangle(ofGetWidth()/2, 0, ofGetWidth()/2, ofGetHeight());
        
            ofSetColor(255,255,255);
        
            
            ofPixels c1 = cam[0].getPixels();
            ofPixels c2 = cam[1].getPixels();
            ofPixels c3 = cam[2].getPixels();
            ofPixels c4 = cam[3].getPixels();
                        
//            c1.rotate90(1);
//            c2.rotate90(-1);
//            c1.mirror(1, 1);
//            c2.mirror(1, 1);
//            c3.mirror(1, 1);
//            c4.mirror(1, 1);
        
            ofImage cam1, cam2, cam3, cam4;
            cam1.setFromPixels(c1);
            cam2.setFromPixels(c2);
            cam3.setFromPixels(c3);
            cam4.setFromPixels(c4);
            
            cam1.draw(0, 0);
            cam2.draw(w+camFix_X, 0);
            cam3.draw(0, h+camFix_Y);
            cam4.draw(w+camFix_X, h+camFix_Y);
            
        
//            cam2.draw(0, h+camFix_Y);
        
        fbo.end();
        
        ofTexture tex = fbo.getTexture();
        tex.readToPixels(cam1cam2_Pixels); // from texture to pixels
        
        blur(cam1cam2_Pixels, 10);
        
        contourFinder.findContours(cam1cam2_Pixels);
#else
        blur(movie, 15);
        contourFinder.findContours(movie);
#endif
    }
    
    homography = ofxHomography::findHomography(originalCorners, distortedCorners);
}

void ofApp::draw() {
	ofSetBackgroundAuto(showLabels);
    
    RectTracker& tracker = contourFinder.getTracker();
    
	if(showLabels) {
		ofSetColor(255);
        #ifdef USE_LIVE_CAM
            fbo.draw(0, 0);
        #else
            movie.draw(0, 0);
        #endif
        
//        contourFinder.draw();
        
        string msgPos = "";
        xyPosOsc.clear();
//        xyPosOsc.resize(4);
        for(int i = 0; i < contourFinder.size(); i++) {
            ofPoint center = toOf(contourFinder.getCenter(i));
            int label = contourFinder.getLabel(i);
//            ofDrawBitmapString(ofToString(label), 20, 50+i*15);
//            ofDrawBitmapString(center, 50, 50+i*15);
//
//            xyPosOsc.at(i) = center;
            xyPosOsc.push_back(center);
            msgPos += ofToString(center) + "|";
            
            ofPushMatrix();
            ofTranslate(center.x, center.y);
//            int label = contourFinder.getLabel(i);
            string msg = ofToString(label) + ":" + ofToString(tracker.getAge(label));
//            ofDrawBitmapString(msg, 10, 0);
            
            ofVec2f velocity = toOf(contourFinder.getVelocity(i));
            ofScale(5, 5);
//            ofDrawLine(0, 0, velocity.x, velocity.y);
            ofPopMatrix();
        }

        
        if(xyPosOsc.size()>=4 && xyPosOsc.at(0) != ofPoint(0,0,0)){
            Gx = 0;
            Gy = 0;
            for(int i=0;i<xyPosOsc.size();i++){
                Gx += xyPosOsc[i].x;
                Gy += xyPosOsc[i].y;
            }
            // 計算臨時重心
            Gx = Gx/xyPosOsc.size();
            Gy = Gy/xyPosOsc.size();
            
            
            for(int i=0;i<xyPosOsc.size();i++){
                xyPosOsc.at(i).z = atan2(xyPosOsc.at(i).y-Gy, xyPosOsc.at(i).x-Gx);
    //            cout << xyPosOsc.at(i).z << endl;
            }

            // 順時針排序
            for (int i = 0; i < xyPosOsc.size(); ++i) {
                for (int j = 0; j < i; ++j) {
                    if (xyPosOsc[j].z > xyPosOsc[i].z) {
                        ofPoint temp = xyPosOsc[j];
                        xyPosOsc[j] = xyPosOsc[i];
                        xyPosOsc[i] = temp;
                    }
                }
            }
            
            vertexPoint.clear();
            for(int i=3;i<=xyPosOsc.size()+3;i++){
                
                ofPoint p1, p2, p3, p4;
                int n = xyPosOsc.size();
                
                if(i==xyPosOsc.size()){
                    p1 = xyPosOsc.at(i%n);
                    p2 = xyPosOsc.at(n-1);
                    p3 = xyPosOsc.at(n-2);
                    p4 = xyPosOsc.at(n-3);
                }else if(i==xyPosOsc.size()+1){
                    p1 = xyPosOsc.at(i%n);
                    p2 = xyPosOsc.at(i%n-1);
                    p3 = xyPosOsc.at(n-1);
                    p4 = xyPosOsc.at(n-2);
                }else if(i==xyPosOsc.size()+2){
                    p1 = xyPosOsc.at(i%n);
                    p2 = xyPosOsc.at(i%n-1);
                    p3 = xyPosOsc.at(i%n-2);
                    p4 = xyPosOsc.at(n-1);
                }else if(i==xyPosOsc.size()+3){
                    p1 = xyPosOsc.at(i%n);
                    p2 = xyPosOsc.at(i%n-1);
                    p3 = xyPosOsc.at(i%n-2);
                    p4 = xyPosOsc.at(i%n-3);
                }else{
                    p1 = xyPosOsc.at(i%n);
                    p2 = xyPosOsc.at(i%n-1);
                    p3 = xyPosOsc.at(i%n-2);
                    p4 = xyPosOsc.at(i%n-3);
                }
                
                float l1 = atan2(p1.y-p2.y, p1.x-p2.x);
                float l2 = atan2(p3.y-p4.y, p3.x-p4.x);
                
//                cout << " p4: " << p4 <<", p3: " << p3 << ", p2: " << p2 <<", p1: " << p1 << endl;
//                cout << i << " l2: " << l2*180/PI <<", l1: " << l1*180/PI << " => "<< (l2-l1)*180/PI << endl;
                
//                ofSetColor(255, 0, 0);
//                ofDrawCircle(p4, 20-i*2);
                
//                if(abs(l2-l1)>0.3){
                if((abs((l2-l1)*180/PI) > 80 && abs((l2-l1)*180/PI) < 100) || (abs((l2-l1)*180/PI) > 260 && abs((l2-l1)*180/PI) < 280)){
                    ofPoint cross = line.GetCrossPoint(line.GetLine(p1, p2), line.GetLine(p3, p4));
                    vertexPoint.push_back(cross);
                    
                    // 移除相近點    10, 15
                    if(vertexPoint.size() > 1 && abs(vertexPoint.back().x -vertexPoint.at(vertexPoint.size()-2).x)<10 && abs(vertexPoint.back().y-vertexPoint.at(vertexPoint.size()-2).y)<10){
                        vertexPoint.pop_back();
                    }
                    else if(vertexPoint.size() > 1  && abs(vertexPoint.back().x-vertexPoint.at(0).x)<10 && abs(vertexPoint.back().y-vertexPoint.at(0).y)<10){
                        vertexPoint.pop_back();
                    }
                    
                    if(showPos) {
                        ofSetColor(112, 213, 237);
                        ofDrawLine(p1, p2);
                        ofDrawLine(p3, p4);
                    }
//                    ofSetColor(255, 0, 0);
//                    ofDrawCircle(cross, 20);
                }
            }
            
            
            // 計算正確重心
            ofPoint Gxy = getCenterOfGravityPoint(vertexPoint);
            Gx = Gxy.x;
            Gy = Gxy.y;
            
            // 以距離過濾出四個頂點
            for(int i=0;i<vertexPoint.size();i++){
                distance = ofDist(vertexPoint.at(i).x, vertexPoint.at(i).y, Gx, Gy);
                vertexPoint.at(i).z = distance;
//                cout << i << ", " << distance << endl;
                if(distance < vertexDistance){
                    vertexPoint.erase(vertexPoint.begin()+i);
                }
            }
            
            for (int i = 0; i < vertexPoint.size(); ++i) {
                for (int j = 0; j < i; ++j) {
                    if (vertexPoint[j].z < vertexPoint[i].z) {
                        ofPoint temp = vertexPoint[j];
                        vertexPoint[j] = vertexPoint[i];
                        vertexPoint[i] = temp;
                    }
                }
            }
            
            // 重新順時針排序
            for(int i=0;i<vertexPoint.size();i++){
                vertexPoint.at(i).z = atan2(vertexPoint.at(i).y-Gy, vertexPoint.at(i).x-Gx);
            }
            for (int i = 0; i < vertexPoint.size(); ++i) {
                for (int j = 0; j < i; ++j) {
                    if (vertexPoint[j].z > vertexPoint[i].z) {
                        ofPoint temp = vertexPoint[j];
                        vertexPoint[j] = vertexPoint[i];
                        vertexPoint[i] = temp;
                    }
                }
            }
            
            for(int i=0;i<vertexPoint.size();i++){
//                cout << "vertexPoint: " << vertexPoint.at(i) << endl;
                string pos = ofToString(vertexPoint.at(i));
//                ofDrawBitmapString(pos, vertexPoint.at(i).x+13, vertexPoint.at(i).y);
            }
            
            if(vertexPoint.size()==4){
                
                // ================== TODO: be function
                ofPoint p1(vertexPoint.at(0).x,vertexPoint.at(0).y);
                ofPoint p2(vertexPoint.at(1).x,vertexPoint.at(1).y);
                ofPoint p3(vertexPoint.at(2).x,vertexPoint.at(2).y);
                ofPoint p4(vertexPoint.at(3).x,vertexPoint.at(3).y);
                
                ofPushMatrix();
                ofMultMatrix(homography);
                
                ofSetColor(ofColor::yellow);
        //        ofDrawRectangle(100, 100, 100, 100);
                
                ofSetColor(ofColor::black);
            //    ofDrawCircle(50, 50, 30);
                
                
                // Draw a point in the warped space
                ofSetColor(255, 0, 255);
                ofDrawCircle(p1, 7);
                ofDrawCircle(p2, 7);
                ofDrawCircle(p3, 7);
                ofDrawCircle(p4, 7);
                
                ofPopMatrix();
                
                // Draw the screen coordinates of that point
                ofPoint sP1 = ofxHomography::toScreenCoordinates(p1, homography);
                ofPoint sP2 = ofxHomography::toScreenCoordinates(p2, homography);
                ofPoint sP3 = ofxHomography::toScreenCoordinates(p3, homography);
                ofPoint sP4 = ofxHomography::toScreenCoordinates(p4, homography);
                
                
                ofxOscMessage m1;
                m1.setAddress("/p1");
                m1.addFloatArg(sP1.x);
                m1.addFloatArg(sP1.y);
                sender.sendMessage(m1, false);

                ofxOscMessage m2;
                m2.setAddress("/p2");
                m2.addFloatArg(sP2.x);
                m2.addFloatArg(sP2.y);
                sender.sendMessage(m2, false);

                ofxOscMessage m3;
                m3.setAddress("/p3");
                m3.addFloatArg(sP3.x);
                m3.addFloatArg(sP3.y);
                sender.sendMessage(m3, false);

                ofxOscMessage m4;
                m4.setAddress("/p4");
                m4.addFloatArg(sP4.x);
                m4.addFloatArg(sP4.y);
                sender.sendMessage(m4, false);
                
                ofSetColor(ofColor::white);
                string points = "OSC Sending Points: \n"
                                + ofToString(sP1.x) + "," + ofToString(sP1.y) + "\n"
                                + ofToString(sP2.x) + "," + ofToString(sP2.y) + "\n"
                                + ofToString(sP3.x) + "," + ofToString(sP3.y) + "\n"
                                + ofToString(sP4.x) + "," + ofToString(sP4.y);
//                ofDrawBitmapString(points, 20, ofGetHeight()-140);
                
//                cout << sP1 << ", " << sP2 << ", " << sP3 << ", " << sP4 << endl;
            }
            
        }
        
        if(showPos) {
            
            ofSetColor(255, 255, 255);
            
            for(int i=0;i<20;i++){
                ofDrawLine(ofVec2f(i*100,0), ofVec2f(i*100,ofGetHeight()));
                ofDrawLine(ofVec2f(0,i*100), ofVec2f(ofGetWidth(),i*100));
            }
            
            ofSetColor(0, 255, 255);
            for(int i=0;i<xyPosOsc.size();i++){
                ofPushMatrix();
                ofTranslate(xyPosOsc[i].x, xyPosOsc[i].y);
                ofDrawCircle(0, 0, 8);
                ofPopMatrix();
            }

            ofSetColor(255, 255, 0);
            for(int i=0;i<vertexPoint.size();i++){
                ofPushMatrix();
                ofTranslate(vertexPoint[i].x, vertexPoint[i].y);
                ofDrawCircle(0, 0, 10);
                ofPopMatrix();
            }
            

            ofSetColor(255, 0, 0);
            ofDrawCircle(Gx, Gy, 5);
            ofSetColor(255, 255, 255);
        }
        
	} else {
		for(int i = 0; i < contourFinder.size(); i++) {
			unsigned int label = contourFinder.getLabel(i);
			// only draw a line if this is not a new label
			if(tracker.existsPrevious(label)) {
				// use the label to pick a random color
				ofSeedRandom(label << 24);
				ofSetColor(ofColor::fromHsb(ofRandom(255), 255, 255));
				// get the tracked object (cv::Rect) at current and previous position
				const cv::Rect& previous = tracker.getPrevious(label);
				const cv::Rect& current = tracker.getCurrent(label);
				// get the centers of the rectangles
				ofVec2f previousPosition(previous.x + previous.width / 2, previous.y + previous.height / 2);
				ofVec2f currentPosition(current.x + current.width / 2, current.y + current.height / 2);
				ofDrawLine(previousPosition, currentPosition);
			}
		}
	}

//    ofDrawBitmapString(ofGetFrameRate(), 20, 20);
    string widowsSize = "Cam Size: " + ofToString(ofGetWidth())  + ", " + ofToString(ofGetHeight());
//    ofDrawBitmapString(widowsSize, 20, ofGetHeight()-20);
}

void ofApp::mousePressed(int x, int y, int button) {
    cout << x << ", " << y << endl;
}

void ofApp::keyPressed(int key) {
    
    if(key == 'p') {
#ifdef USE_LIVE_CAM
        
#else
        movie.setPaused(movie.isPlaying());
#endif
    }

    if(key == ' ') {
        showLabels = !showLabels;
    }
    
    if(key == 's'){
        showPos = !showPos;
    }

    if(key == OF_KEY_LEFT) {
        camFix_X --;
    }
    
    if(key == OF_KEY_RIGHT) {
        camFix_X ++;
    }
    
    if(key == OF_KEY_UP) {
        camFix_Y --;
    }

    if(key == OF_KEY_DOWN) {
        camFix_Y ++;
    }
    
    if(key == 't') {
        fixP1.x ++;
    }
    if(key == 'g') {
        fixP1.y ++;
    }
    
    if(key == 'y') {
        fixP2.x ++;
    }
    if(key == 'h') {
        fixP2.y ++;
    }

    if(key == 'u') {
        fixP3.x ++;
    }
    if(key == 'j') {
        fixP3.y ++;
    }
    
    if(key == 'i') {
        fixP4.x ++;
    }
    if(key == 'k') {
        fixP4.y ++;
    }
    
    originalCorners[0].set(27+fixP1.x, 40+fixP1.y);
    originalCorners[1].set(1156+fixP2.x, 25+fixP2.y);
    originalCorners[2].set(1160+fixP3.x, 892+fixP3.y);
    originalCorners[3].set(23+fixP4.x, 884+fixP4.y);
    
#ifdef USE_LIVE_CAM
    ofSetWindowShape(w*2+camFix_X, h*2+camFix_Y);
#else
        
#endif
    
    
    cout << camFix_X << ", " << camFix_Y << endl;
    
    
    if(key == 'o'){
        originalCorners[oCornerIndex].set(mouseX, mouseY);
        oCornerIndex++;
        if(oCornerIndex>=4){
            oCornerIndex = 0;
        }
        cout << oCornerIndex << ": " << mouseX << ", " << mouseY << endl;
    }
    
    if(key == 'd'){
        distortedCorners[dCornerIndex].set(mouseX, mouseY);
        dCornerIndex++;
        if(dCornerIndex>=4){
            dCornerIndex = 0;
        }
        cout << mouseX << ", " << mouseY << endl;
    }
}

void ofApp::bubble_sort(ofPoint pos[], int n) {
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < i; ++j) {
            if (pos[j].x > pos[i].x) {
                ofPoint temp = pos[j];
                pos[j] = pos[i];
                pos[i] = temp;
            }
        }
    }
    
    if(pos[1].y>pos[2].y){
        ofPoint temp = pos[1];
        pos[1] = pos[2];
        pos[2] = temp;
        
        temp = pos[2];
        pos[2] = pos[3];
        pos[3] = temp;
    }else{
        ofPoint temp = pos[2];
        pos[2] = pos[3];
        pos[3] = temp;
    }
}

void ofApp::checkPos(ofPoint pos[]) {
    
}

ofPoint ofApp::getCenterOfGravityPoint(vector<ofPoint> pos){
    double area = 0.0;//多边形面积
    double Gx = 0.0, Gy = 0.0;// 重心的x、y
//    array<int, 10> arr = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    
    for (int i = 1; i <= pos.size(); i++) {
        double px = pos[i % pos.size()].x;
        double py = pos[i % pos.size()].y;
        double next_px = pos[i-1].x;
        double next_py = pos[i-1].y;
        double temp = (px * next_py - py * next_px) / 2.0;
        area += temp;
        Gx += temp * (px + next_px) / 3.0;
        Gy += temp * (py + next_py) / 3.0;
        
//        cout << i-1 << " ================" << endl;
//        cout << pos.at(i-1) << endl;
//
//        string _pos = ofToString(i-1) + ": " + ofToString(pos.at(i-1));
//        ofDrawBitmapString(_pos, pos.at(i-1).x+13, pos.at(i-1).y);
//        cout << "temp: " << temp << endl;
//        cout << "p: " << px << "," << py<< endl;
//        cout << "next: " << next_px << "," << next_py<< endl;
    }
    Gx = Gx / area;
    Gy = Gy / area;
//
//    cout << "array size: " << pos.size() << endl;
//    cout << "Area: " << area << ", Gravity: " << Gx << "," << Gy<< endl;
    

    return ofPoint(Gx, Gy);
}
