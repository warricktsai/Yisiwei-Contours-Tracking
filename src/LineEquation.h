//
//  LineEquation.h
//  Yisiwei-Contours-Tracking
//
//  Created by Warrick on 2021/7/12.
//
#pragma once

#include "ofMain.h"

#ifndef LineEquation_h
#define LineEquation_h

class LineEquation{
    
public:
    double a;
    double b;
    double c;
    
    LineEquation GetLine(ofPoint ptSource, ofPoint ptDestination);
    ofPoint GetCrossPoint(LineEquation l1, LineEquation l2);
    void CrossPointShow(ofPoint ptCross);

};

#endif /* LineEquation_h */
