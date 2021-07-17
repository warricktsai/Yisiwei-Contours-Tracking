//
//  LineEquation.cpp
//  Yisiwei-Contours-Tracking
//
//  Created by Warrick on 2021/7/12.
//

#include <stdio.h>
#include "LineEquation.h"

LineEquation LineEquation::GetLine(ofPoint ptSource, ofPoint ptDestination){
    LineEquation lTemp;
    lTemp.a = ptSource.y - ptDestination.y;
    lTemp.b = ptDestination.x - ptSource.x;
    lTemp.c = ptSource.x*ptDestination.y - ptDestination.x*ptSource.y;
    return lTemp;
}


ofPoint LineEquation::GetCrossPoint(LineEquation l1, LineEquation l2){
    ofPoint pTemp;
    double D;
    D = l1.a*l2.b - l2.a*l1.b;
    ofPoint p;
    pTemp.x = (l1.b*l2.c - l2.b*l1.c)/D;
    pTemp.y = (l1.c*l2.a - l2.c*l1.a)/D;
    return pTemp;
}

void LineEquation::CrossPointShow(ofPoint ptCross){
    cout << "兩條直線交點的橫座標：" << ptCross.x << endl;
    cout << "兩條直線交點的縱座標：" << ptCross.y << endl;
}
