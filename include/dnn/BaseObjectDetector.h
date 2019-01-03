//
// Created by kandithws on 22/12/2561.
//

#ifndef ORB_SLAM2_BASEOBJECTDETECTOR_H
#define ORB_SLAM2_BASEOBJECTDETECTOR_H

#include <vector>
#include <opencv2/dnn/dnn.hpp>

namespace ORB_SLAM2 {
class BaseObjectDetector {
public:
    BaseObjectDetector();

    virtual void detectObject() = 0;
    virtual void drawBoxes() = 0;

};
}


#endif //ORB_SLAM2_BASEOBJECTDETECTOR_H
