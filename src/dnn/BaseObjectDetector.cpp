//
// Created by kandithws on 22/12/2561.
//

#include "dnn/BaseObjectDetector.h"

namespace ORB_SLAM2 {

PredictedObject::PredictedObject() {}

PredictedObject::PredictedObject(int label, float conf) :
        _label(label),
        _confidence(conf) {}

PredictedObject::PredictedObject(int label, float conf, cv::Rect &box) :
    _label(label),
    _confidence(conf) {
    setPolyfromRect(box);
    _bbox = std::shared_ptr<cv::Rect2f>(new cv::Rect2f(box));
}

PredictedObject::PredictedObject(int label, float conf, std::vector<cv::Point2f> &poly) :
    _label(label),
    _confidence(conf),
    _poly(poly) {}

cv::Rect2f& PredictedObject::box() {
    // TODO -- Throws runtime assertion
    if (!_bbox)
        generateBoundingBox();

    return *_bbox;
}

void PredictedObject::setPolyfromRect(cv::Rect &rect) {
    if (!_poly.empty())
        _poly.clear();

    _poly.push_back(rect.tl());
    _poly.push_back(rect.br());
}

void PredictedObject::generateBoundingBox() {
    double x_min = std::numeric_limits<double>::max(), y_min = std::numeric_limits<double>::max();
    double x_max = std::numeric_limits<double>::min(), y_max = std::numeric_limits<double>::min();
    for (const auto &pt : _poly) {
        if (pt.x < x_min)
            x_min = pt.x;
        if (pt.x > x_max)
            x_max = pt.x;
        if (pt.y < y_min)
            y_min = pt.y;
        if (pt.y > y_max)
            y_max = pt.y;
    }

    _bbox = std::shared_ptr<cv::Rect2f>(new cv::Rect2f(cv::Point2f(x_min, y_min), cv::Point2f(x_max, y_max)));
}


void BaseObjectDetector::drawPredictionBoxes(cv::Mat &img, std::vector<PredictedObject> &preds) {
    for (auto &pred : preds){
        int top = (int)pred.box().tl().x;
        int left = (int)pred.box().tl().y;
        cv::rectangle(img, pred.box().tl(), pred.box().br(), cv::Scalar(0, 255, 0));

        char conf_str[10];
        sprintf(conf_str, "%.2f", pred._confidence);
        std::string label(conf_str);
        if (!_label_map.size())
        {
            CV_Assert(pred._label < (int)_label_map.size());
            label = _label_map[pred._label] + ": " + label;
        }

        int baseLine;
        cv::Size labelSize = cv::getTextSize(label, CV_FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        top = std::max(top, labelSize.height);
        cv::putText(img, label, cv::Point(left, top), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
    }
}

}