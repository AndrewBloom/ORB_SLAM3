//
// Created by bloom on 21/02/2024.
//

#ifndef ORBSLAM3APP_FEATURESDRAWER_H
#define ORBSLAM3APP_FEATURESDRAWER_H

#include<opencv2/core/core.hpp>
#include "Tracking.h"

namespace ORB_SLAM3 {
    class FeaturesDrawer {
    public:
        FeaturesDrawer(Tracking *t, Atlas* a) : pTracker(t), mpAtlas(a) {}
        void DrawFrame(cv::Mat &im, cv::Size2f &scale, const Eigen::Vector3f &tr, const Eigen::Vector3f &ang);

    private:
        void DrawTextInfo(cv::Mat &im, int nState, int mnTracked, int mnTrackedVO, const Eigen::Vector3f &tr, const Eigen::Vector3f &ang);
        Tracking *pTracker = nullptr;
        Atlas* mpAtlas = nullptr;
    };
}

#endif //ORBSLAM3APP_FEATURESDRAWER_H
