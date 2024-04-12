//
// Created by bloom on 21/02/2024.
//

#include "FeaturesDrawer.h"

namespace ORB_SLAM3 {

    void FeaturesDrawer::DrawFrame(cv::Mat &im, cv::Size2f &scale, const Eigen::Vector3f &tr, const Eigen::Vector3f &ang) {
        const cv::Scalar standardColor(0, 255, 0);
        const cv::Scalar odometryColor(255, 0, 0);
        int mnTracked = 0, mnTrackedVO = 0;
        Tracking::eTrackingState state = pTracker->mLastProcessedState;
        vector<cv::KeyPoint> &vCurrentKeys = pTracker->mCurrentFrame.mvKeys; // KeyPoints in current frame

        if (state == Tracking::NOT_INITIALIZED) {
            vector<int> &vMatches = pTracker->mvIniMatches;; // Initialization: correspondeces with reference keypoints
            vector<cv::KeyPoint> &vIniKeys = pTracker->mInitialFrame.mvKeys; // Initialization: KeyPoints in reference frame
            for (unsigned int i = 0; i < vMatches.size(); i++) {
                if (vMatches[i] >= 0) {
                    cv::Point2f pt1, pt2;
                    pt1.x = vIniKeys[i].pt.x / scale.width;
                    pt1.y = vIniKeys[i].pt.y / scale.height;
                    pt2.x = vCurrentKeys[vMatches[i]].pt.x / scale.width;
                    pt2.y = vCurrentKeys[vMatches[i]].pt.y / scale.height;
                    cv::line(im, pt1, pt2, standardColor);
                }
            }
        } else if (state == Tracking::OK) //TRACKING
        {
            vector<bool> vbVO = vector<bool>(vCurrentKeys.size(), false);
            vector<bool> vbMap = vector<bool>(vCurrentKeys.size(), false);
            for (int i = 0; i < vCurrentKeys.size(); i++) {
                MapPoint *pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
                if (pMP != nullptr) {
                    if (!pTracker->mCurrentFrame.mvbOutlier[i]) {
                        if (pMP->Observations() > 0) vbMap[i] = true;
                        else vbVO[i] = true;
                    }
                }
            }
            const float r = 5;
            int n = vCurrentKeys.size();
            for (int i = 0; i < n; i++) {
                if (vbVO[i] || vbMap[i]) {
                    cv::Point2f pt1, pt2;
                    cv::Point2f point;

                    point.x = vCurrentKeys[i].pt.x / scale.width;
                    point.y = vCurrentKeys[i].pt.y / scale.height;
                    pt1.x = point.x - r;
                    pt1.y = point.y - r;
                    pt2.x = point.x + r;
                    pt2.y = point.y + r;

                    // This is a match to a MapPoint in the map
                    if (vbMap[i]) {
                        cv::rectangle(im, pt1, pt2, standardColor);
                        cv::circle(im, point, 2, standardColor, -1);
                        mnTracked++;
                    } else // This is match to a "visual odometry" MapPoint created in the last frame
                    {
                        cv::rectangle(im, pt1, pt2, odometryColor);
                        cv::circle(im, point, 2, odometryColor, -1);
                        mnTrackedVO++;
                    }
                }
            }
        }
        DrawTextInfo(im, state, mnTracked, mnTrackedVO, tr, ang);
    }

    void FeaturesDrawer::DrawTextInfo(cv::Mat &im, int nState, int mnTracked, int mnTrackedVO, const Eigen::Vector3f &tr, const Eigen::Vector3f &ang) {
        stringstream s;
        if (nState == Tracking::NO_IMAGES_YET)
            s << " WAITING FOR IMAGES";
        else if (nState == Tracking::NOT_INITIALIZED)
            s << " TRYING TO INITIALIZE ";
        else if (nState == Tracking::OK) {
            if (!pTracker->mbOnlyTracking)
                s << "SLAM MODE |  ";
            else
                s << "LOCALIZATION | ";
            int nMaps = mpAtlas->CountMaps();
            int nKFs = mpAtlas->KeyFramesInMap();
            int nMPs = mpAtlas->MapPointsInMap();
            s << "Maps: " << nMaps << ", KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: "
              << mnTracked;
            if (mnTrackedVO > 0)
                s << ", + VO matches: " << mnTrackedVO;
        } else if (nState == Tracking::LOST) {
            s << " TRACK LOST. TRYING TO RELOCALIZE ";
        } else if (nState == Tracking::SYSTEM_NOT_READY) {
            s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
        }
        stringstream pos, ori;
        pos << setw(9) << setprecision(5) << "Translation [" << tr.x() << ", " << tr.y() << ", " << tr.z() << "]";
        ori << setw(9) << setprecision(5) << "Angles [" << ang.x() << ", " << ang.y() << ", " << ang.z() << "]";

        int baseline = 0;
        constexpr int fontscale = 2, thickness = 2;
        cv::Size ts = cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, fontscale, thickness, &baseline);
        cv::putText(im, s.str(), cv::Point(5, im.rows - ts.height - 5), cv::FONT_HERSHEY_PLAIN, fontscale,
                    cv::Scalar(255, 255, 255), thickness, 8, true);
        ts = cv::getTextSize(pos.str(), cv::FONT_HERSHEY_PLAIN, fontscale, thickness, &baseline);
        cv::putText(im, pos.str(), cv::Point(5, 3*ts.height + 5), cv::FONT_HERSHEY_PLAIN, fontscale,
                    cv::Scalar(255, 255, 255), thickness, 8, true);
        ts = cv::getTextSize(ori.str(), cv::FONT_HERSHEY_PLAIN, fontscale, thickness, &baseline);
        cv::putText(im, ori.str(), cv::Point(5, ts.height + 5), cv::FONT_HERSHEY_PLAIN, fontscale,
                    cv::Scalar(255, 255, 255), thickness, 8, true);
    }
}