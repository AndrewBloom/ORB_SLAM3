//
// Created by Andrea Fiorito on 11/01/2024.
//

#ifndef ORB_SLAM3_ORBSLAM3_H
#define ORB_SLAM3_ORBSLAM3_H

#include <string>

using namespace std;
/** This class is the main interface for the library with the external world
 *  and eliminates any dependency on other headers
 */
namespace ORB_SLAM3 {

    class System;
    class FeaturesDrawer;

    class OrbSlam3 {
    private:
        atomic_bool isInitialised;
        int lastTrackingState = 0;
        System* _system;
        FeaturesDrawer* featDrawer;
    public:
        // Input sensor
        enum eSensor{
            MONOCULAR=0,
            STEREO=1,
            RGBD=2,
            IMU_MONOCULAR=3,
            IMU_STEREO=4,
            IMU_RGBD=5,
        };

        OrbSlam3(const string &rootPath, const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true, const int initFr = 0, const string &strSequence = std::string());
        ~OrbSlam3();
        void TrackMonocular(float *outMat, void *im, int32_t w, int32_t h, int8_t chan, const double &timestamp);
    };
}
#endif //ORB_SLAM3_ORBSLAM3_H