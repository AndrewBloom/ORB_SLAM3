//
// Created by Andrea Fiorito on 11/01/2024.
//

#include "include/public/OrbSlam3.h"
#include<opencv2/core.hpp>
#include "System.h"
#include "FeaturesDrawer.h"
#include "Eigen/src/Core/util/Memory.h"

#define XSTR(x) STR(x)
#define STR(x) #x

using namespace ORB_SLAM3;

void checks()
{
#if defined (EIGEN_VECTORIZE)
    std::cout << "Eigen3 vectorize on" << std::endl;
#else
    std::cout << "Eigen3 vectorize off" << std::endl;
#endif
    std::cout << "Eigen3 aligned new definition (empty for c++17): " << XSTR(EIGEN_MAKE_ALIGNED_OPERATOR_NEW) << std::endl;
}

OrbSlam3::OrbSlam3(const string &rootPath, const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer, const int initFr, const string &strSequence) :
isInitialised(false)
{
    checks();
    _system = new System(strVocFile, strSettingsFile, sensor, bUseViewer, initFr, strSequence, rootPath);
    featDrawer = new FeaturesDrawer(_system->sysUI->getTracker(), _system->sysUI->getAtlas());
    isInitialised = true;
}

OrbSlam3::~OrbSlam3()
{
    _system->Shutdown();
    delete _system;
}

void OrbSlam3::TrackMonocular(float *outMat, void *im, int32_t w, int32_t h, int8_t chan, const double &timestamp)
{
    if (!isInitialised) return;
    int ch;
    switch(chan) {
        case 1: ch = CV_8UC1; break;
        case 3: ch = CV_8UC3; break;
        case 4: ch = CV_8UC4; break;
        default: abort();
    };
    cv::Mat cvMat = cv::Mat(h, w, ch, im);
    assert(_system!= nullptr);
    Sophus::SE3<float> mat = _system->TrackMonocular(cvMat, timestamp);
    Sophus::SE3f::Transformation t = mat.matrix();
    assert(t.data()!= nullptr);
    cout << "InnerStride: " << t.innerStride() << "OuterStride: " << t.outerStride() << endl;
    Eigen::Map<Eigen::MatrixXf>(outMat, 4, 4) = t;
    cv::Size2f scale = _system->getImageSizeScale();
    Eigen::Vector3f translation = mat.translation();
    Eigen::Vector3f angles = { mat.angleX(), mat.angleY(), mat.angleZ() };
    featDrawer->DrawFrame(cvMat, scale, translation, angles);
/*    cout << "Mat is [" << outMat[0] << " " << outMat[4] << " " << outMat[8] << " " << outMat[12] << " \n"
            << "       ["  << outMat[1] << " " << outMat[5] << " " << outMat[9] << " " << outMat[13] << " \n"
            << "       ["  << outMat[2] << " " << outMat[6] << " " << outMat[10] << " " << outMat[14] << " \n"
            << "       ["  << outMat[3] << " " << outMat[7] << " " << outMat[11] << " " << outMat[15] << "]" << endl;
    cout << "Mat is [" << t(0,0) << " " << t(0,1) << " " << t(0,2) << " " << t(0,3) << " \n"
            << "       ["  << t(1,0) << " " << t(1,1) << " " << t(1,2) << " " << t(1,3) << " \n"
            << "       ["  << t(2,0) << " " << t(2,1) << " " << t(2,2) << " " << t(2,3) << " \n"
            << "       ["  << t(3,0) << " " << t(3,1) << " " << t(3,2) << " " << t(3,3) << "]" << endl;*/
    //Eigen::Quaternionf q = mat.unit_quaternion();
/*    cout <<  setprecision(9) << "Translation: [" << twc(0) << ", " << twc(1) << ", " << twc(2) << "]\n";
    cout << "Quaternion: [" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << "]\n";
    cout << "Angles: [" << mat.angleX() << ", " << mat.angleY() << ", " << mat.angleZ() << "]" << endl;*/
    if (_system->GetTrackingState() != lastTrackingState) {
        lastTrackingState = _system->GetTrackingState();
        cout << "Tracking state is " << _system->GetTrackingState() << endl;
    }
}
