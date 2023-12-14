//
// Created by Andrea Fiorito on 07/12/2023.
//

#ifndef ORB_SLAM3_DUMMYMAPDRAWER_H
#define ORB_SLAM3_DUMMYMAPDRAWER_H

#include "IMapDrawer.h"

namespace ORB_SLAM3 {

    class DummyMapDrawer : public IMapDrawer {
    public:
        virtual void SetCurrentCameraPose(const Sophus::SE3f &Tcw) {};
    };

} // ORB_SLAM3

#endif //ORB_SLAM3_DUMMYMAPDRAWER_H
