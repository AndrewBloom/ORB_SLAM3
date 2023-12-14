//
// Created by Andrea Fiorito on 07/12/2023.
//

#ifndef ORB_SLAM3_IMAPDRAWER_H
#define ORB_SLAM3_IMAPDRAWER_H

#include "../Thirdparty/Sophus/sophus/se3.hpp"

namespace ORB_SLAM3 {

    class IMapDrawer {
    public:
        virtual void SetCurrentCameraPose(const Sophus::SE3f &Tcw) = 0;
    };

}
#endif //ORB_SLAM3_IMAPDRAWER_H
