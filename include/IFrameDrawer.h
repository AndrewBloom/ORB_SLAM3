//
// Created by Andrea Fiorito on 07/12/2023.
//

#ifndef ORB_SLAM3_IFRAMEDRAWER_H
#define ORB_SLAM3_IFRAMEDRAWER_H

#include "Tracking.h"

namespace ORB_SLAM3 {

    class Tracking;

    class IFrameDrawer {
    public:
        bool both;
        virtual void Update(Tracking *pTracker) = 0;
    };
}
#endif //ORB_SLAM3_IFRAMEDRAWER_H
