//
// Created by Andrea Fiorito on 07/12/2023.
//

#ifndef ORB_SLAM3_DUMMYFRAMEDRAWER_H
#define ORB_SLAM3_DUMMYFRAMEDRAWER_H

#include "IFrameDrawer.h"

namespace ORB_SLAM3 {

    class DummyFrameDrawer : public IFrameDrawer {
        virtual void Update(Tracking *pTracker) {};
    };

} // ORB_SLAM3

#endif //ORB_SLAM3_DUMMYFRAMEDRAWER_H
