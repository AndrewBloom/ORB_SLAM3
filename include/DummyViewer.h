//
// Created by Andrea Fiorito on 07/12/2023.
//

#ifndef ORB_SLAM3_DUMMYVIEWER_H
#define ORB_SLAM3_DUMMYVIEWER_H

#include "IViewer.h"

namespace ORB_SLAM3 {

    class DummyViewer : public IViewer {
        virtual void RequestStop() {};
        virtual bool isStopped() { return true; };
        virtual void Release() {};
    };

} // ORB_SLAM3

#endif //ORB_SLAM3_DUMMYVIEWER_H
