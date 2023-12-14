//
// Created by Andrea Fiorito on 07/12/2023.
//

#ifndef ORB_SLAM3_IVIEWER_H
#define ORB_SLAM3_IVIEWER_H

namespace ORB_SLAM3 {

    class IViewer {
    public:
        virtual void RequestStop() = 0;

        virtual bool isStopped() = 0;

        virtual void Release() = 0;
    };
}
#endif //ORB_SLAM3_IVIEWER_H
