//
// Created by Andrea Fiorito on 24/10/2023.
//

#ifndef ORB_SLAM3_SYSTEMUI_H
#define ORB_SLAM3_SYSTEMUI_H

#include<thread>
#ifdef PANGOLIN_UI
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Viewer.h"
#else
#include "DummyFrameDrawer.h"
#include "DummyMapDrawer.h"
#include "DummyViewer.h"
#endif

namespace ORB_SLAM3 {

    class SystemUI {
    private:
        // The viewer draws the map and the current camera pose. It uses Pangolin.
#ifdef PANGOLIN_UI
        Viewer* mpViewer;
        FrameDrawer* mpFrameDrawer;
        MapDrawer* mpMapDrawer;
#else
        DummyViewer *mpViewer;
        DummyFrameDrawer *mpFrameDrawer;
        DummyMapDrawer *mpMapDrawer;
        Tracking* tracker;
        Atlas* atlas;
#endif

        std::thread *mptViewer;
    public:
        SystemUI(Atlas *pAtlas, const string &strSettingsFile, Settings *settings_);
        IFrameDrawer* getFrameDrawer();
        IMapDrawer* getMapDrawer();
        Tracking* getTracker();
        Atlas* getAtlas();
        void startUIThread(Tracking *pTracker, const string &strSettingsFile, Settings *settings_);
    };
}
#endif //ORB_SLAM3_SYSTEMUI_H
