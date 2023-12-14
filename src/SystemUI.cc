//
// Created by Andrea Fiorito on 24/10/2023.
//
#include "SystemUI.h"
//#define PANGOLIN_UI // for debugging

namespace ORB_SLAM3 {
    SystemUI::SystemUI(Atlas *mpAtlas, const string &strSettingsFile, Settings *settings_)
    {
#ifdef PANGOLIN_UI
        //Create Drawers. These are used by the Viewer
        mpFrameDrawer = new FrameDrawer(mpAtlas);
        mpMapDrawer = new MapDrawer(mpAtlas, strSettingsFile, settings_);
#else
        mpFrameDrawer = new DummyFrameDrawer();
        mpMapDrawer = new DummyMapDrawer();
#endif
    }

    void SystemUI::startUIThread(Tracking *mpTracker, const string &strSettingsFile, Settings *settings_) {
#ifdef PANGOLIN_UI
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile,settings_);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
        // mpLoopCloser->mpViewer = mpViewer;
        mpViewer->both = mpFrameDrawer->both;
#else
        mpViewer = new DummyViewer();
        mpTracker->SetViewer(mpViewer);
#endif


    }

    IFrameDrawer* SystemUI::getFrameDrawer() { return mpFrameDrawer; };
    IMapDrawer* SystemUI::getMapDrawer() { return mpMapDrawer; };
}