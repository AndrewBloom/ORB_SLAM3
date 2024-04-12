//
// Created by Andrea Fiorito on 24/10/2023.
//
#include "SystemUI.h"
//#define PANGOLIN_UI // for debugging

namespace ORB_SLAM3 {
    SystemUI::SystemUI(Atlas *pAtlas, const string &strSettingsFile, Settings *settings_)
    {
        atlas = pAtlas;
#ifdef PANGOLIN_UI
        //Create Drawers. These are used by the Viewer
        mpFrameDrawer = new FrameDrawer(pAtlas);
        mpMapDrawer = new MapDrawer(pAtlas, strSettingsFile, settings_);
#else
        mpFrameDrawer = new DummyFrameDrawer();
        mpMapDrawer = new DummyMapDrawer();
#endif
    }

    void SystemUI::startUIThread(Tracking *pTracker, const string &strSettingsFile, Settings *settings_) {
        tracker = pTracker;
#ifdef PANGOLIN_UI
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,pTracker,strSettingsFile,settings_);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
        // mpLoopCloser->mpViewer = mpViewer;
        mpViewer->both = mpFrameDrawer->both;
#else
        mpViewer = new DummyViewer();
        pTracker->SetViewer(mpViewer);
#endif


    }

    IFrameDrawer* SystemUI::getFrameDrawer() { return mpFrameDrawer; };
    IMapDrawer* SystemUI::getMapDrawer() { return mpMapDrawer; };
    Tracking* SystemUI::getTracker() { return tracker; };
    Atlas* SystemUI::getAtlas() { return atlas; };
}