//
// Created by Andrea Fiorito on 11/01/2024.
//

#include "OrbSlam3.h"
#include "System.h"

using namespace ORB_SLAM3;

OrbSlam3::OrbSlam3(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer, const int initFr, const string &strSequence)
{
    _system = new System(strVocFile, strSettingsFile, sensor, bUseViewer, initFr, strSequence);
}

OrbSlam3::~OrbSlam3()
{
    delete _system;
}

void OrbSlam3::TrackMonocular(void *im, const double &timestamp)
{

}
