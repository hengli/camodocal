#ifndef CAMRIGTHREAD_H
#define CAMRIGTHREAD_H

#include <glibmm.h>

#include "camodocal/camera_models/Camera.h"
#include "camodocal/camera_systems/CameraRigExtrinsics.h"
#include "camodocal/sparse_graph/SparseGraph.h"

namespace camodocal
{

class CamRigThread
{
public:
    explicit CamRigThread(const std::vector<CameraPtr>& cameras,
                          CameraRigExtrinsics& cameraRigExt,
                          SparseGraph& graph,
                          int beginStage = 1,
                          bool optimizeIntrinsics = true,
                          bool saveWorkingData = false,
                          std::string dataDir = "data",
                          bool verbose = false);
    virtual ~CamRigThread();

    void launch(void);
    void join(void);
    bool running(void) const;
    sigc::signal<void>& signalFinished(void);

private:
    void threadFunction(void);

    Glib::Threads::Thread* mThread;
    bool mRunning;
    sigc::signal<void> mSignalFinished;

    const std::vector<CameraPtr>& mCameras;
    CameraRigExtrinsics& mCameraRigExt;
    SparseGraph& mGraph;

    int mBeginStage;
    bool mOptimizeIntrinsics;
    bool mSaveWorkingData;
    std::string mDataDir;
    bool mVerbose;
};

}

#endif
