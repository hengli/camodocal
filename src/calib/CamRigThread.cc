#include "CamRigThread.h"

#include "CameraRigBA.h"

namespace camodocal
{

CamRigThread::CamRigThread(CameraSystem& cameraSystem,
                           SparseGraph& graph,
                           int beginStage,
                           bool optimizeIntrinsics,
                           bool saveWorkingData,
                           std::string dataDir,
                           bool verbose)
 : mThread(0)
 , mRunning(false)
 , mCameraSystem(cameraSystem)
 , mGraph(graph)
 , mBeginStage(beginStage)
 , mOptimizeIntrinsics(optimizeIntrinsics)
 , mSaveWorkingData(saveWorkingData)
 , mDataDir(dataDir)
 , mVerbose(verbose)
{

}

CamRigThread::~CamRigThread()
{
    g_return_if_fail(mThread == 0);
}

void
CamRigThread::launch(void)
{
    mThread = Glib::Threads::Thread::create(sigc::mem_fun(*this, &CamRigThread::threadFunction));
}

void
CamRigThread::join(void)
{
    if (mRunning)
    {
        mThread->join();
    }
    mThread = 0;
}

bool
CamRigThread::running(void) const
{
    return mRunning;
}

sigc::signal<void>&
CamRigThread::signalFinished(void)
{
    return mSignalFinished;
}

void
CamRigThread::threadFunction(void)
{
    mRunning = true;

    CameraRigBA ba(mCameraSystem, mGraph);
    ba.setVerbose(mVerbose);
    ba.run(mBeginStage, mOptimizeIntrinsics, mSaveWorkingData, mDataDir);

    mRunning = false;

    mSignalFinished();
}

}
