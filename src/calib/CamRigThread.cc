#include "CamRigThread.h"

#include <boost/make_shared.hpp>

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
 : m_running(false)
 , m_cameraSystem(cameraSystem)
 , m_graph(graph)
 , m_beginStage(beginStage)
 , m_optimizeIntrinsics(optimizeIntrinsics)
 , m_saveWorkingData(saveWorkingData)
 , m_dataDir(dataDir)
 , m_verbose(verbose)
{

}

CamRigThread::~CamRigThread()
{

}

void
CamRigThread::launch(void)
{
    m_running = true;

    m_thread = boost::make_shared<boost::thread>(&CamRigThread::threadFunction, this);
}

void
CamRigThread::join(void)
{
    if (m_running)
    {
        m_thread->join();
    }
}

bool
CamRigThread::running(void) const
{
    return m_running;
}

boost::signals2::signal<void ()>&
CamRigThread::signalFinished(void)
{
    return m_signalFinished;
}

void
CamRigThread::threadFunction(void)
{
    CameraRigBA ba(m_cameraSystem, m_graph);
    ba.setVerbose(m_verbose);
    ba.run(m_beginStage, m_optimizeIntrinsics, m_saveWorkingData, m_dataDir);

    m_running = false;

    m_signalFinished();
}

}
