#ifndef CAMRIGTHREAD_H
#define CAMRIGTHREAD_H

#include <boost/signals2.hpp>
#include <boost/thread.hpp>

#include "camodocal/camera_models/Camera.h"
#include "camodocal/camera_systems/CameraSystem.h"
#include "camodocal/sparse_graph/SparseGraph.h"

namespace camodocal
{

class CamRigThread
{
public:
    explicit CamRigThread(CameraSystem& cameraSystem,
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
    boost::signals2::signal<void ()>& signalFinished(void);

private:
    void threadFunction(void);

    boost::shared_ptr<boost::thread> m_thread;
    bool m_running;
    boost::signals2::signal<void ()> m_signalFinished;

    CameraSystem& m_cameraSystem;
    SparseGraph& m_graph;

    int m_beginStage;
    bool m_optimizeIntrinsics;
    bool m_saveWorkingData;
    std::string m_dataDir;
    bool m_verbose;
};

}

#endif
