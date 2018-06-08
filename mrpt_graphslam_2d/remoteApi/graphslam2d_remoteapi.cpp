#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include "mrpt/obs/CObservation2DRangeScan.h"
#include "mrpt/obs/CObservationOdometry.h"
#include "mrpt/obs/CSensoryFrame.h"
#include "mrpt/obs/CObservation.h"
#include "mrpt/obs/CActionCollection.h"
#include "mrpt/obs/CActionRobotMovement2D.h"
#include "mrpt/poses/CPose3D.h"
#include "mrpt/system/COutputLogger.h"
#include "mrpt/graphs/CNetworkOfPoses.h"
#include "mrpt/graphslam/CGraphSlamEngine.h"
#include "mrpt/graphslam/apps_related/TUserOptionsChecker.h"
#include "../../conversions/include/vrep_conversion.h"
extern "C" 
{
    #include "extApi.h"
}
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace vrep_bridge;
using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::graphs;
using namespace mrpt::graphslam;
using namespace mrpt::graphslam::deciders;
using namespace mrpt::graphslam::optimizers;
using namespace mrpt::graphslam::apps;
using namespace mrpt::opengl;
using namespace mrpt::gui;

template <class GRAPH_T>
CGraphSlamEngine<GRAPH_T>* initGraphslam(mrpt::system::COutputLogger* logger)
{
    
    TUserOptionsChecker<GRAPH_T> options_checker;
    options_checker.createDeciderOptimizerMappings();
    options_checker.populateDeciderOptimizerProperties();
    string node_reg = "CEmptyNRD";
    string edge_reg = "CEmptyERD";
    string optimizer = "CEmptyGSO";
    string ini_fname = "../config/odometry_2DRangeScans.ini";
    logger->logFmt(LVL_INFO, "Node registration decider: %s", node_reg.c_str());
    logger->logFmt(LVL_INFO, "Edge registration decider: %s", edge_reg.c_str());
    logger->logFmt(LVL_INFO, "graphSLAM Optimizer: %s", optimizer.c_str());

    //CGraphSlamHandler<GRAPH_T> graphslam_handler(logger, &options_checker, true);
    //graphslam_handler.setFNames(ini_fname, "", "");

    //graphslam_handler.initEngine(node_reg, edge_reg, optimizer);
    mrpt::graphslam::CWindowManager* m_win_manager;
    mrpt::graphslam::CWindowObserver* m_win_observer;
    mrpt::gui::CDisplayWindow3D* m_win;


    m_win_observer = new CWindowObserver();
    m_win = new CDisplayWindow3D("GraphSlam building procedure", 800, 600);
    m_win->setPos(400, 200);
    m_win_observer->observeBegin(*m_win);
    {
        COpenGLScene::Ptr& scene = m_win->get3DSceneAndLock();
        COpenGLViewport::Ptr main_view = scene->getViewport("main");
        m_win_observer->observeBegin(*main_view);
        m_win->unlockAccess3DScene();
    }

    logger->logFmt(mrpt::system::LVL_DEBUG, "Initialized CDisplayWindow3D");
    logger->logFmt(mrpt::system::LVL_DEBUG, "Listening to CDisplayWindow3D events");

    m_win_manager = new mrpt::graphslam::CWindowManager();
    m_win_manager->setCDisplayWindow3DPtr(m_win);
    m_win_manager->setWindowObserverPtr(m_win_observer);
    CGraphSlamEngine<GRAPH_T>* engine = 
        new CGraphSlamEngine<GRAPH_T>(ini_fname,"","",m_win_manager,
            options_checker.node_regs_map[node_reg](),
            options_checker.edge_regs_map[edge_reg](),
            options_checker.optimizers_map[optimizer]());
    return engine;  
}

template <class GRAPH_T>
void saveresults(CGraphSlamEngine<GRAPH_T>* engine,mrpt::system::COutputLogger* logger)
{
    string ini_fname = "../config/odometry_2DRangeScans.ini";
    mrpt::config::CConfigFile cfg_file(ini_fname);
    bool m_user_decides_about_output_dir = cfg_file.read_bool(
        "GeneralConfiguration", "user_decides_about_output_dir", false, false);
    string m_output_dir_fname = cfg_file.read_string(
        "GeneralConfiguration", "output_dir_fname", "graphslam_results", false);
    bool m_save_graph =
        cfg_file.read_bool("GeneralConfiguration", "save_graph", true, false);
    bool m_save_3DScene =
        cfg_file.read_bool("GeneralConfiguration", "save_3DScene", true, false);
    bool m_save_map =
        cfg_file.read_bool("GeneralConfiguration", "save_map", true, false);
    string m_save_graph_fname = cfg_file.read_string(
        "GeneralConfiguration", "save_graph_fname", "output_graph.graph",
        false);
    string m_save_3DScene_fname = cfg_file.read_string(
        "GeneralConfiguration", "save_3DScene_fname", "scene.3DScene", false);
    string m_save_map_fname = cfg_file.read_string(
        "GeneralConfiguration", "save_map_fname", "output_map", false);
    if(directoryExists(m_output_dir_fname))
    {
        logger->logFmt(mrpt::system::LVL_INFO, "Found output directory.");
    }
    else
    {
        createDirectory(m_output_dir_fname);
        logger->logFmt(mrpt::system::LVL_INFO, "Finished initializing output directory.");        
    }

    engine->generateReportFiles(m_output_dir_fname);
}
int main()
{   /**
    *This is a sample VREP remote api which fetches the following data from the simulator
    *- <b>object handle[]</b> -> Using simxGetObjectHandle.
    *- <b>object position</b> -> Using simxGetObjectPosition
    *- <b>object velocity</b> -> Using simxGetObjectVelocity
    *- <b>object quaternion</b> -> Using simxGetObjectQuaternion
    *
    * <i> This api also makes use of the conversion methods defined in ../../conversions/vrep_conversion.cpp<i> 
    */
    simxFinish(-1); //close all existing connections.
    int clientID=simxStart((simxChar*)"127.0.0.1",19997,true,true,2000,5);
    simxInt handle;
    simxFloat position[3],scanningAngle,maxScanDistance;
    simxUChar* dataSignal;
    int dataSignalSize,dataCount;
    simxFloat quaternion[4],vel[3],angularvel[3];
    COutputLogger logger("graphslam_remoteApi");
    logger.logging_enable_keep_record = true;

    CGraphSlamEngine<CNetworkOfPoses2DInf>* graphslam_engine = initGraphslam<CNetworkOfPoses2DInf>(&logger);
    graphslam_engine->printParams();

    if (clientID!=-1)
    {
        printf("Simulation started \n");
        while (simxGetConnectionId(clientID)!=-1)
        {
            simxGetObjectHandle(clientID,"fastHokuyo",&handle,simx_opmode_streaming);//get object handlesimxGetObjectPosition(clientID,handle,-1,position,simx_opmode_streaming); // get object position
            simxGetObjectPosition(clientID,handle,-1,position,simx_opmode_streaming); // get object orientation(returns euler angles)
            simxGetObjectQuaternion(clientID,handle,-1,quaternion,simx_opmode_streaming);
            simxGetFloatSignal(clientID,"scanningAngle",&scanningAngle,simx_opmode_streaming);
            simxGetFloatSignal(clientID,"maxScanDistance",&maxScanDistance,simx_opmode_streaming);
            simxGetStringSignal(clientID,"measuredDataAtThisTime",&dataSignal,&dataSignalSize,simx_opmode_streaming);
            simxGetObjectVelocity(clientID,handle,vel,angularvel,simx_opmode_streaming);

            if (simxGetStringSignal(clientID,"measuredDataAtThisTime",&dataSignal,&dataSignalSize,simx_opmode_buffer)==simx_return_ok
                && simxGetFloatSignal(clientID,"scanningAngle",&scanningAngle,simx_opmode_buffer)==simx_return_ok
                && simxGetFloatSignal(clientID,"maxScanDistance",&maxScanDistance,simx_opmode_buffer)==simx_return_ok)
            {
                dataCount=dataSignalSize/12;
                float range[dataCount];
                for (int i=0;i<dataCount;i++)
                {
                    float x =((float*)dataSignal)[3*i];
                    float y =((float*)dataSignal)[3*i+1];
                    range[i] = sqrt(x*x + y*y);
                }
                CPose3D sensor_pose;
                bool pose_convert = convert(position,quaternion,sensor_pose);
                CObservation2DRangeScan obj;
                bool rangescan_convert = convert(range,dataCount,maxScanDistance,scanningAngle,sensor_pose,obj);
                printf("Aperture : %f\n",obj.aperture );
                CObservationOdometry odom;
                bool odometry_convert = convert(sensor_pose, vel, angularvel,odom);
                CObservation2DRangeScan::Ptr obs = CObservation2DRangeScan::Create(obj);
                CActionCollection action;
                CActionRobotMovement2D myAction;
                CPose2D sensor_pose_2d = odom.odometry;
                CActionRobotMovement2D::TMotionModelOptions motion_model_options;          
                myAction.computeFromOdometry(sensor_pose_2d,motion_model_options);
                action.insert(myAction);
                CSensoryFrame observations;
                observations.insert(obs);
                CObservation::Ptr observation;
                CActionCollection::Ptr action_ptr = CActionCollection::Create(action);
                CSensoryFrame::Ptr observations_ptr = CSensoryFrame::Create(observations);
                size_t size=0;
                graphslam_engine->_execGraphSlamStep(action_ptr, observations_ptr, observation,size);
            }
            printf("Handle : %u\n",handle);
            printf("Laser Position : (%f,%f,%f) \n",position[0],position[1],position[2]);
            extApi_sleepMs(100);
        }
        saveresults(graphslam_engine,&logger);
        simxFinish(clientID);
    }
    return(0);
}