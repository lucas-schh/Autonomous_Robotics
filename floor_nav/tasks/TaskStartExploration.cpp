#include <math.h>
#include "TaskStartExploration.h"
#include "floor_nav/TaskStartExplorationConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_GOTO

TaskIndicator TaskStartExploration::initialise() 
{
    std_msgs::Bool msg;
    msg.data=true; 
    env->publishExploration(msg);
    return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskStartExploration::iterate()
{
    std_msgs::Bool msg;
    msg.data=true; 
    env->publishExploration(msg);   
	return TaskStatus::TASK_RUNNING;
    if(env->getBatteryLvl()<20){
        return TaskStatus::TASK_COMPLETED;
    }
}

TaskIndicator TaskStartExploration::terminate()
{
    std_msgs::Bool msg;
    msg.data=false; 
    env->publishExploration(msg);
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryStartExploration);
