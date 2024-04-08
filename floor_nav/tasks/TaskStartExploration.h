#ifndef TASK_START_EXPLORATION_H
#define TASK_START_EXPLORATION_H

#include "task_manager_lib/TaskDefinition.h"
#include "floor_nav/SimTasksEnv.h"
#include "floor_nav/TaskStartExplorationConfig.h"
#include "std_msgs/Bool.h"

using namespace task_manager_lib;

namespace floor_nav {
    class TaskStartExploration : public TaskInstance<TaskStartExplorationConfig,SimTasksEnv>
    {
        protected: 
            double initial_heading;
        public:
            TaskStartExploration(TaskDefinitionPtr def, TaskEnvironmentPtr env) : Parent(def,env) {}
            virtual ~TaskStartExploration() {};

            virtual TaskIndicator initialise() ;

            virtual TaskIndicator iterate();

            virtual TaskIndicator terminate();
    };
    class TaskFactoryStartExploration : public TaskDefinition<TaskStartExplorationConfig, SimTasksEnv, TaskStartExploration>
    {

        public:
            TaskFactoryStartExploration(TaskEnvironmentPtr env) : 
                Parent("StartExploration","Reach a desired heading angle",true,env) {}
            virtual ~TaskFactoryStartExploration() {};
    };
};

#endif // TASK_START_EXPLORATION_H