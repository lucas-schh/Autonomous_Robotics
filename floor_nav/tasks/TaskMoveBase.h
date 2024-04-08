#ifndef TASK_MOVE_BASE_H
#define TASK_MOVE_BASE_H

#include "task_manager_lib/TaskDefinition.h"
#include "floor_nav/SimTasksEnv.h"
#include "task_manager_action/TaskActionMoveBase.h"

using namespace task_manager_lib;
using namespace task_manager_action;

// There is no move_base for turtlesim, this is to test the principle of the
// generic TaskActionMoveBase
namespace floor_nav {
    class TaskMoveBase : public TaskActionMoveBase<SimTasksEnv>
    {
            TaskMoveBase(TaskDefinitionPtr def, TaskEnvironmentPtr env) :
                TaskActionMoveBase<SimTasksEnv>(def,env) {}
            virtual ~TaskMoveBase() {};
    };

    class TaskFactoryMoveBase : public TaskFactoryActionMoveBase<SimTasksEnv>
    {

        public:
            TaskFactoryMoveBase(TaskEnvironmentPtr env) :
                TaskFactoryActionMoveBase<SimTasksEnv>(env) {}
            virtual ~TaskFactoryMoveBase() {};
    };
};

#endif // TASK_MOVE_BASE_H
