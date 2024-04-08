#include <math.h>
#include "TaskGoToPose.h"
#include "floor_nav/TaskGoToPoseConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_GOTOPOSE
#ifdef DEBUG_GOTOPOSE
#warning Debugging task GOTOPOSE
#endif


TaskIndicator TaskGoToPose::initialise() 
{
    ROS_INFO("Going to %.2f %.2f, with angle %.2f",cfg.goal_x,cfg.goal_y,cfg.goal_theta*180./M_PI);
    if (cfg.relative) {
        const geometry_msgs::Pose2D & tpose = env->getPose2D();
        x_init = tpose.x;
        y_init = tpose.y;
        theta_init = tpose.theta;
    } else {
        x_init = 0.0;
        y_init = 0.0;
        theta_init = 0.0;
    }
    return TaskStatus::TASK_INITIALISED;
}


TaskIndicator TaskGoToPose::iterate()
{
    const geometry_msgs::Pose2D & tpose = env->getPose2D();
    if(!cfg.holonomic_mode){
        if(cfg.ctrlmtd){
            double r = hypot(y_init + cfg.goal_y-tpose.y,x_init + cfg.goal_x-tpose.x);
            
            double alpha = remainder(atan2((y_init + cfg.goal_y-tpose.y),x_init + cfg.goal_x-tpose.x)-tpose.theta,2*M_PI);
            double beta =remainder((-alpha -(tpose.theta+cfg.goal_theta-theta_init)), 2*M_PI);    
            //ROS_INFO("beta: %.2lf, alpha: %.2lf, theta: %.2f", beta*180./M_PI, alpha*180./M_PI, cfg.goal_theta*180./M_PI);

            #ifdef DEBUG_GOTOPOSE
                printf("c %.1f %.1f %.1f g %.1f %.1f r %.3f alpha %.1f\n",
                        tpose.x, tpose.y, tpose.theta*180./M_PI,
                        cfg.goal_x,cfg.goal_y,r,alpha*180./M_PI);
            #endif
            if (fabs(beta) < cfg.angle_threshold && r < cfg.dist_threshold) {
                return TaskStatus::TASK_COMPLETED;
            }
            double vel = cfg.k_v * r;
            if (vel > cfg.max_velocity) vel = cfg.max_velocity;
            double rot = std::max(std::min(cfg.k_alpha*alpha+ cfg.k_beta*beta,cfg.max_angular_velocity),-cfg.max_angular_velocity);
            env->publishVelocity(vel, rot);
            return TaskStatus::TASK_RUNNING;

        } else{
            double r = hypot(y_init + cfg.goal_y-tpose.y,x_init + cfg.goal_x-tpose.x);
            if (r < cfg.dist_threshold) {
                return TaskStatus::TASK_COMPLETED;
            }
            double alpha = remainder(atan2((y_init + cfg.goal_y-tpose.y),x_init + cfg.goal_x-tpose.x)-tpose.theta,2*M_PI);
            #ifdef DEBUG_GOTO
                printf("c %.1f %.1f %.1f g %.1f %.1f r %.3f alpha %.1f\n",
                tpose.x, tpose.y, tpose.theta*180./M_PI,
                cfg.goal_x,cfg.goal_y,r,alpha*180./M_PI);
            #endif
            if (fabs(alpha) > M_PI/9) {
                double rot = ((alpha>0)?+1:-1)*cfg.max_angular_velocity;
            #ifdef DEBUG_GOTO
                    printf("Cmd v %.2f r %.2f\n",0.,rot);
            #endif
                env->publishVelocity(0,rot);
            } else {
                double vel = cfg.k_v * r;
                double rot = std::max(std::min(cfg.k_alpha*alpha,cfg.max_angular_velocity),-cfg.max_angular_velocity);
                if (vel > cfg.max_velocity) vel = cfg.max_velocity;
            #ifdef DEBUG_GOTO
                printf("Cmd v %.2f r %.2f\n",vel,rot);
            #endif
                env->publishVelocity(vel, rot);
            }
            return TaskStatus::TASK_RUNNING;
        }
    }else{
        if(cfg.ctrlmtd){
            // double r = hypot(y_init + cfg.goal_y-tpose.y,x_init + cfg.goal_x-tpose.x);
            // double alpha = remainder(atan2(y_init + cfg.goal_y-tpose.y,x_init + cfg.goal_x-tpose.x),2*M_PI);
            // if (r < cfg.dist_threshold) {
            //     return TaskStatus::TASK_COMPLETED;
            // }
            // double vel_x = cfg.k_v*r*cos(alpha);
            // double vel_y = cfg.k_v*r*sin(alpha);
            // if (vel_x > cfg.max_velocity) vel_x = cfg.max_velocity;
            // if (vel_y > cfg.max_velocity) vel_y = cfg.max_velocity;
            // env->publishVelocity(vel_x, vel_y, 1);
            // return TaskStatus::TASK_RUNNING;

            double dx=x_init + cfg.goal_x-tpose.x;
            double dy=y_init + cfg.goal_y-tpose.y;
            double r = hypot(y_init + cfg.goal_y-tpose.y,x_init + cfg.goal_x-tpose.x);
            double alpha = remainder(cfg.goal_theta-tpose.theta+theta_init,2*M_PI);
            // double beta =remainder((-alpha -(tpose.theta+cfg.goal_theta-theta_init)), 2*M_PI);    
            
            //ROS_INFO("beta: %.2lf, alpha: %.2lf, theta: %.2f", beta*180./M_PI, alpha*180./M_PI, cfg.goal_theta*180./M_PI);

            #ifdef DEBUG_GOTOPOSE
                printf("c %.1f %.1f %.1f g %.1f %.1f r %.3f alpha %.1f\n",
                        tpose.x, tpose.y, tpose.theta*180./M_PI,
                        cfg.goal_x,cfg.goal_y,r,alpha*180./M_PI);
            #endif
            if (fabs(alpha) < cfg.angle_threshold && r < cfg.dist_threshold) {
                return TaskStatus::TASK_COMPLETED;
            }
            double vel_x = cfg.k_v*(cos(tpose.theta)*dx+sin(tpose.theta)*dy);
            double vel_y = cfg.k_v*(-sin(tpose.theta)*dx+cos(tpose.theta)*dy);
            double vnorm = hypot(vel_x,vel_y);
            if (vnorm > cfg.max_velocity) {
                double scale = cfg.max_velocity/vnorm;
                vel_x *= scale;
                vel_y *= scale;
            }
            double rot = std::max(std::min(cfg.k_goal*alpha,cfg.max_angular_velocity),-cfg.max_angular_velocity);
            env->publishVelocity(vel_x, vel_y, rot);
            return TaskStatus::TASK_RUNNING;

        }else{
            double dx=x_init + cfg.goal_x-tpose.x;
            double dy=y_init + cfg.goal_y-tpose.y;
            double r = hypot(y_init + cfg.goal_y-tpose.y,x_init + cfg.goal_x-tpose.x);
            if (r < cfg.dist_threshold) {
                return TaskStatus::TASK_COMPLETED;
            }
            double vel_x = cfg.k_v*(cos(tpose.theta)*dx+sin(tpose.theta)*dy);
            double vel_y = cfg.k_v*(-sin(tpose.theta)*dx+cos(tpose.theta)*dy);
            double vnorm = hypot(vel_x,vel_y);
            if (vnorm > cfg.max_velocity) {
                double scale = cfg.max_velocity/vnorm;
                vel_x *= scale;
                vel_y *= scale;
            }
            printf("VX %f VY %f W %f\n",vel_x,vel_y,0.0);
            env->publishVelocity(vel_x, vel_y, 0);
            return TaskStatus::TASK_RUNNING;

        }
        



    }
        
    
}

TaskIndicator TaskGoToPose::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryGoToPose);