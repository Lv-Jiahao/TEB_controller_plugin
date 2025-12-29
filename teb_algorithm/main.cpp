
/*
 * @Function:using Trajectory Optimize Method 
 * @Create by:juchunyu@qq.com
 * @Date:2025-09-13 16:10:01
 */
#include "planner_manager.h"
#include "matplotlib-cpp/matplotlibcpp.h"

namespace plt = matplotlibcpp;//可视化

int main()
{
    std::vector<double> glob_x;
    std::vector<double> glob_y;

    std::vector<double> plan_x;
    std::vector<double> plan_y;

    std::vector<double> obs_x;
    std::vector<double> obs_y;

    std::vector<double> color;
    // 1. 配置初始化
    teb_local_planner::TebConfig cfg;
    cfg.no_inner_iterations = 10; // 5;
    cfg.no_outer_iterations = 4;
    cfg.min_obstacle_dist = 0.8; //2.0;
    cfg.penalty_epsilon = 0.05;
    cfg.obstacle_weight = 200;                    
    cfg.optimization_verbose = true;
    cfg.weight_viapoint  = 1;
    cfg.max_vel           = 1.0;
    cfg.max_vel_theta     = 1.0;
    cfg.max_vel_x_backwards = 0.3;
    cfg.weight_max_vel_x    = 10;
    cfg.weight_max_vel_theta = 10;
    cfg.weight_kinematics_nh  = 10;  // 1000;
    cfg.weight_kinematics_forward_drive = 1;

    // 添加jerk权重
    cfg.weight_jerk = 5.0;                     
    cfg.max_jerk = 2.0;
    cfg.max_jerk_theta = 2.0;


    // 2. 创建规划器并运行优化
    // teb_local_planner::plannerManager planner(cfg);
    std::shared_ptr<teb_local_planner::plannerManager> planner = std::make_shared<teb_local_planner::plannerManager>(cfg);

    std::vector<tools::pathInfo> gloablPlan;   //(x,y,theat)
    std::vector<tools::obstacleInfo> obstacles;   //(x,y,theat)
    std::cout << "globa_plan_start" << std::endl;
    for(float i = 1;i < 10; i = i+0.1)
    {
        tools::pathInfo temp;  // (x,y,theat)
        temp.x = i;
        temp.y = i;
        temp.theta = 3.1415926/4;
         
        //gloablPlan 全局路径信息
        gloablPlan.push_back(temp);
        std::cout << "(" << temp.x  << ", " << temp.y << ")" << std::endl;
        glob_x.push_back(temp.x);  // 全是
        glob_y.push_back(temp.y);
    }
    std::cout << "globa_plan_end" << std::endl;


    // add obstacles 1
    tools::obstacleInfo obstemp;
    obstemp.x = 3.0;
    obstemp.y = 3.5;
    obs_x.push_back(obstemp.x);
    obs_y.push_back(obstemp.y);
    color.push_back(1.0);
    obstacles.push_back(obstemp);

    // add obstacles 2
    obstemp.x = 7.0;
    obstemp.y = 7.5;
    obs_x.push_back(obstemp.x);
    obs_y.push_back(obstemp.y);
    color.push_back(1.0);
    obstacles.push_back(obstemp);
   
    planner->setpathInfo(gloablPlan);
    planner->setObstacleInfo(obstacles);

    planner->runOptimization();

    std::vector<tools::pathInfo> planResult;
    
    // 3. 获取规划结果  这里实际还是(x,y,theta) 传了个引用进去
    planner->getPlannerResults(planResult);

    // 4. 可视化
    for(int i = 0;i < planResult.size();i++)
    {
        plan_x.push_back(planResult[i].x);
        plan_y.push_back(planResult[i].y);
    }

    std::map<std::string, std::string> keywords1;
    keywords1.insert(std::pair<std::string, std::string>("label", "ref_traj") );
    keywords1.insert(std::pair<std::string, std::string>("linewidth", "3.5") );
    plt::plot(glob_x,glob_y,keywords1);


    std::map<std::string, std::string> keywords4;
    keywords4.insert(std::pair<std::string, std::string>("label", "planTraj") );
    keywords4.insert(std::pair<std::string, std::string>("linewidth", "3.5") );
    plt::plot(plan_x,plan_y,keywords4);

    double point_size = 100.0;

    std::cout<<"原始点数："<<gloablPlan.size()<<std::endl;
    std::cout<<"优化后点数："<<planResult.size()<<std::endl;

    plt::scatter_colored(obs_x, obs_y,color,point_size,{{"cmap", "viridis"}});

    plt::legend();
    plt::title("Trajectory Optimize Method ");
    plt::show();

    return 0;
}