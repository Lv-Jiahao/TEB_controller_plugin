
/*
关键改变是：

分离数据和顶点：timediff_vec_ 存储数值（double），timediff_vertices_ 存储顶点指针
每次迭代重建顶点：在 AddVertices() 中根据 timediff_vec_ 的值重新创建顶点
避免悬空指针：clearGraph() 后，旧顶点被销毁，但我们有数值备份可以重建
*/
#include "planner_manager.h"
 

namespace teb_local_planner
{

    void plannerManager::runOptimization()
    {
        std::cout << "===== TEB Style Optimization Start =====" << std::endl;
        std::cout << "\n----- 开始优化 -----" << std::endl;

        // 1. 清理所有旧顶点，防止内存泄漏/野指针
        for (auto v : pose_vertices_) delete v;
        pose_vertices_.clear();
        for (auto v : timediff_vertices_) delete v;
        timediff_vertices_.clear();

        // 2. 重建全新graph，彻底清空上一轮内容
        optimizer_ = initOptimizer();
        vertexId_ = 0;

        // 3. 重建顶点、边
        AddVertices();          // 添加顶点
        AddObstacleEdges();     // 添加障碍物约束边
        AddViaPointEdges();     // 添加路径跟随约束边
        AddVelocityEdgs();      // 添加速度约束边
        AddEdgeKinematics();    // 添加运动学约束边
        AddJerkEdges();         // 添加Jerk约束边
        if (!optimizeGraph())
            std::cerr << "优化失败！" << std::endl;

        std::cout << "\n===== TEB Style Optimization End =====" << std::endl;
    }

    boost::shared_ptr<g2o::SparseOptimizer> plannerManager::initOptimizer()
    {
        // 线程安全注册自定义类型
        static boost::once_flag flag = BOOST_ONCE_INIT;
        boost::call_once(&registerG2OTypes, flag);

        // 配置求解器（二维顶点，动态残差维度）
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

        // 创建求解器
        auto linear_solver = std::make_unique<LinearSolverType>();
        auto block_solver = std::make_unique<BlockSolverType>(std::move(linear_solver));
        auto solver = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

        // 初始化优化器
        auto optimizer = boost::make_shared<g2o::SparseOptimizer>();
        optimizer->setAlgorithm(solver);
        optimizer->setVerbose(cfg_.optimization_verbose);  // 从配置读取verbose

        return optimizer;
    }



    // 注册g2o类型（顶点+边）
    void plannerManager::registerG2OTypes()
    {
        g2o::Factory* factory = g2o::Factory::instance();
        factory->registerType("VERTEX_POINT2D", new g2o::HyperGraphElementCreator<VertexPoint2D>);
        factory->registerType("EDGE_obstacle_CONSTRAINT", new g2o::HyperGraphElementCreator<EdgeObstacleConstraint>);
        factory->registerType("EDGE_via_point_CONSTRAINT", new g2o::HyperGraphElementCreator<EdgeViaPointConstraint>);
        factory->registerType("EDGE_velocity_CONSTRAINT", new g2o::HyperGraphElementCreator<EdgeVelocityConstraint>);
        factory->registerType("EDGE_jerk_CONSTRAINT", new g2o::HyperGraphElementCreator<EdgeJerkConstraint>);  // 新增

    }

    // 这里实际 传入的是 所有路径信息点 (x,y,theta)
    // void plannerManager::setpathInfo(std::vector<tools::pathInfo>& path)
    // {
    //     pathPointArr_ = path;
    //     vertexId_ = 0;
    //     std::cout << "timedff real s" << std::endl;
    //     for(int i = 1; i < path.size();i++)
    //     {
    //         double dist = fabs(tools::distanceBetweenTwoPoint(path[i],path[i-1]));
    //         // 两点直接 距离/时间
    //         double dt   = dist / cfg_.max_vel;
    //         double dt2  = 0;
    //         if(i <  path.size() - 1)
    //         {
    //             // double theta1 = atan2(path[i].y-path[i-1].y,path[i].x-path[i-1].x);
    //             // double theta2 = atan2(path[i+1].y-path[i].y,path[i+1].x-path[i].x);
    //             dt2   = fabs(tools::normalize_theta(path[i].theta - path[i-1].theta))/cfg_.max_vel_theta;
    //         }
    //         dt = std::max(dt,dt2);
    //         // timediff_vec_.push_back(dt);
    //         vertexTimeDiff* v = new vertexTimeDiff();
    //         v->setId(vertexId_++);
    //         v->setFixed(false);
    //         v->setEstimate(dt);
    //         std::cout << "cfg_.max_vel =" << cfg_.max_vel << "cfg_.max_vel_theta =" << cfg_.max_vel_theta<<std::endl;
    //         std::cout << "dt =" << dt << std::endl;
    //         timediff_vec_.push_back(v);
    //         optimizer_->addVertex(v);
    //     }
    //     std::cout << "timedff real end" << std::endl;

    // }
    void plannerManager::setpathInfo(std::vector<tools::pathInfo>& path)
    {
        pathPointArr_ = path;
        vertexId_ = 0;
        
        // 清空旧的时间差数据
        timediff_vec_.clear();
        
        std::cout << "timediff real start" << std::endl;
        for(int i = 1; i < path.size(); i++)
        {
            double dist = fabs(tools::distanceBetweenTwoPoint(path[i], path[i-1]));
            double dt = dist / cfg_.max_vel;
            double dt2 = 0;
        
            if(i < path.size() - 1)
            {
                dt2 = fabs(tools::normalize_theta(path[i].theta - path[i-1].theta)) / cfg_.max_vel_theta;
            }
            // 机器人在移动/转弯 时两者所需的最大时间
            dt = std::max(dt, dt2);
            
            // 存储 dt 值，而不是顶点指针
            timediff_vec_.push_back(dt);  // 改变：存储 double 值
            
            std::cout << "cfg_.max_vel =" << cfg_.max_vel 
                    << " cfg_.max_vel_theta =" << cfg_.max_vel_theta << std::endl;
            std::cout << "dt =" << dt << std::endl;
        }
        std::cout << "timediff real end" << std::endl;
    }

    void plannerManager::setObstacleInfo(std::vector<tools::obstacleInfo>& obs)
    {
        obstaclePointInfo_ = obs;
    }

    void plannerManager::getPlannerResults(std::vector<tools::pathInfo>& path)
    {
        for(int i = 0; i < pose_vertices_.size(); i++)
        {
            Eigen::Vector3d v = pose_vertices_[i]->estimate();
            std::cout << "x = " << std::fixed << std::setprecision(4) 
                    << v.x() << ", " << v.y() << ", " << v[2] << std::endl;
            
            if(i < timediff_vertices_.size())
            {
                std::cout << "dt = " << timediff_vertices_[i]->estimate() << std::endl;
            }
            
            tools::pathInfo temp;
            temp.x = v.x();
            temp.y = v.y();
            temp.theta = v[2];
            path.push_back(temp);
        }
    }


    void plannerManager::AddVertices()
    {
        // 清空旧顶点容器
        pose_vertices_.clear();
        timediff_vertices_.clear();
        
        std::cout << "开始添加顶点，当前 vertexId_ = " << vertexId_ << std::endl;
        
        // 添加位姿顶点
        for(size_t i = 0; i < pathPointArr_.size(); i++)
        {
            // 这里应该使用 VertexSE2()
            VertexPoint2D* v = new VertexPoint2D();  //这里使用的是 g2o的 数据类型(x,y)
            v->setId(vertexId_++);
            
            // 第一个和最后一个点固定
            if (i == 0 || i == pathPointArr_.size() - 1)
            {
                v->setFixed(true);
            }
            else
            {
                v->setFixed(false);
            }
            // 这里传入的是3d，最后的 theta 应该是舍弃了
            v->setEstimate(Eigen::Vector3d(pathPointArr_[i].x, pathPointArr_[i].y, pathPointArr_[i].theta));
            pose_vertices_.push_back(v);
            
            if (!optimizer_->addVertex(v))
            {
                std::cerr << "添加位姿顶点 " << v->id() << " 失败！" << std::endl;
                delete v;
                pose_vertices_.pop_back();
            }
        }
        
        // 添加时间差顶点
        for(size_t i = 0; i < timediff_vec_.size(); i++)
        {
            vertexTimeDiff* v = new vertexTimeDiff();
            v->setId(vertexId_++);
            v->setFixed(false);
            v->setEstimate(timediff_vec_[i]);
            timediff_vertices_.push_back(v);
            
            if (!optimizer_->addVertex(v))
            {
                std::cerr << "添加时间差顶点 " << v->id() << " 失败！" << std::endl;
                delete v;
                timediff_vertices_.pop_back();
            }
        }
        
        std::cout << "顶点添加完成：pose=" << pose_vertices_.size() 
                << ", timediff=" << timediff_vertices_.size() 
                << ", 下一个ID=" << vertexId_ << std::endl;
    }
 
    void plannerManager::AddEdgeKinematics()
    {
        Eigen::Matrix<double,2,2> information_kinematics;
        information_kinematics.fill(0.0);
        information_kinematics(0, 0) = cfg_.weight_kinematics_nh;
        information_kinematics(1, 1) = cfg_.weight_kinematics_forward_drive;
        
        for(size_t i = 0; i + 1 < pathPointArr_.size(); i++)
        {
            EdgeKineticConstraint* edge_kinetic = new EdgeKineticConstraint();
            // edge_kinetic->setId(edgeId_++);  // ← 使用全局 edgeId_
            edge_kinetic->setVertex(0, pose_vertices_[i]);
            edge_kinetic->setVertex(1, pose_vertices_[i+1]);
            edge_kinetic->setInformation(information_kinematics);
            edge_kinetic->setcfg(&cfg_);
            optimizer_->addEdge(edge_kinetic);
        }
    }


    void plannerManager::AddVelocityEdgs()
    {
                
        Eigen::Matrix<double,2,2> information;
        information(0,0) = cfg_.weight_max_vel_x;  // 限制正向移动的能力，针对机器人前进方向的具体约束。
        information(1,1) = cfg_.weight_max_vel_theta;
        information(0,1) = 0.0;
        information(1,0) = 0.0;
        
        std::cout << "AddVelocityEdgs: pose=" << pose_vertices_.size()
                << ", timediff=" << timediff_vertices_.size() << std::endl;
        
        for (size_t i = 0; i + 1 < pose_vertices_.size(); ++i)
        {
            if (i >= timediff_vertices_.size())
            {
                std::cerr << "[错误] timediff 索引越界: i=" << i 
                        << ", size=" << timediff_vertices_.size() << std::endl;
                break;
            }
            
            // 检查顶点是否在优化器中
            if (!optimizer_->vertex(pose_vertices_[i]->id()) ||
                !optimizer_->vertex(pose_vertices_[i+1]->id()) ||
                !optimizer_->vertex(timediff_vertices_[i]->id()))
            {
                std::cerr << "[错误] 顶点不在优化器中" << std::endl;
                continue;
            }
            
            EdgeVelocityConstraint* edge_velocity = new EdgeVelocityConstraint();
            edge_velocity->setVertex(0, pose_vertices_[i]);
            edge_velocity->setVertex(1, pose_vertices_[i+1]);
            edge_velocity->setVertex(2, timediff_vertices_[i]);
            edge_velocity->setInformation(information);
            edge_velocity->setcfg(&cfg_);
            
            // 检查边的有效性
            if (!edge_velocity->allVerticesFixed())  // 如果 $P_i$、$P_{i+1}$ 和 $\Delta T_i$ 全部被设为 Fixed（固定不动）
            {
                if (!optimizer_->addEdge(edge_velocity))
                {
                    std::cerr << "[错误] 添加速度约束边失败 i=" << i << std::endl;
                    delete edge_velocity;
                }
                else
                {
                    std::cout << "[成功] 添加速度约束边 " << i << std::endl;
                }
            }
            else
            {
                std::cout << "[跳过] 所有顶点都固定的边 " << i << std::endl;
                delete edge_velocity;
            }
        }
    }

    // 添加边
    void plannerManager::AddObstacleEdges()
    {
        std::cout << "\n=== AddObstacleEdges 开始 ===" << std::endl;
        std::cout << "位姿顶点数量: " << pose_vertices_.size() << std::endl;
        std::cout << "障碍物数量: " << obstaclePointInfo_.size() << std::endl;
        std::cout << "障碍物权重: " << cfg_.obstacle_weight << std::endl;
        std::cout << "最小距离: " << cfg_.min_obstacle_dist << std::endl;
        
        if (obstaclePointInfo_.empty())
        {
            std::cout << "⚠️  警告: 没有障碍物信息,跳过障碍物约束!" << std::endl;
            return;
        }
        
        // 正确设置information矩阵
        Eigen::Matrix<double,1,1> information;
        information.fill(cfg_.obstacle_weight);  // 使用配置的权重
        
        int edge_count = 0;
        for (size_t i = 0; i < pose_vertices_.size(); ++i)
        {
            for(size_t j = 0; j < obstaclePointInfo_.size(); j++)
            {
                // 这里被 g2o 封装了，会产生 pose_vertices_.size()*obstaclePointInfo_.size() 条边，每条边都产生一个cost值，
                // 在config里会有 min_obstacle_dist 值，
                EdgeObstacleConstraint* e_soft = new EdgeObstacleConstraint();
                // 告诉这个边：“你应该去监视第 i 个顶点的坐标变化”
                e_soft->setVertex(0, pose_vertices_[i]);
                e_soft->setTebConfig(cfg_);
                e_soft->setObstcele(obstaclePointInfo_[j], &cfg_);
                e_soft->setInformation(information);  // 权重矩阵
                
                // 把边正式注册到 g2o 的优化池子里
                if (!optimizer_->addEdge(e_soft))
                {
                    std::cerr << "[错误] 添加障碍物约束边失败 pose=" << i 
                            << " obs=" << j << std::endl;
                    delete e_soft;
                }
                else
                {
                    edge_count++;
                }
            }
        }
        
        std::cout << "[成功] 添加了 " << edge_count << " 条障碍物约束边" << std::endl;
        std::cout << "预期数量: " << (pose_vertices_.size() * obstaclePointInfo_.size()) << std::endl;
    }

    int plannerManager::findClosestTrajectoryPose(tools::pathInfo &ref_point,int& idx)
    {
        int n = pose_vertices_.size();
        if (idx < 0 || idx >= n)
            return -1;

        double min_dist_sq = std::numeric_limits<double>::max();
        int min_idx = -1;

        for (int i = idx; i < n; i++)
        {
            VertexPoint2D* v        =  static_cast<VertexPoint2D*>(pose_vertices_[i]);
            Eigen::Vector3d point   =  v->estimate();

            double dist_sq = sqrt(pow(ref_point.x - point[0],2) + pow(ref_point.y - point[1],2));
            if (dist_sq < min_dist_sq)
            {
                min_dist_sq = dist_sq;
                min_idx = i;
            }
        }

        idx = min_idx;

        return min_idx;
    }

    void plannerManager::AddViaPointEdges()
    {
        int start_index = 0;
        // 相当于，参考轨迹有 n个点，待优化也有n个，要计算之间的 距离error，n条边
        for(size_t i = 0; i < pathPointArr_.size(); i++)
        {
            int index = findClosestTrajectoryPose(pathPointArr_[i], start_index);
            
            Eigen::Matrix<double,1,1> information;
            information.fill(cfg_.weight_viapoint);
            
            EdgeViaPointConstraint* edge_viapoint = new EdgeViaPointConstraint;
            
            edge_viapoint->setVertex(0, pose_vertices_[index]);
            edge_viapoint->setInformation(information);
            edge_viapoint->setPathPoint(pathPointArr_[i], &cfg_);
            optimizer_->addEdge(edge_viapoint);
        }
    }

    // 执行优化
    bool plannerManager::optimizeGraph()
    {
        if (pose_vertices_.empty())
        {
            std::cerr << "Error: No vertices to optimize!" << std::endl;
            return false;
        }
        optimizer_->initializeOptimization();

        double chi2_before = optimizer_->chi2();
        int actual_iter = optimizer_->optimize(cfg_.no_inner_iterations);  // 从配置读取最大迭代次数
        double chi2_after = optimizer_->chi2();
        std::cout << "优化前chi2: " << chi2_before << ", 优化后chi2: " << chi2_after << std::endl;

        if (actual_iter == 0)
        {
            std::cerr << "Optimization failed: No iterations executed!" << std::endl;
            return false;
        }
        return true;

    }

    // 打印优化结果
    void plannerManager::printResult()
    {

        std::cout << "\n===== Optimization Result =====" << std::endl;
        for(int i = 0; i < pose_vertices_.size();i++)
        {
            Eigen::Vector3d v = pose_vertices_[i]->estimate();
            std::cout << "x = " << std::fixed << std::setprecision(4) << v.x()  << ", " << v.y() << ")" << std::endl;

        }
        // Eigen::Vector2d v1_res = pose_vertices_[0]->estimate();
        // Eigen::Vector2d v2_res = pose_vertices_[1]->estimate();
        // double final_dist = (v1_res - v2_res).norm();

        // std::cout << "顶点1坐标：(" << std::fixed << std::setprecision(4) << v1_res.x() 
        //           << ", " << v1_res.y() << ")" << std::endl;
        // std::cout << "顶点2坐标：(" << std::fixed << std::setprecision(4) << v2_res.x() 
        //           << ", " << v2_res.y() << ")" << std::endl;
        // std::cout << "实际距离：" << std::fixed << std::setprecision(4) << final_dist << std::endl;
        // std::cout << "期望距离：" << std::fixed << std::setprecision(4) << cfg_.distance_constraint_exp_dist << std::endl;

    }

    void plannerManager::AddJerkEdges()
    {
        std::cout << "=== AddJerkEdges 开始 ===" << std::endl;
        
        Eigen::Matrix<double,2,2> information;
        information.setZero();
        information(0,0) = cfg_.weight_jerk;        // 线性jerk权重
        information(1,1) = cfg_.weight_jerk;        // 角度jerk权重
        
        int edge_count = 0;
        
        // 遍历所有可以构成jerk约束的顶点组合
        // 需要5个连续位姿 + 4个连续时间差
        for (size_t i = 0; i + 4 < pose_vertices_.size(); ++i)
        {
            // 检查时间差顶点索引是否有效
            if (i + 3 >= timediff_vertices_.size())
            {
                std::cerr << "[警告] timediff索引越界: i=" << i << std::endl;
                break;
            }
            
            // 创建jerk约束边
            EdgeJerkConstraint* edge_jerk = new EdgeJerkConstraint();
            
            // 连接5个位姿顶点 内存中建立指针级联 edge_jerk 与下述9个顶点相关联
            edge_jerk->setVertex(0, pose_vertices_[i]);
            edge_jerk->setVertex(1, pose_vertices_[i+1]);
            edge_jerk->setVertex(2, pose_vertices_[i+2]);
            edge_jerk->setVertex(3, pose_vertices_[i+3]);
            edge_jerk->setVertex(4, pose_vertices_[i+4]);
            
            // 连接4个时间差顶点
            edge_jerk->setVertex(5, timediff_vertices_[i]);
            edge_jerk->setVertex(6, timediff_vertices_[i+1]);
            edge_jerk->setVertex(7, timediff_vertices_[i+2]);
            edge_jerk->setVertex(8, timediff_vertices_[i+3]);
            
            edge_jerk->setInformation(information);
            edge_jerk->setjerkcfg(&cfg_);
            
            if (!optimizer_->addEdge(edge_jerk))
            {
                std::cerr << "[错误] 添加jerk约束边失败 i=" << i << std::endl;
                delete edge_jerk;
            }
            else
            {
                edge_count++;
            }
        }
        
        std::cout << "[成功] 添加了 " << edge_count << " 条jerk约束边" << std::endl;
    }

}
