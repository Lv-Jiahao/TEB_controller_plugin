// planner_manager.cpp - 保留 Jerk 约束的完整版
#include "planner_manager.h"

namespace teb_local_planner
{

void plannerManager::runOptimization()
{
    std::cout << "===== TEB Style Optimization Start =====" << std::endl;

    if (optimizer_ && optimizer_->vertices().size() > 0)
    {
        optimizer_->edges().clear();
        optimizer_->clearParameters();
    }
    
    pose_vertices_.clear();
    timediff_vertices_.clear();
    vertexId_ = 0;
    
    if (!optimizer_)
    {
        optimizer_ = initOptimizer();
        if (!optimizer_)
        {
            std::cerr << "创建optimizer失败!" << std::endl;
            return;
        }
    }
    else
    {
        optimizer_->clear();
    }
    
    AddVertices();
    AddGoalPoseEdge();
    AddObstacleEdges();
    AddViaPointEdges();
    AddVelocityEdgs();
    AddEdgeKinematics();
    AddJerkEdges();    
    
    if (!optimizeGraph())
        std::cerr << "优化失败!" << std::endl;

    std::cout << "===== TEB Style Optimization End =====" << std::endl;
}

boost::shared_ptr<g2o::SparseOptimizer> plannerManager::initOptimizer()
{
    static boost::once_flag flag = BOOST_ONCE_INIT;
    boost::call_once(&registerG2OTypes, flag);

    try {
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

        auto linear_solver = std::make_unique<LinearSolverType>();
        auto block_solver = std::make_unique<BlockSolverType>(std::move(linear_solver));
        auto solver = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

        auto optimizer = boost::make_shared<g2o::SparseOptimizer>();
        optimizer->setAlgorithm(solver);
        optimizer->setVerbose(cfg_.optimization_verbose);

        std::cout << "Optimizer 创建成功" << std::endl;
        return optimizer;
    }
    catch (const std::exception& e)
    {
        std::cerr << "创建optimizer异常: " << e.what() << std::endl;
        return nullptr;
    }
}

void plannerManager::registerG2OTypes()
{
    g2o::Factory* factory = g2o::Factory::instance();
    
    if (!factory->knowsTag("VERTEX_POINT2D"))
    {
        factory->registerType("VERTEX_POINT2D", new g2o::HyperGraphElementCreator<VertexPoint2D>);
        factory->registerType("EDGE_obstacle_CONSTRAINT", new g2o::HyperGraphElementCreator<EdgeObstacleConstraint>);
        factory->registerType("EDGE_via_point_CONSTRAINT", new g2o::HyperGraphElementCreator<EdgeViaPointConstraint>);
        factory->registerType("EDGE_velocity_CONSTRAINT", new g2o::HyperGraphElementCreator<EdgeVelocityConstraint>);
        factory->registerType("EDGE_kinetic_pose_CONSTRAINT", new g2o::HyperGraphElementCreator<EdgeKineticConstraint>);  
        factory->registerType("EDGE_jerk_CONSTRAINT", new g2o::HyperGraphElementCreator<EdgeJerkConstraint>);
        factory->registerType("EDGE_goal_pose_CONSTRAINT", new g2o::HyperGraphElementCreator<EdgeGoalPoseConstraint>);  
        std::cout << "G2O类型注册完成" << std::endl;
    }
}

void plannerManager::setpathInfo(std::vector<tools::pathInfo>& path)
{
    pathPointArr_ = path;
    timediff_vec_.clear();
    
    for(size_t i = 1; i < path.size(); i++)
    {
        double dx = path[i].x - path[i-1].x;
        double dy = path[i].y - path[i-1].y;
        double dist = sqrt(dx*dx + dy*dy);
        
        double dtheta = tools::normalize_theta(path[i].theta - path[i-1].theta);
        
        double dt_linear = dist / cfg_.max_vel;
        double dt_angular = fabs(dtheta) / cfg_.max_vel_theta;
        double dt = std::max(dt_linear, dt_angular);
        
        dt = std::max(dt, 0.1);
        dt = std::min(dt, 2.0);
        
        timediff_vec_.push_back(dt);
    }
}

void plannerManager::setObstacleInfo(std::vector<tools::obstacleInfo>& obs)
{
    obstaclePointInfo_ = obs;
}

void plannerManager::getPlannerResults(std::vector<tools::pathInfo>& path)
{
    path.clear();
    
    for(size_t i = 0; i < pose_vertices_.size(); i++)
    {
        if (!pose_vertices_[i]) continue;
        
        Eigen::Vector3d v = pose_vertices_[i]->estimate();
        
        tools::pathInfo temp;
        temp.x = v.x();
        temp.y = v.y();
        temp.theta = v[2];
        path.push_back(temp);
    }
}

void plannerManager::AddVertices()
{
    if (!optimizer_)
    {
        std::cerr << "optimizer_ 为空!" << std::endl;
        return;
    }
    
    pose_vertices_.reserve(pathPointArr_.size());
    timediff_vertices_.reserve(timediff_vec_.size());
    
    // 添加位姿顶点
    for(size_t i = 0; i < pathPointArr_.size(); i++)
    {
        VertexPoint2D* v = new VertexPoint2D();
        v->setId(vertexId_++);
        
        // 固定起点和终点
        // if (i == 0 || i == pathPointArr_.size() - 1)
        if (i == 0)
            v->setFixed(true);
        else
            v->setFixed(false);
            
        v->setEstimate(Eigen::Vector3d(pathPointArr_[i].x, 
                                       pathPointArr_[i].y, 
                                       pathPointArr_[i].theta));
        
        if (!optimizer_->addVertex(v))
        {
            std::cerr << "添加位姿顶点 " << v->id() << " 失败!" << std::endl;
            delete v;
            continue;
        }
        
        pose_vertices_.push_back(v);
    }
    
    // 添加时间差顶点
    for(size_t i = 0; i < timediff_vec_.size(); i++)
    {
        vertexTimeDiff* v = new vertexTimeDiff();
        v->setId(vertexId_++);
        v->setFixed(false);
        
        double dt = timediff_vec_[i];
        v->setEstimate(dt);
        
        if (!optimizer_->addVertex(v))
        {
            std::cerr << "添加时间差顶点 " << v->id() << " 失败!" << std::endl;
            delete v;
            continue;
        }
        
        timediff_vertices_.push_back(v);
    }
    
    std::cout << "顶点添加完成: pose=" << pose_vertices_.size() 
              << ", timediff=" << timediff_vertices_.size() << std::endl;
}


void plannerManager::AddEdgeKinematics()
{
    if (pose_vertices_.size() < 2) return;
    
    Eigen::Matrix<double,2,2> information_kinematics;
    information_kinematics.fill(0.0);
    information_kinematics(0, 0) = cfg_.weight_kinematics_nh;
    information_kinematics(1, 1) = cfg_.weight_kinematics_forward_drive;
    
    for(size_t i = 0; i + 1 < pose_vertices_.size(); i++)
    {
        EdgeKineticConstraint* edge_kinetic = new EdgeKineticConstraint();
        edge_kinetic->setVertex(0, pose_vertices_[i]);
        edge_kinetic->setVertex(1, pose_vertices_[i+1]);
        edge_kinetic->setInformation(information_kinematics);
        edge_kinetic->setcfg(&cfg_);
        
        if (!optimizer_->addEdge(edge_kinetic))
        {
            delete edge_kinetic;
        }
    }
}

void plannerManager::AddVelocityEdgs()
{
    if (pose_vertices_.size() < 2 || timediff_vertices_.empty()) return;
    
    Eigen::Matrix<double,2,2> information;
    information(0,0) = cfg_.weight_max_vel_x;
    information(1,1) = cfg_.weight_max_vel_theta;
    information(0,1) = 0.0;
    information(1,0) = 0.0;
    int velocity_count = 0;
    for (size_t i = 0; i + 1 < pose_vertices_.size(); ++i)
    {
        if (i >= timediff_vertices_.size()) break;
        
        EdgeVelocityConstraint* edge_velocity = new EdgeVelocityConstraint();
        edge_velocity->setVertex(0, pose_vertices_[i]);
        edge_velocity->setVertex(1, pose_vertices_[i+1]);
        edge_velocity->setVertex(2, timediff_vertices_[i]);
        edge_velocity->setInformation(information);
        edge_velocity->setcfg(&cfg_);
        
        if (!edge_velocity->allVerticesFixed())
        {
            if (!optimizer_->addEdge(edge_velocity))
            {
                delete edge_velocity;
            }else{
                velocity_count++;

            }
        }
        else
        {
            delete edge_velocity;
        }
    }
    std::cout << "[成功] 添加了 " << velocity_count << " 条 velocity 约束边" << std::endl;
}

void plannerManager::AddObstacleEdges()
{
    if (obstaclePointInfo_.empty() || pose_vertices_.empty()) return;
    
    Eigen::Matrix<double,1,1> information;
    information.fill(cfg_.obstacle_weight);
    
    size_t start_idx = 0;
    size_t end_idx = std::min(pose_vertices_.size(), size_t(20));
    
    for (size_t i = start_idx; i < end_idx; ++i)
    {
        for(size_t j = 0; j < obstaclePointInfo_.size(); j++)
        {
            EdgeObstacleConstraint* e_soft = new EdgeObstacleConstraint();
            e_soft->setVertex(0, pose_vertices_[i]);
            e_soft->setTebConfig(cfg_);
            e_soft->setObstcele(obstaclePointInfo_[j], &cfg_);
            e_soft->setInformation(information);
            
            if (!optimizer_->addEdge(e_soft))
            {
                delete e_soft;
            }
        }
    }
}

int plannerManager::findClosestTrajectoryPose(tools::pathInfo &ref_point, int& idx)
{
    int n = pose_vertices_.size();
    if (idx < 0 || idx >= n) return -1;

    double min_dist_sq = std::numeric_limits<double>::max();
    int min_idx = -1;

    for (int i = idx; i < n; i++)
    {
        if (!pose_vertices_[i]) continue;
        
        VertexPoint2D* v = static_cast<VertexPoint2D*>(pose_vertices_[i]);
        Eigen::Vector3d point = v->estimate();

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
    if (pose_vertices_.empty() || pathPointArr_.empty()) return;
    
    Eigen::Matrix<double,1,1> information;
    information.fill(cfg_.weight_viapoint);
    
    int via_count = 0;
    // 对所有点添加约束（不跳跃）
    int start_index = 0;
    for(size_t i = 0; i < pathPointArr_.size(); i++)
    {
        int index = findClosestTrajectoryPose(pathPointArr_[i], start_index);
        if (index < 0) continue;
        
        EdgeViaPointConstraint* edge_viapoint = new EdgeViaPointConstraint;
        edge_viapoint->setVertex(0, pose_vertices_[index]);
        edge_viapoint->setInformation(information);
        edge_viapoint->setPathPoint(pathPointArr_[i], &cfg_);
        
        if (!optimizer_->addEdge(edge_viapoint))
        {
            delete edge_viapoint;
        }else{
            via_count++;
        }
    }
    std::cout << "[成功] 添加了 " << via_count << " 条 Via 约束边" << std::endl;
}

bool plannerManager::optimizeGraph()
{
    if (pose_vertices_.empty())
    {
        std::cerr << "Error: No vertices to optimize!" << std::endl;
        return false;
    }
    
    if (!optimizer_->initializeOptimization())
    {
        std::cerr << "Error: Failed to initialize optimization!" << std::endl;
        return false;
    }

    double chi2_before = optimizer_->chi2();
    int actual_iter = optimizer_->optimize(cfg_.no_inner_iterations);
    double chi2_after = optimizer_->chi2();
    
    std::cout << "优化: chi2 " << chi2_before << " → " << chi2_after 
              << ", 迭代: " << actual_iter << std::endl;

    if (actual_iter == 0)
    {
        std::cerr << "Optimization failed!" << std::endl;
        return false;
    }
    return true;
}

void plannerManager::printResult()
{
    std::cout << "\n===== Optimization Result =====" << std::endl;
    for(size_t i = 0; i < pose_vertices_.size(); i++)
    {
        if (!pose_vertices_[i]) continue;
        
        Eigen::Vector3d v = pose_vertices_[i]->estimate();
        std::cout << "(" << std::fixed << std::setprecision(2) 
                  << v.x() << ", " << v.y() << ")" << std::endl;
    }
}

//  Jerk 约束
void plannerManager::AddJerkEdges()
{
    if (pose_vertices_.size() < 5 || timediff_vertices_.size() < 4)
    {
        std::cout << "路径点不足(<5)，跳过Jerk约束" << std::endl;
        return;
    }
    
    std::cout << "=== AddJerkEdges 开始 ===" << std::endl;
    
    Eigen::Matrix<double,2,2> information;
    information.setZero();
    information(0,0) = cfg_.weight_jerk;
    information(1,1) = cfg_.weight_jerk;
    
    int edge_count = 0;
    size_t max_i = std::min(pose_vertices_.size() - 4, timediff_vertices_.size() - 3);
    
    for (size_t i = 0; i < max_i; ++i)
    {
        // 检查顶点有效性
        bool all_valid = true;
        for (size_t j = 0; j < 5; ++j) {
            if (!pose_vertices_[i+j]) {
                all_valid = false;
                break;
            }
        }
        for (size_t j = 0; j < 4; ++j) {
            if (!timediff_vertices_[i+j]) {
                all_valid = false;
                break;
            }
        }
        
        if (!all_valid) continue;
        
        try {
            EdgeJerkConstraint* edge_jerk = new EdgeJerkConstraint();
            
            // 连接5个位姿顶点
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
                delete edge_jerk;
            }
            else
            {
                edge_count++;
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "[异常] 创建jerk边失败 i=" << i << ": " << e.what() << std::endl;
            break;
        }
    }
    
    std::cout << "[成功] 添加了 " << edge_count << " 条jerk约束边" << std::endl;
}

    void plannerManager::AddGoalPoseEdge()
    {
        if (pose_vertices_.empty() || pathPointArr_.empty())
        {
            std::cout << "无法添加目标点约束: 顶点为空" << std::endl;
            return;
        }
        
        // 创建信息矩阵 (3x3: x, y, theta)
        Eigen::Matrix<double,3,3> information;
        information.setZero();
        information(0,0) = cfg_.weight_goal_position;    // x权重
        information(1,1) = cfg_.weight_goal_position;    // y权重
        information(2,2) = cfg_.weight_goal_orientation; // theta权重
        
        // 获取最后一个顶点(终点)
        VertexPoint2D* last_vertex = pose_vertices_.back();
        
        // 获取目标点
        tools::pathInfo goal = pathPointArr_.back();
        
        // 创建目标点约束边
        EdgeGoalPoseConstraint* edge_goal = new EdgeGoalPoseConstraint();
        edge_goal->setVertex(0, last_vertex);
        edge_goal->setGoalPose(goal, &cfg_);
        edge_goal->setInformation(information);
        
        if (!optimizer_->addEdge(edge_goal))
        {
            std::cerr << "添加目标点约束边失败!" << std::endl;
            delete edge_goal;
        }
        else
        {
            std::cout << "[成功] 添加目标点约束边: 目标(" 
                    << goal.x << ", " << goal.y << ", " << goal.theta << ")" << std::endl;
        }
    }
}  // namespace teb_local_planner