/*
 * IRIS Localization and Mapping (LaMa) for ROS
 *
 * Copyright (c) 2019-today, Eurico Pedrosa, University of Aveiro - Portugal
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Aveiro nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "lama/ros/pf_slam2d_ros.h"
#include "rclcpp_components/register_node_macro.hpp"

lama::PFSlam2DROS::PFSlam2DROS(const rclcpp::NodeOptions &options) :
        Node("pf_slam2d_ros", options), transform_tolerance_(0, 100000000) {
    ros_clock = this->get_clock();

    // Load parameters from the server.
    double tmp;
    this->declare_parameter("global_frame_id", "map");
    global_frame_id_ = this->get_parameter("global_frame_id").as_string();
    this->declare_parameter("odom_frame_id", "odom");
    odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
    this->declare_parameter("base_frame_id", "base_link");
    base_frame_id_ = this->get_parameter("base_frame_id").as_string();
    this->declare_parameter("scan_topic", "/scan");
    scan_topic_ =  this->get_parameter("scan_topic").as_string();
    this->declare_parameter("transform_tolerance", 0.1);
    tmp = this->get_parameter("transform_tolerance").as_double();
    transform_tolerance_ = rclcpp::Duration::from_seconds(tmp);

    Vector2d pos;
    this->declare_parameter("initial_pos_x", 0.0);
    pos[0] = this->get_parameter("initial_pos_x").as_double();
    this->declare_parameter("initial_pos_y", 0.0);
    pos[1] = this->get_parameter("initial_pos_y").as_double();
    this->declare_parameter("initial_pos_a", 0.0);
    tmp = this->get_parameter("initial_pos_a").as_double();
    Pose2D prior(pos, tmp);

    PFSlam2D::Options pf_options;
    this->declare_parameter("srr", 0.1);
    pf_options.srr = this->get_parameter("srr").as_double();
    this->declare_parameter("str", 0.2);
    pf_options.str = this->get_parameter("str").as_double();
    this->declare_parameter("stt", 0.1);
    pf_options.stt = this->get_parameter("stt").as_double();
    this->declare_parameter("srt", 0.2);
    pf_options.srt = this->get_parameter("srt").as_double();
    this->declare_parameter("sigma", 0.05);
    pf_options.meas_sigma = this->get_parameter("sigma").as_double();
    this->declare_parameter("lgain", 3.0);
    pf_options.meas_sigma_gain = this->get_parameter("lgain").as_double();
    this->declare_parameter("d_thresh", 0.5);
    pf_options.trans_thresh = this->get_parameter("d_thresh").as_double();
    this->declare_parameter("a_thresh", 0.25);
    pf_options.rot_thresh = this->get_parameter("a_thresh").as_double();
    this->declare_parameter("l2_max", 0.5);
    pf_options.l2_max = this->get_parameter("l2_max").as_double();
    this->declare_parameter("truncate", 0.0);
    pf_options.truncated_ray = this->get_parameter("truncate").as_double();
    this->declare_parameter("resolution", 0.05);
    pf_options.resolution = this->get_parameter("resolution").as_double();
    this->declare_parameter("strategy", "gn");
    pf_options.strategy = this->get_parameter("strategy").as_string();
    this->declare_parameter("use_compression", false);
    pf_options.use_compression = this->get_parameter("use_compression").as_bool();
    this->declare_parameter("compression_algorithm", "zstd");
    pf_options.calgorithm = this->get_parameter("compression_algorithm").as_string();
    this->declare_parameter("mrange", 16.0);
    max_range_ = this->get_parameter("mrange").as_double();
    this->declare_parameter("threads", -1);
    pf_options.threads = this->get_parameter("threads").as_int();

    int itmp;
    this->declare_parameter("patch_size", 32);
    itmp = this->get_parameter("patch_size").as_int();
    pf_options.patch_size = itmp;
    this->declare_parameter("particles", 30);
    itmp = this->get_parameter("particles").as_int();
    pf_options.particles = itmp;
    this->declare_parameter("cache_size", 100);
    itmp = this->get_parameter("cache_size").as_int();
    pf_options.cache_size = itmp;
    // ros param does not have unsigned int??
    this->declare_parameter("seed", 0.0);
    tmp = this->get_parameter("seed").as_double();
    pf_options.seed = tmp;

    this->declare_parameter("create_summary", false);
    pf_options.create_summary = this->get_parameter("create_summary").as_bool();

    this->declare_parameter("map_publish_period", 5.0);
    tmp = this->get_parameter("map_publish_period").as_double();
    periodic_publish_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::milliseconds(static_cast<int>(tmp * 1000))),
            std::bind(&PFSlam2DROS::publishCallback, this));

    slam2d_ = std::make_shared<PFSlam2D>(pf_options);
    slam2d_->setPrior(prior);

    // Setup TF workers ...
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(),
            this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
    tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Syncronized LaserScan messages with odometry transforms. This ensures that an odometry transformation
    // exists when the handler of a LaserScan message is called.
    auto rmw_qos_profile = rclcpp::QoS(rclcpp::SystemDefaultsQoS()).keep_last(100).get_rmw_qos_profile();
    laser_scan_sub_ = std::make_shared < message_filters::Subscriber < sensor_msgs::msg::LaserScan >> (
            this, scan_topic_, rmw_qos_profile);
    laser_scan_filter_ = std::make_shared < tf2_ros::MessageFilter < sensor_msgs::msg::LaserScan >> (
            *laser_scan_sub_, *tf_buffer_, odom_frame_id_, 100,
                    this->get_node_logging_interface(), this->get_node_clock_interface());
    laser_scan_filter_->registerCallback(std::bind(&PFSlam2DROS::onLaserScan, this, std::placeholders::_1));

    // Setup publishers
    // TODO latch https://github.com/ros2/ros2/issues/464
    //      https://answers.ros.org/question/305795/ros2-latching/
    //      http://docs.ros.org/kinetic/api/roscpp/html/classros_1_1NodeHandle.html#a6b655c04f32c4c967d49799ff9312ac6
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose", rclcpp::QoS(2));
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).transient_local()); // latch=true
    dist_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/distance", rclcpp::QoS(1).transient_local()); // latch=true
    patch_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/patch", rclcpp::QoS(1).transient_local()); // latch=true
    poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/poses", rclcpp::QoS(1).transient_local()); // latch=true
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", rclcpp::QoS(1).transient_local()); // latch=true

    ros_occ_.header.frame_id = global_frame_id_;
    ros_cost_.header.frame_id = global_frame_id_;
    ros_patch_.header.frame_id = global_frame_id_;
    poses_.header.frame_id = global_frame_id_;

    // Setup service
    // https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Cpp-Service-And-Client/#write-the-service-node
    // https://answers.ros.org/question/299126/ros2-error-creating-a-service-server-as-a-member-function/
    service = this->create_service<nav_msgs::srv::GetMap>("/dynamic_map",
                                                          std::bind(&PFSlam2DROS::onGetMap, this, std::placeholders::_1,
                                                                    std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "PF SLAM node up and running");
}

lama::PFSlam2DROS::~PFSlam2DROS() {

}

void lama::PFSlam2DROS::onLaserScan(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan) {
    int laser_index = -1;

    // verify if it is from a known source
    if (frame_to_laser_.find(laser_scan->header.frame_id) == frame_to_laser_.end()) {

        laser_index = (int) frame_to_laser_.size();  // simple ID generator :)
        lasers_update_.push_back(false);            // do not update when a laser is added to the known list.
        frame_to_laser_[laser_scan->header.frame_id] = laser_index;

        // find the origin of the sensor
        geometry_msgs::msg::PoseStamped msg_laser_origin;
        try {
            geometry_msgs::msg::PoseStamped msg_identity = lama_utils::createPoseStamped(
                    tf2::Transform(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, 0)),
                    rclcpp::Time(), laser_scan->header.frame_id);
            tf_buffer_->transform(msg_identity, msg_laser_origin, base_frame_id_);
        } catch (tf2::TransformException &e) {
            RCLCPP_ERROR(this->get_logger(), "Could not find origin of %s", laser_scan->header.frame_id.c_str());
            return;
        }
        tf2::Stamped <tf2::Transform> laser_origin = lama_utils::createStampedTransform(msg_laser_origin);
        double roll, pitch, yaw;
        laser_origin.getBasis().getRPY(roll, pitch, yaw);

        lama::Pose3D lp(laser_origin.getOrigin().x(),
                        laser_origin.getOrigin().y(),
                        laser_origin.getOrigin().z(),
                        roll, pitch, yaw);

        lasers_origin_.push_back(lp);

        RCLCPP_INFO(this->get_logger(), "New laser configured (id=%d frame_id=%s)", laser_index,
                    laser_scan->header.frame_id.c_str());
    } else {
        laser_index = frame_to_laser_[laser_scan->header.frame_id];
    }

    // Where was the robot at the time of the scan ?
    geometry_msgs::msg::PoseStamped msg_odom_tf;
    try {
        geometry_msgs::msg::PoseStamped msg_identity = lama_utils::createPoseStamped(
                tf2::Transform(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, 0)),
                rclcpp::Time(laser_scan->header.stamp), base_frame_id_);
        tf_buffer_->transform(msg_identity, msg_odom_tf, odom_frame_id_);
    } catch (tf2::TransformException &e) {
        RCLCPP_WARN(this->get_logger(), "Failed to compute odom pose, skipping scan %s", e.what());
        return;
    }
    tf2::Stamped <tf2::Transform> odom_tf = lama_utils::createStampedTransform(msg_odom_tf);

    Pose2D odom(odom_tf.getOrigin().x(), odom_tf.getOrigin().y(), lama_utils::getYaw(odom_tf.getRotation()));

    bool update;

    size_t size = laser_scan->ranges.size();
    size_t beam_step = 1;

    float max_range;
    if (max_range_ == 0.0 || max_range_ > laser_scan->range_max)
        max_range = laser_scan->range_max;
    else
        max_range = max_range_;

    float min_range = laser_scan->range_min;
    float angle_min = laser_scan->angle_min;
    float angle_inc = laser_scan->angle_increment;

    PointCloudXYZ::Ptr cloud(new PointCloudXYZ);

    cloud->sensor_origin_ = lasers_origin_[laser_index].xyz();
    cloud->sensor_orientation_ = Quaterniond(lasers_origin_[laser_index].state.so3().matrix());

    cloud->points.reserve(laser_scan->ranges.size());
    for(size_t i = 0; i < size; i += beam_step ){

        if (std::isnan(laser_scan->ranges[i]) || std::isinf(laser_scan->ranges[i]))
            continue;

        if ( laser_scan->ranges[i] >= max_range || laser_scan->ranges[i] <= min_range )
            continue;

        Eigen::Vector3d point;
        point << laser_scan->ranges[i] * std::cos(angle_min+(i*angle_inc)),
                 laser_scan->ranges[i] * std::sin(angle_min+(i*angle_inc)),
                 0;

        cloud->points.push_back( point );
    }

    rclcpp::Time start = ros_clock->now();
    update = slam2d_->update(cloud, odom, rclcpp::Time(laser_scan->header.stamp).seconds());
    rclcpp::Time end = ros_clock->now();

    if (update){
        Pose2D pose = slam2d_->getPose();

        // subtracting base to odom from map to base and send map to odom instead
        geometry_msgs::msg::PoseStamped msg_odom_to_map;
        try {
            tf2::Quaternion q;
            q.setRPY(0, 0, pose.rotation());
            geometry_msgs::msg::PoseStamped msg_odom_to_map_baseFrame = lama_utils::createPoseStamped(
                    tf2::Transform(q, tf2::Vector3(pose.x(), pose.y(), 0)).inverse(),
                    rclcpp::Time(laser_scan->header.stamp), base_frame_id_);
            tf_buffer_->transform(msg_odom_to_map_baseFrame, msg_odom_to_map, odom_frame_id_);
        } catch (tf2::TransformException &e) {
            RCLCPP_WARN(this->get_logger(), "Failed to subtract base to odom transform");
            return;
        }
        tf2::Stamped <tf2::Transform> odom_to_map = lama_utils::createStampedTransform(msg_odom_to_map);

        latest_tf_ = tf2::Transform(tf2::Quaternion(odom_to_map.getRotation()),
                                   tf2::Vector3(odom_to_map.getOrigin()));

        // We want to send a transform that is good up until a
        // tolerance time so that odom can be used
        rclcpp::Time transform_expiration = rclcpp::Time(laser_scan->header.stamp) + transform_tolerance_;
        geometry_msgs::msg::TransformStamped tmp_tf_stamped = lama_utils::createTransformStamped(
                latest_tf_.inverse(), transform_expiration, global_frame_id_, odom_frame_id_);
        tfb_->sendTransform(tmp_tf_stamped);

        //--
        nav_msgs::msg::Path path;
        path.header.frame_id = global_frame_id_;
        path.header.stamp    = laser_scan->header.stamp;

        uint32_t idx = slam2d_->getBestParticleIdx();
        const size_t num_poses = slam2d_->getParticles()[idx].poses.size();
        for (size_t i = 0; i < num_poses; ++i){

            geometry_msgs::msg::PoseStamped p;
            p.header.frame_id = global_frame_id_;
            p.header.stamp    = laser_scan->header.stamp;
            p.pose.position.x = slam2d_->getParticles()[idx].poses[i].x();
            p.pose.position.y = slam2d_->getParticles()[idx].poses[i].y();
            p.pose.position.z = 0.0;

            tf2::Quaternion quat_tf;
            quat_tf.setRPY(0.0, 0.0, slam2d_->getParticles()[idx].poses[i].rotation());
            tf2::convert(quat_tf, p.pose.orientation);
            //p.pose.orientation = tf2_ros::createQuaternionMsgFromYaw(slam2d_->getParticles()[idx].poses[i].rotation());

            path.poses.push_back(p);
        }
        path_pub_->publish(path);

        RCLCPP_DEBUG(this->get_logger(), "Update time: %.3fms - NEFF: %.2f",
                static_cast<double>(RCUTILS_NS_TO_MS((end-start).nanoseconds())), slam2d_->getNeff());
	RCLCPP_DEBUG(this->get_logger(), "Sent TF Map->Odom");
    } else {
        // Nothing has change, therefore, republish the last transform.
        rclcpp::Time transform_expiration = rclcpp::Time(laser_scan->header.stamp) + transform_tolerance_;
        geometry_msgs::msg::TransformStamped tmp_tf_stamped = lama_utils::createTransformStamped(
                latest_tf_.inverse(), transform_expiration, global_frame_id_, odom_frame_id_);
        tfb_->sendTransform(tmp_tf_stamped);
	RCLCPP_DEBUG(this->get_logger(), "Nothing sent as TF Map->Odom");
    } // end if (update)

    const size_t num_particles = slam2d_->getParticles().size();
    poses_.poses.resize(num_particles);
    for (size_t i = 0; i < num_particles; ++i){

        poses_.poses[i].position.x = slam2d_->getParticles()[i].pose.x();
        poses_.poses[i].position.y = slam2d_->getParticles()[i].pose.y();
        poses_.poses[i].position.z = 0.0;

        tf2::Quaternion quat_tf;
        quat_tf.setRPY(0.0, 0.0, slam2d_->getParticles()[i].pose.rotation());
        tf2::convert(quat_tf, poses_.poses[i].orientation);
        //poses_.poses[i].orientation = tf2_ros::createQuaternionMsgFromYaw(slam2d_->getParticles()[i].pose.rotation());
    }

    poses_pub_->publish(poses_);
}

void lama::PFSlam2DROS::publishCallback()       // const rclcpp::TimerEvent &
{
    auto time = ros_clock->now(); //rclcpp::Time::now();

    nav_msgs::msg::OccupancyGrid ros_occ;
    ros_occ.header.frame_id = global_frame_id_;
    ros_occ.header.stamp = time;

    if (map_pub_->get_subscription_count() > 0 ){
        OccupancyMsgFromOccupancyMap(ros_occ);
        ros_occ_.header.stamp = time;
        map_pub_->publish(ros_occ);
    }

    if (dist_pub_->get_subscription_count() > 0){
        DistanceMsgFromOccupancyMap(ros_cost_);
        ros_cost_.header.stamp = time;
        dist_pub_->publish(ros_cost_);
    }

    if (patch_pub_->get_subscription_count() > 0){
        PatchMsgFromOccupancyMap(ros_patch_);
        ros_patch_.header.stamp = time;
        patch_pub_->publish(ros_patch_);
    }
}

void lama::PFSlam2DROS::publishMaps() {
    auto time = ros_clock->now(); //rclcpp::Time::now();

    nav_msgs::msg::OccupancyGrid ros_occ;
    ros_occ.header.frame_id = global_frame_id_;
    ros_occ.header.stamp = time;

    OccupancyMsgFromOccupancyMap(ros_occ);
    ros_occ_.header.stamp = time;
    map_pub_->publish(ros_occ);

    DistanceMsgFromOccupancyMap(ros_cost_);
    ros_cost_.header.stamp = time;
    dist_pub_->publish(ros_cost_);

    PatchMsgFromOccupancyMap(ros_patch_);
    ros_patch_.header.stamp = time;
    patch_pub_->publish(ros_patch_);
}

bool lama::PFSlam2DROS::OccupancyMsgFromOccupancyMap(nav_msgs::msg::OccupancyGrid &msg) {

    const FrequencyOccupancyMap* map = slam2d_->getOccupancyMap();
    if (map == 0)
        return false;

    Vector3ui imin, imax;
    map->bounds(imin, imax);

    unsigned int width = imax(0) - imin(0);
    unsigned int height= imax(1) - imin(1);

    if (width == 0 || height == 0)
        return false;

    if ( width*height > msg.data.size() )
        msg.data.resize(width*height);

    Image image;
    image.alloc(width, height, 1);
    image.fill(50);

    map->visit_all_cells([&image, &map, &imin](const Vector3ui& coords){
        Vector3ui adj_coords = coords - imin;

        if (map->isFree(coords))
            image(adj_coords(0), adj_coords(1)) = 0;
        else if (map->isOccupied(coords))
            image(adj_coords(0), adj_coords(1)) = 100;
        else
            image(adj_coords(0), adj_coords(1)) = 0xff;
    });

    memcpy(&msg.data[0], image.data.get(), width*height);

    msg.info.width = width;
    msg.info.height = height;
    msg.info.resolution = map->resolution;

    Vector3d pos = map->m2w(imin);
    msg.info.origin.position.x = pos.x();
    msg.info.origin.position.y = pos.y();
    msg.info.origin.position.z = 0;

    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0.0, 0.0, 0);
    tf2::convert(quat_tf, msg.info.origin.orientation);
    //msg.info.origin.orientation = tf2_ros::createQuaternionMsgFromYaw(0);

    return true;
}

bool lama::PFSlam2DROS::DistanceMsgFromOccupancyMap(nav_msgs::msg::OccupancyGrid &msg) {
    const DynamicDistanceMap* map = slam2d_->getDistanceMap();
    if (map == 0)
        return false;

    Vector3ui imin, imax;
    map->bounds(imin, imax);

    unsigned int width = imax(0) - imin(0);
    unsigned int height= imax(1) - imin(1);

    double factor = 1.0 / map->maxDistance();

    if (width == 0 || height == 0)
        return false;

    if ( width*height > msg.data.size() )
        msg.data.resize(width*height);

    Image image;
    image.alloc(width, height, 1);
    image.fill(50);

    map->visit_all_cells([&image, &map, &imin, factor](const Vector3ui& coords){
        Vector3ui adj_coords = coords - imin;
        image(adj_coords(0), adj_coords(1)) = 100 - 100 * map->distance(coords) * factor;
    });

    memcpy(&msg.data[0], image.data.get(), width*height);

    msg.info.width = width;
    msg.info.height = height;
    msg.info.resolution = map->resolution;

    Vector3d pos = map->m2w(imin);
    msg.info.origin.position.x = pos.x();
    msg.info.origin.position.y = pos.y();
    msg.info.origin.position.z = 0;

    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0.0, 0.0, 0);
    tf2::convert(quat_tf, msg.info.origin.orientation);
    //msg.info.origin.orientation = tf2_ros::createQuaternionMsgFromYaw(0);

    return true;
}

bool lama::PFSlam2DROS::PatchMsgFromOccupancyMap(nav_msgs::msg::OccupancyGrid &msg) {
    const FrequencyOccupancyMap* map = slam2d_->getOccupancyMap();
    if (map == 0)
        return false;

    Vector3d min, max;
    map->bounds(min, max);

    Vector3ui imin = map->w2m(min);
    Vector3ui imax = map->w2m(max);

    unsigned int width = imax(0) - imin(0);
    unsigned int height= imax(1) - imin(1);

    if (width == 0 || height == 0)
        return false;

    msg.data.resize(width*height);

    for (unsigned int j = imin(1); j < imax(1); ++j)
        for (unsigned int i = imin(0); i < imax(0); ++i){
            if (map->patchAllocated(Vector3ui(i,j,0))){
                if (map->patchIsUnique(Vector3ui(i,j,0)))
                    msg.data[ (j-imin[1])*width + (i-imin[0]) ] = 100;
                else
                    msg.data[ (j-imin[1])*width + (i-imin[0]) ] = 50;
            }
            else
                msg.data[ (j-imin[1])*width + (i-imin[0]) ] = -1;
        }

    msg.info.width = width;
    msg.info.height = height;
    msg.info.resolution = map->resolution;

    msg.info.origin.position.x = min[0];
    msg.info.origin.position.y = min[1];
    msg.info.origin.position.z = 0;

    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0.0, 0.0, 0.0);
    tf2::convert(quat_tf, msg.info.origin.orientation);
    //msg.info.origin.orientation = tf2_ros::createQuaternionMsgFromYaw(0);

    return true;
}

void lama::PFSlam2DROS::onGetMap(const std::shared_ptr <nav_msgs::srv::GetMap::Request> req,
                                 std::shared_ptr <nav_msgs::srv::GetMap::Response> res) {
    static_cast<void>(req);  // To suppress compiler warning
    res->map.header.frame_id = global_frame_id_;
    res->map.header.stamp = ros_clock->now();

    OccupancyMsgFromOccupancyMap(res->map);
}

void lama::PFSlam2DROS::printSummary() {
    if (slam2d_->summary)
        std::cout << slam2d_->summary->report() << std::endl;
}

int main(int argc, char *argv[]) {
    //std::cout << argc << " params: " << std::endl;
    //for(int i=0; i<argc; ++i){
    //    std::cout << "  " << argv[i] << std::endl;
    //}

    rclcpp::init(argc, argv);
    auto pf_slam2d_ros = std::make_shared<lama::PFSlam2DROS>();
    pf_slam2d_ros->declare_parameter("rosbag", false);
    bool using_rosbag = pf_slam2d_ros->get_parameter("rosbag").as_bool();
    if(using_rosbag) {
        RCLCPP_INFO(pf_slam2d_ros->get_logger(), "Running SLAM in Rosbag Mode (offline)");
        RCLCPP_INFO(pf_slam2d_ros->get_logger(), "After the rosbag has finished, wait up to 'map_publish_period' for the map to be published. Save your map and use ctrl-c to quit.");
    } else{
        RCLCPP_INFO(pf_slam2d_ros->get_logger(), "Running SLAM in Live Mode");
    }

    rclcpp::spin(pf_slam2d_ros);
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(lama::PFSlam2DROS)

