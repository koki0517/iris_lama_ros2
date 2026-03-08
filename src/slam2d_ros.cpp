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

#include "lama/ros/slam2d_ros.h"
#include "rclcpp_components/register_node_macro.hpp"

lama::Slam2DROS::Slam2DROS(const rclcpp::NodeOptions &options) :
        Node("slam2d_ros", options), transform_tolerance_(0, 100000000) {
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

    Slam2D::Options slam_options;
    this->declare_parameter("d_thresh", 0.01);
    slam_options.trans_thresh = this->get_parameter("d_thresh").as_double();
    this->declare_parameter("a_thresh", 0.25);
    slam_options.rot_thresh = this->get_parameter("a_thresh").as_double();
    this->declare_parameter("l2_max", 0.5);
    slam_options.l2_max = this->get_parameter("l2_max").as_double();
    this->declare_parameter("truncate", 0.0);
    slam_options.truncated_ray = this->get_parameter("truncate").as_double();
    this->declare_parameter("resolution", 0.05);
    slam_options.resolution = this->get_parameter("resolution").as_double();
    this->declare_parameter("strategy", "gn");
    slam_options.strategy = this->get_parameter("strategy").as_string();
    this->declare_parameter("use_compression", false);
    slam_options.use_compression = this->get_parameter("use_compression").as_bool();
    this->declare_parameter("compression_algorithm", "lz4");
    slam_options.calgorithm = this->get_parameter("compression_algorithm").as_string();
    this->declare_parameter("mrange", 16.0);
    max_range_ = this->get_parameter("mrange").as_double();

    int itmp;
    this->declare_parameter("max_iterations", 100);
    itmp = this->get_parameter("max_iterations").as_int();
    slam_options.max_iter = itmp;
    this->declare_parameter("patch_size", 32);
    itmp = this->get_parameter("patch_size").as_int();
    slam_options.patch_size = itmp;
    this->declare_parameter("cache_size", 100);
    itmp = this->get_parameter("cache_size").as_int();
    slam_options.cache_size = itmp;

    this->declare_parameter("create_summary", false);
    slam_options.create_summary = this->get_parameter("create_summary").as_bool();

    //periodic_publish_ = nh->create_wall_timer(rclcpp::Duration(tmp), std::bind(&Slam2DROS::publishCallback, this));
    // https://docs.ros2.org/beta3/api/rclcpp/classrclcpp_1_1node_1_1Node.html
    this->declare_parameter("map_publish_period", 5.0);
    tmp = this->get_parameter("map_publish_period").as_double();
    periodic_publish_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::milliseconds(static_cast<int>(tmp * 1000))),
            std::bind(&Slam2DROS::publishCallback, this));

    slam2d_ = std::make_shared<Slam2D>(slam_options);
    slam2d_->setPose(prior);

    // Setup TF workers ...
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock(), tf2::Duration(30));
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
    laser_scan_filter_->registerCallback(std::bind(&Slam2DROS::onLaserScan, this, std::placeholders::_1));

    // Setup publishers
    // TODO latch https://github.com/ros2/ros2/issues/464
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose", rclcpp::QoS(2));
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).transient_local()); // latch=true
    dist_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/distance", rclcpp::QoS(1).transient_local()); // latch=true

    ros_occ_.header.frame_id = global_frame_id_;
    ros_cost_.header.frame_id = global_frame_id_;

    // Setup service
    // https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Cpp-Service-And-Client/#write-the-service-node
    // https://answers.ros.org/question/299126/ros2-error-creating-a-service-server-as-a-member-function/
    service = this->create_service<nav_msgs::srv::GetMap>("/dynamic_map",
                                                          std::bind(&Slam2DROS::onGetMap, this, std::placeholders::_1,
                                                                    std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Online SLAM node up and running");
}

lama::Slam2DROS::~Slam2DROS() {

}

void lama::Slam2DROS::onLaserScan(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan) {
    int laser_index = -1;

    // verify if it is from a known source
    if (frame_to_laser_.find(laser_scan->header.frame_id) == frame_to_laser_.end()) {
        if (not initLaser(laser_scan))
            return;

        laser_index = frame_to_laser_[laser_scan->header.frame_id];
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

    bool update = slam2d_->enoughMotion(odom);

    if (update) {

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
        for (size_t i = 0; i < size; i += beam_step) {
            double range;

            range = laser_scan->ranges[i];

            if (not std::isfinite(range))
                continue;

            if (range >= max_range || range <= min_range)
                continue;


            Eigen::Vector3d point;
            point << range * std::cos(angle_min + (i * angle_inc)),
                    range * std::sin(angle_min + (i * angle_inc)),
                    0;

            cloud->points.push_back(point);
        }

        slam2d_->update(cloud, odom, rclcpp::Time(laser_scan->header.stamp).seconds());
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
	RCLCPP_DEBUG(this->get_logger(), "Sent TF Map->Odom");
    } else {
        // Nothing has change, therefore, republish the last transform.
        rclcpp::Time transform_expiration = rclcpp::Time(laser_scan->header.stamp) + transform_tolerance_;
        geometry_msgs::msg::TransformStamped tmp_tf_stamped = lama_utils::createTransformStamped(
                latest_tf_.inverse(), transform_expiration, global_frame_id_, odom_frame_id_);
        tfb_->sendTransform(tmp_tf_stamped);
	RCLCPP_DEBUG(this->get_logger(), "Nothing sent as TF Map->Odom");
    } // end if (update)
}

bool lama::Slam2DROS::initLaser(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan) {
    // find the origin of the sensor in the base frame
    geometry_msgs::msg::PoseStamped msg_laser_origin;
    try {
        geometry_msgs::msg::PoseStamped msg_identity = lama_utils::createPoseStamped(
                tf2::Transform(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, 0)),
                rclcpp::Time(), laser_scan->header.frame_id);
        tf_buffer_->transform(msg_identity, msg_laser_origin, base_frame_id_);
    } catch (tf2::TransformException &e) {
        RCLCPP_ERROR(this->get_logger(), "Could not find origin of %s", laser_scan->header.frame_id.c_str());
        return false;
    }
    tf2::Stamped <tf2::Transform> laser_origin = lama_utils::createStampedTransform(msg_laser_origin);

    double roll, pitch, yaw;
    laser_origin.getBasis().getRPY(roll, pitch, yaw);

    lama::Pose3D lp(laser_origin.getOrigin().x(),
                    laser_origin.getOrigin().y(),
                    laser_origin.getOrigin().z(),
                    roll, pitch, yaw);
    lasers_origin_.push_back(lp);

    int laser_index = (int) frame_to_laser_.size();  // simple ID generator :)
    frame_to_laser_[laser_scan->header.frame_id] = laser_index;

    RCLCPP_INFO(this->get_logger(), "New laser configured (id=%d frame_id=%s)", laser_index,
                laser_scan->header.frame_id.c_str());
    return true;
}

bool lama::Slam2DROS::OccupancyMsgFromOccupancyMap(nav_msgs::msg::OccupancyGrid &msg) {
    const FrequencyOccupancyMap *map = slam2d_->getOccupancyMap();
    if (map == 0)
        return false;

    Vector3ui imin, imax;
    map->bounds(imin, imax);

    unsigned int width = imax(0) - imin(0);
    unsigned int height = imax(1) - imin(1);

    if (width == 0 || height == 0)
        return false;

    if (width * height > msg.data.size())
        msg.data.resize(width * height);

    Image image;
    image.alloc(width, height, 1);
    image.fill(50);

    map->visit_all_cells([&image, &map, &imin](const Vector3ui &coords) {
        Vector3ui adj_coords = coords - imin;

        if (map->isFree(coords))
            image(adj_coords(0), adj_coords(1)) = 0;
        else if (map->isOccupied(coords))
            image(adj_coords(0), adj_coords(1)) = 100;
        else
            image(adj_coords(0), adj_coords(1)) = 0xff;
    });

    memcpy(&msg.data[0], image.data.get(), width * height);

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

bool lama::Slam2DROS::DistanceMsgFromOccupancyMap(nav_msgs::msg::OccupancyGrid &msg) {
    const DynamicDistanceMap *map = slam2d_->getDistanceMap();
    if (map == 0)
        return false;

    Vector3ui imin, imax;
    map->bounds(imin, imax);

    unsigned int width = imax(0) - imin(0);
    unsigned int height = imax(1) - imin(1);

    double factor = 1.0 / map->maxDistance();

    if (width == 0 || height == 0)
        return false;

    if (width * height > msg.data.size())
        msg.data.resize(width * height);

    Image image;
    image.alloc(width, height, 1);
    image.fill(50);

    map->visit_all_cells([&image, &map, &imin, factor](const Vector3ui &coords) {
        Vector3ui adj_coords = coords - imin;
        image(adj_coords(0), adj_coords(1)) = 100 - 100 * map->distance(coords) * factor;
    });

    memcpy(&msg.data[0], image.data.get(), width * height);

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

void lama::Slam2DROS::publishCallback() {  // const rclcpp::TimerEvent &
    auto time = ros_clock->now(); //rclcpp::Time::now();
    if (map_pub_->get_subscription_count() > 0) {
        OccupancyMsgFromOccupancyMap(ros_occ_);
        ros_occ_.header.stamp = time;
        map_pub_->publish(ros_occ_);
    }

    if (dist_pub_->get_subscription_count() > 0) {
        DistanceMsgFromOccupancyMap(ros_cost_);
        ros_cost_.header.stamp = time;
        dist_pub_->publish(ros_cost_);
    }
}

void lama::Slam2DROS::onGetMap(const std::shared_ptr <nav_msgs::srv::GetMap::Request> req,
                                 std::shared_ptr <nav_msgs::srv::GetMap::Response> res) {
    static_cast<void>(req);  // To suppress compiler warning
    res->map.header.frame_id = global_frame_id_;
    res->map.header.stamp = ros_clock->now();

    OccupancyMsgFromOccupancyMap(res->map);
}

void lama::Slam2DROS::publishMaps() {
    auto time = ros_clock->now(); //rclcpp::Time::now();

    OccupancyMsgFromOccupancyMap(ros_occ_);
    ros_occ_.header.stamp = time;
    map_pub_->publish(ros_occ_);

    DistanceMsgFromOccupancyMap(ros_cost_);
    ros_cost_.header.stamp = time;
    dist_pub_->publish(ros_cost_);
}

void lama::Slam2DROS::printSummary() {
    if (slam2d_->summary)
        std::cout << slam2d_->summary->report() << std::endl;
}

int main(int argc, char *argv[]) {
    //std::cout << argc << " params: " << std::endl;
    //for(int i=0; i<argc; ++i){
    //    std::cout << "  " << argv[i] << std::endl;
    //}

    rclcpp::init(argc, argv);
    auto slam2d_ros = std::make_shared<lama::Slam2DROS>();
    slam2d_ros->declare_parameter("rosbag", false);
    bool using_rosbag = slam2d_ros->get_parameter("rosbag").as_bool();
    if(using_rosbag) {
        RCLCPP_INFO(slam2d_ros->get_logger(), "Running SLAM in Rosbag Mode (offline)");
        RCLCPP_INFO(slam2d_ros->get_logger(), "After the rosbag has finished, wait up to 'map_publish_period' for the map to be published. Save your map and use ctrl-c to quit.");
    } else{
        RCLCPP_INFO(slam2d_ros->get_logger(), "Running SLAM in Live Mode");
    }

    rclcpp::spin(slam2d_ros);
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(lama::Slam2DROS)

