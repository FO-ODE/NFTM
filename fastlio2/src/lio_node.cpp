#include <mutex>
#include <vector>
#include <queue>
#include <memory>
#include <iostream>
#include <chrono>
#include <cmath>
#include <array>
// #include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <unitree_go/msg/low_state.hpp>

#include "utils.h"
#include "map_builder/commons.h"
#include "map_builder/map_builder.h"

#include <pcl_conversions/pcl_conversions.h>
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;
struct NodeConfig
{
    std::string imu_topic = "/livox/imu";
    std::string lidar_topic = "/livox/lidar";
    std::string lowstate_topic = "/lowstate";
    std::string body_frame = "body";
    std::string world_frame = "lidar";
    bool publish_foot_markers = true;
    std::string foot_marker_topic = "foot_markers";
    std::vector<std::string> foot_marker_names = {"fr", "fl", "rr", "rl"};
    std::array<std::array<float, 3>, 4> foot_marker_colors = {{
        {1.0F, 0.2F, 0.2F},
        {0.2F, 0.8F, 1.0F},
        {1.0F, 0.8F, 0.2F},
        {0.2F, 1.0F, 0.4F},
    }};
    bool print_time_cost = false;
};
struct StateData
{
    bool lidar_pushed = false;
    std::mutex imu_mutex;
    std::mutex lidar_mutex;
    std::mutex lowstate_mutex;
    double last_lidar_time = -1.0;
    double last_imu_time = -1.0;
    double last_lowstate_time = -1.0;
    std::deque<IMUData> imu_buffer;
    std::deque<LowStateData> lowstate_buffer;
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;
    nav_msgs::msg::Path path;
};

class LIONode : public rclcpp::Node
{
public:
    LIONode() : Node("lio_node")
    {
        RCLCPP_INFO(this->get_logger(), "LIO Node Started");
        loadParameters();

        m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(m_node_config.imu_topic, 10, std::bind(&LIONode::imuCB, this, std::placeholders::_1));
        m_lidar_sub = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(m_node_config.lidar_topic, 10, std::bind(&LIONode::lidarCB, this, std::placeholders::_1));
        m_lowstate_sub = this->create_subscription<unitree_go::msg::LowState>(m_node_config.lowstate_topic, 10, std::bind(&LIONode::lowstateCB, this, std::placeholders::_1));

        m_body_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud", 10000);
        m_world_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("world_cloud", 10000);
        m_path_pub = this->create_publisher<nav_msgs::msg::Path>("lio_path", 10000);
        m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("lio_odom", 10000);
        m_foot_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(m_node_config.foot_marker_topic, 10);
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_state_data.path.poses.clear();
        m_state_data.path.header.frame_id = m_node_config.world_frame;

        m_kf = std::make_shared<IESKF>();
        m_kf->setMaxIter(m_builder_config.ieskf_max_iter);
        m_builder = std::make_shared<MapBuilder>(m_builder_config, m_kf);
        m_timer = this->create_wall_timer(20ms, std::bind(&LIONode::timerCB, this));
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);

        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        m_node_config.imu_topic = config["imu_topic"].as<std::string>();
        m_node_config.lidar_topic = config["lidar_topic"].as<std::string>();
        m_node_config.lowstate_topic = config["lowstate_topic"].as<std::string>();
        m_node_config.body_frame = config["body_frame"].as<std::string>();
        m_node_config.world_frame = config["world_frame"].as<std::string>();
        m_node_config.print_time_cost = config["print_time_cost"].as<bool>();
        if (config["publish_foot_markers"])
            m_node_config.publish_foot_markers = config["publish_foot_markers"].as<bool>();
        if (config["foot_marker_topic"])
            m_node_config.foot_marker_topic = config["foot_marker_topic"].as<std::string>();
        if (config["foot_marker_names"])
        {
            std::vector<std::string> foot_marker_names = config["foot_marker_names"].as<std::vector<std::string>>();
            if (foot_marker_names.size() == 4)
                m_node_config.foot_marker_names = foot_marker_names;
            else
                RCLCPP_WARN(this->get_logger(), "foot_marker_names must contain exactly 4 names, keep default foot marker names");
        }
        if (config["foot_marker_colors"])
        {
            std::vector<float> foot_marker_colors = config["foot_marker_colors"].as<std::vector<float>>();
            if (foot_marker_colors.size() == 12)
            {
                for (size_t i = 0; i < 4; i++)
                {
                    m_node_config.foot_marker_colors[i][0] = foot_marker_colors[3 * i];
                    m_node_config.foot_marker_colors[i][1] = foot_marker_colors[3 * i + 1];
                    m_node_config.foot_marker_colors[i][2] = foot_marker_colors[3 * i + 2];
                }
            }
            else
                RCLCPP_WARN(this->get_logger(), "foot_marker_colors must contain exactly 12 RGB values, keep default foot marker colors");
        }

        m_builder_config.lidar_filter_num = config["lidar_filter_num"].as<int>();
        m_builder_config.lidar_min_range = config["lidar_min_range"].as<double>();
        m_builder_config.lidar_max_range = config["lidar_max_range"].as<double>();
        m_builder_config.scan_resolution = config["scan_resolution"].as<double>();
        m_builder_config.map_resolution = config["map_resolution"].as<double>();
        m_builder_config.cube_len = config["cube_len"].as<double>();
        m_builder_config.det_range = config["det_range"].as<double>();
        m_builder_config.move_thresh = config["move_thresh"].as<double>();
        m_builder_config.na = config["na"].as<double>();
        m_builder_config.ng = config["ng"].as<double>();
        m_builder_config.nba = config["nba"].as<double>();
        m_builder_config.nbg = config["nbg"].as<double>();

        m_builder_config.imu_init_num = config["imu_init_num"].as<int>();
        m_builder_config.near_search_num = config["near_search_num"].as<int>();
        m_builder_config.ieskf_max_iter = config["ieskf_max_iter"].as<int>();
        m_builder_config.gravity_align = config["gravity_align"].as<bool>();
        if (config["state_log_enable"])
            m_builder_config.state_log_enable = config["state_log_enable"].as<bool>();
        if (config["state_log_path"])
            m_builder_config.state_log_path = config["state_log_path"].as<std::string>();
        if (config["state_log_flush"])
            m_builder_config.state_log_flush = config["state_log_flush"].as<bool>();
        std::vector<double> t_il_vec = config["t_il"].as<std::vector<double>>();
        std::vector<double> r_il_vec = config["r_il"].as<std::vector<double>>();
        m_builder_config.t_il << t_il_vec[0], t_il_vec[1], t_il_vec[2];
        m_builder_config.r_il << r_il_vec[0], r_il_vec[1], r_il_vec[2], r_il_vec[3], r_il_vec[4], r_il_vec[5], r_il_vec[6], r_il_vec[7], r_il_vec[8];
        m_builder_config.lidar_cov_inv = config["lidar_cov_inv"].as<double>();

        if (config["contact_enable"])
            m_builder_config.contact_enable = config["contact_enable"].as<bool>();
        if (config["contact_force_threshold"])
            m_builder_config.contact_force_threshold = config["contact_force_threshold"].as<double>();
        if (config["contact_position_cov_inv"])
            m_builder_config.contact_position_cov_inv = config["contact_position_cov_inv"].as<double>();
        if (config["contact_velocity_cov_inv"])
            m_builder_config.contact_velocity_cov_inv = config["contact_velocity_cov_inv"].as<double>();
        if (config["contact_foot_position_noise"])
            m_builder_config.contact_foot_position_noise = config["contact_foot_position_noise"].as<double>();
        if (config["contact_abduction_link"])
            m_builder_config.contact_abduction_link = config["contact_abduction_link"].as<double>();
        if (config["contact_thigh_link"])
            m_builder_config.contact_thigh_link = config["contact_thigh_link"].as<double>();
        if (config["contact_calf_link"])
            m_builder_config.contact_calf_link = config["contact_calf_link"].as<double>();
        if (config["contact_foot_link_to_ground"])
            m_builder_config.contact_foot_link_to_ground = config["contact_foot_link_to_ground"].as<double>();
        if (config["contact_r_base_body"])
        {
            std::vector<double> r_base_body = config["contact_r_base_body"].as<std::vector<double>>();
            if (r_base_body.size() == 9)
                m_builder_config.contact_r_base_body << r_base_body[0], r_base_body[1], r_base_body[2],
                    r_base_body[3], r_base_body[4], r_base_body[5],
                    r_base_body[6], r_base_body[7], r_base_body[8];
            else
                RCLCPP_WARN(this->get_logger(), "contact_r_base_body must contain exactly 9 values, keep identity");
        }
        if (config["contact_t_base_body"])
        {
            std::vector<double> t_base_body = config["contact_t_base_body"].as<std::vector<double>>();
            if (t_base_body.size() == 3)
                m_builder_config.contact_t_base_body << t_base_body[0], t_base_body[1], t_base_body[2];
            else
                RCLCPP_WARN(this->get_logger(), "contact_t_base_body must contain exactly 3 values, keep zero");
        }
        if (config["contact_hip_offsets"])
        {
            std::vector<double> hip_offsets = config["contact_hip_offsets"].as<std::vector<double>>();
            if (hip_offsets.size() == 12)
            {
                for (size_t i = 0; i < 12; i++)
                    m_builder_config.contact_hip_offsets(i) = hip_offsets[i];
            }
        }
    }

    void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
        double timestamp = Utils::getSec(msg->header);
        if (timestamp < m_state_data.last_imu_time)
        {
            RCLCPP_WARN(this->get_logger(), "IMU Message is out of order");
            std::deque<IMUData>().swap(m_state_data.imu_buffer);
        }
        m_state_data.imu_buffer.emplace_back(V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z) * 10.0,
                                             V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                                             timestamp);
        m_state_data.last_imu_time = timestamp;
    }

    void lowstateCB(const unitree_go::msg::LowState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(m_state_data.lowstate_mutex);
        double timestamp = this->get_clock()->now().seconds();
        if (timestamp < m_state_data.last_lowstate_time)
        {
            RCLCPP_WARN(this->get_logger(), "LowState Message is out of order");
            std::deque<LowStateData>().swap(m_state_data.lowstate_buffer);
        }

        V12D joint_pos = V12D::Zero();
        V12D joint_vel = V12D::Zero();
        V4D foot_force = V4D::Zero();

        for (size_t i = 0; i < 12; i++)
        {
            const auto &motor = msg->motor_state[i];
            joint_pos(i) = finiteOrZero(motor.q);
            joint_vel(i) = finiteOrZero(motor.dq);
        }
        for (size_t i = 0; i < 4; i++)
            foot_force(i) = static_cast<double>(msg->foot_force[i]);

        m_state_data.lowstate_buffer.emplace_back(joint_pos, joint_vel, foot_force, timestamp);
        m_state_data.last_lowstate_time = timestamp;
    }

    void lidarCB(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        CloudType::Ptr cloud = Utils::livox2PCL(msg, m_builder_config.lidar_filter_num, m_builder_config.lidar_min_range, m_builder_config.lidar_max_range);
        std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
        double timestamp = Utils::getSec(msg->header);
        if (timestamp < m_state_data.last_lidar_time)
        {
            RCLCPP_WARN(this->get_logger(), "Lidar Message is out of order");
            std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(m_state_data.lidar_buffer);
        }
        m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
        m_state_data.last_lidar_time = timestamp;
    }

    bool syncPackage()
    {
        if (m_state_data.imu_buffer.empty() || m_state_data.lidar_buffer.empty())
            return false;
        if (!m_state_data.lidar_pushed)
        {
            m_package.cloud = m_state_data.lidar_buffer.front().second;
            std::sort(m_package.cloud->points.begin(), m_package.cloud->points.end(), [](PointType &p1, PointType &p2)
                      { return p1.curvature < p2.curvature; });
            m_package.cloud_start_time = m_state_data.lidar_buffer.front().first;
            m_package.cloud_end_time = m_package.cloud_start_time + m_package.cloud->points.back().curvature / 1000.0;
            m_state_data.lidar_pushed = true;
        }
        if (m_state_data.last_imu_time < m_package.cloud_end_time)
            return false;

        Vec<IMUData>().swap(m_package.imus);
        Vec<LowStateData>().swap(m_package.lowstates);
        while (!m_state_data.imu_buffer.empty() && m_state_data.imu_buffer.front().time < m_package.cloud_end_time)
        {
            m_package.imus.emplace_back(m_state_data.imu_buffer.front());
            m_state_data.imu_buffer.pop_front();
        }
        {
            std::lock_guard<std::mutex> lock(m_state_data.lowstate_mutex);
            while (!m_state_data.lowstate_buffer.empty() && m_state_data.lowstate_buffer.front().time < m_package.cloud_end_time)
            {
                m_package.lowstates.emplace_back(m_state_data.lowstate_buffer.front());
                m_state_data.lowstate_buffer.pop_front();
            }
            if (m_package.lowstates.empty() && !m_state_data.lowstate_buffer.empty())
                m_package.lowstates.emplace_back(m_state_data.lowstate_buffer.back());
            while (m_state_data.lowstate_buffer.size() > 200)
                m_state_data.lowstate_buffer.pop_front();
        }
        m_state_data.lidar_buffer.pop_front();
        m_state_data.lidar_pushed = false;
        return true;
    }

    void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, CloudType::Ptr cloud, std::string frame_id, const double &time)
    {
        if (pub->get_subscription_count() <= 0)
            return;
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = frame_id;
        cloud_msg.header.stamp = Utils::getTime(time);
        pub->publish(cloud_msg);
    }

    void publishOdometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub, std::string frame_id, std::string child_frame, const double &time)
    {
        if (odom_pub->get_subscription_count() <= 0)
            return;
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = frame_id;
        odom.header.stamp = Utils::getTime(time);
        odom.child_frame_id = child_frame;
        odom.pose.pose.position.x = m_kf->x().t_wi.x();
        odom.pose.pose.position.y = m_kf->x().t_wi.y();
        odom.pose.pose.position.z = m_kf->x().t_wi.z();
        Eigen::Quaterniond q(m_kf->x().r_wi);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        V3D vel = m_kf->x().r_wi.transpose() * m_kf->x().v;
        odom.twist.twist.linear.x = vel.x();
        odom.twist.twist.linear.y = vel.y();
        odom.twist.twist.linear.z = vel.z();
        odom_pub->publish(odom);
    }

    void publishPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub, std::string frame_id, const double &time)
    {
        if (path_pub->get_subscription_count() <= 0)
            return;
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.header.stamp = Utils::getTime(time);
        pose.pose.position.x = m_kf->x().t_wi.x();
        pose.pose.position.y = m_kf->x().t_wi.y();
        pose.pose.position.z = m_kf->x().t_wi.z();
        Eigen::Quaterniond q(m_kf->x().r_wi);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        m_state_data.path.poses.push_back(pose);
        path_pub->publish(m_state_data.path);
    }

    void broadCastTF(std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster, std::string frame_id, std::string child_frame, const double &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = frame_id;
        transformStamped.child_frame_id = child_frame;
        transformStamped.header.stamp = Utils::getTime(time);
        Eigen::Quaterniond q(m_kf->x().r_wi);
        V3D t = m_kf->x().t_wi;
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        broad_caster->sendTransform(transformStamped);
    }

    void publishFootMarkers(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub, const double &time)
    {
        if (marker_pub->get_subscription_count() <= 0)
            return;

        const State &state = m_kf->x();
        const std::array<V3D, 4> foot_positions = {state.p_f1, state.p_f2, state.p_f3, state.p_f4};

        visualization_msgs::msg::MarkerArray marker_array;
        for (size_t i = 0; i < foot_positions.size(); i++)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = m_node_config.world_frame;
            marker.header.stamp = Utils::getTime(time);
            marker.ns = "fastlio_foot";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = foot_positions[i].x();
            marker.pose.position.y = foot_positions[i].y();
            marker.pose.position.z = foot_positions[i].z();
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.08;
            marker.scale.y = 0.08;
            marker.scale.z = 0.08;
            marker.color.r = m_node_config.foot_marker_colors[i][0];
            marker.color.g = m_node_config.foot_marker_colors[i][1];
            marker.color.b = m_node_config.foot_marker_colors[i][2];
            marker.color.a = 1.0;
            marker.lifetime = rclcpp::Duration::from_seconds(0.2);
            marker.text = m_node_config.foot_marker_names[i];
            marker_array.markers.push_back(marker);
        }
        marker_pub->publish(marker_array);
    }

    void timerCB()
    {
        if (!syncPackage())
            return;
        auto t1 = std::chrono::high_resolution_clock::now();
        m_builder->process(m_package);
        auto t2 = std::chrono::high_resolution_clock::now();

        if (m_node_config.print_time_cost)
        {
            auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
            RCLCPP_WARN(this->get_logger(), "Time cost: %.2f ms", time_used);
        }

        if (m_builder->status() != BuilderStatus::MAPPING)
            return;

        broadCastTF(m_tf_broadcaster, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);
        if (m_node_config.publish_foot_markers)
            publishFootMarkers(m_foot_marker_pub, m_package.cloud_end_time);

        publishOdometry(m_odom_pub, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

        CloudType::Ptr body_cloud = m_builder->lidar_processor()->transformCloud(m_package.cloud, m_builder_config.r_il, m_builder_config.t_il);

        publishCloud(m_body_cloud_pub, body_cloud, m_node_config.body_frame, m_package.cloud_end_time);

        CloudType::Ptr world_cloud = m_builder->lidar_processor()->transformCloud(m_package.cloud, m_builder->lidar_processor()->r_wl(), m_builder->lidar_processor()->t_wl());

        publishCloud(m_world_cloud_pub, world_cloud, m_node_config.world_frame, m_package.cloud_end_time);

        publishPath(m_path_pub, m_node_config.world_frame, m_package.cloud_end_time);
    }

    static double finiteOrZero(double value)
    {
        return std::isfinite(value) ? value : 0.0;
    }

private:
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_lidar_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr m_lowstate_sub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_world_cloud_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_foot_marker_pub;

    rclcpp::TimerBase::SharedPtr m_timer;
    StateData m_state_data;
    SyncPackage m_package;
    NodeConfig m_node_config;
    Config m_builder_config;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<MapBuilder> m_builder;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LIONode>());
    rclcpp::shutdown();
    return 0;
}
