#ifndef LIVOX_GPU_POINTS_PLUGIN_H_
#define LIVOX_GPU_POINTS_PLUGIN_H_

#include <gazebo/plugins/GpuRayPlugin.hh>
#include <gazebo/sensors/GpuRaySensor.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/Node.hh>

#include <ignition/math6/ignition/math.hh>

#include <sensor_msgs/PointCloud2.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/callback_queue.h>
#include <tf/transform_broadcaster.h>

#include <sdf/Param.hh>

#include <mutex>
#include <thread>
#include <vector>

namespace gazebo 
{

struct AviaRotateInfo {
    double time;
    double azimuth;
    double zenith;
};

class LivoxGpuPointsPlugin : public GpuRayPlugin
{
public:
    LivoxGpuPointsPlugin();

    ~LivoxGpuPointsPlugin();

    void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

private:

    void SendRosTf(const ignition::math::Pose3d& pose, const std::string& father_frame, const std::string& child_frame);

    void OnNewLaserAnglesScans(ConstLaserScanAnglesStampedPtr& _msg);

    ros::NodeHandle* nh_;
    ros::Publisher pub_;
    std::string topic_name_;

    int64_t samplesStep {0};
    int64_t currStartIndex {0};
    int64_t downSample {1};

    double maxDist {400.0};
    double minDist {0.1};

    bool publish_pointcloud2 {false};

    // Custom Callback Queue
    ros::CallbackQueue laser_queue_;
    void laserQueueThread();
    std::thread callback_laser_queue_thread_;

    std::shared_ptr<tf::TransformBroadcaster> tfBroadcaster;

    gazebo::sensors::GpuRaySensorPtr raySensor;

    // Subscribe to gazebo laserscan
    gazebo::transport::NodePtr gazebo_node_;
    gazebo::transport::SubscriberPtr sub_;

    std::vector<AviaRotateInfo> aviaInfos;
    std::mutex lock_;
};
}


#endif /* LIVOX_GPU_POINTS_PLUGIN_H_ */
