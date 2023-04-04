#include <livox_laser_simulation/livox_gpu_points_plugin.h>

#include "livox_laser_simulation/csv_reader.hpp"

namespace gazebo
{

GZ_REGISTER_SENSOR_PLUGIN(LivoxGpuPointsPlugin)

LivoxGpuPointsPlugin::LivoxGpuPointsPlugin() : nh_(nullptr), maxDist(400.0), minDist(0.1)
{
    ROS_ERROR("LivoxGpuPointsPlugin constructor");
    gzerr << "Construct\n";
}

LivoxGpuPointsPlugin::~LivoxGpuPointsPlugin()
{}
#if 0
{
    laser_queue_.clear();
    laser_queue_.disable();
    if (nh_) 
    {
        nh_->shutdown();
        delete nh_;
        nh_ = nullptr;
    }
    callback_laser_queue_thread_.join();
}
#endif
void LivoxGpuPointsPlugin::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
    // Load plugin
    GpuRayPlugin::Load(_parent, _sdf);

    gazebo_node_ = transport::NodePtr(new transport::Node());
    gazebo_node_->Init(); //

    //sdfPtr = sdf;
    auto rayElem = _sdf->GetElement("ray");
    auto scanElem = rayElem->GetElement("scan");
    auto rangeElem = rayElem->GetElement("range");
    auto curr_scan_topic = _sdf->Get<std::string>("ros_topic");

    //ros::init(argc, argv, curr_scan_topic);
    //rosNode.reset(new ros::NodeHandle);
    // Create Node Handle
    nh_ = new ros::NodeHandle();
    pub_ = nh_->advertise<sensor_msgs::PointCloud2>(curr_scan_topic, 5);

    raySensor = std::dynamic_pointer_cast<sensors::GpuRaySensor>(_parent);
    raySensor->SetActive(false);
    auto sensor_pose = raySensor->Pose();
    SendRosTf(sensor_pose, raySensor->ParentName(), raySensor->Name());

    //node = transport::NodePtr(new transport::Node());
    //node->Init(raySensor->WorldName());
    //scanPub = gazebo_node_->Advertise<msgs::LaserScanStamped>(raySensor->Topic(), 50);

    // Load csv file
    std::vector<std::vector<double>> datas;
    std::string file_name = _sdf->Get<std::string>("csv_file_name");
    if (!CsvReader::ReadCsvFile(file_name, datas)) {
        ROS_FATAL_STREAM("cannot get csv file!" << file_name << "will return !");
        return;
    }
    aviaInfos.clear();
    convertDataToRotateInfo(datas, aviaInfos);
    maxPointSize = aviaInfos.size();
    ROS_INFO_STREAM("scan info size:" << maxPointSize);
    samplesStep = _sdf->Get<int>("samples");
    downSample = _sdf->Get<int>("downsample");

    if (downSample < 1) {
        ROS_WARN_STREAM("Downsample must be greater or equal 1");
        downSample = 1;
    }
    raySensor->SetActive(true);
    if (raySensor->IsActive())
    {
        ROS_INFO_STREAM("Sensor is active");
    }
    ROS_INFO_STREAM("Topic: " << raySensor->Topic());
    sub_ = gazebo_node_->Subscribe(raySensor->Topic(), &LivoxGpuPointsPlugin::OnNewLaserScans, this);
}

void LivoxGpuPointsPlugin::ConnectCb()
{
#if 0
  std::lock_guard<std::mutex> lock(lock_);
  if (pub_.getNumSubscribers()) {
    if (!sub_) {
      sub_ = gazebo_node_->Subscribe(this->parentSensor->Topic(), &LivoxGpuPointsPlugin::OnScan, this);
    }
    parentSensor->SetActive(true);
  } else {
#if GAZEBO_MAJOR_VERSION >= 7
    if (sub_) {
      sub_->Unsubscribe();
      sub_.reset();
    }
#endif
    parentSensor->SetActive(false);
  }
#endif
}

void LivoxGpuPointsPlugin::SendRosTf(const ignition::math::Pose3d &pose, const std::string &father_frame,
                                     const std::string &child_frame) {
    if (!tfBroadcaster) {
        tfBroadcaster.reset(new tf::TransformBroadcaster);
    }
    tf::Transform tf;
    auto rot = pose.Rot();
    auto pos = pose.Pos();
    tf.setRotation(tf::Quaternion(rot.X(), rot.Y(), rot.Z(), rot.W()));
    tf.setOrigin(tf::Vector3(pos.X(), pos.Y(), pos.Z()));
    tfBroadcaster->sendTransform(
        tf::StampedTransform(tf, ros::Time::now(), raySensor->ParentName(), raySensor->Name()));
}

void LivoxGpuPointsPlugin::convertDataToRotateInfo(const std::vector<std::vector<double>> &datas, std::vector<AviaRotateInfo> &avia_infos) {
    avia_infos.reserve(datas.size());
    double deg_2_rad = M_PI / 180.0;
    for (auto &data : datas) {
        if (data.size() == 3) {
            avia_infos.emplace_back();
            avia_infos.back().time = data[0];
            avia_infos.back().azimuth = data[1] * deg_2_rad;
            avia_infos.back().zenith = data[2] * deg_2_rad - M_PI_2;  //转化成标准的右手系角度
        } else {
            ROS_INFO_STREAM("data size is not 3!");
        }
    }
}

void LivoxGpuPointsPlugin::OnNewLaserScans(ConstLaserScanStampedPtr& _msg)
{
  ROS_ERROR_STREAM("OnScan called");
  const ignition::math::Angle maxAngle = raySensor->AngleMax();
  const ignition::math::Angle minAngle = raySensor->AngleMin();
  const std::vector<std::vector<double>>* angles = raySensor->getDatas();
  const double maxRange = raySensor->RangeMax();
  const double minRange = raySensor->RangeMin();

  const int rayCount = raySensor->RayCount();
  const int rangeCount = raySensor->RangeCount();

  const int verticalRayCount = raySensor->VerticalRayCount();
  const int verticalRangeCount = raySensor->VerticalRangeCount();

  const ignition::math::Angle verticalMaxAngle = raySensor->VerticalAngleMax();
  const ignition::math::Angle verticalMinAngle = raySensor->VerticalAngleMin();

  const double yDiff = maxAngle.Radian() - minAngle.Radian();
  const double pDiff = verticalMaxAngle.Radian() - verticalMinAngle.Radian();

  const double MIN_RANGE = minRange; //std::max(min_range_, minRange);
  const double MAX_RANGE = maxRange; //std::min(max_range_, maxRange);
  const double MIN_INTENSITY = 0.0; //min_intensity_;
  constexpr bool organize_cloud_ {false};
  // Populate message fields
  const uint32_t POINT_STEP = 22;
  sensor_msgs::PointCloud2 msg;
  msg.header.frame_id = raySensor->Name();
  msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
  msg.fields.resize(6);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[2].count = 1;
  msg.fields[3].name = "intensity";
  msg.fields[3].offset = 12;
  msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[3].count = 1;
  msg.fields[4].name = "ring";
  msg.fields[4].offset = 16;
  msg.fields[4].datatype = sensor_msgs::PointField::UINT16;
  msg.fields[4].count = 1;
  msg.fields[5].name = "time";
  msg.fields[5].offset = 18;
  msg.fields[5].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[5].count = 1;
  msg.data.resize(20000 * POINT_STEP); //verticalRangeCount * rangeCount * POINT_STEP);

#if 1
  int i, j;
  uint8_t *ptr = msg.data.data();
  unsigned int datasCount {0};
  ROS_ERROR_STREAM("Gazebo Plugin DatasCount " << datasCount << "\n");
  for (i = 0; i < 20000; i++) {

      if (datasCount + i >= angles->size() - 1)
      {
        datasCount = 0;
      }
    //for (j = 0; j < verticalRangeCount; j++) {

      // Range
      double r = _msg->scan().ranges(i); // + j * rangeCount);
      // Intensity
      double intensity = _msg->scan().intensities(i);// + j * rangeCount);
      // Ignore points that lay outside range bands or optionally, beneath a
      // minimum intensity level.
      /*if ((MIN_RANGE >= r) || (r >= MAX_RANGE) || (intensity < MIN_INTENSITY)) {
        if (!organize_cloud_) {
          continue;
        }
      }*/

      // Noise
      /*if (gaussian_noise_ != 0.0) {
        r += 0.0; //gaussianKernel(0,gaussian_noise_);
      }*/

      // TODO: Georg: Get angles from csv file?
      // Get angles of ray to get xyz for point
      double yAngle;
      double pAngle;
      constexpr double deg_2_rad = M_PI / 180.0;
      yAngle = angles->at(datasCount+i)[1] * deg_2_rad;
      pAngle = angles->at(datasCount+i)[2] * deg_2_rad - M_PI_2;
      /*if (rangeCount > 1) {
        yAngle = i * yDiff / (rangeCount -1) + minAngle.Radian();
      } else {
        yAngle = minAngle.Radian();
      }

      if (verticalRayCount > 1) {
        pAngle = j * pDiff / (verticalRangeCount -1) + verticalMinAngle.Radian();
      } else {
        pAngle = verticalMinAngle.Radian();
      }*/

      // pAngle is rotated by yAngle:
      if ((MIN_RANGE < r) && (r < MAX_RANGE)) {
        *((float*)(ptr + 0)) = r * cos(pAngle) * cos(yAngle); // x
        *((float*)(ptr + 4)) = r * cos(pAngle) * sin(yAngle); // y
        *((float*)(ptr + 8)) = r * sin(pAngle); // z
        *((float*)(ptr + 12)) = intensity; // intensity
        *((uint16_t*)(ptr + 16)) = j; // ring
        *((float*)(ptr + 18)) = 0.0; // time
        ptr += POINT_STEP;
      } else if (organize_cloud_) {
        *((float*)(ptr + 0)) = nanf(""); // x
        *((float*)(ptr + 4)) = nanf(""); // y
        *((float*)(ptr + 8)) = nanf(""); // x
        *((float*)(ptr + 12)) = nanf(""); // intensity
        *((uint16_t*)(ptr + 16)) = j; // ring
        *((float*)(ptr + 18)) = 0.0; // time
        ptr += POINT_STEP;
      }
    //}
  }
  //datasCount = datasCount + i;
  // Populate message with number of valid points
  msg.data.resize(ptr - msg.data.data()); // Shrink to actual size
  msg.point_step = POINT_STEP;
  msg.is_bigendian = false;
  if (organize_cloud_) {
    msg.width = verticalRangeCount;
    msg.height = msg.data.size() / POINT_STEP / msg.width;
    msg.row_step = POINT_STEP * msg.width;
    msg.is_dense = false;
  } else {
    msg.width = msg.data.size() / POINT_STEP;
    msg.height = 1;
    msg.row_step = msg.data.size();
    msg.is_dense = true;
  }
#endif
  // Publish output
  pub_.publish(msg);
}

}
