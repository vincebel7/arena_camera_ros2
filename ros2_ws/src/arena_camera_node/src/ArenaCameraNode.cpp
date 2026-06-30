#include <algorithm>  // std::max (ptp-lock attempt count)
#include <cstring>    // memcopy
#include <stdexcept>  // std::runtime_err
#include <string>

// ROS
#include "rmw/types.h"

// ArenaSDK
#include "ArenaCameraNode.h"
#include "light_arena/deviceinfo_helper.h"
#include "rclcpp_adapter/pixelformat_translation.h"
#include "rclcpp_adapter/quilty_of_service_translation.cpp"

void ArenaCameraNode::parse_parameters_()
{
  std::string nextParameterToDeclare = "";
  try {
    int serial_integer = this->declare_parameter<int>("serial", 0);
    serial_ = std::to_string(serial_integer);
    is_passed_serial_ = serial_integer != 0;
      
    nextParameterToDeclare = "pixelformat";
    pixelformat_ros_ = this->declare_parameter("pixelformat", "");
    is_passed_pixelformat_ros_ = pixelformat_ros_ != "";

    nextParameterToDeclare = "width";
    width_ = this->declare_parameter("width", 0);
    is_passed_width = width_ > 0;

    nextParameterToDeclare = "height";
    height_ = this->declare_parameter("height", 0);
    is_passed_height = height_ > 0;

    nextParameterToDeclare = "gain";
    gain_ = this->declare_parameter("gain", -1.0);
    is_passed_gain_ = gain_ >= 0;

    nextParameterToDeclare = "exposure_time";
    exposure_time_ = this->declare_parameter("exposure_time", -1.0);
    is_passed_exposure_time_ = exposure_time_ >= 0;

    nextParameterToDeclare = "target_brightness";
    target_brightness_ = this->declare_parameter("target_brightness", -1);
    is_passed_target_brightness_ = target_brightness_ >= 0;

    nextParameterToDeclare = "gamma";
    gamma_ = this->declare_parameter("gamma", -1.0);
    is_passed_gamma_ = gamma_ >= 0;

    // Image flip (matches the ReverseX/ReverseY toggles in ArenaView).
    nextParameterToDeclare = "reverse_x";
    reverse_x_ = this->declare_parameter("reverse_x", false);

    nextParameterToDeclare = "reverse_y";
    reverse_y_ = this->declare_parameter("reverse_y", false);

    nextParameterToDeclare = "trigger_mode";
    trigger_mode_activated_ = this->declare_parameter("trigger_mode", false);
    // no need to is_passed_trigger_mode_ because it is already a boolean

    nextParameterToDeclare = "frame_rate";
    frame_rate_ = this->declare_parameter("frame_rate", 30.0);

    // -- synchronized (PTP + scheduled GigE Vision Action Command) triggering --
    // When trigger_mode is true the cameras capture on TriggerSource=Action0.
    // action_master designates the single node that broadcasts the action
    // command. The device/group key/mask MUST be identical on all six cameras
    // and on the fire command.
    nextParameterToDeclare = "action_master";
    action_master_ = this->declare_parameter("action_master", false);

    nextParameterToDeclare = "action_device_key";
    action_device_key_ =
        this->declare_parameter<int64_t>("action_device_key", 1);

    nextParameterToDeclare = "action_group_key";
    action_group_key_ = this->declare_parameter<int64_t>("action_group_key", 1);

    nextParameterToDeclare = "action_group_mask";
    action_group_mask_ =
        this->declare_parameter<int64_t>("action_group_mask", 1);

    nextParameterToDeclare = "action_lead_time";
    action_lead_time_ = this->declare_parameter("action_lead_time", 0.05);

    nextParameterToDeclare = "action_trigger_rate";
    action_trigger_rate_ = this->declare_parameter("action_trigger_rate", 0.0);

    nextParameterToDeclare = "action_get_image_timeout";
    action_get_image_timeout_ms_ =
        this->declare_parameter<int64_t>("action_get_image_timeout", 1000);

    nextParameterToDeclare = "ptp_domain";
    ptp_domain_ = this->declare_parameter<int64_t>("ptp_domain", 0);

    nextParameterToDeclare = "ptp_lock_timeout";
    ptp_lock_timeout_sec_ = this->declare_parameter("ptp_lock_timeout", 60.0);

    // GigE Vision transmission shaping. gev_scftd staggers per-camera frame
    // transmission so synchronized frames do not all arrive at the host at once
    // (helps high-rate multi-camera recording); gev_scpd spaces out packets.
    // 0 = leave the camera default. These affect transmission only, not capture.
    nextParameterToDeclare = "gev_scpd";
    gev_scpd_ = this->declare_parameter<int64_t>("gev_scpd", 0);

    nextParameterToDeclare = "gev_scftd";
    gev_scftd_ = this->declare_parameter<int64_t>("gev_scftd", 0);

    nextParameterToDeclare = "camera_name";
    camera_name_ = this->declare_parameter("camera_name", "arena_camera");
    // no need to is_passed_camera_name_

    nextParameterToDeclare = "topic";
    topic_ = this->declare_parameter(
        "topic", std::string("/") + this->get_name() + "/images");
    // no need to is_passed_topic_
    
    nextParameterToDeclare = "qos_history";
    pub_qos_history_ = this->declare_parameter("qos_history", "");
    is_passed_pub_qos_history_ = pub_qos_history_ != "";

    nextParameterToDeclare = "qos_history_depth";
    pub_qos_history_depth_ = this->declare_parameter("qos_history_depth", 0);
    is_passed_pub_qos_history_depth_ = pub_qos_history_depth_ > 0;

    nextParameterToDeclare = "qos_reliability";
    pub_qos_reliability_ = this->declare_parameter("qos_reliability", "");
    is_passed_pub_qos_reliability_ = pub_qos_reliability_ != "";

  } catch (rclcpp::ParameterTypeException& e) {
    log_err(nextParameterToDeclare + " argument");
    throw;
  }
}

void ArenaCameraNode::initialize_()
{
  using namespace std::chrono_literals;
  // ARENASDK ---------------------------------------------------------------
  // Custom deleter for system
  m_pSystem =
      std::shared_ptr<Arena::ISystem>(nullptr, [=](Arena::ISystem* pSystem) {
        if (pSystem) {  // this is an issue for multi devices
          Arena::CloseSystem(pSystem);
          log_info("System is destroyed");
        }
      });
  m_pSystem.reset(Arena::OpenSystem());

  // Custom deleter for device
  m_pDevice =
      std::shared_ptr<Arena::IDevice>(nullptr, [=](Arena::IDevice* pDevice) {
        if (m_pSystem && pDevice) {
          m_pSystem->DestroyDevice(pDevice);
          log_info("Device is destroyed");
        }
      });

  //
  // CHECK DEVICE CONNECTION ( timer ) --------------------------------------
  //
  // TODO
  // - Think of design that allow the node to start stream as soon as
  // it is initialized without waiting for spin to be called
  // - maybe change 1s to a smaller value
  m_wait_for_device_timer_callback_ = this->create_wall_timer(
      1s, std::bind(&ArenaCameraNode::wait_for_device_timer_callback_, this));

  //
  // TRIGGER (service) ------------------------------------------------------
  //
  using namespace std::placeholders;
  m_trigger_an_image_srv_ = this->create_service<std_srvs::srv::Trigger>(
      std::string(this->get_name()) + "/trigger_image",
      std::bind(&ArenaCameraNode::publish_an_image_on_trigger_, this, _1, _2));

  //
  // TRIGGER ALL (service) --------------------------------------------------
  //
  // Only the designated action master exposes /trigger_all. The name is
  // absolute (leading '/') so a single node owns it; the broadcast action
  // command it sends reaches every camera. Creating it only on the master
  // avoids the collision that would happen if all six nodes registered it.
  if (trigger_mode_activated_ && action_master_) {
    m_trigger_all_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/trigger_all",
        std::bind(&ArenaCameraNode::trigger_all_callback_, this, _1, _2));
    log_info(
        "\taction master: /trigger_all service is ready (fires one "
        "synchronized shot across all cameras)");
  }

  //
  // Publisher --------------------------------------------------------------
  //
  // m_pub_qos is rclcpp::SensorDataQoS has these defaults
  // https://github.com/ros2/rmw/blob/fb06b57975373b5a23691bb00eb39c07f1660ed7/rmw/include/rmw/qos_profiles.h#L25

  /*
  static const rmw_qos_profile_t rmw_qos_profile_sensor_data =
  {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    5, // history depth
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false // avoid ros namespace conventions
  };
  */
  rclcpp::SensorDataQoS pub_qos_;
  // QoS history
  if (is_passed_pub_qos_history_) {
    if (is_supported_qos_histroy_policy(pub_qos_history_)) {
      pub_qos_.history(
          K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY[pub_qos_history_]);
    } else {
      log_err(pub_qos_history_ + " is not supported for this node");
      // TODO
      // should thorow instead??
      // should this keeps shutting down if for some reasons this node is kept
      // alive
      throw;
    }
  }
  // QoS depth
  if (is_passed_pub_qos_history_depth_ &&
      K_CMDLN_PARAMETER_TO_QOS_HISTORY_POLICY[pub_qos_history_] ==
          RMW_QOS_POLICY_HISTORY_KEEP_LAST) {
    // TODO
    // test err msg withwhen -1
    pub_qos_.keep_last(pub_qos_history_depth_);
  }

  // Qos reliability
  if (is_passed_pub_qos_reliability_) {
    if (is_supported_qos_reliability_policy(pub_qos_reliability_)) {
      pub_qos_.reliability(
          K_CMDLN_PARAMETER_TO_QOS_RELIABILITY_POLICY[pub_qos_reliability_]);
    } else {
      log_err(pub_qos_reliability_ + " is not supported for this node");
      throw;
    }
  }

  // rmw_qos_history_policy_t history_policy_ = RMW_QOS_
  // rmw_qos_history_policy_t;
  // auto pub_qos_init = rclcpp::QoSInitialization(history_policy_, );

  m_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      this->get_parameter("topic").as_string(), pub_qos_);

  std::stringstream pub_qos_info;
  auto pub_qos_profile = pub_qos_.get_rmw_qos_profile();
  pub_qos_info
      << '\t' << "QoS history     = "
      << K_QOS_HISTORY_POLICY_TO_CMDLN_PARAMETER[pub_qos_profile.history]
      << '\n';
  pub_qos_info << "\t\t\t\t"
               << "QoS depth       = " << pub_qos_profile.depth << '\n';
  pub_qos_info << "\t\t\t\t"
               << "QoS reliability = "
               << K_QOS_RELIABILITY_POLICY_TO_CMDLN_PARAMETER[pub_qos_profile
                                                                  .reliability]
               << '\n';

  log_info(pub_qos_info.str());
}

void ArenaCameraNode::wait_for_device_timer_callback_()
{
  // something happend while checking for cameras
  if (!rclcpp::ok()) {
    log_err("Interrupted while waiting for arena camera. Exiting.");
    rclcpp::shutdown();
  }

  // camera discovery
  m_pSystem->UpdateDevices(100);  // in millisec
  auto device_infos = m_pSystem->GetDevices();

  // no camera is connected
  if (!device_infos.size()) {
    log_info("No arena camera is connected. Waiting for device(s)...");
  }
  // at least on is found
  else {
    m_wait_for_device_timer_callback_->cancel();
    log_info(std::to_string(device_infos.size()) +
             " arena device(s) has been discoved.");
    run_();
  }
}

void ArenaCameraNode::run_()
{
  auto device = create_device_ros_();
  m_pDevice.reset(device);
  set_nodes_();
  m_pDevice->StartStream();

  if (is_passed_target_brightness_) {
    Arena::SetNodeValue<GenICam::gcstring>(m_pDevice->GetNodeMap(), "ExposureAuto", "Once");
    log_info("\tExposureAuto set to Once (will lock after convergence)");
  }

  if (!trigger_mode_activated_) {
    // free-run: blocking publish loop, exactly as before.
    publish_images_();
  } else {
    // trigger_mode=true: synchronized capture via PTP + scheduled action
    // commands. Frames arrive only when a scheduled action command fires, so
    // run the GetImage/publish loop in a background thread; the node keeps
    // spinning to serve /trigger_all (master) and the continuous timer.
    m_stop_acquisition_ = false;
    m_acquisition_thread_ =
        std::thread(&ArenaCameraNode::publish_images_action_, this);

    if (action_master_ && action_trigger_rate_ > 0.0) {
      auto period_ns = std::chrono::nanoseconds(
          static_cast<int64_t>(1e9 / action_trigger_rate_));
      m_continuous_trigger_timer_ = this->create_wall_timer(
          period_ns,
          std::bind(&ArenaCameraNode::continuous_trigger_timer_callback_,
                    this));
      log_info(std::string("\tcontinuous action triggering enabled at ") +
               std::to_string(action_trigger_rate_) + " Hz");
    }
  }
}

void ArenaCameraNode::publish_images_()
{
  Arena::IImage* pImage = nullptr;
  while (rclcpp::ok()) {
    try {
      auto p_image_msg = std::make_unique<sensor_msgs::msg::Image>();
      pImage = m_pDevice->GetImage(1000);
      msg_form_image_(pImage, *p_image_msg);

      m_pub_->publish(std::move(p_image_msg));

      log_info(std::string("image ") + std::to_string(pImage->GetFrameId()) +
               " published to " + topic_);
      this->m_pDevice->RequeueBuffer(pImage);
      pImage = nullptr;

    } catch (GenICam::GenericException& e) {
      if (pImage) {
        this->m_pDevice->RequeueBuffer(pImage);
        pImage = nullptr;
      }
      log_warn(std::string("GenICam exception while publishing an image\n") +
               e.what());
    } catch (std::exception& e) {
      if (pImage) {
        this->m_pDevice->RequeueBuffer(pImage);
        pImage = nullptr;
      }
      log_warn(std::string("Exception occurred while publishing an image\n") +
               e.what());
    }
  };
}

void ArenaCameraNode::publish_images_action_()
{
  // Action-mode acquisition loop. Frames arrive only when a scheduled action
  // command fires, so GetImage() routinely times out while idle; those
  // timeouts are expected and swallowed quietly to avoid log spam.
  Arena::IImage* pImage = nullptr;
  while (rclcpp::ok() && !m_stop_acquisition_) {
    try {
      pImage = m_pDevice->GetImage(
          static_cast<uint64_t>(action_get_image_timeout_ms_));
      auto p_image_msg = std::make_unique<sensor_msgs::msg::Image>();
      msg_form_image_(pImage, *p_image_msg);
      m_pub_->publish(std::move(p_image_msg));
      log_info(std::string("image ") + std::to_string(pImage->GetFrameId()) +
               " published to " + topic_);
      m_pDevice->RequeueBuffer(pImage);
      pImage = nullptr;
    } catch (GenICam::TimeoutException&) {
      // [verify] GenICam::TimeoutException is the standard Arena/LUCID type
      // thrown by GetImage() on timeout. Expected between triggers -> keep
      // waiting quietly. No buffer was handed out, so nothing to requeue.
      continue;
    } catch (GenICam::GenericException& e) {
      if (pImage) {
        m_pDevice->RequeueBuffer(pImage);
        pImage = nullptr;
      }
      log_warn(std::string("GenICam exception in action acquisition loop\n") +
               e.what());
    } catch (std::exception& e) {
      if (pImage) {
        m_pDevice->RequeueBuffer(pImage);
        pImage = nullptr;
      }
      log_warn(std::string("Exception in action acquisition loop\n") +
               e.what());
    }
  }
}

bool ArenaCameraNode::wait_for_ptp_lock_()
{
  if (m_ptp_locked_) {
    return true;
  }
  auto nodemap = m_pDevice->GetNodeMap();

  // "Settled" is approximated by a sustained run of consecutive "Slave" reads
  // (PTP can briefly report Slave before the offset converges).
  const int required_consecutive = 5;
  const auto poll_interval = std::chrono::milliseconds(500);
  const int max_attempts =
      std::max(1, static_cast<int>((ptp_lock_timeout_sec_ * 1000.0) /
                                   static_cast<double>(poll_interval.count())));

  int consecutive = 0;
  for (int attempt = 0; attempt < max_attempts && rclcpp::ok(); ++attempt) {
    std::string status;
    try {
      // [verify] PtpStatus enum string "Slave" on the vehicle SDK.
      status = Arena::GetNodeValue<GenICam::gcstring>(nodemap, "PtpStatus");
    } catch (GenICam::GenericException& e) {
      log_warn(std::string("could not read PtpStatus: ") + e.what());
    }

    if (status == "Slave") {
      if (++consecutive >= required_consecutive) {
        m_ptp_locked_ = true;
        log_info(
            "PTP locked: this camera reports 'Slave' (synchronized to the RTK "
            "grandmaster)");
        return true;
      }
    } else {
      consecutive = 0;
      log_info(std::string("waiting for PTP lock; PtpStatus=") + status);
    }
    std::this_thread::sleep_for(poll_interval);
  }

  log_err(
      "PTP did not settle on 'Slave' within ptp_lock_timeout; refusing to fire "
      "the action command. Check the grandmaster (RTK), the PTP domain, and "
      "the switch.");
  return false;
}

void ArenaCameraNode::fire_scheduled_action_command_(bool single_shot)
{
  std::lock_guard<std::mutex> lock(m_fire_mutex_);

  auto deviceNodeMap = m_pDevice->GetNodeMap();
  auto systemNodeMap = m_pSystem->GetTLSystemNodeMap();

  // Latch this device's current PTP time (disciplined to the RTK GPS
  // grandmaster) and read it back in nanoseconds.
  Arena::ExecuteNode(deviceNodeMap, "PtpDataSetLatch");
  const int64_t curr_ptp =
      Arena::GetNodeValue<int64_t>(deviceNodeMap, "PtpDataSetLatchValue");

  const int64_t lead_ns = static_cast<int64_t>(action_lead_time_ * 1e9);
  const int64_t one_second_ns = 1000000000LL;

  int64_t exec_time_ns;
  if (single_shot) {
    // Round the latched time UP to the next whole second, then add the lead so
    // even a latch landing just before a second boundary keeps full margin.
    const int64_t next_second =
        ((curr_ptp / one_second_ns) + 1) * one_second_ns;
    exec_time_ns = next_second + lead_ns;
  } else {
    // Continuous: schedule just `lead` ahead of the freshly latched time so at
    // most ONE scheduled action command is pending at the camera. Cameras hold
    // a single pending scheduled action, so a large lead (e.g. rounding to the
    // next whole second) backlogs commands that then get dropped, throttling
    // the rate. Inter-frame spacing follows the trigger timer period; every
    // shot is still PTP-synchronized across all cameras (they all receive the
    // same single broadcast with the same execute time).
    //
    // lead must exceed the worst-case latch+fire+network latency or the command
    // arrives after its execute time and is ignored. If you see dropped frames
    // at high rates, raise action_lead_time (keeping it below 1/rate).
    exec_time_ns = curr_ptp + lead_ns;
  }

  // Configure and fire the broadcast action command on the system nodemap.
  // [verify] the six ActionCommand* system-nodemap node names on the vehicle
  // SDK. The keys/mask MUST match what every camera was configured with.
  Arena::SetNodeValue<int64_t>(systemNodeMap, "ActionCommandDeviceKey",
                               action_device_key_);
  Arena::SetNodeValue<int64_t>(systemNodeMap, "ActionCommandGroupKey",
                               action_group_key_);
  Arena::SetNodeValue<int64_t>(systemNodeMap, "ActionCommandGroupMask",
                               action_group_mask_);
  // Broadcast to every camera on the subnet.
  Arena::SetNodeValue<int64_t>(systemNodeMap, "ActionCommandTargetIP",
                               static_cast<int64_t>(0xFFFFFFFF));
  Arena::SetNodeValue<int64_t>(systemNodeMap, "ActionCommandExecuteTime",
                               exec_time_ns);
  Arena::ExecuteNode(systemNodeMap, "ActionCommandFireCommand");

  log_info(std::string("fired scheduled action command: exec_time(ns, RTK)=") +
           std::to_string(exec_time_ns) +
           " latched_ptp(ns)=" + std::to_string(curr_ptp) +
           (single_shot ? " [single shot]" : " [continuous]"));
}

void ArenaCameraNode::trigger_all_callback_(
    std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!trigger_mode_activated_ || !action_master_) {
    response->success = false;
    response->message =
        "this node is not the action master (needs trigger_mode:=true and "
        "action_master:=true)";
    log_warn(response->message);
    return;
  }

  if (!wait_for_ptp_lock_()) {
    response->success = false;
    response->message = "PTP is not locked; refusing to fire";
    return;
  }

  try {
    fire_scheduled_action_command_(true);
    response->success = true;
    response->message =
        "scheduled action command fired (single synchronized shot)";
    log_info(response->message);
  } catch (GenICam::GenericException& e) {
    response->success = false;
    response->message =
        std::string("GenICam exception while firing action command: ") +
        e.what();
    log_warn(response->message);
  } catch (std::exception& e) {
    response->success = false;
    response->message =
        std::string("exception while firing action command: ") + e.what();
    log_warn(response->message);
  }
}

void ArenaCameraNode::continuous_trigger_timer_callback_()
{
  if (!wait_for_ptp_lock_()) {
    log_warn("continuous trigger: PTP not locked yet; skipping this tick");
    return;
  }
  try {
    fire_scheduled_action_command_(false);
  } catch (GenICam::GenericException& e) {
    log_warn(std::string("continuous fire GenICam exception: ") + e.what());
  } catch (std::exception& e) {
    log_warn(std::string("continuous fire exception: ") + e.what());
  }
}

void ArenaCameraNode::msg_form_image_(Arena::IImage* pImage,
                                      sensor_msgs::msg::Image& image_msg)
{
  try {
    // 1 ) Header
    //      - stamp.sec
    //      - stamp.nanosec
    //      - Frame ID
    image_msg.header.stamp.sec =
        static_cast<uint32_t>(pImage->GetTimestampNs() / 1000000000);
    image_msg.header.stamp.nanosec =
        static_cast<uint32_t>(pImage->GetTimestampNs() % 1000000000);
    image_msg.header.frame_id = camera_name_;

    //
    // 2 ) Height
    //
    image_msg.height = height_;

    //
    // 3 ) Width
    //
    image_msg.width = width_;

    //
    // 4 ) encoding
    //
    image_msg.encoding = pixelformat_ros_;

    //
    // 5 ) is_big_endian
    //
    // TODO what to do if unknown
    image_msg.is_bigendian = pImage->GetPixelEndianness() ==
                             Arena::EPixelEndianness::PixelEndiannessBig;
    //
    // 6 ) step
    //
    // TODO could be optimized by moving it out
    auto pixel_length_in_bytes = pImage->GetBitsPerPixel() / 8;
    auto width_length_in_bytes = pImage->GetWidth() * pixel_length_in_bytes;
    image_msg.step =
        static_cast<sensor_msgs::msg::Image::_step_type>(width_length_in_bytes);

    //
    // 7) data
    //
    auto image_data_length_in_bytes = width_length_in_bytes * height_;
    image_msg.data.resize(image_data_length_in_bytes);
    auto x = pImage->GetData();
    std::memcpy(&image_msg.data[0], pImage->GetData(),
                image_data_length_in_bytes);

  } catch (...) {
    log_warn(
        "Failed to create Image ROS MSG. Published Image Msg might be "
        "corrupted");
  }
}

void ArenaCameraNode::publish_an_image_on_trigger_(
    std::shared_ptr<std_srvs::srv::Trigger::Request> request /*unused*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!trigger_mode_activated_) {
    std::string msg =
        "Failed to trigger image because the device is not in trigger mode."
        "run `ros2 run arena_camera_node run --ros-args -p trigger_mode:=true`";
    log_warn(msg);
    response->message = msg;
    response->success = false;
  }

  log_info("A client triggered an image request");

  Arena::IImage* pImage = nullptr;
  try {
    // trigger
    bool triggerArmed = false;
    auto waitForTriggerCount = 10;
    do {
      // infinite loop when I step in (sometimes)
      triggerArmed =
          Arena::GetNodeValue<bool>(m_pDevice->GetNodeMap(), "TriggerArmed");

      if (triggerArmed == false && (waitForTriggerCount % 10) == 0) {
        log_info("waiting for trigger to be armed");
      }

    } while (triggerArmed == false);

    log_debug("trigger is armed; triggering an image");
    Arena::ExecuteNode(m_pDevice->GetNodeMap(), "TriggerSoftware");

    // get image
    auto p_image_msg = std::make_unique<sensor_msgs::msg::Image>();

    log_debug("getting an image");
    pImage = m_pDevice->GetImage(1000);
    auto msg = std::string("image ") + std::to_string(pImage->GetFrameId()) +
               " published to " + topic_;
    msg_form_image_(pImage, *p_image_msg);
    m_pub_->publish(std::move(p_image_msg));
    response->message = msg;
    response->success = true;

    log_info(msg);
    this->m_pDevice->RequeueBuffer(pImage);

  }

  catch (std::exception& e) {
    if (pImage) {
      this->m_pDevice->RequeueBuffer(pImage);
      pImage = nullptr;
    }
    auto msg =
        std::string("Exception occurred while grabbing an image\n") + e.what();
    log_warn(msg);
    response->message = msg;
    response->success = false;

  }

  catch (GenICam::GenericException& e) {
    if (pImage) {
      this->m_pDevice->RequeueBuffer(pImage);
      pImage = nullptr;
    }
    auto msg =
        std::string("GenICam Exception occurred while grabbing an image\n") +
        e.what();
    log_warn(msg);
    response->message = msg;
    response->success = false;
  }
}

Arena::IDevice* ArenaCameraNode::create_device_ros_()
{
  m_pSystem->UpdateDevices(100);  // in millisec
  auto device_infos = m_pSystem->GetDevices();
  if (!device_infos.size()) {
    // TODO: handel disconnection
    throw std::runtime_error(
        "camera(s) were disconnected after they were discovered");
  }

  auto index = 0;
  if (is_passed_serial_) {
    index = DeviceInfoHelper::get_index_of_serial(device_infos, serial_);
  }

  auto pDevice = m_pSystem->CreateDevice(device_infos.at(index));
  log_info(std::string("device created ") +
           DeviceInfoHelper::info(device_infos.at(index)));
  return pDevice;
}

void ArenaCameraNode::set_nodes_()
{
  set_nodes_load_default_profile_();
  set_nodes_roi_();
  set_nodes_gain_();
  // Apply the flip BEFORE PixelFormat: flipping changes the effective Bayer
  // order, so the camera must already be reversed for the matching Bayer
  // PixelFormat (e.g. bayer_bggr8 for a 180 flip of an RGGB sensor) to be valid.
  set_nodes_reverse_();
  set_nodes_pixelformat_();
  set_nodes_exposure_();
  set_nodes_target_brightness_();
  set_nodes_gamma_();
  if (trigger_mode_activated_) {
    // trigger_mode=true: synchronized capture via PTP + scheduled GigE Vision
    // action commands. The action command gates every frame, so the free-run
    // AcquisitionFrameRate is intentionally NOT enabled here (see
    // set_nodes_action_trigger_mode_()).
    set_nodes_action_trigger_mode_();
    set_nodes_ptp_();
  } else {
    // trigger_mode=false: free-run, exactly as before.
    set_nodes_trigger_mode_();
    set_nodes_ptp_();
    set_nodes_frame_rate_();
  }
  // configure Auto Negotiate Packet Size and Packet Resend
  Arena::SetNodeValue<bool>(m_pDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
  Arena::SetNodeValue<bool>(m_pDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);
  set_nodes_transmission_delay_();

  //set_nodes_test_pattern_image_();
}

void ArenaCameraNode::set_nodes_load_default_profile_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  // device run on default profile all the time if no args are passed
  // otherwise, overwise only these params
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "UserSetSelector", "Default");
  // execute the profile
  Arena::ExecuteNode(nodemap, "UserSetLoad");
  log_info("\tdefault profile is loaded");
}

void ArenaCameraNode::set_nodes_roi_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  // Width -------------------------------------------------
  if (is_passed_width) {
    Arena::SetNodeValue<int64_t>(nodemap, "Width", width_);
  } else {
    width_ = Arena::GetNodeValue<int64_t>(nodemap, "Width");
  }

  // Height ------------------------------------------------
  if (is_passed_height) {
    Arena::SetNodeValue<int64_t>(nodemap, "Height", height_);
  } else {
    height_ = Arena::GetNodeValue<int64_t>(nodemap, "Height");
  }

  // TODO only if it was passed by ros arg
  log_info(std::string("\tROI set to ") + std::to_string(width_) + "X" +
           std::to_string(height_));
}

void ArenaCameraNode::set_nodes_gain_()
{
  if (is_passed_gain_) {  // not default
    auto nodemap = m_pDevice->GetNodeMap();
    Arena::SetNodeValue<double>(nodemap, "Gain", gain_);
    log_info(std::string("\tGain set to ") + std::to_string(gain_));
  }
}

void ArenaCameraNode::set_nodes_pixelformat_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  // TODO ---------------------------------------------------------------------
  // PIXEL FORMAT HANDLEING

  if (is_passed_pixelformat_ros_) {
    pixelformat_pfnc_ = K_ROS2_PIXELFORMAT_TO_PFNC[pixelformat_ros_];
    if (pixelformat_pfnc_.empty()) {
      throw std::invalid_argument("pixelformat is not supported!");
    }

    try {
      Arena::SetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat",
                                             pixelformat_pfnc_.c_str());
      log_info(std::string("\tPixelFormat set to ") + pixelformat_pfnc_);

    } catch (GenICam::GenericException& e) {
      // TODO
      // an rcl expectation might be expected
      auto x = std::string("pixelformat is not supported by this camera");
      x.append(e.what());
      throw std::invalid_argument(x);
    }
  } else {
    pixelformat_pfnc_ =
        Arena::GetNodeValue<GenICam::gcstring>(nodemap, "PixelFormat");
    pixelformat_ros_ = K_PFNC_TO_ROS2_PIXELFORMAT[pixelformat_pfnc_];

    if (pixelformat_ros_.empty()) {
      log_warn(
          "the device current pixelfromat value is not supported by ROS2. "
          "please use --ros-args -p pixelformat:=\"<supported pixelformat>\".");
      // TODO
      // print list of supported pixelformats
    }
  }
}

void ArenaCameraNode::set_nodes_exposure_()
{
  if (is_passed_exposure_time_) {
    auto nodemap = m_pDevice->GetNodeMap();
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAuto", "Off");
    Arena::SetNodeValue<double>(nodemap, "ExposureTime", exposure_time_);
  }
}

void ArenaCameraNode::set_nodes_target_brightness_()
{
  if (is_passed_target_brightness_) {
    auto nodemap = m_pDevice->GetNodeMap();
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ExposureAuto", "Continuous");
    Arena::SetNodeValue<int64_t>(nodemap, "TargetBrightness", target_brightness_);
    log_info(std::string("\tTargetBrightness set to ") + std::to_string(target_brightness_));
  }
}

void ArenaCameraNode::set_nodes_gamma_()
{
  if (is_passed_gamma_) {
    auto nodemap = m_pDevice->GetNodeMap();
    Arena::SetNodeValue<double>(nodemap, "Gamma", gamma_);
    log_info(std::string("\tGamma set to ") + std::to_string(gamma_));
  }
}

void ArenaCameraNode::set_nodes_reverse_()
{
  // Flip the image in the camera (same as the ReverseX/ReverseY toggles in
  // ArenaView). Set unconditionally so the value is deterministic each launch
  // (defaults are false). Must be set before StartStream.
  auto nodemap = m_pDevice->GetNodeMap();
  Arena::SetNodeValue<bool>(nodemap, "ReverseX", reverse_x_);
  Arena::SetNodeValue<bool>(nodemap, "ReverseY", reverse_y_);
  log_info(std::string("\tReverseX=") + (reverse_x_ ? "true" : "false") +
           " ReverseY=" + (reverse_y_ ? "true" : "false"));
}

void ArenaCameraNode::set_nodes_trigger_mode_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  if (trigger_mode_activated_) {
    if (exposure_time_ < 0) {
      log_warn(
          "\tavoid long waits wating for triggered images by providing proper "
          "exposure_time.");
    }
    // Enable trigger mode before setting the source and selector
    // and before starting the stream. Trigger mode cannot be turned
    // on and off while the device is streaming.

    // Make sure Trigger Mode set to 'Off' after finishing this example
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "On");

    // Set the trigger source to software in order to trigger buffers
    // without the use of any additional hardware.
    // Lines of the GPIO can also be used to trigger.
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSource",
                                           "Software");
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSelector",
                                           "FrameStart");
    auto msg =
        std::string(
            "\ttrigger_mode is activated. To trigger an image run `ros2 run ") +
        this->get_name() + " trigger_image`";
    log_warn(msg);
  }
  // unset device from being in trigger mode if user did not pass trigger
  // mode parameter because the trigger nodes are not rest when loading
  // the user default profile
  else {
    Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "Off");
  }
}

void ArenaCameraNode::set_nodes_action_trigger_mode_()
{
  auto nodemap = m_pDevice->GetNodeMap();

  if (exposure_time_ < 0) {
    log_warn(
        "\ttrigger_mode (synchronized): no fixed exposure_time provided. A "
        "fixed (bounded) exposure is recommended for deterministic "
        "synchronized capture under triggering.");
  }

  // Trigger configuration: capture exactly one frame per scheduled action
  // command. Select the trigger first, then enable it and route it to Action0.
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSelector",
                                         "FrameStart");
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerMode", "On");
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TriggerSource", "Action0");

  // Action command configuration. These keys/mask MUST be identical on all six
  // cameras and on the fire command (see fire_scheduled_action_command_()).
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "ActionUnconditionalMode",
                                         "On");
  Arena::SetNodeValue<int64_t>(nodemap, "ActionSelector", 0);
  Arena::SetNodeValue<int64_t>(nodemap, "ActionDeviceKey", action_device_key_);
  Arena::SetNodeValue<int64_t>(nodemap, "ActionGroupKey", action_group_key_);
  Arena::SetNodeValue<int64_t>(nodemap, "ActionGroupMask", action_group_mask_);

  // The scheduled action command fully gates acquisition; do not also cap the
  // free-run acquisition frame rate (which could silently drop triggered
  // frames). Guarded because the node may be read-only in some device states.
  try {
    Arena::SetNodeValue<bool>(nodemap, "AcquisitionFrameRateEnable", false);
  } catch (GenICam::GenericException&) {
    // not fatal; leave the camera's default frame-rate gating in place
  }

  log_info(
      std::string(
          "\ttrigger_mode activated (synchronized): TriggerSource=Action0 "
          "(scheduled GigE Vision action commands). device_key=") +
      std::to_string(action_device_key_) +
      " group_key=" + std::to_string(action_group_key_) +
      " group_mask=" + std::to_string(action_group_mask_));
}

void ArenaCameraNode::set_nodes_ptp_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  Arena::SetNodeValue<bool>(nodemap, "PtpEnable", true);
  Arena::SetNodeValue<bool>(nodemap, "PtpSlaveOnly", true);
  // The PTP domain must match the grandmaster (OXTS RTK) and the switch.
  // Default (0) leaves the camera at its default assumption, so the original
  // behavior is unchanged. A non-zero value is attempted in a try/catch since
  // not every LUCID model exposes a writable domain node.
  if (ptp_domain_ != 0) {
    try {
      // [verify] node name on the vehicle's Arena SDK; many LUCID models
      // assume domain 0 and may not expose a writable PTP domain node.
      Arena::SetNodeValue<int64_t>(nodemap, "PtpDomainNumber", ptp_domain_);
      log_info(std::string("\tPTP domain set to ") +
               std::to_string(ptp_domain_));
    } catch (GenICam::GenericException& e) {
      log_warn(
          std::string(
              "\tcould not set PTP domain (camera may not expose a writable "
              "domain node); ensure RTK/switch/cameras all use the same PTP "
              "domain. ") +
          e.what());
    }
  }
  log_info("\tPTP enabled (slave-only mode)");
}

void ArenaCameraNode::set_nodes_frame_rate_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  Arena::SetNodeValue<bool>(nodemap, "AcquisitionFrameRateEnable", true);
  Arena::SetNodeValue<double>(nodemap, "AcquisitionFrameRate", frame_rate_);
  log_info(std::string("\tAcquisitionFrameRate set to ") + std::to_string(frame_rate_));
}

void ArenaCameraNode::set_nodes_transmission_delay_()
{
  // GigE Vision stream-channel transmission shaping. Staggering gev_scftd
  // per-camera spreads when each camera transmits its already-captured frame so
  // the six synchronized frames do not all arrive at the host at the same
  // instant (which overflows the receive buffers at high rates). This shapes
  // TRANSMISSION only: exposure/capture stays driven by the shared trigger, so
  // the PTP capture timestamp (header.stamp) and cross-camera sync are
  // unchanged. Defaults of 0 leave the camera at its normal behavior.
  //
  // [verify] GevSCPD / GevSCFTD node names, nodemap, and units on the vehicle
  // SDK (LUCID Arena typically exposes these on the device nodemap; units are
  // commonly nanoseconds). Guarded so a missing/read-only node is non-fatal.
  auto nodemap = m_pDevice->GetNodeMap();
  if (gev_scpd_ > 0) {
    try {
      Arena::SetNodeValue<int64_t>(nodemap, "GevSCPD", gev_scpd_);
      log_info(std::string("\tGevSCPD (packet delay) set to ") +
               std::to_string(gev_scpd_));
    } catch (GenICam::GenericException& e) {
      log_warn(std::string("\tcould not set GevSCPD: ") + e.what());
    }
  }
  if (gev_scftd_ > 0) {
    try {
      Arena::SetNodeValue<int64_t>(nodemap, "GevSCFTD", gev_scftd_);
      log_info(std::string("\tGevSCFTD (frame transmission delay) set to ") +
               std::to_string(gev_scftd_));
    } catch (GenICam::GenericException& e) {
      log_warn(std::string("\tcould not set GevSCFTD: ") + e.what());
    }
  }
}

// just for debugging
void ArenaCameraNode::set_nodes_test_pattern_image_()
{
  auto nodemap = m_pDevice->GetNodeMap();
  Arena::SetNodeValue<GenICam::gcstring>(nodemap, "TestPattern", "Pattern3");
}
