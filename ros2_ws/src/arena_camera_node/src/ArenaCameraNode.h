#pragma once

// TODO
// - remove m_ before private members
// - add const to member functions
// fix includes in all files
// - should we rclcpp::shutdown in construction instead
//

// std
#include <atomic>      // std::atomic (acquisition-thread / ptp-lock flags)
#include <chrono>      //chrono_literals
#include <cstdint>     // int64_t (action keys / ptp latch value)
#include <functional>  // std::bind , std::placeholders
#include <mutex>       // std::mutex (serialize action-command fires)
#include <thread>      // std::thread (action-mode acquisition loop)

// ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>           // WallTimer
#include <sensor_msgs/msg/image.hpp>  //image msg published
#include <std_srvs/srv/trigger.hpp>   // Trigger

// arena sdk
#include "ArenaApi.h"

class ArenaCameraNode : public rclcpp::Node
{
 public:
  ArenaCameraNode() : Node("arena_camera_node")
  {
    // set stdout buffer size for ROS defined size BUFSIZE
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    log_info(std::string("Creating \"") + this->get_name() + "\" node");
    parse_parameters_();
    initialize_();
    log_info(std::string("Created \"") + this->get_name() + "\" node");
  }

  ~ArenaCameraNode()
  {
    log_info("Destroying node");
    // Stop the action-mode acquisition loop (if running) before tearing the
    // stream down. join() returns once the loop's current GetImage() returns
    // or times out. For non-action nodes the thread is never started, so
    // joinable() is false and this is a no-op.
    m_stop_acquisition_ = true;
    if (m_acquisition_thread_.joinable()) {
      m_acquisition_thread_.join();
    }
    if (m_pDevice) {
      try {
        m_pDevice->StopStream();
      } catch (...) {}
      m_pSystem->DestroyDevice(m_pDevice.get());
      m_pDevice = nullptr;
    }
    if (m_pSystem) {
      Arena::CloseSystem(m_pSystem.get());
      m_pSystem = nullptr;
    }
  }

  void log_debug(std::string msg) { RCLCPP_DEBUG(this->get_logger(), msg.c_str()); };
  void log_info(std::string msg) { RCLCPP_INFO(this->get_logger(), msg.c_str()); };
  void log_warn(std::string msg) { RCLCPP_WARN(this->get_logger(), msg.c_str()); };
  void log_err(std::string msg) { RCLCPP_ERROR(this->get_logger(), msg.c_str()); };

 private:
  std::shared_ptr<Arena::ISystem> m_pSystem;
  std::shared_ptr<Arena::IDevice> m_pDevice;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub_;
  rclcpp::TimerBase::SharedPtr m_wait_for_device_timer_callback_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_trigger_an_image_srv_;

  // -- synchronized (PTP + scheduled GigE Vision Action Command) triggering --
  // Service exposed only on the designated action master; fires one
  // synchronized shot across all cameras. Absolute name "/trigger_all".
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_trigger_all_srv_;
  // Optional steady-rate continuous triggering (action master only).
  rclcpp::TimerBase::SharedPtr m_continuous_trigger_timer_;
  // Action-mode acquisition runs in its own thread so the node can still spin
  // and serve services while waiting for triggered frames.
  std::thread m_acquisition_thread_;
  std::atomic<bool> m_stop_acquisition_{false};
  // Latched once this device's PtpStatus has settled on "Slave".
  std::atomic<bool> m_ptp_locked_{false};
  // Serializes manual (/trigger_all) and continuous-timer fires.
  std::mutex m_fire_mutex_;

  std::string serial_;
  bool is_passed_serial_;

  std::string topic_;
  std::string camera_name_;

  size_t width_;
  bool is_passed_width;

  size_t height_;
  bool is_passed_height;

  double gain_;
  bool is_passed_gain_;

  double exposure_time_;
  bool is_passed_exposure_time_;

  int64_t target_brightness_;
  bool is_passed_target_brightness_;

  double gamma_;
  bool is_passed_gamma_;

  std::string pixelformat_pfnc_;
  std::string pixelformat_ros_;
  bool is_passed_pixelformat_ros_;

  bool trigger_mode_activated_;

  double frame_rate_;

  // -- params for synchronized action-command triggering (used when
  //    trigger_mode is true; see set_nodes_action_trigger_mode_()) --
  bool action_master_;                  // exactly one node fires the broadcast
  int64_t action_device_key_;           // identical on all six cameras
  int64_t action_group_key_;            // identical on all six cameras
  int64_t action_group_mask_;           // identical on all six cameras
  double action_lead_time_;             // seconds added ahead of exec time
  double action_trigger_rate_;          // Hz; >0 enables continuous triggering
  int64_t action_get_image_timeout_ms_; // GetImage timeout in action loop
  int64_t ptp_domain_;                  // applied only when non-zero
  double ptp_lock_timeout_sec_;         // max wait for PtpStatus == "Slave"
  // GigE Vision stream-channel transmission shaping (0 = leave camera default).
  // Used to stagger when each camera transmits its already-captured frame so
  // the six synchronized frames do not all hit the host at once. Affects
  // TRANSMISSION only, not exposure/capture, so header.stamp sync is unchanged.
  int64_t gev_scpd_;                    // packet delay
  int64_t gev_scftd_;                   // frame-transmission delay (per-camera)
  int64_t mtu_;                         // GevSCPSPacketSize; 1500=standard, up to 9000 for jumbo frames

  bool white_balance_enable_;           // BalanceWhiteEnable; master toggle for white balance correction
  std::string white_balance_auto_;      // BalanceWhiteAuto; "Off", "Once", "Continuous"

  std::string pub_qos_history_;
  bool is_passed_pub_qos_history_;

  size_t pub_qos_history_depth_;
  bool is_passed_pub_qos_history_depth_;

  std::string pub_qos_reliability_;
  bool is_passed_pub_qos_reliability_;

  void parse_parameters_();
  void initialize_();

  void wait_for_device_timer_callback_();

  void run_();
  // TODO :
  // - handle misconfigured device
  Arena::IDevice* create_device_ros_();
  void set_nodes_();
  void set_nodes_load_default_profile_();
  void set_nodes_roi_();
  void set_nodes_gain_();
  void set_nodes_pixelformat_();
  void set_nodes_exposure_();
  void set_nodes_target_brightness_();
  void set_nodes_gamma_();
  void set_nodes_trigger_mode_();
  void set_nodes_action_trigger_mode_();
  void set_nodes_ptp_();
  void set_nodes_frame_rate_();
  void set_nodes_transmission_delay_();
  void set_nodes_mtu_();
  void set_nodes_white_balance_();
  void set_nodes_test_pattern_image_();
  void publish_images_();
  // Action-mode acquisition loop (runs in m_acquisition_thread_).
  void publish_images_action_();
  // Action master only: schedule + broadcast one GigE Vision action command.
  // single_shot=true rounds up to the next whole second; false steps by the
  // continuous period.
  void fire_scheduled_action_command_(bool single_shot);
  // Block until this device's PtpStatus settles on "Slave" (or timeout).
  bool wait_for_ptp_lock_();
  // /trigger_all service handler (one synchronized shot).
  void trigger_all_callback_(
      std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  // Continuous-rate timer handler.
  void continuous_trigger_timer_callback_();

  void publish_an_image_on_trigger_(
      std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void msg_form_image_(Arena::IImage* pImage,
                       sensor_msgs::msg::Image& image_msg);
};
