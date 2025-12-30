/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019,  Intel Corporation.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Yu Yan */

#include <moveit/handeye_calibration_rviz_plugin/handeye_control_widget.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rclcpp/rclcpp.hpp>

namespace moveit_rviz_plugin
{
ProgressBarWidget::ProgressBarWidget(QWidget* parent, int min, int max, int value) : QWidget(parent)
{
  QHBoxLayout* row = new QHBoxLayout(this);
  row->setContentsMargins(0, 10, 0, 10);

  // QLabel init
  name_label_ = new QLabel("Recorded joint state progress:", this);
  name_label_->setContentsMargins(0, 0, 0, 0);
  row->addWidget(name_label_);

  value_label_ = new QLabel(QString::number(value), this);
  value_label_->setContentsMargins(0, 0, 0, 0);
  row->addWidget(value_label_);

  // QProgressBar init
  bar_ = new QProgressBar(this);
  bar_->setTextVisible(true);
  bar_->setMinimum(min);
  bar_->setMaximum(max);
  bar_->setValue(value);
  bar_->setContentsMargins(0, 0, 0, 0);
  bar_->setDisabled(max == 0);
  row->addWidget(bar_);

  max_label_ = new QLabel(QString::number(max), this);
  max_label_->setContentsMargins(0, 0, 0, 0);
  row->addWidget(max_label_);

  this->setLayout(row);
}

void ProgressBarWidget::setMax(int value)
{
  bar_->setMaximum(value);
  bar_->setDisabled(value == 0);
  max_label_->setText(QString::number(value));
}

void ProgressBarWidget::setMin(int value)
{
  bar_->setMinimum(value);
}

void ProgressBarWidget::setValue(int value)
{
  bar_->setValue(value);
  value_label_->setText(QString::number(value));
}

int ProgressBarWidget::getValue()
{
  return bar_->value();
}

ControlTabWidget::ControlTabWidget(rclcpp::Node::SharedPtr node, HandEyeCalibrationDisplay* pdisplay, QWidget* parent)
  : QWidget(parent)
  , node_(node)
  , calibration_display_(pdisplay)
  , tf_buffer_(new tf2_ros::Buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME),
                                   (tf2::Duration)(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME), node_))
  , tf_listener_(*tf_buffer_, node_)
  , sensor_mount_type_(mhc::EYE_TO_HAND)
  , from_frame_tag_("base")
  , solver_plugins_loader_(nullptr)
  , solver_(nullptr)
  , move_group_(nullptr)
  , camera_robot_pose_(Eigen::Isometry3d::Identity())
  , auto_started_(false)
  , planning_res_(ControlTabWidget::SUCCESS)
  , full_auto_calibration_running_(false)
  , full_auto_current_pose_index_(0)
  , full_auto_total_poses_(15)
  , auto_calib_state_(AUTO_IDLE)
{
  QVBoxLayout* layout = new QVBoxLayout();
  this->setLayout(layout);

  QHBoxLayout* calib_layout = new QHBoxLayout();
  layout->addLayout(calib_layout);

  // Calibration progress
  auto_progress_ = new ProgressBarWidget(this);
  layout->addWidget(auto_progress_);

  // Pose sample tree view area
  QGroupBox* sample_group = new QGroupBox("Pose samples");
  sample_group->setMinimumWidth(280);
  calib_layout->addWidget(sample_group);
  QVBoxLayout* sample_layout = new QVBoxLayout();
  sample_group->setLayout(sample_layout);

  sample_tree_view_ = new QTreeView(this);
  sample_tree_view_->setAutoScroll(true);
  sample_tree_view_->setAlternatingRowColors(true);
  tree_view_model_ = new QStandardItemModel(sample_tree_view_);
  sample_tree_view_->setModel(tree_view_model_);
  sample_tree_view_->setHeaderHidden(true);
  sample_tree_view_->setIndentation(10);
  sample_layout->addWidget(sample_tree_view_);
  reprojection_error_label_ = new QLabel("Reprojection error: N/A");
  sample_layout->addWidget(reprojection_error_label_);

  // Settings area - use QScrollArea to make it scrollable
  QScrollArea* scroll_area = new QScrollArea();
  scroll_area->setWidgetResizable(true);
  scroll_area->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scroll_area->setFrameShape(QFrame::NoFrame);
  calib_layout->addWidget(scroll_area);

  QWidget* scroll_content = new QWidget();
  scroll_area->setWidget(scroll_content);
  QVBoxLayout* layout_right = new QVBoxLayout(scroll_content);
  layout_right->setContentsMargins(0, 0, 0, 0);

  QGroupBox* setting_group = new QGroupBox("Settings");
  layout_right->addWidget(setting_group);
  QVBoxLayout* setting_layout = new QVBoxLayout();
  setting_group->setLayout(setting_layout);
  QFormLayout* setting_layout_top = new QFormLayout();
  QGridLayout* setting_layout_bottom = new QGridLayout();
  setting_layout->insertLayout(0, setting_layout_top);
  setting_layout->insertLayout(1, setting_layout_bottom);

  calibration_solver_ = new QComboBox();
  setting_layout_top->addRow("AX=XB Solver", calibration_solver_);

  group_name_ = new QComboBox();
  connect(group_name_, SIGNAL(activated(const QString&)), this, SLOT(planningGroupNameChanged(const QString&)));
  setting_layout_top->addRow("Planning Group", group_name_);

  load_joint_state_btn_ = new QPushButton("Load joint states");
  connect(load_joint_state_btn_, SIGNAL(clicked(bool)), this, SLOT(loadJointStateBtnClicked(bool)));
  setting_layout_bottom->addWidget(load_joint_state_btn_, 0, 0);

  save_joint_state_btn_ = new QPushButton("Save joint states");
  connect(save_joint_state_btn_, SIGNAL(clicked(bool)), this, SLOT(saveJointStateBtnClicked(bool)));
  setting_layout_bottom->addWidget(save_joint_state_btn_, 0, 1);

  load_samples_btn_ = new QPushButton("Load samples");
  connect(load_samples_btn_, SIGNAL(clicked(bool)), this, SLOT(loadSamplesBtnClicked(bool)));
  setting_layout_bottom->addWidget(load_samples_btn_, 1, 0);

  save_samples_btn_ = new QPushButton("Save samples");
  connect(save_samples_btn_, SIGNAL(clicked(bool)), this, SLOT(saveSamplesBtnClicked(bool)));
  setting_layout_bottom->addWidget(save_samples_btn_, 1, 1);

  save_camera_pose_btn_ = new QPushButton("Save camera pose");
  connect(save_camera_pose_btn_, SIGNAL(clicked(bool)), this, SLOT(saveCameraPoseBtnClicked(bool)));
  setting_layout_bottom->addWidget(save_camera_pose_btn_, 2, 0, 1, 2);

  // Manual calibration area
  QGroupBox* manual_cal_group = new QGroupBox("Manual Calibration");
  layout_right->addWidget(manual_cal_group);
  QHBoxLayout* control_cal_layout = new QHBoxLayout();
  manual_cal_group->setLayout(control_cal_layout);

  take_sample_btn_ = new QPushButton("Take sample");
  take_sample_btn_->setMinimumHeight(35);
  connect(take_sample_btn_, SIGNAL(clicked(bool)), this, SLOT(takeSampleBtnClicked(bool)));
  control_cal_layout->addWidget(take_sample_btn_);

  delete_sample_btn_ = new QPushButton("Delete selected");
  delete_sample_btn_->setMinimumHeight(35);
  delete_sample_btn_->setToolTip("Delete the selected sample from the list");
  connect(delete_sample_btn_, SIGNAL(clicked(bool)), this, SLOT(deleteSelectedSampleBtnClicked(bool)));
  control_cal_layout->addWidget(delete_sample_btn_);

  reset_sample_btn_ = new QPushButton("Clear all");
  reset_sample_btn_->setMinimumHeight(35);
  connect(reset_sample_btn_, SIGNAL(clicked(bool)), this, SLOT(clearSamplesBtnClicked(bool)));
  control_cal_layout->addWidget(reset_sample_btn_);

  solve_btn_ = new QPushButton("Solve");
  solve_btn_->setMinimumHeight(35);
  connect(solve_btn_, SIGNAL(clicked(bool)), this, SLOT(solveBtnClicked(bool)));
  control_cal_layout->addWidget(solve_btn_);

  // Auto calibration area
  QGroupBox* auto_cal_group = new QGroupBox("Calibrate With Recorded Joint States");
  layout_right->addWidget(auto_cal_group);
  QVBoxLayout* auto_cal_layout = new QVBoxLayout();
  auto_cal_group->setLayout(auto_cal_layout);

  QHBoxLayout* auto_btns_layout = new QHBoxLayout();
  auto_cal_layout->addLayout(auto_btns_layout);
  auto_plan_btn_ = new QPushButton("Plan");
  auto_plan_btn_->setMinimumHeight(35);
  auto_plan_btn_->setToolTip("Plan next calibration pose");
  connect(auto_plan_btn_, SIGNAL(clicked(bool)), this, SLOT(autoPlanBtnClicked(bool)));
  auto_btns_layout->addWidget(auto_plan_btn_);

  auto_execute_btn_ = new QPushButton("Execute");
  auto_execute_btn_->setMinimumHeight(35);
  auto_execute_btn_->setToolTip("Execute the planned motion to next calibration pose");
  connect(auto_execute_btn_, SIGNAL(clicked(bool)), this, SLOT(autoExecuteBtnClicked(bool)));
  auto_btns_layout->addWidget(auto_execute_btn_);

  auto_skip_btn_ = new QPushButton("Skip");
  auto_skip_btn_->setMinimumHeight(35);
  auto_skip_btn_->setToolTip("Skip the current robot state target");
  connect(auto_skip_btn_, SIGNAL(clicked(bool)), this, SLOT(autoSkipBtnClicked(bool)));
  auto_btns_layout->addWidget(auto_skip_btn_);

  // Full Auto calibration area (spherical sampling)
  QGroupBox* full_auto_cal_group = new QGroupBox("Full Auto Calibration (Spherical Sampling)");
  layout_right->addWidget(full_auto_cal_group);
  QVBoxLayout* full_auto_cal_layout = new QVBoxLayout();
  full_auto_cal_group->setLayout(full_auto_cal_layout);

  // Number of poses setting
  QHBoxLayout* poses_setting_layout = new QHBoxLayout();
  full_auto_cal_layout->addLayout(poses_setting_layout);
  QLabel* num_poses_label = new QLabel("Number of poses:");
  poses_setting_layout->addWidget(num_poses_label);
  num_poses_spinbox_ = new QSpinBox();
  num_poses_spinbox_->setRange(5, 30);
  num_poses_spinbox_->setValue(15);
  num_poses_spinbox_->setToolTip("Number of calibration poses to generate (5-30)");
  poses_setting_layout->addWidget(num_poses_spinbox_);

  // Status label
  auto_calib_status_label_ = new QLabel("Status: Ready");
  auto_calib_status_label_->setStyleSheet("QLabel { color: gray; }");
  full_auto_cal_layout->addWidget(auto_calib_status_label_);

  // Progress bar for full auto calibration
  full_auto_progress_ = new ProgressBarWidget(this, 0, 15, 0);
  full_auto_progress_->name_label_->setText("Auto calibration progress:");
  full_auto_cal_layout->addWidget(full_auto_progress_);

  // Buttons
  QHBoxLayout* full_auto_btns_layout = new QHBoxLayout();
  full_auto_cal_layout->addLayout(full_auto_btns_layout);

  auto_calibrate_btn_ = new QPushButton("Start Auto Calibrate");
  auto_calibrate_btn_->setMinimumHeight(40);
  auto_calibrate_btn_->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }");
  auto_calibrate_btn_->setToolTip("Start full automatic calibration with spherical sampling");
  connect(auto_calibrate_btn_, SIGNAL(clicked(bool)), this, SLOT(autoCalibrateBtnClicked(bool)));
  full_auto_btns_layout->addWidget(auto_calibrate_btn_);

  stop_auto_calibrate_btn_ = new QPushButton("Stop");
  stop_auto_calibrate_btn_->setMinimumHeight(40);
  stop_auto_calibrate_btn_->setStyleSheet("QPushButton { background-color: #f44336; color: white; font-weight: bold; }");
  stop_auto_calibrate_btn_->setToolTip("Stop automatic calibration");
  stop_auto_calibrate_btn_->setEnabled(false);
  connect(stop_auto_calibrate_btn_, SIGNAL(clicked(bool)), this, SLOT(stopAutoCalibrateBtnClicked(bool)));
  full_auto_btns_layout->addWidget(stop_auto_calibrate_btn_);

  // Timer for auto calibration state machine
  auto_calib_timer_ = new QTimer(this);
  connect(auto_calib_timer_, SIGNAL(timeout()), this, SLOT(autoCalibrationTimerCallback()));

  // Initialize handeye solver plugins
  std::vector<std::string> plugins;
  if (loadSolverPlugin(plugins))
    fillSolverTypes(plugins);

  // Connect PSM and get group names
  fillPlanningGroupNameComboBox();

  // Set plan and execution watcher
  plan_watcher_ = new QFutureWatcher<void>(this);
  connect(plan_watcher_, &QFutureWatcher<void>::finished, this, &ControlTabWidget::planFinished);

  execution_watcher_ = new QFutureWatcher<void>(this);
  connect(execution_watcher_, &QFutureWatcher<void>::finished, this, &ControlTabWidget::executeFinished);

  // Set initial status
  calibration_display_->setStatus(rviz_common::properties::StatusProperty::Ok, "Calibration",
                                  "Collect 5 samples to start calibration.");
}

void ControlTabWidget::loadWidget(const rviz_common::Config& config)
{
  QString group_name;
  config.mapGetString("group", &group_name);
  if (!group_name.isEmpty() && planning_scene_monitor_ && planning_scene_monitor_->getRobotModel())
  {
    const std::vector<std::string> groups = planning_scene_monitor_->getRobotModel()->getJointModelGroupNames();
    std::vector<std::string>::const_iterator it = std::find(groups.begin(), groups.end(), group_name.toStdString());
    if (it != groups.end())
    {
      group_name_->setCurrentText(group_name);
      Q_EMIT group_name_->activated(group_name);
    }
  }
  QString solver_name;
  config.mapGetString("solver", &solver_name);
  if (!solver_name.isEmpty())
  {
    for (size_t i = 0; i < calibration_solver_->count(); ++i)
    {
      if (calibration_solver_->itemText(i) == solver_name)
      {
        calibration_solver_->setCurrentText(solver_name);
        Q_EMIT calibration_solver_->activated(solver_name);
        break;
      }
    }
  }
}

void ControlTabWidget::saveWidget(rviz_common::Config& config)
{
  config.mapSetValue("solver", calibration_solver_->currentText());
  config.mapSetValue("group", group_name_->currentText());
}

bool ControlTabWidget::loadSolverPlugin(std::vector<std::string>& plugins)
{
  if (!solver_plugins_loader_)
  {
    try
    {
      solver_plugins_loader_ = std::make_unique<pluginlib::ClassLoader<moveit_handeye_calibration::HandEyeSolverBase>>(
          "moveit_calibration_plugins", "moveit_handeye_calibration::HandEyeSolverBase");
    }
    catch (pluginlib::PluginlibException& ex)
    {
      calibration_display_->setStatus(rviz_common::properties::StatusProperty::Error, "Calibration",
                                      "Couldn't create solver plugin loader.");
      QMessageBox::warning(this, tr("Exception while creating handeye solver plugin loader "), tr(ex.what()));
      return false;
    }
  }

  // Get available plugins
  plugins = solver_plugins_loader_->getDeclaredClasses();
  return !plugins.empty();
}

bool ControlTabWidget::createSolverInstance(const std::string& plugin_name)
{
  try
  {
    solver_ = solver_plugins_loader_->createUniqueInstance(plugin_name);
    solver_->initialize();
  }
  catch (pluginlib::PluginlibException& ex)
  {
    calibration_display_->setStatus(rviz_common::properties::StatusProperty::Error, "Calibration",
                                    "Couldn't load solver plugin.");
    RCLCPP_ERROR_STREAM(node_->get_logger(),
                        "Exception while loading handeye solver plugin: " << plugin_name << ex.what());
    solver_ = nullptr;
  }

  return solver_ != nullptr;
}

void ControlTabWidget::fillSolverTypes(const std::vector<std::string>& plugins)
{
  for (const std::string& plugin : plugins)
    if (!plugin.empty() && createSolverInstance(plugin))
    {
      const std::vector<std::string>& solvers = solver_->getSolverNames();
      for (const std::string& solver : solvers)
      {
        std::string solver_name = plugin + "/" + solver;  // solver name format is "plugin_name/solver_name"
        calibration_solver_->addItem(tr(solver_name.c_str()));
      }
    }
}

std::string ControlTabWidget::parseSolverName(const std::string& solver_name, char delimiter)
{
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream tokenStream(solver_name);
  while (std::getline(tokenStream, token, delimiter))
  {
    tokens.push_back(token);
  }
  return tokens.back();
}

bool ControlTabWidget::takeTransformSamples()
{
  // Store the pair of two tf transforms and calculate camera_robot pose
  try
  {
    geometry_msgs::msg::TransformStamped camera_to_object_tf;
    geometry_msgs::msg::TransformStamped base_to_eef_tf;

    // Get the transform of the object w.r.t the camera
    camera_to_object_tf = tf_buffer_->lookupTransform(frame_names_["sensor"], frame_names_["object"], rclcpp::Time(0));

    // Check if the target transform is recent (within 1 second)
    // This ensures we only use samples where the target is actually visible
    rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();
    rclcpp::Time tf_time(camera_to_object_tf.header.stamp);
    double tf_age = (now - tf_time).seconds();
    if (tf_age > 1.0)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "Target transform is %.2f seconds old. Target may not be visible. Skipping sample.", tf_age);
      return false;
    }

    // Get the transform of the end-effector w.r.t the robot base
    base_to_eef_tf = tf_buffer_->lookupTransform(frame_names_["base"], frame_names_["eef"], rclcpp::Time(0));

    // Renormalize quaternions, to avoid numerical issues
    tf2::Quaternion tf2_quat;
    tf2::fromMsg(camera_to_object_tf.transform.rotation, tf2_quat);
    tf2_quat.normalize();
    camera_to_object_tf.transform.rotation = tf2::toMsg(tf2_quat);
    tf2::fromMsg(base_to_eef_tf.transform.rotation, tf2_quat);
    tf2_quat.normalize();
    base_to_eef_tf.transform.rotation = tf2::toMsg(tf2_quat);

    // save the pose samples
    effector_wrt_world_.push_back(tf2::transformToEigen(base_to_eef_tf));
    object_wrt_sensor_.push_back(tf2::transformToEigen(camera_to_object_tf));

    ControlTabWidget::addPoseSampleToTreeView(camera_to_object_tf, base_to_eef_tf, effector_wrt_world_.size());
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_WARN(node_->get_logger(), "TF exception: %s", e.what());
    return false;
  }

  return true;
}

void ControlTabWidget::solveBtnClicked(bool clicked)
{
  solveCameraRobotPose();
}

bool ControlTabWidget::solveCameraRobotPose()
{
  if (solver_ && !calibration_solver_->currentText().isEmpty())
  {
    std::string error_message;
    bool res = solver_->solve(effector_wrt_world_, object_wrt_sensor_, sensor_mount_type_,
                              parseSolverName(calibration_solver_->currentText().toStdString(), '/'), &error_message);
    if (res)
    {
      camera_robot_pose_ = solver_->getCameraRobotPose();

      // Update camera pose guess in context tab
      Eigen::Vector3d t = camera_robot_pose_.translation();
      Eigen::Vector3d r = camera_robot_pose_.rotation().eulerAngles(0, 1, 2);
      Q_EMIT sensorPoseUpdate(t[0], t[1], t[2], r[0], r[1], r[2]);

      // Calculate reprojection error
      const auto& reproj_err = solver_->getReprojectionError(effector_wrt_world_, object_wrt_sensor_,
                                                             camera_robot_pose_, sensor_mount_type_);
      std::ostringstream reproj_err_text;
      reproj_err_text << "Reprojection error:\n" << reproj_err.first << " m, " << reproj_err.second << " rad";
      RCLCPP_WARN(node_->get_logger(), "%s", reproj_err_text.str().c_str());
      reprojection_error_label_->setText(QString(reproj_err_text.str().c_str()));

      // Publish camera pose tf
      const std::string& from_frame = frame_names_[from_frame_tag_];
      const std::string& to_frame = frame_names_["sensor"];
      if (!from_frame.empty() && !to_frame.empty())
      {
        tf_tools_->clearAllTransforms();
        calibration_display_->setStatus(rviz_common::properties::StatusProperty::Ok, "Calibration",
                                        "Calibration successful.");
        RCLCPP_INFO_STREAM(node_->get_logger(), "Publish camera transformation"
                                                    << std::endl
                                                    << camera_robot_pose_.matrix() << std::endl
                                                    << "from " << from_frame_tag_ << " frame '" << from_frame << "'"
                                                    << "to sensor frame '" << to_frame << "'");
        return tf_tools_->publishTransform(camera_robot_pose_, from_frame, to_frame);
      }
      else
      {
        // CLI warning message without formatting
        {
          std::stringstream warn_msg;
          warn_msg << "Found camera pose:" << std::endl
                   << camera_robot_pose_.matrix() << std::endl
                   << "but " << from_frame_tag_ << " or sensor frame is undefined.";
          RCLCPP_ERROR_STREAM(node_->get_logger(), warn_msg.str());
        }
        // GUI warning message with formatting
        {
          std::stringstream warn_msg;
          warn_msg << "Found camera pose:<pre>" << std::endl
                   << camera_robot_pose_.matrix() << std::endl
                   << "</pre>but <b>" << from_frame_tag_ << "</b> or <b>sensor</b> frame is undefined.";
          QMessageBox::warning(this, "Solver Failed", QString::fromStdString(warn_msg.str()));
        }
        calibration_display_->setStatus(rviz_common::properties::StatusProperty::Warn, "Calibration",
                                        "Calibration successful but frames are undefined.");
        return false;
      }
    }
    else
    {
      calibration_display_->setStatus(rviz_common::properties::StatusProperty::Error, "Calibration", "Solver failed.");
      QMessageBox::warning(this, tr("Solver Failed"), tr((std::string("Error: ") + error_message).c_str()));
      return false;
    }
  }
  else
  {
    calibration_display_->setStatus(rviz_common::properties::StatusProperty::Error, "Calibration",
                                    "No solver available.");
    QMessageBox::warning(this, tr("No Solver Available"), tr("No available handeye calibration solver instance."));
    return false;
  }
}

bool ControlTabWidget::frameNamesEmpty()
{
  // All of four frame names needed for getting the pair of two tf transforms
  if (frame_names_["sensor"].empty() || frame_names_["object"].empty() || frame_names_["base"].empty() ||
      frame_names_["eef"].empty())
  {
    QMessageBox::warning(this, tr("Empty Frame Name"), tr("At least one of the four frame names is empty."));
    return true;
  }
  return false;
}

bool ControlTabWidget::checkJointStates()
{
  if (joint_names_.empty() || joint_states_.empty())
    return false;

  for (const std::vector<double>& state : joint_states_)
    if (state.size() != joint_names_.size())
      return false;

  return true;
}

void ControlTabWidget::setTFTool(rviz_visual_tools::TFVisualToolsPtr& tf_pub)
{
  tf_tools_ = tf_pub;
}

void ControlTabWidget::addPoseSampleToTreeView(const geometry_msgs::msg::TransformStamped& camera_to_object_tf,
                                               const geometry_msgs::msg::TransformStamped& base_to_eef_tf, int id)
{
  std::string item_name = "Sample " + std::to_string(id);
  QStandardItem* parent = new QStandardItem(QString(item_name.c_str()));
  tree_view_model_->appendRow(parent);

  std::ostringstream ss;

  QStandardItem* child_1 = new QStandardItem("TF base-to-eef");
  ss << "((" << base_to_eef_tf.transform.translation.x << ", " << base_to_eef_tf.transform.translation.y << ", "
     << base_to_eef_tf.transform.translation.z << ",), (" << base_to_eef_tf.transform.rotation.x << ", "
     << base_to_eef_tf.transform.rotation.y << ", " << base_to_eef_tf.transform.rotation.z << ", "
     << base_to_eef_tf.transform.rotation.w << "))";
  child_1->appendRow(new QStandardItem(ss.str().c_str()));
  parent->appendRow(child_1);

  QStandardItem* child_2 = new QStandardItem("TF camera-to-target");
  ss.str("");
  ss << "((" << camera_to_object_tf.transform.translation.x << ", " << camera_to_object_tf.transform.translation.y
     << ", " << camera_to_object_tf.transform.translation.z << ",), (" << camera_to_object_tf.transform.rotation.x
     << ", " << camera_to_object_tf.transform.rotation.y << ", " << camera_to_object_tf.transform.rotation.z << ", "
     << camera_to_object_tf.transform.rotation.w << "))";
  child_2->appendRow(new QStandardItem(ss.str().c_str()));
  parent->appendRow(child_2);
}

void ControlTabWidget::UpdateSensorMountType(int index)
{
  if (0 <= index && index <= 1)
  {
    sensor_mount_type_ = static_cast<mhc::SensorMountType>(index);
    switch (sensor_mount_type_)
    {
      case mhc::EYE_TO_HAND:
        from_frame_tag_ = "base";
        break;
      case mhc::EYE_IN_HAND:
        from_frame_tag_ = "eef";
        break;
      default:
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Error sensor mount type.");
        break;
    }
  }
}

void ControlTabWidget::updateFrameNames(std::map<std::string, std::string> names)
{
  frame_names_ = names;
  RCLCPP_DEBUG(node_->get_logger(), "Frame names changed:");
  for (const std::pair<const std::string, std::string>& name : frame_names_)
    RCLCPP_DEBUG_STREAM(node_->get_logger(), name.first << " : " << name.second);
}

void ControlTabWidget::takeSampleBtnClicked(bool clicked)
{
  if (frameNamesEmpty() || !takeTransformSamples())
    return;

  if (effector_wrt_world_.size() == object_wrt_sensor_.size() && effector_wrt_world_.size() > 4)
    if (!solveCameraRobotPose())
      return;

  // Save the joint values of current robot state
  if (planning_scene_monitor_)
  {
    planning_scene_monitor_->waitForCurrentRobotState(rclcpp::Clock(RCL_ROS_TIME).now(), 0.1);  // Revisit this change
    const planning_scene_monitor::LockedPlanningSceneRO& ps =
        planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_);
    if (ps)
    {
      const moveit::core::RobotState& state = ps->getCurrentState();
      const moveit::core::JointModelGroup* jmg = state.getJointModelGroup(group_name_->currentText().toStdString());
      const std::vector<std::string>& names = jmg->getActiveJointModelNames();
      if (joint_names_.size() != names.size() || joint_names_ != names)
      {
        joint_names_.clear();
        joint_states_.clear();
      }
      std::vector<double> state_joint_values;
      state.copyJointGroupPositions(jmg, state_joint_values);
      if (names.size() == state_joint_values.size())
      {
        joint_names_ = names;
        joint_states_.push_back(state_joint_values);
        auto_progress_->setMax(joint_states_.size());
      }
    }
  }
}

void ControlTabWidget::clearSamplesBtnClicked(bool clicked)
{
  // Clear recorded transforms
  effector_wrt_world_.clear();
  object_wrt_sensor_.clear();
  tree_view_model_->clear();

  // Clear recorded joint states
  joint_states_.clear();
  auto_progress_->setMax(0);
  auto_progress_->setValue(0);
}

void ControlTabWidget::deleteSelectedSampleBtnClicked(bool clicked)
{
  // Get selected index from tree view
  QModelIndexList selected = sample_tree_view_->selectionModel()->selectedIndexes();
  if (selected.isEmpty())
  {
    QMessageBox::warning(this, tr("No Selection"), tr("Please select a sample to delete."));
    return;
  }

  // Get the row index of the selected item (top-level item)
  QModelIndex index = selected.first();
  // If a child item is selected, get its parent
  if (index.parent().isValid())
  {
    index = index.parent();
  }
  int row = index.row();

  // Check if the row is valid
  if (row < 0 || row >= static_cast<int>(effector_wrt_world_.size()))
  {
    QMessageBox::warning(this, tr("Invalid Selection"), tr("Invalid sample selected."));
    return;
  }

  // Remove the sample from data vectors
  effector_wrt_world_.erase(effector_wrt_world_.begin() + row);
  object_wrt_sensor_.erase(object_wrt_sensor_.begin() + row);

  // Remove from tree view
  tree_view_model_->removeRow(row);

  // Rename remaining samples to keep numbering consistent
  for (int i = row; i < tree_view_model_->rowCount(); ++i)
  {
    QStandardItem* item = tree_view_model_->item(i);
    if (item)
    {
      item->setText(QString("Sample %1").arg(i + 1));
    }
  }

  // Update status
  QString status_msg = QString("Sample deleted. %1 samples remaining.").arg(effector_wrt_world_.size());
  calibration_display_->setStatus(rviz_common::properties::StatusProperty::Ok, "Calibration", status_msg);
}

void ControlTabWidget::saveCameraPoseBtnClicked(bool clicked)
{
  std::string& from_frame = frame_names_[from_frame_tag_];
  std::string& to_frame = frame_names_["sensor"];

  if (from_frame.empty() || to_frame.empty())
  {
    QMessageBox::warning(this, tr("Empty Frame Name"),
                         tr("Make sure you have selected the correct frames in the Context tab."));
    return;
  }

  // DontUseNativeDialog option set to avoid this issue: https://github.com/ros-planning/moveit/issues/2357
  QString file_name = QFileDialog::getSaveFileName(
      this, tr("Save Camera Robot Pose"), "",
      tr("Launch scripts - ALL (*.launch* *.py *.xml *.yaml *.yml);;Launch scripts - "
         "PYTHON (*.launch.py *.py);;Launch scripts - XML (*.launch *.launch.xml *.xml);;Launch "
         "scripts - YAML (*.launch.yaml *.launch.yml *.yaml *.yml);;All Files (*)"),
      nullptr, QFileDialog::DontUseNativeDialog);

  if (file_name.isEmpty())
    return;

  if (!file_name.contains("."))
    file_name += ".launch.py";
  else if (file_name.endsWith(".launch"))
    file_name += ".py";

  QFile file(file_name);
  if (!file.open(QIODevice::WriteOnly))
  {
    QMessageBox::warning(this, tr("Unable to open file"), file.errorString());
    return;
  }

  Eigen::Vector3d t = camera_robot_pose_.translation();
  Eigen::Quaterniond r_quat(camera_robot_pose_.rotation());
  Eigen::Vector3d r_euler = camera_robot_pose_.rotation().eulerAngles(0, 1, 2);

  std::string mount_type;
  switch (sensor_mount_type_)
  {
    case mhc::EYE_TO_HAND:
      mount_type = "EYE-TO-HAND";
      break;
    case mhc::EYE_IN_HAND:
      mount_type = "EYE-IN-HAND";
      break;
    default:
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Error sensor mount type.");
      break;
  }

  std::stringstream ss;
  if (file_name.endsWith(".py"))
  {
    ss = generateCalibrationPython(from_frame, to_frame, t, r_quat, r_euler, mount_type);
  }
  else if (file_name.endsWith(".xml"))
  {
    ss = generateCalibrationXml(from_frame, to_frame, t, r_quat, r_euler, mount_type);
  }
  else if (file_name.endsWith(".yaml") || file_name.endsWith(".yml"))
  {
    ss = generateCalibrationYaml(from_frame, to_frame, t, r_quat, r_euler, mount_type);
  }
  else
  {
    QMessageBox::warning(
        this, tr("Unknown file type"),
        tr("Unable to save file, unknown file type. Only `.py`, `.xml`, and `.yaml`/`.yml` are currently "
           "supported for ROS 2 launch scripts."));
    return;
  }

  QTextStream out(&file);
  out << ss.str().c_str();
}

void ControlTabWidget::planningGroupNameChanged(const QString& text)
{
  if (!text.isEmpty())
  {
    setGroupName(text.toStdString());
  }
  else
  {
    QMessageBox::warning(this, tr("Invalid Group Name"), "Group name is empty");
  }
}

void ControlTabWidget::setGroupName(const std::string& group_name)
{
  if (move_group_ && move_group_->getName() == group_name)
    return;

  try
  {
    moveit::planning_interface::MoveGroupInterface::Options opt(group_name);
    // opt.node_handle_ = ros::NodeHandle(calibration_display_->move_group_ns_property_->getStdString());  <- Do I need
    // to assign opt.robot_model and opt.robot_description ?
    move_group_.reset(
        new moveit::planning_interface::MoveGroupInterface(node_, opt, tf_buffer_, rclcpp::Duration(5, 0)));

    // Clear the joint values from any previous group
    joint_states_.clear();
    auto_progress_->setMax(0);
  }
  catch (std::exception& ex)
  {
    RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
  }
}

void ControlTabWidget::fillPlanningGroupNameComboBox()
{
  group_name_->clear();
  // Fill in available planning group names
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(node_, "robot_description", tf_buffer_,
                                                                                 "planning_scene_monitor"));
  if (planning_scene_monitor_)
  {
    planning_scene_monitor_->startSceneMonitor(calibration_display_->planning_scene_topic_property_->getStdString());
    std::string service_name = planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_SERVICE;
    if (!calibration_display_->move_group_ns_property_->getStdString().empty())
      service_name = rclcpp::names::append(calibration_display_->move_group_ns_property_->getStdString(), service_name);
    if (planning_scene_monitor_->requestPlanningSceneState(service_name))
    {
      const moveit::core::RobotModelConstPtr& kmodel = planning_scene_monitor_->getRobotModel();
      for (const std::string& group_name : kmodel->getJointModelGroupNames())
      {
        group_name_->addItem(group_name.c_str());
      }
      if (!group_name_->currentText().isEmpty())
        setGroupName(group_name_->currentText().toStdString());
    }
  }
}

void ControlTabWidget::saveJointStateBtnClicked(bool clicked)
{
  if (!checkJointStates())
  {
    QMessageBox::warning(this, tr("Error"), tr("No joint states or joint state doesn't match joint names."));
    return;
  }

  // DontUseNativeDialog option set to avoid this issue: https://github.com/ros-planning/moveit/issues/2357
  QString file_name =
      QFileDialog::getSaveFileName(this, tr("Save Joint States"), "", tr("Target File (*.yaml);;All Files (*)"),
                                   nullptr, QFileDialog::DontUseNativeDialog);

  if (file_name.isEmpty())
    return;

  if (!file_name.endsWith(".yaml"))
    file_name += ".yaml";

  QFile file(file_name);
  if (!file.open(QIODevice::WriteOnly))
  {
    QMessageBox::warning(this, tr("Unable to open file"), file.errorString());
    return;
  }

  YAML::Emitter emitter;
  emitter << YAML::BeginMap;

  // Joint Names
  emitter << YAML::Key << "joint_names";
  emitter << YAML::Value << YAML::BeginSeq;
  for (size_t i = 0; i < joint_names_.size(); ++i)
    emitter << YAML::Value << joint_names_[i];
  emitter << YAML::EndSeq;

  // Joint Values
  emitter << YAML::Key << "joint_values";
  emitter << YAML::Value << YAML::BeginSeq;
  for (size_t i = 0; i < joint_states_.size(); ++i)
  {
    emitter << YAML::BeginSeq;
    for (size_t j = 0; j < joint_states_[i].size(); ++j)
      emitter << YAML::Value << joint_states_[i][j];
    emitter << YAML::EndSeq;
  }
  emitter << YAML::EndSeq;

  emitter << YAML::EndMap;

  QTextStream out(&file);
  out << emitter.c_str();
}

void ControlTabWidget::loadSamplesBtnClicked(bool clicked)
{
  QString file_name = QFileDialog::getOpenFileName(this, tr("Load Samples"), "", tr("Target File (*.yaml)"), nullptr,
                                                   QFileDialog::DontUseNativeDialog);

  if (file_name.isEmpty())
    return;

  effector_wrt_world_.clear();
  object_wrt_sensor_.clear();

  // transformations are serialised as 4x4 row-major matrices
  typedef Eigen::Matrix<double, 4, 4, Eigen::RowMajor> Matrix4d_rm;

  YAML::Node yaml_states = YAML::LoadFile(file_name.toStdString());
  try
  {
    for (std::size_t i = 0; i < yaml_states.size(); i++)
    {
      effector_wrt_world_.emplace_back(
          Eigen::Map<const Matrix4d_rm>(yaml_states[i]["effector_wrt_world"].as<std::vector<double>>().data()));

      object_wrt_sensor_.emplace_back(
          Eigen::Map<const Matrix4d_rm>(yaml_states[i]["object_wrt_sensor"].as<std::vector<double>>().data()));

      // add to GUI
      ControlTabWidget::addPoseSampleToTreeView(tf2::eigenToTransform(object_wrt_sensor_.back()),
                                                tf2::eigenToTransform(effector_wrt_world_.back()),
                                                effector_wrt_world_.size());
    }

    auto_progress_->setMax(yaml_states.size());
    auto_progress_->setValue(yaml_states.size());
  }
  catch (const YAML::Exception& e)
  {
    QMessageBox::critical(this, "YAML Exception",
                          QString::fromStdString("YAML exception: " + std::string(e.what()) +
                                                 "\nCheck that the sample file has the correct format."));
  }
}

void ControlTabWidget::saveSamplesBtnClicked(bool clicked)
{
  if (effector_wrt_world_.size() != object_wrt_sensor_.size())
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Different number of poses");
    return;
  }

  // DontUseNativeDialog option set to avoid this issue: https://github.com/ros-planning/moveit/issues/2357
  QString file_name =
      QFileDialog::getSaveFileName(this, tr("Save Samples"), "", tr("Target File (*.yaml);;All Files (*)"), nullptr,
                                   QFileDialog::DontUseNativeDialog);

  if (file_name.isEmpty())
    return;

  if (!file_name.endsWith(".yaml"))
    file_name += ".yaml";

  QFile file(file_name);
  if (!file.open(QIODevice::WriteOnly))
  {
    QMessageBox::warning(this, tr("Unable to open file"), file.errorString());
    return;
  }

  YAML::Emitter emitter;
  emitter << YAML::BeginSeq;
  for (size_t i = 0; i < effector_wrt_world_.size(); i++)
  {
    emitter << YAML::Value << YAML::BeginMap;
    emitter << YAML::Key << "effector_wrt_world";
    emitter << YAML::Value << YAML::BeginSeq;
    for (size_t y = 0; y < 4; y++)
    {
      for (size_t x = 0; x < 4; x++)
      {
        emitter << YAML::Value << effector_wrt_world_[i](y, x);
      }
    }
    emitter << YAML::EndSeq;
    emitter << YAML::Key << "object_wrt_sensor";
    emitter << YAML::Value << YAML::BeginSeq;
    for (size_t y = 0; y < 4; y++)
    {
      for (size_t x = 0; x < 4; x++)
      {
        emitter << YAML::Value << object_wrt_sensor_[i](y, x);
      }
    }
    emitter << YAML::EndSeq;
    emitter << YAML::EndMap;
  }
  emitter << YAML::EndSeq;

  QTextStream out(&file);
  out << emitter.c_str();
}

void ControlTabWidget::loadJointStateBtnClicked(bool clicked)
{
  // DontUseNativeDialog option set to avoid this issue: https://github.com/ros-planning/moveit/issues/2357
  QString file_name =
      QFileDialog::getOpenFileName(this, tr("Load Joint States"), "", tr("Target File (*.yaml);;All Files (*)"),
                                   nullptr, QFileDialog::DontUseNativeDialog);

  if (file_name.isEmpty() || !file_name.endsWith(".yaml"))
    return;

  QFile file(file_name);
  if (!file.open(QIODevice::ReadOnly))
  {
    QMessageBox::warning(this, tr("Unable to open file"), file.errorString());
    return;
  }

  // Begin parsing
  try
  {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Load joint states from file: " << file_name.toStdString().c_str());
    YAML::Node doc = YAML::LoadFile(file_name.toStdString());
    if (!doc.IsMap())
      return;

    // Read joint names
    const YAML::Node& names = doc["joint_names"];
    if (!names.IsNull() && names.IsSequence())
    {
      joint_names_.clear();
      for (YAML::const_iterator it = names.begin(); it != names.end(); ++it)
        joint_names_.push_back(it->as<std::string>());
    }
    else
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Can't find 'joint_names' in the opened file.");
      return;
    }

    // Read joint values
    const YAML::Node& values = doc["joint_values"];
    if (!values.IsNull() && values.IsSequence())
    {
      joint_states_.clear();
      for (YAML::const_iterator state_it = values.begin(); state_it != values.end(); ++state_it)
      {
        std::vector<double> jv;
        if (!state_it->IsNull() && state_it->IsSequence())
          for (YAML::const_iterator joint_it = state_it->begin(); joint_it != state_it->end(); ++joint_it)
            jv.push_back(joint_it->as<double>());
        if (jv.size() == joint_names_.size())
          joint_states_.push_back(jv);
      }
    }
    else
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Can't find 'joint_values' in the opened file.");
      return;
    }
  }
  catch (YAML::ParserException& e)  // Catch errors
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), e.what());
    return;
  }

  if (joint_states_.size() > 0)
  {
    auto_progress_->setMax(joint_states_.size());
    auto_progress_->setValue(0);
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), "Loaded and parsed: " << file_name.toStdString());
}

void ControlTabWidget::autoPlanBtnClicked(bool clicked)
{
  auto_plan_btn_->setEnabled(false);
  plan_watcher_->setFuture(QtConcurrent::run(this, &ControlTabWidget::computePlan));
}

void ControlTabWidget::computePlan()
{
  planning_res_ = ControlTabWidget::SUCCESS;
  int max = auto_progress_->bar_->maximum();

  if (max != joint_states_.size() || auto_progress_->getValue() == max)
  {
    planning_res_ = ControlTabWidget::FAILURE_NO_JOINT_STATE;
    return;
  }

  if (!checkJointStates())
  {
    planning_res_ = ControlTabWidget::FAILURE_INVALID_JOINT_STATE;
    return;
  }

  if (!planning_scene_monitor_)
  {
    planning_res_ = ControlTabWidget::FAILURE_NO_PSM;
    return;
  }

  if (!move_group_)
  {
    planning_res_ = ControlTabWidget::FAILURE_NO_MOVE_GROUP;
    return;
  }

  if (move_group_->getActiveJoints() != joint_names_)
  {
    planning_res_ = ControlTabWidget::FAILURE_WRONG_MOVE_GROUP;
    return;
  }

  // Get current joint state as start state
  moveit::core::RobotStatePtr start_state = move_group_->getCurrentState();
  planning_scene_monitor_->waitForCurrentRobotState(rclcpp::Clock(RCL_ROS_TIME).now(), 0.1);
  const planning_scene_monitor::LockedPlanningSceneRO& ps =
      planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_);
  if (ps)
    start_state.reset(new moveit::core::RobotState(ps->getCurrentState()));

  // Plan motion to the recorded joint state target
  if (auto_progress_->getValue() < joint_states_.size())
  {
    move_group_->setStartState(*start_state);
    move_group_->setJointValueTarget(joint_states_[auto_progress_->getValue()]);
    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.5);
    current_plan_.reset(new moveit::planning_interface::MoveGroupInterface::Plan());
    planning_res_ = (move_group_->plan(*current_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS) ?
                        ControlTabWidget::SUCCESS :
                        ControlTabWidget::FAILURE_PLAN_FAILED;

    if (planning_res_ == ControlTabWidget::SUCCESS)
      RCLCPP_DEBUG_STREAM(node_->get_logger(), "Planning succeed.");
    else
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Planning failed.");
  }
}

void ControlTabWidget::autoExecuteBtnClicked(bool clicked)
{
  if (plan_watcher_->isRunning())
  {
    plan_watcher_->waitForFinished();
  }

  auto_execute_btn_->setEnabled(false);
  execution_watcher_->setFuture(QtConcurrent::run(this, &ControlTabWidget::computeExecution));
}

void ControlTabWidget::computeExecution()
{
  if (move_group_ && current_plan_)
    planning_res_ = (move_group_->execute(*current_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS) ?
                        ControlTabWidget::SUCCESS :
                        ControlTabWidget::FAILURE_PLAN_FAILED;

  if (planning_res_ == ControlTabWidget::SUCCESS)
  {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Execution succeed.");
  }
  else
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Execution failed.");
}

void ControlTabWidget::planFinished()
{
  // Check if this is for full auto calibration
  if (full_auto_calibration_running_)
  {
    autoCalibrationPlanFinished();
    return;
  }

  auto_plan_btn_->setEnabled(true);
  switch (planning_res_)
  {
    case ControlTabWidget::FAILURE_NO_JOINT_STATE:
      QMessageBox::warning(this, tr("Error"),
                           tr("Could not compute plan. No more prerecorded joint states to execute."));
      break;
    case ControlTabWidget::FAILURE_INVALID_JOINT_STATE:
      QMessageBox::warning(this, tr("Error"),
                           tr("Could not compute plan. Invalid joint states (names wrong or missing)."));
      break;
    case ControlTabWidget::FAILURE_NO_PSM:
      QMessageBox::warning(this, tr("Error"), tr("Could not compute plan. No planning scene monitor."));
      break;
    case ControlTabWidget::FAILURE_NO_MOVE_GROUP:
      QMessageBox::warning(this, tr("Error"), tr("Could not compute plan. Missing move_group."));
      break;
    case ControlTabWidget::FAILURE_WRONG_MOVE_GROUP:
      QMessageBox::warning(
          this, tr("Error"),
          tr("Could not compute plan. Joint names for recorded state do not match names from current planning group."));
      break;
    case ControlTabWidget::FAILURE_PLAN_FAILED:
      QMessageBox::warning(this, tr("Error"), tr("Could not compute plan. Planning failed."));
      break;
    case ControlTabWidget::SUCCESS:
      break;
  }
  RCLCPP_DEBUG(node_->get_logger(), "Plan finished");
}

void ControlTabWidget::executeFinished()
{
  // Check if this is for full auto calibration
  if (full_auto_calibration_running_)
  {
    autoCalibrationExecuteFinished();
    return;
  }

  auto_execute_btn_->setEnabled(true);
  if (planning_res_ == ControlTabWidget::SUCCESS)
  {
    auto_progress_->setValue(auto_progress_->getValue() + 1);
    if (!frameNamesEmpty())
      takeTransformSamples();

    if (effector_wrt_world_.size() == object_wrt_sensor_.size() && effector_wrt_world_.size() > 4)
      solveCameraRobotPose();
  }
  RCLCPP_DEBUG(node_->get_logger(), "Execution finished");
}

void ControlTabWidget::autoSkipBtnClicked(bool clicked)
{
  auto_progress_->setValue(auto_progress_->getValue() + 1);
}

std::stringstream ControlTabWidget::generateCalibrationPython(std::string& from_frame, std::string& to_frame,
                                                              Eigen::Vector3d& t, Eigen::Quaterniond& r_quat,
                                                              Eigen::Vector3d& r_euler, std::string& mount_type)
{
  std::stringstream ss;
  ss << "\"\"\" Static transform publisher acquired via MoveIt 2 hand-eye calibration \"\"\"" << std::endl;
  ss << "\"\"\" " << mount_type << ": " << from_frame << " -> " << to_frame << " \"\"\"" << std::endl;
  ss << "from launch import LaunchDescription" << std::endl;
  ss << "from launch_ros.actions import Node" << std::endl;
  ss << std::endl;
  ss << std::endl;
  ss << "def generate_launch_description() -> LaunchDescription:" << std::endl;
  ss << "    nodes = [" << std::endl;
  ss << "        Node(" << std::endl;
  ss << "            package=\"tf2_ros\"," << std::endl;
  ss << "            executable=\"static_transform_publisher\"," << std::endl;
  ss << "            output=\"log\"," << std::endl;
  ss << "            arguments=[" << std::endl;
  ss << "                \"--frame-id\"," << std::endl;
  ss << "                \"" << from_frame << "\"," << std::endl;
  ss << "                \"--child-frame-id\"," << std::endl;
  ss << "                \"" << to_frame << "\"," << std::endl;
  ss << "                \"--x\"," << std::endl;
  ss << "                \"" << t[0] << "\"," << std::endl;
  ss << "                \"--y\"," << std::endl;
  ss << "                \"" << t[1] << "\"," << std::endl;
  ss << "                \"--z\"," << std::endl;
  ss << "                \"" << t[2] << "\"," << std::endl;
  ss << "                \"--qx\"," << std::endl;
  ss << "                \"" << r_quat.x() << "\"," << std::endl;
  ss << "                \"--qy\"," << std::endl;
  ss << "                \"" << r_quat.y() << "\"," << std::endl;
  ss << "                \"--qz\"," << std::endl;
  ss << "                \"" << r_quat.z() << "\"," << std::endl;
  ss << "                \"--qw\"," << std::endl;
  ss << "                \"" << r_quat.w() << "\"," << std::endl;
  ss << "                # \"--roll\"," << std::endl;
  ss << "                # \"" << r_euler[0] << "\"," << std::endl;
  ss << "                # \"--pitch\"," << std::endl;
  ss << "                # \"" << r_euler[1] << "\"," << std::endl;
  ss << "                # \"--yaw\"," << std::endl;
  ss << "                # \"" << r_euler[2] << "\"," << std::endl;
  ss << "            ]," << std::endl;
  ss << "        )," << std::endl;
  ss << "    ]" << std::endl;
  ss << "    return LaunchDescription(nodes)" << std::endl;
  return ss;
}

std::stringstream ControlTabWidget::generateCalibrationXml(std::string& from_frame, std::string& to_frame,
                                                           Eigen::Vector3d& t, Eigen::Quaterniond& r_quat,
                                                           Eigen::Vector3d& r_euler, std::string& mount_type)
{
  std::stringstream ss;
  ss << "<!-- Static transform publisher acquired via MoveIt 2 hand-eye calibration -->" << std::endl;
  ss << "<!-- " << mount_type << ": " << from_frame << " -> " << to_frame << " -->" << std::endl;
  ss << std::endl;
  ss << "<launch>" << std::endl;
  ss << "    <node" << std::endl;
  ss << "        pkg=\"tf2_ros\"" << std::endl;
  ss << "        exec=\"static_transform_publisher\"" << std::endl;
  ss << "        output=\"log\"" << std::endl;
  ss << "        args=\"" << std::endl;
  ss << "            --frame-id " << from_frame << std::endl;
  ss << "            --child-frame-id " << to_frame << std::endl;
  ss << "            --x " << t[0] << std::endl;
  ss << "            --y " << t[1] << std::endl;
  ss << "            --z " << t[2] << std::endl;
  ss << "            --qx " << r_quat.x() << std::endl;
  ss << "            --qy " << r_quat.y() << std::endl;
  ss << "            --qz " << r_quat.z() << std::endl;
  ss << "            --qw " << r_quat.w() << std::endl;
  ss << "        \"" << std::endl;
  ss << "    />" << std::endl;
  ss << "    <!--" << std::endl;
  ss << "            roll " << r_euler[0] << std::endl;
  ss << "            pitch " << r_euler[1] << std::endl;
  ss << "            yaw " << r_euler[2] << std::endl;
  ss << "    -->" << std::endl;
  ss << "</launch>" << std::endl;
  return ss;
}

std::stringstream ControlTabWidget::generateCalibrationYaml(std::string& from_frame, std::string& to_frame,
                                                            Eigen::Vector3d& t, Eigen::Quaterniond& r_quat,
                                                            Eigen::Vector3d& r_euler, std::string& mount_type)
{
  std::stringstream ss;
  ss << "# Static transform publisher acquired via MoveIt 2 hand-eye calibration" << std::endl;
  ss << "# " << mount_type << ": " << from_frame << " -> " << to_frame << std::endl;
  ss << std::endl;
  ss << "launch:" << std::endl;
  ss << "    - node:" << std::endl;
  ss << "          pkg: tf2_ros" << std::endl;
  ss << "          exec: static_transform_publisher" << std::endl;
  ss << "          output: log" << std::endl;
  ss << "          args:" << std::endl;
  ss << "              \"" << std::endl;
  ss << "              --frame-id " << from_frame << std::endl;
  ss << "              --child-frame-id " << to_frame << std::endl;
  ss << "              --x " << t[0] << std::endl;
  ss << "              --y " << t[1] << std::endl;
  ss << "              --z " << t[2] << std::endl;
  ss << "              --qx " << r_quat.x() << std::endl;
  ss << "              --qy " << r_quat.y() << std::endl;
  ss << "              --qz " << r_quat.z() << std::endl;
  ss << "              --qw " << r_quat.w() << std::endl;
  ss << "              \"" << std::endl;
  ss << "              # --roll " << r_euler[0] << std::endl;
  ss << "              # --pitch " << r_euler[1] << std::endl;
  ss << "              # --yaw " << r_euler[2] << std::endl;
  return ss;
}

// ============================================================================
// Full Auto Calibration Implementation (Spherical Sampling)
// ============================================================================

std::vector<Eigen::Isometry3d> ControlTabWidget::generateSphericalSamplingPoses(const Eigen::Isometry3d& center_pose,
                                                                                  int num_poses)
{
  std::vector<Eigen::Isometry3d> poses;

  // ============================================================================
  // Optimal hand-eye calibration sampling strategy:
  //
  // Key principles for high-quality calibration:
  // 1. Keep end-effector position nearly constant (minimize translation)
  // 2. Maximize rotation angle for each pose (larger rotation = better data)
  // 3. Maximize angle between rotation axes of different poses
  // 4. This way, only joints 3,4,5,6 change while end-effector stays in place
  // ============================================================================

  // Keep position fixed (or with very minimal offset for safety)
  Eigen::Vector3d fixed_position = center_pose.translation();
  Eigen::Matrix3d base_rotation = center_pose.rotation();

  // Maximum tilt angle from the base orientation (25-30 degrees is good)
  const double max_tilt_angle = 28.0 * M_PI / 180.0;

  // Use Fibonacci sphere algorithm to uniformly distribute rotation axes
  // This maximizes the angle between rotation axes of different poses
  const double golden_ratio = (1.0 + std::sqrt(5.0)) / 2.0;

  for (int i = 0; i < num_poses; ++i)
  {
    // Fibonacci sphere sampling for rotation axis direction
    // This ensures rotation axes are uniformly distributed on a sphere
    double theta = 2.0 * M_PI * i / golden_ratio;  // Azimuthal angle
    double phi = std::acos(1.0 - (2.0 * (i + 0.5)) / num_poses);  // Polar angle (0 to PI)

    // Create rotation axis from spherical coordinates
    // The axis points in different directions for each pose
    Eigen::Vector3d rotation_axis;
    rotation_axis << std::sin(phi) * std::cos(theta),
                     std::sin(phi) * std::sin(theta),
                     std::cos(phi);
    rotation_axis.normalize();

    // Use consistent large rotation angle for all poses
    // Larger angles provide better calibration data
    double rot_angle = max_tilt_angle;

    // Apply rotation around the computed axis
    Eigen::AngleAxisd rotation_variation(rot_angle, rotation_axis);
    Eigen::Matrix3d new_rotation = base_rotation * rotation_variation.toRotationMatrix();

    // Create the pose with fixed position but varied orientation
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = fixed_position;  // Position stays the same
    pose.linear() = new_rotation;         // Only orientation changes

    poses.push_back(pose);

    RCLCPP_DEBUG(node_->get_logger(),
                 "Generated pose %d: fixed pos(%.3f, %.3f, %.3f), rot_axis(%.2f, %.2f, %.2f), angle=%.1f deg",
                 i, fixed_position.x(), fixed_position.y(), fixed_position.z(),
                 rotation_axis.x(), rotation_axis.y(), rotation_axis.z(),
                 rot_angle * 180.0 / M_PI);
  }

  RCLCPP_INFO(node_->get_logger(),
              "Generated %d poses with fixed position and uniformly distributed rotation axes (max angle: %.1f deg)",
              num_poses, max_tilt_angle * 180.0 / M_PI);

  return poses;
}

void ControlTabWidget::planToGeneratedPose(int pose_index)
{
  planning_res_ = ControlTabWidget::SUCCESS;

  if (!planning_scene_monitor_)
  {
    planning_res_ = ControlTabWidget::FAILURE_NO_PSM;
    return;
  }

  if (!move_group_)
  {
    planning_res_ = ControlTabWidget::FAILURE_NO_MOVE_GROUP;
    return;
  }

  if (pose_index < 0 || pose_index >= static_cast<int>(generated_poses_.size()))
  {
    planning_res_ = ControlTabWidget::FAILURE_NO_JOINT_STATE;
    return;
  }

  // Get current state as start state
  moveit::core::RobotStatePtr start_state = move_group_->getCurrentState();
  planning_scene_monitor_->waitForCurrentRobotState(rclcpp::Clock(RCL_ROS_TIME).now(), 0.1);
  const planning_scene_monitor::LockedPlanningSceneRO& ps =
      planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_);
  if (ps)
    start_state.reset(new moveit::core::RobotState(ps->getCurrentState()));

  // Set target pose
  const Eigen::Isometry3d& target_pose = generated_poses_[pose_index];
  geometry_msgs::msg::Pose target_pose_msg = tf2::toMsg(target_pose);

  move_group_->setStartState(*start_state);
  move_group_->setPoseTarget(target_pose_msg);
  move_group_->setMaxVelocityScalingFactor(0.3);
  move_group_->setMaxAccelerationScalingFactor(0.3);

  current_plan_.reset(new moveit::planning_interface::MoveGroupInterface::Plan());
  planning_res_ = (move_group_->plan(*current_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS) ?
                      ControlTabWidget::SUCCESS :
                      ControlTabWidget::FAILURE_PLAN_FAILED;

  if (planning_res_ == ControlTabWidget::SUCCESS)
    RCLCPP_INFO(node_->get_logger(), "Auto calibration: Planning to pose %d succeeded", pose_index);
  else
    RCLCPP_WARN(node_->get_logger(), "Auto calibration: Planning to pose %d failed", pose_index);
}

void ControlTabWidget::executeToGeneratedPose()
{
  if (move_group_ && current_plan_)
  {
    planning_res_ = (move_group_->execute(*current_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS) ?
                        ControlTabWidget::SUCCESS :
                        ControlTabWidget::FAILURE_PLAN_FAILED;
  }
  else
  {
    planning_res_ = ControlTabWidget::FAILURE_PLAN_FAILED;
  }

  if (planning_res_ == ControlTabWidget::SUCCESS)
    RCLCPP_INFO(node_->get_logger(), "Auto calibration: Execution succeeded");
  else
    RCLCPP_WARN(node_->get_logger(), "Auto calibration: Execution failed");
}

void ControlTabWidget::autoCalibrateBtnClicked(bool clicked)
{
  // Check prerequisites
  if (frameNamesEmpty())
  {
    QMessageBox::warning(this, tr("Error"), tr("Please set all frame names in the Context tab first."));
    return;
  }

  if (!move_group_)
  {
    QMessageBox::warning(this, tr("Error"), tr("No move group available. Please select a planning group."));
    return;
  }

  if (!planning_scene_monitor_)
  {
    QMessageBox::warning(this, tr("Error"), tr("No planning scene monitor available."));
    return;
  }

  // Clear previous samples
  effector_wrt_world_.clear();
  object_wrt_sensor_.clear();
  tree_view_model_->clear();

  // Get current end-effector pose as the center for spherical sampling
  planning_scene_monitor_->waitForCurrentRobotState(rclcpp::Clock(RCL_ROS_TIME).now(), 0.5);
  const planning_scene_monitor::LockedPlanningSceneRO& ps =
      planning_scene_monitor::LockedPlanningSceneRO(planning_scene_monitor_);
  if (!ps)
  {
    QMessageBox::warning(this, tr("Error"), tr("Could not get current robot state."));
    return;
  }

  const moveit::core::RobotState& state = ps->getCurrentState();
  const std::string& eef_link = move_group_->getEndEffectorLink();
  initial_eef_pose_ = state.getGlobalLinkTransform(eef_link);

  // Generate sampling poses
  // Generate more poses than needed to account for failures
  // We want full_auto_total_poses_ successful samples, so generate 2x candidates
  full_auto_total_poses_ = num_poses_spinbox_->value();  // This is the TARGET number of successful samples
  int num_candidate_poses = full_auto_total_poses_ * 2;  // Generate 2x candidates to account for failures
  generated_poses_ = generateSphericalSamplingPoses(initial_eef_pose_, num_candidate_poses);

  if (generated_poses_.empty())
  {
    QMessageBox::warning(this, tr("Error"), tr("Failed to generate calibration poses."));
    return;
  }

  // Initialize state machine
  full_auto_calibration_running_ = true;
  full_auto_current_pose_index_ = 0;
  auto_calib_state_ = AUTO_PLANNING;

  // Update UI
  auto_calibrate_btn_->setEnabled(false);
  stop_auto_calibrate_btn_->setEnabled(true);
  num_poses_spinbox_->setEnabled(false);
  full_auto_progress_->setMax(full_auto_total_poses_);
  full_auto_progress_->setValue(0);
  auto_calib_status_label_->setText("Status: Starting auto calibration...");
  auto_calib_status_label_->setStyleSheet("QLabel { color: blue; }");

  RCLCPP_INFO(node_->get_logger(), "Starting full auto calibration with %d poses", full_auto_total_poses_);

  // ============================================================================
  // First, take a sample at the initial position (user's starting position)
  // This position is guaranteed to have the target visible
  // ============================================================================
  auto_calib_status_label_->setText("Status: Taking initial sample at starting position...");
  if (takeTransformSamples())
  {
    RCLCPP_INFO(node_->get_logger(), "Auto calibration: Initial sample taken successfully at starting position");
    full_auto_progress_->setValue(1);
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(), "Auto calibration: Failed to take initial sample, but continuing...");
  }

  // Start planning to first generated pose
  plan_watcher_->setFuture(
      QtConcurrent::run(this, &ControlTabWidget::planToGeneratedPose, full_auto_current_pose_index_));
}

void ControlTabWidget::stopAutoCalibrateBtnClicked(bool clicked)
{
  RCLCPP_INFO(node_->get_logger(), "Stopping auto calibration");

  full_auto_calibration_running_ = false;
  auto_calib_state_ = AUTO_IDLE;
  auto_calib_timer_->stop();

  // Update UI
  auto_calibrate_btn_->setEnabled(true);
  stop_auto_calibrate_btn_->setEnabled(false);
  num_poses_spinbox_->setEnabled(true);
  auto_calib_status_label_->setText("Status: Stopped by user");
  auto_calib_status_label_->setStyleSheet("QLabel { color: orange; }");

  // If we have enough samples, try to solve
  if (effector_wrt_world_.size() >= 5)
  {
    auto_calib_status_label_->setText(
        QString("Status: Stopped. %1 samples collected. Solving...").arg(effector_wrt_world_.size()));
    if (solveCameraRobotPose())
    {
      auto_calib_status_label_->setText(QString("Status: Calibration successful with %1 samples")
                                            .arg(effector_wrt_world_.size()));
      auto_calib_status_label_->setStyleSheet("QLabel { color: green; }");
    }
  }
}

void ControlTabWidget::autoCalibrationTimerCallback()
{
  // This timer is used for waiting states (e.g., waiting for robot to stabilize)
  auto_calib_timer_->stop();

  if (!full_auto_calibration_running_)
    return;

  if (auto_calib_state_ == AUTO_WAITING_STABLE)
  {
    // Robot should be stable now, take sample
    auto_calib_state_ = AUTO_SAMPLING;
    auto_calib_status_label_->setText(QString("Status: Pose %1 - Checking target visibility...")
                                          .arg(full_auto_current_pose_index_ + 1));

    // Take the transform sample
    if (takeTransformSamples())
    {
      int num_samples = static_cast<int>(effector_wrt_world_.size());
      RCLCPP_INFO(node_->get_logger(), "Auto calibration: Sample %d/%d taken at pose %d",
                  num_samples, full_auto_total_poses_, full_auto_current_pose_index_ + 1);

      // Update progress bar to show actual successful samples
      full_auto_progress_->setValue(num_samples);

      // If we have enough samples, solve
      if (num_samples >= 5)
      {
        solveCameraRobotPose();
      }

      // Check if we have reached the target number of successful samples
      if (num_samples >= full_auto_total_poses_)
      {
        // Target reached! We're done
        auto_calib_state_ = AUTO_DONE;
        full_auto_calibration_running_ = false;

        auto_calibrate_btn_->setEnabled(true);
        stop_auto_calibrate_btn_->setEnabled(false);
        num_poses_spinbox_->setEnabled(true);

        if (solveCameraRobotPose())
        {
          auto_calib_status_label_->setText(QString("Status: Done! %1 samples collected successfully").arg(num_samples));
          auto_calib_status_label_->setStyleSheet("QLabel { color: green; font-weight: bold; }");
          RCLCPP_INFO(node_->get_logger(), "Auto calibration completed with target %d samples!", num_samples);
        }
        else
        {
          auto_calib_status_label_->setText(QString("Status: %1 samples collected but solve failed").arg(num_samples));
          auto_calib_status_label_->setStyleSheet("QLabel { color: orange; }");
        }
        return;
      }

      // Move to next pose
      full_auto_current_pose_index_++;

      if (full_auto_current_pose_index_ >= static_cast<int>(generated_poses_.size()))
      {
        // All candidate poses exhausted
        auto_calib_state_ = AUTO_DONE;
        full_auto_calibration_running_ = false;

        auto_calibrate_btn_->setEnabled(true);
        stop_auto_calibrate_btn_->setEnabled(false);
        num_poses_spinbox_->setEnabled(true);

        if (num_samples >= 5 && solveCameraRobotPose())
        {
          auto_calib_status_label_->setText(QString("Status: Done with %1/%2 samples (poses exhausted)").arg(num_samples).arg(full_auto_total_poses_));
          auto_calib_status_label_->setStyleSheet("QLabel { color: orange; }");
        }
        else
        {
          auto_calib_status_label_->setText(QString("Status: Failed - only %1 samples (need %2)").arg(num_samples).arg(full_auto_total_poses_));
          auto_calib_status_label_->setStyleSheet("QLabel { color: red; }");
        }
      }
      else
      {
        // Plan to next pose
        auto_calib_state_ = AUTO_PLANNING;
        auto_calib_status_label_->setText(QString("Status: %1/%2 samples, trying pose %3...")
                                              .arg(num_samples)
                                              .arg(full_auto_total_poses_)
                                              .arg(full_auto_current_pose_index_ + 1));
        plan_watcher_->setFuture(
            QtConcurrent::run(this, &ControlTabWidget::planToGeneratedPose, full_auto_current_pose_index_));
      }
    }
    else
    {
      int num_samples = static_cast<int>(effector_wrt_world_.size());
      RCLCPP_WARN(node_->get_logger(), "Auto calibration: Target not visible at pose %d, skipping (have %d/%d samples)",
                  full_auto_current_pose_index_ + 1, num_samples, full_auto_total_poses_);

      // Skip this pose and continue
      full_auto_current_pose_index_++;

      if (full_auto_current_pose_index_ >= static_cast<int>(generated_poses_.size()))
      {
        // All candidate poses exhausted
        auto_calib_state_ = AUTO_DONE;
        full_auto_calibration_running_ = false;

        auto_calibrate_btn_->setEnabled(true);
        stop_auto_calibrate_btn_->setEnabled(false);
        num_poses_spinbox_->setEnabled(true);

        if (num_samples >= 5 && solveCameraRobotPose())
        {
          auto_calib_status_label_->setText(QString("Status: Done with %1/%2 samples (poses exhausted)").arg(num_samples).arg(full_auto_total_poses_));
          auto_calib_status_label_->setStyleSheet("QLabel { color: orange; }");
        }
        else
        {
          auto_calib_status_label_->setText(QString("Status: Failed - only %1 samples (need %2)").arg(num_samples).arg(full_auto_total_poses_));
          auto_calib_status_label_->setStyleSheet("QLabel { color: red; }");
        }
      }
      else
      {
        // Try next pose
        auto_calib_state_ = AUTO_PLANNING;
        auto_calib_status_label_->setText(QString("Status: %1/%2 samples, skipped pose, trying %3...")
                                              .arg(num_samples)
                                              .arg(full_auto_total_poses_)
                                              .arg(full_auto_current_pose_index_ + 1));
        plan_watcher_->setFuture(
            QtConcurrent::run(this, &ControlTabWidget::planToGeneratedPose, full_auto_current_pose_index_));
      }
    }
  }
}

void ControlTabWidget::autoCalibrationPlanFinished()
{
  if (!full_auto_calibration_running_)
    return;

  int num_samples = static_cast<int>(effector_wrt_world_.size());

  if (planning_res_ == ControlTabWidget::SUCCESS)
  {
    // Planning succeeded, execute
    auto_calib_state_ = AUTO_EXECUTING;
    auto_calib_status_label_->setText(QString("Status: %1/%2 samples, moving to pose %3...")
                                          .arg(num_samples)
                                          .arg(full_auto_total_poses_)
                                          .arg(full_auto_current_pose_index_ + 1));
    execution_watcher_->setFuture(QtConcurrent::run(this, &ControlTabWidget::executeToGeneratedPose));
  }
  else
  {
    // Planning failed, skip to next pose
    RCLCPP_WARN(node_->get_logger(), "Auto calibration: Skipping pose %d due to planning failure",
                full_auto_current_pose_index_ + 1);

    full_auto_current_pose_index_++;

    if (full_auto_current_pose_index_ >= static_cast<int>(generated_poses_.size()))
    {
      // All candidate poses exhausted
      auto_calib_state_ = AUTO_DONE;
      full_auto_calibration_running_ = false;

      auto_calibrate_btn_->setEnabled(true);
      stop_auto_calibrate_btn_->setEnabled(false);
      num_poses_spinbox_->setEnabled(true);

      if (num_samples >= 5 && solveCameraRobotPose())
      {
        auto_calib_status_label_->setText(QString("Status: Done with %1/%2 samples (poses exhausted)").arg(num_samples).arg(full_auto_total_poses_));
        auto_calib_status_label_->setStyleSheet("QLabel { color: orange; }");
      }
      else
      {
        auto_calib_status_label_->setText(QString("Status: Failed - only %1 samples (need %2)").arg(num_samples).arg(full_auto_total_poses_));
        auto_calib_status_label_->setStyleSheet("QLabel { color: red; }");
      }
    }
    else
    {
      // Try next pose
      auto_calib_status_label_->setText(QString("Status: %1/%2 samples, plan failed, trying pose %3...")
                                            .arg(num_samples)
                                            .arg(full_auto_total_poses_)
                                            .arg(full_auto_current_pose_index_ + 1));
      plan_watcher_->setFuture(
          QtConcurrent::run(this, &ControlTabWidget::planToGeneratedPose, full_auto_current_pose_index_));
    }
  }
}

void ControlTabWidget::autoCalibrationExecuteFinished()
{
  if (!full_auto_calibration_running_)
    return;

  int num_samples = static_cast<int>(effector_wrt_world_.size());

  if (planning_res_ == ControlTabWidget::SUCCESS)
  {
    // Execution succeeded, wait for robot to stabilize then take sample
    auto_calib_state_ = AUTO_WAITING_STABLE;
    auto_calib_status_label_->setText(QString("Status: %1/%2 samples, waiting to stabilize...")
                                          .arg(num_samples)
                                          .arg(full_auto_total_poses_));

    // Wait 1 second for robot to stabilize and camera to see the target
    auto_calib_timer_->start(1000);
  }
  else
  {
    // Execution failed, skip to next pose
    RCLCPP_WARN(node_->get_logger(), "Auto calibration: Skipping pose %d due to execution failure",
                full_auto_current_pose_index_ + 1);

    full_auto_current_pose_index_++;

    if (full_auto_current_pose_index_ >= static_cast<int>(generated_poses_.size()))
    {
      // All candidate poses exhausted
      auto_calib_state_ = AUTO_DONE;
      full_auto_calibration_running_ = false;

      auto_calibrate_btn_->setEnabled(true);
      stop_auto_calibrate_btn_->setEnabled(false);
      num_poses_spinbox_->setEnabled(true);

      if (num_samples >= 5 && solveCameraRobotPose())
      {
        auto_calib_status_label_->setText(QString("Status: Done with %1/%2 samples (poses exhausted)").arg(num_samples).arg(full_auto_total_poses_));
        auto_calib_status_label_->setStyleSheet("QLabel { color: orange; }");
      }
      else
      {
        auto_calib_status_label_->setText(QString("Status: Failed - only %1 samples (need %2)").arg(num_samples).arg(full_auto_total_poses_));
        auto_calib_status_label_->setStyleSheet("QLabel { color: red; }");
      }
    }
    else
    {
      // Try next pose
      auto_calib_state_ = AUTO_PLANNING;
      auto_calib_status_label_->setText(QString("Status: %1/%2 samples, exec failed, trying pose %3...")
                                            .arg(num_samples)
                                            .arg(full_auto_total_poses_)
                                            .arg(full_auto_current_pose_index_ + 1));
      plan_watcher_->setFuture(
          QtConcurrent::run(this, &ControlTabWidget::planToGeneratedPose, full_auto_current_pose_index_));
    }
  }
}

}  // namespace moveit_rviz_plugin
