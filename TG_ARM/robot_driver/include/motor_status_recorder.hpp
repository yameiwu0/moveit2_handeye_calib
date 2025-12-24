#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <fstream>
#include <memory>
#include <vector>
#include <mutex>
#include <iomanip>
#include "robot_can_driver.hpp"

class MotorStatusRecorder
{
public:
    MotorStatusRecorder(double record_rate_hz = 1000.0)
        : recording_(false)
    {
        record_period_ = (record_rate_hz > 0.0) ? 1.0 / record_rate_hz : 0.001;
    }

    ~MotorStatusRecorder() { stop(); }

    // 开始记录，传入文件路径和 RobotCanDriver 指针
    bool start(const std::string &file_path,
               std::shared_ptr<motor_driver::RobotCanDriver> driver)
    {
        if (recording_) return false;

        file_path_ = file_path;
        driver_ = driver;
        data_cache_.clear();
        start_time_ = rclcpp::Clock(RCL_ROS_TIME).now();
        recording_ = true;

        RCLCPP_INFO(rclcpp::get_logger("MotorStatusRecorder"), "Started recording to %s", file_path_.c_str());
        return true;
    }

    // 停止记录并写入之前指定的文件
    void stop()
    {
        recording_ = false;

        if (file_path_.empty() || data_cache_.empty())
            return;

        std::ofstream file(file_path_, std::ios::out);
        if (!file.is_open())
        {
            RCLCPP_ERROR(rclcpp::get_logger("MotorStatusRecorder"), "Failed to open file: %s", file_path_.c_str());
            return;
        }

        // 写表头
        file << "timestamp";
        for (size_t i = 0; i < motor_driver::MOTOR_NUM; i++) file << ",pos" << i;
        for (size_t i = 0; i < motor_driver::MOTOR_NUM; i++) file << ",vel" << i;
        file << "\n";

        // 写缓存
        for (auto &row : data_cache_)
        {
            file << std::fixed << std::setprecision(6) << row.timestamp;
            for (auto &p : row.positions) file << "," << p;
            for (auto &v : row.velocities) file << "," << v;
            file << "\n";
        }
        file.close();
        data_cache_.clear();
        RCLCPP_INFO(rclcpp::get_logger("MotorStatusRecorder"), "Saved %zu points to %s", data_cache_.size(), file_path_.c_str());
    }

    // 1kHz 或任意频率调用
    void record()
    {
        if (!recording_ || !driver_) return;

        rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();
        double t = (now - start_time_).seconds();

        std::vector<double> positions, velocities;
        for (uint8_t motor_id = 1; motor_id <= motor_driver::MOTOR_NUM; motor_id++)
        {
            auto motor_status = driver_->get_motor_status(motor_id);
            if (motor_status)
            {
                positions.push_back(motor_status->position * motor_driver::PI / 180.0); // deg -> rad
                velocities.push_back(motor_status->velocity);
            }
            else
            {
                positions.push_back(0.0);
                velocities.push_back(0.0);
            }
        }

        std::lock_guard<std::mutex> lock(data_mutex_);
        data_cache_.push_back({t, positions, velocities});
    }

private:
    struct RowData
    {
        double timestamp;
        std::vector<double> positions;
        std::vector<double> velocities;
    };

    std::shared_ptr<motor_driver::RobotCanDriver> driver_;
    std::atomic<bool> recording_;
    double record_period_;
    rclcpp::Time start_time_;
    std::vector<RowData> data_cache_;
    std::mutex data_mutex_;
    std::string file_path_;
};