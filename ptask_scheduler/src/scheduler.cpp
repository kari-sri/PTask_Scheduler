#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include <thread>

using namespace std::chrono_literals;

class TaskScheduler : public rclcpp::Node
{
public:
  TaskScheduler() : Node("task_scheduler"), task_a_id_(0), task_b_id_(0), task_c_id_(0)
  {
    // Task A: Period = 16ms, Deadline = 10ms, Execution time = 4ms
    timer_a_ = this->create_wall_timer(16ms, std::bind(&TaskScheduler::task_a, this));

    // Task B: Period = 12ms, Deadline = 8ms, Execution time = 2ms
    timer_b_ = this->create_wall_timer(12ms, std::bind(&TaskScheduler::task_b, this));

    // Task C: Period = 32ms, Deadline = 26ms, Execution time = 12ms
    timer_c_ = this->create_wall_timer(32ms, std::bind(&TaskScheduler::task_c, this));
  }

private:
  // Job IDs to track individual releases of tasks
  int task_a_id_, task_b_id_, task_c_id_;

  void task_a()
  {
    ++task_a_id_;  // Increment Job ID for Task A
    auto release_time = this->now();
    RCLCPP_INFO(this->get_logger(), "Task A (Job ID: %d) released at time: %f", task_a_id_, release_time.seconds());

    // Simulate execution time of 4ms
    std::this_thread::sleep_for(4ms);
    auto completion_time = this->now();
    double response_time = (completion_time - release_time).seconds();

    RCLCPP_INFO(this->get_logger(), "Task A (Job ID: %d) completed at time: %f", task_a_id_, completion_time.seconds());
    RCLCPP_INFO(this->get_logger(), "Task A (Job ID: %d) response time: %f ms", task_a_id_, response_time * 1000);

    // Check for deadline miss (Deadline = 10ms)
    if (response_time > 0.010) {
      RCLCPP_WARN(this->get_logger(), "Task A (Job ID: %d) missed its deadline!", task_a_id_);
    } else {
      RCLCPP_INFO(this->get_logger(), "Task A (Job ID: %d) met its deadline.", task_a_id_);
    }
  }

  void task_b()
  {
    ++task_b_id_;  // Increment Job ID for Task B
    auto release_time = this->now();
    RCLCPP_INFO(this->get_logger(), "Task B (Job ID: %d) released at time: %f", task_b_id_, release_time.seconds());

    // Simulate execution time of 2ms
    std::this_thread::sleep_for(2ms);
    auto completion_time = this->now();
    double response_time = (completion_time - release_time).seconds();

    RCLCPP_INFO(this->get_logger(), "Task B (Job ID: %d) completed at time: %f", task_b_id_, completion_time.seconds());
    RCLCPP_INFO(this->get_logger(), "Task B (Job ID: %d) response time: %f ms", task_b_id_, response_time * 1000);

    // Check for deadline miss (Deadline = 8ms)
    if (response_time > 0.008) {
      RCLCPP_WARN(this->get_logger(), "Task B (Job ID: %d) missed its deadline!", task_b_id_);
    } else {
      RCLCPP_INFO(this->get_logger(), "Task B (Job ID: %d) met its deadline.", task_b_id_);
    }
  }

  void task_c()
  {
    ++task_c_id_;  // Increment Job ID for Task C
    auto release_time = this->now();
    RCLCPP_INFO(this->get_logger(), "Task C (Job ID: %d) released at time: %f", task_c_id_, release_time.seconds());

    // Simulate execution time of 12ms
    std::this_thread::sleep_for(12ms);
    auto completion_time = this->now();
    double response_time = (completion_time - release_time).seconds();

    RCLCPP_INFO(this->get_logger(), "Task C (Job ID: %d) completed at time: %f", task_c_id_, completion_time.seconds());
    RCLCPP_INFO(this->get_logger(), "Task C (Job ID: %d) response time: %f ms", task_c_id_, response_time * 1000);

    // Check for deadline miss (Deadline = 26ms)
    if (response_time > 0.026) {
      RCLCPP_WARN(this->get_logger(), "Task C (Job ID: %d) missed its deadline!", task_c_id_);
    } else {
      RCLCPP_INFO(this->get_logger(), "Task C (Job ID: %d) met its deadline.", task_c_id_);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_a_;
  rclcpp::TimerBase::SharedPtr timer_b_;
  rclcpp::TimerBase::SharedPtr timer_c_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskScheduler>());
  rclcpp::shutdown();
  return 0;
}
