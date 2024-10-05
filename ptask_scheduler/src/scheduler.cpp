#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"

class Scheduler : public rclcpp::Node {
public:
    Scheduler() : Node("scheduler") {
        // Set timers for tasks A, B, C
        timer_a_ = create_wall_timer(
            std::chrono::milliseconds(16), std::bind(&Scheduler::task_a_callback, this));
        timer_b_ = create_wall_timer(
            std::chrono::milliseconds(12), std::bind(&Scheduler::task_b_callback, this));
        timer_c_ = create_wall_timer(
            std::chrono::milliseconds(32), std::bind(&Scheduler::task_c_callback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_a_;
    rclcpp::TimerBase::SharedPtr timer_b_;
    rclcpp::TimerBase::SharedPtr timer_c_;

    void task_a_callback() {
        auto now = this->now();
        RCLCPP_INFO(this->get_logger(), "Task A (Release) executed at time: %ldms", now.seconds() * 1000);
        // Log additional data for completion, response time, and deadline
    }

    void task_b_callback() {
        auto now = this->now();
        RCLCPP_INFO(this->get_logger(), "Task B (ka_scheduler) (Release) executed at time: %ldms", now.seconds() * 1000);
        // Log additional data for completion, response time, and deadline
    }

    void task_c_callback() {
        auto now = this->now();
        RCLCPP_INFO(this->get_logger(), "Task C (Release) executed at time: %ldms", now.seconds() * 1000);
        // Log additional data for completion, response time, and deadline
    }

    // Additional methods to calculate and log response times and deadlines
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Scheduler>());
    rclcpp::shutdown();
    return 0;
}

