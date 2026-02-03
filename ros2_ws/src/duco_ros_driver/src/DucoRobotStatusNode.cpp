/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017-2018, SIASUN Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
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
 */


#include <string>
#include "rclcpp/rclcpp.hpp"
#include "duco_ros_driver/DucoRobotStatus.h"

using namespace duco_driver;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<DucoRobotStatus> n=std::make_shared<DucoRobotStatus>();

    // ros::AsyncSpinner spinner(2);
    // spinner.start();

    DucoRobotStatus robot_status;
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(n);
    while(rclcpp::ok())
    {
        executor->spin_some();  // 处理当前已经准备好的所有回调
        // 这里可以执行其他需要周期性执行的代码
        // ...
        // 让出 CPU 时间或执行其他非回调相关任务
        n->run();

        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 模拟工作负载
    }

    RCLCPP_WARN(n->get_logger(),"Exiting duco robot status");
    return(0);
}


