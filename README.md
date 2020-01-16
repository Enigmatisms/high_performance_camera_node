# high_performance_camera_node
Hikvision High Performance Grabbing

//test_frame.cpp是ros的另一个节点，接收图像的例程
//CameraCtl.hpp
//CameraParam.cc是最新的摄像头控制模块
//修改了bug：长时间取流导致程序崩溃
//程序可读性以及可调试性++
//!!!CameraCtl中的namespace Camera重命名成cm，因为个人觉得太长
