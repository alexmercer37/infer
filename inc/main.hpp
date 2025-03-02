/**
 * @file main.hpp
 * @author alexmercer37 (3450141407@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-08-17
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef MAIN_H
#define MAIN_H
// #define use_cu

#include "cpm.hpp"
#include "infer.hpp"
#include "yolo.hpp"

#ifndef use_cu
#include "OpencvHead.hpp"
#include "detect.hpp"
#include "uart.hpp"
#include "camera.hpp"
#include "pthread.hpp"
#include "tcp.hpp"
#include "realsense.hpp"
#include "pcl.hpp"
#include "PclHead.hpp"
#include <k4a/k4a.hpp>
#include <k4arecord/record.h>
#include <k4arecord/playback.h>
#endif

#include <pthread.h>
#include <unistd.h>
#include <cstdlib>
#include <semaphore.h>
#include <iostream>
#include <initializer_list>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <string.h>
#include <stdio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <signal.h>
#include <sys/prctl.h>
#include <time.h>

#define test_tcp
#define test_uart
// #define use_realsense
#define use_k4a
#define use_detect

#endif