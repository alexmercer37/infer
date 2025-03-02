/**
 * @file main.cpp
 * @author alexmercer37 (3450141407@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-08-17
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "../inc/main.hpp"

using namespace std;

int main()
{

  pthread_mutex_init(&buff_mutex, nullptr);

  matBuff.reset(new cv::Mat);
  depthBuff.reset(new cv::Mat);
  greyBuff.reset(new cv::Mat);

  pthread_t threads[10] = {0};

#ifdef use_realsense

  pthread_create(&threads[1], NULL, pthread::realsenseUpdate, NULL);

#endif

#ifdef use_k4a

  pthread_create(&threads[0], NULL, pthread::k4aUpdate, NULL);

#endif

#ifdef use_detect

  pthread_create(&threads[1], NULL, pthread::create_infer, NULL);
  // pthread_create(&threads[2], NULL, pthread::create_infer_seg, NULL);
  // pthread_create(&threads[3], NULL, pthread::usb_camera_infer, NULL);

#endif

#ifndef test_uart

  tcflush(fd, TCIOFLUSH);
  int receive_data;
  UART uart;
  while (1)
  {
    uart.uart_init();
    sleep(1);
    uart.libtty_receive(fd, receive_data);

    cout << receive_data << endl;

    if (receive_data == 1)
    {
      pthread_create(&threads[0], NULL, pthread::k4aUpdate, NULL);
      pthread_create(&threads[1], NULL, pthread::create_infer, NULL);
      pthread_create(&threads[2], NULL, pthread::create_infer_seg, NULL);
      pthread_create(&threads[3], NULL, pthread::usb_camera_infer, NULL);

      receive_data = 0;

      while (1)
        usleep(100);
    }
    else if (receive_data == 2)
    {
      pthread_create(&threads[0], NULL, pthread::k4aUpdate, NULL);
      pthread_create(&threads[1], NULL, pthread::create_infer, NULL);
      pthread_create(&threads[2], NULL, pthread::create_infer_seg, NULL);
      pthread_create(&threads[3], NULL, pthread::usb_camera_infer, NULL);

      receive_data = 0;
      while (1)
        usleep(100);
    }
  }
#endif

#ifndef test_tcp
  pthread_create(&threads[4], NULL, TCP::TCP_Manage::tcp_recv, NULL);
  pthread_create(&threads[4], NULL, TCP::TCP_Manage::tcp_send, NULL);
#endif

#ifndef python_test_succeed
  pthread_create(&threads[3], NULL, pthread::detect_python, NULL);
#endif

  while (1)
  {
    usleep(100);
  }
  return 0;
}
