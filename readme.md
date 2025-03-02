# dxy-infer配置
### locate的问题
ubuntu可以利用locate进行目标文件定位，但在使用过程中出现了添加或者删除文件不及时的问题，需要对数据库进行更新，因为locate指令是针对数据库运行的
> https://blog.csdn.net/accumulate_zhang/article/details/77587136
### gcc,g++编译器不支持问题
由于当前环境需要nvcc以及gcc,g++联合编译，系统为24.04，默认安装的gcc,g++版本很高，是13.2,而当前只支持小于12版本的编译器，所以需要对其进行降级处理，cmake版本目前是3.28,不过目前来看没有太大影响
> https://huanghailiang.github.io/2017/07/22/ubuntu-Reduce-the-version-gcc-g++/

> https://blog.csdn.net/CharlieVV/article/details/111242143
### undefined reference to 问题解决
首先怀疑是编译器冲突，所以对nvcc与c++作了分开处理，首先对.cu文件使用nvcc编译器进行编译，生成一个静态链接库，之后通过调用静态链接库编译.cpp文件，没有解决，其次发现没有被链接的函数并不是全部，所以对于一部分函数作了处理，直接在.hpp文件中进行定义，需要用inline函数，解决了一部分问题，但是opencv的函数仍然有相同报错，所以怀疑是opencv的版本问题，于是将版本从4.9.0降到了4.5.1,仍然没有解决问题，最后在查找相关资料时发现可能是cmake的配置问题，opencv的配置和cuda的配置可能不匹配，对cmakelists.txt进行了修改，发现问题解决了
> https://blog.csdn.net/LLY_A_/article/details/120385775

> https://mbd.baidu.com/ug_share/mbox/4a83aa9e65/share?product=smartapp&tk=e3c721c7f5214c2c21bcee5a54cecc3b&share_url=https%3A%2F%2Fyebd1h.smartapps.cn%2Fpages%2Fblog%2Findex%3FblogId%3D139819798%26_swebfr%3D1%26_swebFromHost%3Dbaiduboxapp&domain=mbd.baidu.com
### 安装opencv
降低opencv版本时碰到了问题，首先是cmake进行编译时与anaconda有冲突，导致安装不了，可以暂时将anaconda的安装包删除，可以解决冲突问题，其他出现的问题详见下方网页
> https://blog.csdn.net/KIK9973/article/details/118830187

> https://blog.csdn.net/u010003609/article/details/100087245

> https://blog.csdn.net/CSDNWWC/article/details/129205596
### tensorRT版本问题
目前所用的版本是8.6.1.6,可以正常运行，高版本目前测试10.0以上版本无法正常运行，安装的版本没有报错即可使用。报错如下
> class "nvinfer1::ICudaEngine" has no member "getNbBindings"
> 
> class "nvinfer1::ICudaEngine" has no member "bindingIsInput"
> 
> error: class "nvinfer1::ICudaEngine" has no member "getBindingName"
> 
> error: class "nvinfer1::ICudaEngine" has no member "getBindingDimensions"
### engine文件的生成
直接用tensorRT即可通过.onnx文件生成.engine文件，注意使用的文件目前支持fp16，不支持fp32，本架构支持yolov5,yolov7,yolov8,yolov8-seg，对于yolov5，有成熟的修改后的安装包，可作保底方案，但对于目前的比赛来说有些过时，包括今年比赛的推理环境也需要更新，所以今年可以尝试用此推理环境和yolov8进行项目搭建
> https://blog.csdn.net/qq_43799400/article/details/134182459
### 运行代码报错解决
- 报错Error Code 1: Cuda Runtime (an illegal memory access was encountered)，CUDA Runtime error💥 cudaFreeHost(cpu_) # an illegal memory access was encountered, code = cudaErrorIllegalAddress [ 700 ]
原因是加载的engine的类型不正确（fp32不支持），需要在训练时指定类型

- 报错CUDA Runtime error💥 cudaPeekAtLastError() # no kernel image is available for execution on the device, code = cudaErrorNoKernelImageForDevice [ 209 ]
原因是cmakelists.txt设置的gpu算力和本电脑的显卡gpu的算力不匹配
> https://blog.csdn.net/FL1623863129/article/details/124207995
> https://en.wikipedia.org/wiki/CUDA#GPUs_supported
出现报错 libstdc++.so.6 GLIBCXX_3.4.32 not found以及相关的.so文件没有找到，一般是软链接的问题，可以参考以下网址
> https://blog.csdn.net/HITORANGE/article/details/135315149
### 环境配置
- 显卡驱动rtx2060，版本535.183.01，最高支持cuda12.2
- cuda12.2,cudnn和cuda版本匹配即可
  >https://blog.csdn.net/tangjiahao10/article/details/125225786
- 架构代码时使用的opencv4.5.1，在测试opencv的dnn时更新了opencv的版本，目前版本是4.9.0，安装方便没有错误，可以适配该代码
- pcl1.13.1，目前没有深入进行编译测试
- gcc,g++版本11.4,cmake版本3.28.3
- tensorRT版本8.6.1.6
- gpu算力75
### 使用方法
- 进入tensorRT的路径，例如
  > /home/ddxy/project/TensorRT-8.6.1.6.Linux.x86_64-gnu.cuda-12.0/TensorRT-8.6.1.6/targets/x86_64-linux-gnu/bin 

- 利用可执行文件trtexec将onnx文件进行转换，例如
  > ./trtexec --onnx=yolov8n.transd.onnx \
    --minShapes=images:1x3x640x640 \
    --maxShapes=images:16x3x640x640 \
    --optShapes=images:1x3x640x640 \
    --saveEngine=yolov8n.transd.engine

  > ./trtexec --onnx=yolov8n-seg.b1.transd.onnx \
    --saveEngine=yolov8n-seg.b1.transd.engine
- #### 注意其中的报错或者警告
- .pt文件转换onnx文件教程后续添加
- 将转换的.onnx通过tensorRT转换为.engine文件，放在指定路径下，通过函数load即可调用
- 本文件需要分别对.cu文件与.cpp文件进行分别编译，在进行.cu编译时可以将针对.cpp文件的CMakeLists.txt改名，将src/kernels中的.cu文件和CMakeLists.txt文件放在src文件夹外，对main.hpp进行更改，将// #define use_cu的注释解除，编译完成后再放入src/kernels中，将之前改名的cmakelists.txt文件的名字改回（一个目录下只允许有一个cmakelist.txt文件），在build里面找到libcu.a文件
- 对文件中的src/kernels进行单独编译后，生成的libcu.a静态链接库可以在build文件夹中找到，复制到workspace文件夹中，之后删除build文件夹，对main.hpp进行更改，将#define use_cu重新进行注释，对针对.cpp的cmakelists.txt进行编译，得到可执行文件pro，在workspace文件夹中，运行即可
- 可以使用time函数进行速度检测，经测试，速度可以达到0.5ms一次推理，但由于相机帧数的限制，只能达到30帧的速度
### 代码介绍
- cpm.hpp：生产者-消费者模型，对于直接推理任务，cpm.hpp 可以转变为自动多批次生产者-消费者模型
- infer.hpp：tensorRT 的重新包装，接口简单
- yolo.hpp：yolo 任务的包装器。基于 infer.hpp
---
# 版本更新`
### 8.17
#### `代码的完善`
- 多线程更新，更新了调用k4a相机的线程，通过线程成功调用yolov8的目标检测和分割，同时测试了调用电脑的原装相机的线程，对多线程做了一定的封装
  >https://immortalqx.github.io/2021/12/05/cpp-notes-4/
- 串口调用的更新，封装了串口的类，在主函数写入了通过串口接受不同的消息来启动不同的线程
- 更新了c++_python的联合编译，加入了自己搭建的python环境下的pytorch的目标检测文件
- 更新了24年上车的自启动脚本
- 更新了tcp网络通信，进行了简单封装，并且在主函数中可以通过多线程调用
- 加入了linux的can通信相关内容，加入了循环队列，配合can通信的使用
- 利用宏定义对代码进行了注释，方便调用和修改
- 解决了相机数据突然丢失但是不退出运行的遗留问题，主要针对深度图丢失导致的rgb图像的不更新，从而只针对一张固定的图片进行推理，利用rgb转灰度图，标准库的函数对连续两张灰度图的差异进行检测，判定如果连续十张图片都不变化，则认为相机不更新，进而抛出异常，中断程序，等待自启动脚本的重新启动。
- 增加了一些抛异常，使整个代码更加严谨，更方便找到问题所在
### 8.18
#### `yolov8相关内容`
- 更新yolov8的使用教程，安装包在下方网址：
  >https://github.com/ultralytics/ultralytics/tree/main?tab=readme-ov-file
- yolov8使用教程，需要自己写train.py，export.py等等文件，可以在本文档的yolov8中找到
  >https://docs.ultralytics.com/tasks/detect/
- 使用方法，首先通过指定训练的预训练集，本文使用的是yolov8n.pt，适用于处理模型较为简单的检测，可以指定训练中的参数，详见下方网址
  >https://mmyolo.readthedocs.io/zh-cn/latest/recommended_topics/algorithm_descriptions/yolov8_description.html

  >https://blog.csdn.net/StopAndGoyyy/article/details/140071942
- 指定需要使用的yaml文件，主要是cfg类型和data类型，cfg主要是指明需要检测的目标的一些参数，通过nc可以对被检测目标的类型个数进行指定，data类型主要是指明数据集的位置
- 在终端中运行python脚本，可以通过指令指明一些需要的参数，例如--cache ram可以加速训练，有显卡驱动可以有效加速训练，记得安装anaconda进行python环境的配置
- 成功训练之后可以在run文件夹下找到训练完成的.pt文件，以及训练中的参数，例如损失函数的值以及目标检测的准确度，通过export.py脚本可以将.pt文件转换成.onnx文件
- 一般来说训练的文件是TF32类型，由于tensorRT不支持该类型，需要将其转换为TF16类型，通过v8trans.py进行转换，将转换完成的.onnx文件在tensorRT中利用可执行文件转换成engine文件，在代码中调用
- 发现了一个segmentation fault，由于auto caption = cv::format("%s %.2f", name, obj.confidence);存在风险，可能造成内存溢出，所以用了数据流进行更加安全的操作
- 利用yolov8对红球和蓝球进行了训练，数据集只有不到1000,训练的epoch达到1000，效果超出预期，说明代码及其使用的架构的更新非常重要
- 数据集的制作详见yolov5数据集的制作，两者没有大的差异
- yolov8目标检测的相关信息的介绍

| Model                                                                                | size<br><sup>(pixels) | mAP<sup>val<br>50-95 | Speed<br><sup>CPU ONNX<br>(ms) | Speed<br><sup>A100 TensorRT<br>(ms) | params<br><sup>(M) | FLOPs<br><sup>(B) |
| ------------------------------------------------------------------------------------ | --------------------- | -------------------- | ------------------------------ | ----------------------------------- | ------------------ | ----------------- |
| [YOLOv8n](https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.pt) | 640                   | 37.3                 | 80.4                           | 0.99                                | 3.2                | 8.7               |
| [YOLOv8s](https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8s.pt) | 640                   | 44.9                 | 128.4                          | 1.20                                | 11.2               | 28.6              |
| [YOLOv8m](https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8m.pt) | 640                   | 50.2                 | 234.7                          | 1.83                                | 25.9               | 78.9              |
| [YOLOv8l](https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8l.pt) | 640                   | 52.9                 | 375.2                          | 2.39                                | 43.7               | 165.2             |
| [YOLOv8x](https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8x.pt) | 640                   | 53.9                 | 479.1                          | 3.53                                | 68.2               | 257.8             |


### `8.19`
#### 更新python环境配置中的一些问题
- PackagesNotFoundError: The following packages are not available from current channels的解决办法
  >https://blog.csdn.net/weixin_45552562/article/details/109668589
- ModuleNotFoundError: No module named 'PIL'
  >https://blog.csdn.net/weixin_44037416/article/details/96842058
- ModuleNotFoundError: No module named 'yaml'
  >https://blog.csdn.net/HuanCaoO/article/details/104629625
- 更新了推理的可视化，目标框的颜色可以进行随机变化
### `8.20`
#### 更新环境配置中碰到的问题
- pcl编译时报错没有flann，安装一个flann的包编译安装即可，记得make和make install
  >https://blog.csdn.net/weixin_42518636/article/details/122079825
- pcl编译时报错fatal error: lz4.h: No such file or directory
 #include "lz4.h"
  >https://askubuntu.com/questions/774806/fatal-error-lz4-h-no-such-file-or-directory-include-lz4-h
- pcl编译时报错`/bin/ld: …/…/lib/libpcl_kdtree.so.1.8.0: undefined reference to LZ4_resetStreamHC'/bin/ld: ../../lib/libpcl_kdtree.so.1.8.0: undefined reference to LZ4_setStreamDecode’/bin/ld: …/…/lib/libpcl_kdtree.so.1.8.0: undefined reference to LZ4_decompress_safe' /bin/ld: ../../lib/libpcl_kdtree.so.1.8.0: undefined reference to LZ4_decompress_safe_continue’/bin/ld: …/…/lib/libpcl_kdtree.so.1.8.0: undefined reference to LZ4_compress_HC_continue’ `
  >https://blog.csdn.net/weixin_45534376/article/details/132864676
- 中文输入法的简单安装
  >https://blog.csdn.net/windson_f/article/details/124932523
- 一键安装和卸载ros
  >https://www.cnblogs.com/tdyizhen1314/p/16854333.html
- 在其他电脑上配置了该文件，发现有几个问题，在编译.cu文件时会涉及到main.hpp中的文件，可能会报错，在此作了用宏定义去注释部分代码来解决，并更新了使用说明
### `8.29`
#### 重装系统中的问题更新
- opencv安装时报错`error: (-2) The function is not implemented. Rebuild the library with Windows, GTK+ 2.x or Carbon support. If you are on Ubuntu or Debian, install libgtk2.0-dev and pkg-config, then re-run cmake or configure script in function cvNamedWindow`，忘记安装依赖的环境libgtk2.0-dev，记得安装后对opencv重新编译
> https://blog.csdn.net/u011622208/article/details/109117436
- 换源教程,24.04换20.04
> https://blog.csdn.net/orcer1994/article/details/107497822?utm_medium=distribute.pc_relevant.none-task-blog-2~default~baidujs_baidulandingword~default-0-107497822-blog-106069030.235^v43^pc_blog_bottom_relevance_base2&spm=1001.2101.3001.4242.1&utm_relevant_index=3
- 更新了队列的模板封装
- 更新了安装realsenseD435的教程
  >https://blog.csdn.net/qq_41721951/article/details/123507939
- 安装相机驱动的问题解决:udev的权限问题，通过在udev中设置权限解决
  >https://blog.csdn.net/chrisspf/article/details/122428884
- 更新了realsense的调用代码，可以通过多线程进行调用
### `9.2`
#### 更新realsense的调用
- 更新了realsense的深度图和彩色图的调用，并进行图像配准和对齐
### `9.5`
#### 更新yolov8环境的配置问题
- Python:PackageNotFoundError No package metadata was found for <myproject>
  >https://stackoverflow.com/questions/77368894/python-packagenotfounderror-no-package-metadata-was-found-for-myproject
- numpy的报错问题，一般是版本的问题，经过测试numpy版本在2.0.0以上时会出现报错，本电脑版本安装为1.26.0,可以正常运行
- 更新数据集之后的实验

### `9.11`
#### 更新dzx的卡尔曼滤波

- 2025RC传感器
    1. 场外定位？
    2. 球的轨迹预测
    3. 雷达对于篮筐的定位
    4. 全场定位的实现（传统方案能否使用？）
    5. 雷达与相机结合的定位？
    6. 高帧数的相机™
    7. 高性能的迷你主机
    8. 2015年电科视觉
    9. 