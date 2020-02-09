/*==============ROS高性能取流节点(c++ std::thread实现)================
作者：hqy
创建日期：2020.1.22 10:39
最后修改：2020.2.5 14:03
主要内容：对HPGrabbing节点使用的全局函数的封装
    1.双线程同时取流并且解码，解码后直接进行图像处理（解码和处理是同一线程）
    2.现在估计的取流会比解码更慢，故解码位置是由取流位置决定的
    3.目前帧数最大值：88fps
    处理思路：
        两个线程，不对外发送图像，图像解码后立即进入处理，读取、解码、处理、发送信息给serial_com是同一线程
        第一个线程用于取流，写入，第二个进程的循环是用subscirber驱动的，只有接收到gimbal的位置
        才能在下一次装甲板检测，判断云台位置时用上本次云台位置
    2.3日修改：使grab操作更加集成化，再循环内不要有太多内部的操作
        1.取流直接调用函数返回一个Mat
        2.需要防止在使用Mat时对HPG对于Mat的修改
        3.
    2.5日修改：线程管理封装，在类内部开启子线程
TODO: 可能的日后修改：使用mutex实现互斥操作
*/

#ifndef _HP_GRABBING_HPP 
#define _HP_GRABBING_HPP 

#include <iostream>
#include <thread>
#include <opencv2/core.hpp>
#include "CameraCtl.hpp"
#ifndef DEBUG
    #define print(...)                                //空串替换
#else
    #define print printf
#endif

namespace hpg{

const int NULL_POS = -1;
const int GRAB_OK = 0, GRAB_FAILED = 1;
const char* const DEFAUL_LOAD_PATH= "/home/sentinel/ROSWorkspace/Autoaim/src/armor/cv_output2.avi"; 

enum STATUS{
    BUSY        = 0,     //繁忙，不支持读写
    AVAIILABLE  = 1,     //支持读写
};

enum MODE{
    FROM_CAM                = 0,           //高性能
    FROM_VIDEO              = 1,           //从视频取流
    FROM_CAM_LOW_PERFORM    = 2,           //低性能
};

struct ImageBuffer{
    volatile int r_tag = BUSY;          
    volatile int w_tag = AVAIILABLE;          
    MV_FRAME_OUT frame;                 //帧数据
};

class HPGrabbing{
public:
    HPGrabbing(int mode = FROM_CAM, const char* const path = DEFAUL_LOAD_PATH);
    ~HPGrabbing();

    int getMat(cv::Mat &src);                               //src入参/输出  

    void start();                                //外部开启并detach进程
    /**
     * @brief 设置buf[pos]的读写标签：设为可读，不可写，在read、图像处理完毕后调用
     * @param pos 缓冲区下标
    */
    void unlockBuffer();                                    //重置对应缓冲区，使其可以被写入（读取处理成功后调用）
public:
    bool grab;                                              //是否进行grab的标签
private:
    /** 
     * @brief 写入操作线程主循环，
     * @param pUser CameraCtl中的相机操作句柄（handle）
    */
    void* writeWorkThread(void *pUser);  
    bool isWritable(const int pos);                         //bool:pos是否可写
    bool isReadable(const int pos);                         //bool:pos是否可读                
    void decoding(const int pos, cv::Mat &src);             //缓冲区图像解码
    void readBuffer(const int pos, cv::Mat &src); //对pos进行写入操作，包括解码和图像处理

    /**
     * @brief 对缓冲区的pos位置进行写入操作
     * @param pUser 相机控制模块中的handle
     * @param pos 缓冲区下标，取值为0或1（双缓冲）
     * @param e 入参/输出-SDK错误代码
     * @return 下一个写入的缓冲区下标
    */
    int writeBuffer(void* pUser, int pos, int &e);
private:
    int next_pos;                                   //下一次读取的位置
    int last_pos;                                   //上一次读取的时间
    int grab_mode;                                  //取流方式from_cam还是from_video
    cm::CameraCtl ctl;    
    cv::Mat frame;
    cv::VideoCapture cap;                           //对视频播放进行集成
    ImageBuffer buf[2];                             //乒乓缓冲池
};

HPGrabbing::HPGrabbing(int mode, const char* const path){
    for(int i=0; i<2; ++i){
        buf[i].r_tag = BUSY;                        //初始时默认不可读取，需等待第一次写入成功后方可读入
        buf[i].w_tag = AVAIILABLE;
        buf[i].frame.pBufAddr = NULL;
    }
    grab = true;
    next_pos = 1;
    last_pos = NULL_POS;
    frame.create(1080, 1440, CV_8UC3);                          //定义frame格式    
    if(mode == FROM_VIDEO){
        cap.open(path);
    }
    grab_mode = mode;
}

HPGrabbing::~HPGrabbing(){;}

void HPGrabbing::start(){
    switch(grab_mode){
        case FROM_CAM:{
            ctl.startGrabbing();
            auto func = std::bind(&HPGrabbing::writeWorkThread, this, ctl.handle);
            std::thread w_thread(func);
            w_thread.detach();                                          //主线程创建后，子线程一直在工作
            break;
        }
        case FROM_VIDEO:
            {printf("Video loaded from the specified location.\n"); break;}
        case FROM_CAM_LOW_PERFORM:{
            ctl.startGrabbing();
            printf("Low performance grabbing.\n"); break;
            }
        default:
            printf("Error occurs. Un-existing grab_mode:%d", grab_mode);
    }
}

void* HPGrabbing::writeWorkThread(void* pUser){
    int next = 0, pos = 0, error = MV_OK;
    while(1){
        pos = next;                                             //保留本次写入位置
        next = writeBuffer(pUser, next, error);                 //每次写入之前会free对应g_buf
        #ifdef DEBUG
            if(next == NULL_POS){
                print("No data[0x%x].\n", error);
            }
        #endif
        if(buf[pos].frame.pBufAddr != NULL){                //先释放存在已经写入数据的位置
            error = MV_CC_FreeImageBuffer(pUser, &buf[pos].frame);
            
            #ifdef DEBUG
                if(error != MV_OK){
                    printf("Freeing previous frame failed[0x%x].\n", error);
                }
                printf("Freed one spot.\n");
            #endif

        }
        if(!grab) break;
    }
    printf("Exiting thread: writing...\n");
    return 0;
}

int HPGrabbing::getMat(cv::Mat &src){
    if(grab_mode == FROM_CAM){
        bool judge = isReadable(next_pos);
        if(judge) {
            readBuffer(next_pos, src);
            last_pos = next_pos;
            return GRAB_OK;
        }
        else{
            judge = isReadable(1 - next_pos);
            if(judge) {
                readBuffer(1 - next_pos, src);
                last_pos = 1 - next_pos;
                return GRAB_OK;
            }
        }
        return GRAB_FAILED;
    }
    else if(grab_mode == FROM_VIDEO){
        return cap.read(src) ? GRAB_OK : GRAB_FAILED;
    }
    else{
        src = ctl.getOpencvMat();
        return src.empty() ? GRAB_FAILED : GRAB_OK;
    }
}

int HPGrabbing::writeBuffer(void* pUser, int pos, int &e){
    if(isWritable(pos)){
        buf[pos].w_tag = BUSY;
        buf[pos].r_tag = BUSY;
        e = MV_CC_GetImageBuffer(pUser, &buf[pos].frame, 1000);
        if(e != MV_OK) {
            print("Failed to get image buffer.[0x%x]\n", e);
            return NULL_POS;
        }
    }
    buf[pos].w_tag = BUSY;                     
    buf[pos].r_tag = AVAIILABLE;                //对应位置写入完毕，不可写入，可以读取
    print("Writing succeeded. [%d].r_tag, w_tag=(%d, %d)\n",
        pos, buf[pos].r_tag, buf[pos].w_tag);
    next_pos = pos;                             //可以read的位置在pos处（本次读取位置）
    return 1-pos;                               //返回下一个读取位置                   
}

void HPGrabbing::readBuffer(const int pos, cv::Mat &src){
    buf[pos].r_tag = BUSY;
    buf[pos].w_tag = BUSY;
    decoding(pos, src);
}

void HPGrabbing::decoding(const int pos, cv::Mat &src){
    cv::Mat yuyv_img(1080, 1440, CV_8UC2, buf[pos].frame.pBufAddr);
    //if(src == nullptr) cv::cvtColor(yuyv_img, frame, cv::COLOR_YUV2BGR_YUYV);
    cv::cvtColor(yuyv_img, src, cv::COLOR_YUV2BGR_YUYV);
    print("Decoding is a success.\n");
}

void HPGrabbing::unlockBuffer(){                              
    if(grab_mode == FROM_CAM){
        buf[last_pos].r_tag = BUSY;                               //一个位置只能解码一次
        buf[last_pos].w_tag = AVAIILABLE;          
        print("Reading succeeded. [%d].r_tag, w_tag=(%d, %d)\n",
            pos, buf[last_pos].r_tag, buf[last_pos].w_tag);
    }
}

inline bool HPGrabbing::isReadable(const int pos){
    return (buf[pos].r_tag == AVAIILABLE);             //此处可读
}

bool HPGrabbing::isWritable(const int pos){
    print("Writing to [%d], isWritable:%d, isReadable:%d.\n", 
        pos, buf[pos].w_tag, buf[pos].r_tag);
    while(buf[pos].w_tag == BUSY){;}                 //自旋锁,无超时保护            
    return true;
}
}       //namespace hpg;                        
#endif  //_HP_GRABBING_HPP