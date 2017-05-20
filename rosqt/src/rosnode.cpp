#include "rosnode.h"
#include "asmOpencv.h"
#include "rosqt/setVF.h"
#include <QApplication>
#include <QLabel>
#include <QTimer>
#include <QtMath>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <QDebug>
Rosnode::Rosnode(QWidget* parent, const ros::NodeHandle &nh):
    QWidget(parent),pixmap(0),nh_(nh),DrivingRobot(false),
temp_cloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>()){
    //painter = new QPainter(this);
    //painter->setPen(Qt::red);
    
    record_depth_flag = false;
    dragPoint=QPoint(320,240);
    lastPoint = QPoint(320,225);
    QVBoxLayout *vbox = new QVBoxLayout(this);
    QHBoxLayout *hbox1 = new QHBoxLayout();
    QHBoxLayout *hbox2 = new QHBoxLayout();
    cmdBtn = new QPushButton("Command",this);
    quitBtn = new QPushButton("Quit",this);
    vfBtn = new QPushButton("ComputeVF",this);
    funcBtn = new QPushButton("GestureMode",this);
    pointPairs = new QVector<QPoint>;

    scribbling = false;
    pixmap=new QPixmap("/home/ab/Downloads/image3.png"); 
    piclabel = new QLabel("haha",this);
    piclabel->setPixmap(*pixmap);
    campixmap=new QPixmap("/home/ab/Downloads/image3.png"); 
    camlabel = new QLabel("hdfaha",this);
    camlabel->setPixmap(*campixmap);
    my_pen = QPen(QColor(0xFF, 0, 0, 0x80));
    my_pen.setWidth(8);
    /* start time to update msg callback function and update windows */
    update_timer_ = new QTimer(this);
    update_timer_->setInterval(10);
    update_timer_->start();
    connect(update_timer_,&QTimer::timeout,this,&Rosnode::nodeupdate);

    /* initialize the ros publish and subscribe function*/
    msgsub = nh_.subscribe("/kinect/depth/image_raw",1, &Rosnode::msgCallback,this);
    camsub = nh_.subscribe("/cam/image_raw",1, &Rosnode::camCallback,this);
    pointcloudSub = nh_.subscribe("/kinect/depth/points",1,&Rosnode::pointcloudCallback,this);
    cmdPub = nh_.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command",1);
    drivePub = nh_.advertise<geometry_msgs::Vector3>("drive_stroke",1); 
    vfclient = nh_.serviceClient<rosqt::setVF>("setVF");
    /* the layout design of the window */ 
    hbox2->addWidget(cmdBtn,1,Qt::AlignRight);
    hbox2->addWidget(vfBtn,0);
    hbox2->addWidget(funcBtn,0);
    hbox2->addWidget(quitBtn,0);

    hbox1->addWidget(piclabel);
    hbox1->addWidget(camlabel);
    vbox->addLayout(hbox1);
    vbox->addLayout(hbox2);
    setLayout(vbox);
    connect(quitBtn, &QPushButton::clicked,qApp,&QApplication::quit);
    connect(cmdBtn,&QPushButton::clicked,this,&Rosnode::cmdpublish);
    connect(vfBtn,&QPushButton::clicked,this,&Rosnode::computeVF);
    connect(funcBtn,&QPushButton::clicked,this,&Rosnode::functionSwitch);

}
void Rosnode::cmdpublish(){
    
    trajectory_msgs::JointTrajectory traj;
    trajectory_msgs::JointTrajectoryPoint trajpoint;
    traj.joint_names.push_back("shoulder_pan_joint");
    traj.joint_names.push_back("shoulder_lift_joint");
    traj.joint_names.push_back("elbow_joint");
    traj.joint_names.push_back("wrist_1_joint");
    traj.joint_names.push_back("wrist_2_joint");
    traj.joint_names.push_back("wrist_3_joint");

    trajpoint.positions.push_back(-1.5707963);
    trajpoint.positions.push_back(-1.31);
    trajpoint.positions.push_back(1.56);
    //trajpoint.positions.push_back(3.9432);
    trajpoint.positions.push_back(-2.34);
    trajpoint.positions.push_back(-1.5707963);
    trajpoint.positions.push_back(0.0);

    ros::Duration duration(5.0);
    trajpoint.time_from_start = duration; 
    traj.points.push_back(trajpoint);
    ros::Rate loop_rate(10);
    int count = 0;
    while (count <20){
        count++;
        cmdPub.publish(traj);
        loop_rate.sleep();
    }
}

void Rosnode::computeVF(){

    QVector<QPoint>::iterator iter = pointPairs->begin();    
    QVector<tf::Vector3> pseries;
    int count = 0;
    double diff = 0;
    /* find the start point for the manipulator 
     * find the mid point on the handle for further estimate the length
     * save the coresponding 3D point Series in pseries
     */
    QPoint gesture_startP,gesture_midP,gesture_endP;
    
    pcl::PointXYZ tmppt;
    for(iter=pointPairs->begin();iter!=pointPairs->end();iter++){
        tmppt = temp_cloud->at(iter->x(),iter->y());
        tf::Vector3 tmpvec(tmppt.x,tmppt.y,tmppt.z);
        pseries.push_back(tmpvec);
        if (count>0){
            diff = tmpvec.z()-pseries[count-1].z();
            if (diff <-0.05){
                gesture_startP = *iter; 
                qDebug()<<"ges_start point: "<<tmppt.x<<" "<<tmppt.y<<" "<<tmppt.z;    
            }else if(diff>0.05){
                gesture_endP = *iter;
                qDebug()<<"ges_end point: "<<tmppt.x<<" "<<tmppt.y<<" "<<tmppt.z;    
            }
        }
        count++;
    }
    gesture_midP = (gesture_startP + gesture_endP)/2;

    /* estimate the length of the door handle by
     * first go and find left edge then
     * go and find the right edge */
    diff = 0;
    count = 1;
    double oldZ;
    oldZ = temp_cloud->at(gesture_midP.x(),gesture_midP.y()).z;
    QPoint handle_startP, handle_endP;
    while (diff<0.05){
        tmppt = temp_cloud->at(gesture_midP.x()-2*count,gesture_midP.y());
        diff = tmppt.z-oldZ;
        oldZ = tmppt.z;
        count++;
        handle_startP = QPoint(gesture_midP.x()-2*count,gesture_midP.y());
    }
    qDebug()<<"gesture_midP"<<gesture_midP.x()<<" "<<gesture_midP.y();
    qDebug()<<"handle start "<<handle_startP.x()<<" "<<handle_startP.y();
    diff = 0;
    count = 1;
    oldZ = temp_cloud->at(gesture_midP.x(),gesture_midP.y()).z;
    while(diff<0.05){
     
        tmppt = temp_cloud->at(gesture_midP.x()+2*count,gesture_midP.y());
        diff = tmppt.z-oldZ;
        oldZ = tmppt.z;
        count++;
        handle_endP = QPoint(gesture_midP.x()+2*count,gesture_midP.y());
    }

    qDebug()<<"handle end "<<handle_endP.x()<<" "<<handle_endP.y();
    /* compute the length of the handle */
    double handle_length;
    double tmpx1,tmpx2;
    tmpx1 = temp_cloud->at(handle_startP.x(),handle_startP.y()).x;
    tmpx2 = temp_cloud->at(handle_endP.x(),handle_endP.y()).x;
    qDebug()<<"handle left: "<<tmpx1;
    qDebug()<<"handle right: "<<tmpx2;

    handle_length =std::abs(tmpx1 - tmpx2);
    ROS_INFO("handle length is %f",handle_length);

    rosqt::setVF srv;
    srv.request.length = handle_length;
    srv.request.rot_dir = 1;
    srv.request.leftp.x = temp_cloud->at(handle_startP.x(),handle_startP.y()).x;
    srv.request.leftp.y = temp_cloud->at(handle_startP.x(),handle_startP.y()).y;
    srv.request.leftp.z = temp_cloud->at(handle_startP.x(),handle_startP.y()).z;
    srv.request.rightp.x = temp_cloud->at(handle_endP.x(),handle_endP.y()).x;
    srv.request.rightp.y = temp_cloud->at(handle_endP.x(),handle_endP.y()).y;
    srv.request.rightp.z = temp_cloud->at(handle_endP.x(),handle_endP.y()).z;
    srv.request.robot_startP.x = temp_cloud->at(gesture_startP.x(),gesture_startP.y()).x;
    srv.request.robot_startP.y = temp_cloud->at(gesture_startP.x(),gesture_startP.y()).y;
    srv.request.robot_startP.z = temp_cloud->at(gesture_startP.x(),gesture_startP.y()).z;
    srv.request.initial_move = true;
    
    if(vfclient.call(srv)){
        ROS_INFO("already set the vf param"); 
    }

}


void Rosnode::functionSwitch(){
    if (DrivingRobot){
        DrivingRobot = false; 
        funcBtn->setText("GestureMode");
        record_depth_flag = false;
        pointPairs = new QVector<QPoint>;
    }else{
        DrivingRobot = true; 
        lastPoint = QPoint(320,225);
        record_depth_flag = false;
        funcBtn->setText("DrivingMode");
    } 
}
void Rosnode::mousePressEvent(QMouseEvent *event){
    //if (event->button() == Qt::LeftButton){
    if(!DrivingRobot){
        pointPairs = new QVector<QPoint>;
    }else{
        lastPoint = event->pos();
    }
    scribbling = true;
    record_depth_flag = false;
    //}

}
void Rosnode::mouseMoveEvent(QMouseEvent *event){
    //ROS_INFO("the coordinate of the mouse pos is (%d,%d)",event->pos().x(),event->pos().y());
    //if ((event->button() == Qt::LeftButton) && scribbling){
        //drawLineto(event->pos());
    if (!DrivingRobot){
        pointPairs->append(event->pos());
        update();
    }else{
        lastPoint = event->pos();

    }
    //}
}
void Rosnode::mouseReleaseEvent(QMouseEvent *event){
    //if(event->button() == Qt::LeftButton && scribbling){
        //drawLineto(event->pos());
    Q_UNUSED(event);
    scribbling = false;
    record_depth_flag = true;
    //}
}
//void Rosnode::drawLineto(const QPoint &endpoint){
//    painter->drawLine(lastPoint,endpoint);
//    update();
//    lastPoint = endpoint;
//}
//void Rosnode::paintEvent(QPaintEvent *event){
//    Q_UNUSED(event);
//    QPen my_pen = QPen(Qt::red);
//    my_pen.setWidth(30);
//    QPainter painter(this);
//    painter.setPen(my_pen);
//    painter.drawLines(*pointPairs);
//    ROS_INFO("this size of the pointPairs is %d",pointPairs->size());
//}

void Rosnode::camCallback(const sensor_msgs::ImageConstPtr &cam_img){
    
    

}
void Rosnode::msgCallback(const sensor_msgs::ImageConstPtr &msg_img){
    cv::Mat tmpimg = cv_bridge::toCvShare(msg_img,"bgr8")->image;
//    cv::imshow("view", cv_bridge::toCvShare(msg_img, "bgr8")->image);
    //cv::Mat feed_to_q_img ;
    //cv::cvtColor(tmpimg,feed_to_q_img,CV_BGR2RGB);
   //img_ = QImage(tmpimg.data,tmpimg.cols,tmpimg.rows,tmpimg.step,QImage::Format_RGB888); 
    //cvConvertImage(tmpimg, feed_to_q_img, CV_CVTIMG_SWAP_RB);
   //img_ = QImage((uchar*)feed_to_q_img.data,feed_to_q_img.cols,feed_to_q_img.rows,feed_to_q_img.step,QImage::Format_RGB888); 
    img_ = ASM::cvMatToQImage(tmpimg);
    *pixmap = QPixmap::fromImage(img_);
    QPainter painter(pixmap);
    painter.setPen(my_pen);
    QPoint arrowPub;
    if (!DrivingRobot){
        painter.drawLines(*pointPairs);
    }else{
        painter.drawLine(dragPoint,lastPoint);
        /* compute the end points of the arrow */
        QPoint arrowDir, arrowPoint1,arrowPoint2;
        arrowPub = lastPoint - dragPoint;
        arrowDir =0.8*(arrowPub);
        double tmpangle = 0.3491;
        arrowPoint1.setX(arrowDir.x()*qCos(tmpangle) - arrowDir.y()*qSin(tmpangle));
        arrowPoint1.setY(arrowDir.x()*qSin(tmpangle) + arrowDir.y()*qCos(tmpangle));
         
        arrowPoint2.setX(arrowDir.x()*qCos(-tmpangle) - arrowDir.y()*qSin(-tmpangle));
        arrowPoint2.setY(arrowDir.x()*qSin(-tmpangle) + arrowDir.y()*qCos(-tmpangle));

        painter.drawLine(lastPoint,arrowPoint1+dragPoint);
        painter.drawLine(lastPoint,arrowPoint2+dragPoint);
        my_pen = QPen(QColor(0xFF, 0, 0, 0x50));
        
        my_pen.setWidth(8);
        painter.setPen(my_pen);
        painter.drawEllipse(dragPoint,40,40);
        painter.drawEllipse(dragPoint,100,100);
    }
    piclabel->setPixmap(*pixmap);
   //ROS_INFO("the width is %d, the hight is %d",ig_.width(),img_.height());
    geometry_msgs::Vector3 mouse_stroke; 
    mouse_stroke.x = -arrowPub.x();
    mouse_stroke.z = -arrowPub.y();
    drivePub.publish(mouse_stroke);
}

void Rosnode::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr & pcmsg){
    //ROS_INFO("field name: %s \n offset: %d \n datatype: %d \n count: %d", pcmsg->fields.name.c_str(),pcmsg->fields.offset,pcmsg->fields.datatype, pcmsg->fields.count);
    //ROS_INFO("\n 1stNum: %u  2ndNum: %u 3rd Num: %u 4thNum: %u \nrow_step: %d \npoint_step:%d \n height:%d \n width:%d",pcmsg->data[0], pcmsg->data[1], pcmsg->data[2], pcmsg->data[3], pcmsg->row_step,pcmsg->point_step,pcmsg->height,pcmsg->width);
    //sensor_msgs::PointCloud2 cloud_msg;
    //cloud_msg.data = pcmsg->data;
    //cloud_msg.fields = pcmsg->fields;
    //sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg,"x");
    //ROS_INFO("%f ",*(iter_x+240*640 +440));
    if (record_depth_flag && (!DrivingRobot) && (!pointPairs->isEmpty())){
     
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*pcmsg,pcl_pc2);
        //pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
        //ROS_INFO("%f",temp_cloud->at(420,240).x);
    }
}
bool Rosnode::nodeupdate(){
   //QPixmap pixmap("/home/ab/Downloads/images.jpg");

    ros::spinOnce();
    return true;
 //  QPixmap pixmap = QPixmap::fromImage(img_);
}


