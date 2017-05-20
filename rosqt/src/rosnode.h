#include <ros/ros.h>
#include <QWidget>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <QLabel>
#include <QPixmap>
#include <QImage>
#include <QMouseEvent>
#include <QPainter>
#include <QPaintEvent>
#include <QPushButton>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
class Rosnode :public QWidget{
    
    Q_OBJECT
public:
    Rosnode(QWidget *parent, const ros::NodeHandle& nh);
    bool nodeupdate();
private:
    void msgCallback(const sensor_msgs::ImageConstPtr & msg_img);
    void camCallback(const sensor_msgs::ImageConstPtr & cam_img);
    void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& pcmsg);
    QLabel *piclabel;
    QLabel *camlabel;
    QPixmap *pixmap;
    QPixmap *campixmap;
    QImage img_;
    ros::NodeHandle nh_;
    QTimer *update_timer_;
    ros::Subscriber msgsub;
    ros::Subscriber camsub;
    ros::Subscriber pointcloudSub;
    ros::Publisher cmdPub;
    ros::Publisher drivePub;
    ros::ServiceClient vfclient;
    //QPainter *painter;
protected:
    void mouseMoveEvent(QMouseEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    //void paintEvent(QPaintEvent *event);
private:
    void drawLineto(const QPoint &endPoint);
    bool scribbling;
    void cmdpublish();
    void computeVF();
    void functionSwitch();
    bool DrivingRobot;
    bool record_depth_flag;
    QPoint lastPoint;
    QPoint dragPoint;
    QVector<QPoint> *pointPairs;
    QPen my_pen;
    QPushButton *cmdBtn;
    QPushButton *quitBtn;
    QPushButton *vfBtn;
    QPushButton *funcBtn;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud;
};
