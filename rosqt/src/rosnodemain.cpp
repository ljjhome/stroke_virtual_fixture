#include <QApplication>

#include <ros/ros.h>
#include "rosnode.h"
/*
class Nodeapp :public QApplication
{
public:
    ros::NodeHandle nh_;
    Rosnode node;
    Nodeapp(int argc, char** argv):QApplication(argc,argv){
        ros::init(argc, argv,"my_node"); 
        node(0,nh_);
    
    }
}
*/
int main(int argc, char** argv){
    QApplication app(argc, argv);
    ros::init(argc,argv,"my_node",ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    Rosnode window(0,nh);
    window.show();
    return app.exec();
}
