#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/Imu.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstring>

ros::Publisher pose_measn_Pub;
ros::Subscriber real_pose_Sub_,real_imu_Sub_,real_pronto_pose_Sub_;
geometry_msgs::PoseWithCovarianceStamped pose_means;
ros::Time br_time;
tf::Transform base_to_odom,odom_to_footprint;
std::string tf_publish_way;
void positionCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& position){
    if(tf_publish_way=="kin_and_imu"){
        ROS_WARN_ONCE("get the position!!!");
        pose_means.header.frame_id = "odom";
        pose_means.header.stamp = ros::Time::now();
        pose_means.pose.pose.position.x = position->pose.pose.position.x;
        pose_means.pose.pose.position.y = position->pose.pose.position.y;
        pose_means.pose.pose.position.z = position->pose.pose.position.z;
        base_to_odom.setOrigin(tf::Vector3(pose_means.pose.pose.position.x,pose_means.pose.pose.position.y
                                                                                 ,pose_means.pose.pose.position.z));
        odom_to_footprint.setOrigin(tf::Vector3(pose_means.pose.pose.position.x,
                                    pose_means.pose.pose.position.y,
                                    0));
    }

}
void orientationCB(const sensor_msgs::Imu::ConstPtr& orientation){
    if(tf_publish_way=="kin_and_imu"){
        ROS_WARN_ONCE("get the orientation!!!");
        pose_means.pose.pose.orientation.w = orientation->orientation.w;
        pose_means.pose.pose.orientation.x = orientation->orientation.x;
        pose_means.pose.pose.orientation.y = orientation->orientation.y;
        pose_means.pose.pose.orientation.z = orientation->orientation.z;

        //两个四元数需要注意顺序对应
            Eigen::Quaterniond q(pose_means.pose.pose.orientation.w,pose_means.pose.pose.orientation.x,
                                 pose_means.pose.pose.orientation.y,pose_means.pose.pose.orientation.z);
            Eigen::Quaterniond q1 = q.normalized();
            tf::Quaternion q_rot;
            q_rot.setW(q1.w());
            q_rot.setX(q1.x());
            q_rot.setY(q1.y());
            q_rot.setZ(q1.z());
            double yaw, pitch, roll;
            tf::Matrix3x3(q_rot).getRPY(roll, pitch, yaw);
            odom_to_footprint.setRotation(tf::createQuaternionFromYaw(yaw));

    //        ROS_INFO("q1.w(): %f", q1.w());
    //        ROS_INFO("q1.x(): %f", q1.x());
    //        ROS_INFO("q1.y(): %f", q1.y());
    //        ROS_INFO("q1.z(): %f", q1.z());
    //        std::cout<<"mode    "<<sqrt(q1.w()*q1.w()+q1.x()*q1.x()+q1.y()*q1.y()+q1.z()*q1.z())<<std::endl;
            base_to_odom.setRotation(q_rot);
    }

}
void prontoCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose){
    if(tf_publish_way=="pronto_way"){
        ROS_WARN_ONCE("get pronto pose!!!");
        pose_means.header.frame_id = "odom";
        pose_means.header.stamp = ros::Time::now();
        pose_means.pose.pose.position.x = pose->pose.pose.position.x;
        pose_means.pose.pose.position.y = pose->pose.pose.position.y;
        pose_means.pose.pose.position.z = pose->pose.pose.position.z;
        base_to_odom.setOrigin(tf::Vector3(pose_means.pose.pose.position.x,pose_means.pose.pose.position.y
                                                                                 ,pose_means.pose.pose.position.z));

        pose_means.pose.pose.orientation.w = pose->pose.pose.orientation.w;
        pose_means.pose.pose.orientation.x = pose->pose.pose.orientation.x;
        pose_means.pose.pose.orientation.y = pose->pose.pose.orientation.y;
        pose_means.pose.pose.orientation.z = pose->pose.pose.orientation.z;

        Eigen::Quaterniond q(pose_means.pose.pose.orientation.w,pose_means.pose.pose.orientation.x,
                             pose_means.pose.pose.orientation.y,pose_means.pose.pose.orientation.z);
        Eigen::Quaterniond q1 = q.normalized();
        tf::Quaternion q_rot;
        q_rot.setW(q1.w());
        q_rot.setX(q1.x());
        q_rot.setY(q1.y());
        q_rot.setZ(q1.z());
        base_to_odom.setRotation(q_rot);

        double yaw, pitch, roll;
        tf::Matrix3x3(q_rot).getRPY(roll, pitch, yaw);
        odom_to_footprint.setRotation(tf::createQuaternionFromYaw(yaw));
        odom_to_footprint.setOrigin(tf::Vector3(pose_means.pose.pose.position.x,
                                    pose_means.pose.pose.position.y,
                                    0));
    }

}
int main(int argc, char** argv) {
    ros::init(argc,argv,"pose_means");
    ros::NodeHandle nh_("~");


    tf::TransformBroadcaster tfBoardcaster_;//把这行放到初始化函数的外面会报上面的错。You must call ros::init() before creating the first NodeHandle

    nh_.param("/tf_publish_way",tf_publish_way,std::string("kin_and_imu"));
    real_pose_Sub_ = nh_.subscribe("/laikago_pronto/foot_odom",1,&positionCB);
    real_imu_Sub_ = nh_.subscribe("/imu/data",1,&orientationCB);
    real_pronto_pose_Sub_ = nh_.subscribe("/laikago_state/pose",1,&prontoCB);


//    pose_measn_Pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/laikago_pose",1);

//    pose_means.pose.pose.position.x = 0.0;
//    pose_means.pose.pose.position.y = 0.0;
//    pose_means.pose.pose.position.z = 0.083;
//    pose_means.pose.pose.orientation.w = 1.0;
//    pose_means.pose.pose.orientation.x = 0.0;
//    pose_means.pose.pose.orientation.y = 0.0;
//    pose_means.pose.pose.orientation.z = 0.0;
//    pose_means.header.frame_id = "odom";
//    pose_means.header.stamp = ros::Time::now();


//    Eigen::Quaterniond q(pose_means.pose.pose.orientation.w,pose_means.pose.pose.orientation.x,
//                         pose_means.pose.pose.orientation.y,pose_means.pose.pose.orientation.z);
//    base_to_odom.setRotation(tf::Quaternion(pose_means.pose.pose.orientation.w,pose_means.pose.pose.orientation.x,
//                        pose_means.pose.pose.orientation.y,pose_means.pose.pose.orientation.z));
//    base_to_odom.setOrigin(tf::Vector3(pose_means.pose.pose.position.x,pose_means.pose.pose.position.y
//                                       ,pose_means.pose.pose.position.z));







    ros::Rate rate(100);
    while (ros::ok()) {
        //必须放在while循环里面,否则时间戳不更新
        //br_time  = pose_means.header.stamp;
        br_time = ros::Time::now();
        //pose_measn_Pub.publish(pose_means);
        tfBoardcaster_.sendTransform(tf::StampedTransform(base_to_odom, br_time, "/odom", "/base"));
        tfBoardcaster_.sendTransform(tf::StampedTransform(odom_to_footprint,br_time,"/odom","/foot_print"));
        ros::spinOnce();
        rate.sleep();
      }
    return 0;
}
