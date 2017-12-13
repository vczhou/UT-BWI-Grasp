// Converts GPD-format grasp to Agile Grasp-format grasp

#include <ros/ros.h>
#include <gpd/GraspConfigList.h>
#include <bwi_grasp/GraspWithScoreList.h>

ros::Publisher grasp_pub;

void grasp_cb(const gpd::GraspConfigList::ConstPtr& gpd_grasps) {
    bwi_grasp::GraspWithScoreList grasps;
    grasps.header = gpd_grasps->header;
    ROS_INFO_STREAM("Header is " << grasps.header);
    grasps.grasps.resize(gpd_grasps->grasps.size());
    int i;
    for (i=0; i < gpd_grasps->grasps.size(); i++) {
        grasps.grasps[i].approach = gpd_grasps->grasps[i].approach;
        grasps.grasps[i].axis = gpd_grasps->grasps[i].axis;
        grasps.grasps[i].width = gpd_grasps->grasps[i].width;
        grasps.grasps[i].surface_center.x = gpd_grasps->grasps[i].surface.x;
        grasps.grasps[i].surface_center.y = gpd_grasps->grasps[i].surface.y;
        grasps.grasps[i].surface_center.z = gpd_grasps->grasps[i].surface.z;
        grasps.grasps[i].center.x = (gpd_grasps->grasps[i].top.x+gpd_grasps->grasps[i].bottom.x)/2;
        grasps.grasps[i].center.y = (gpd_grasps->grasps[i].top.y+gpd_grasps->grasps[i].bottom.y)/2;
        grasps.grasps[i].center.z = (gpd_grasps->grasps[i].top.z+gpd_grasps->grasps[i].bottom.z)/2;
        grasps.grasps[i].score = gpd_grasps->grasps[i].score;
    }
    ROS_INFO_STREAM("Found " << i << " grasps");
    grasp_pub.publish(grasps);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "convert_grasp");
    ros::NodeHandle n;

    ros::Subscriber grasp_sub = n.subscribe("/detect_grasps/clustered_grasps", 1, grasp_cb);
    grasp_pub = n.advertise<bwi_grasp::GraspWithScoreList>("/bwi_grasp/grasps", 1);

    ros::spin();

    return 0;
}
