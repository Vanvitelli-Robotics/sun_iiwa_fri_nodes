#include "sun_iiwa_fri_nodes/IIWA_ClikNode.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "iiwa_clik");
    
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    sun::iiwa::IIWAClikNode node(nh, nh_private);

    node.run();
    
    return 0;
}
