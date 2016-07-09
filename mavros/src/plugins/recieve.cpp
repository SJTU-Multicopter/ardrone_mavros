#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <mavros/Ardrone.h>

//Don't forget to add codes in mavros_plugins.xml
namespace mavplugin {
class ReceiverPlugin : public MavRosPlugin{

public:
    ReceiverPlugin():
    receiver_nh("~receiver"),
    uas(nullptr)         
    { };

    void initialize(UAS &uas_)
	{
		uas = &uas_;
		receiver_nh.param<std::string>("frame_id", frame_id, "receiver");
        receiver_pub = receiver_nh.advertise<mavros::Ardrone>("/ardrone/robot_position", 1); //add publisher to handler
	}

	const message_map get_rx_handlers() {
		return {
		  MESSAGE_HANDLER(MAVLINK_MSG_ID_ARDRONE_POSITION, &ReceiverPlugin::handle_ardrone_data)
		};
	}

private:
    ros::NodeHandle receiver_nh;
    ros::Publisher receiver_pub;
    UAS *uas; 
    std::string frame_id;
    

    void handle_ardrone_data(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
    	mavlink_ardrone_position_t ardrone_msg;
    	mavlink_msg_ardrone_position_decode(msg, &ardrone_msg);//decode 
        auto ardrone_mav_msg = boost::make_shared<mavros::Ardrone>();//define a msg the same type as  mavros_extras::ClarenceNewMavros
        if(ardrone_msg.moving == 0){
            ardrone_mav_msg->moving = false; 
        }
        else {
            ardrone_mav_msg->moving = true;
        }
        ardrone_mav_msg->robot_x = ardrone_msg.robot_x; 
        ardrone_mav_msg->robot_y = ardrone_msg.robot_y; 
        ardrone_mav_msg->catch_x = ardrone_msg.catch_x; 
        ardrone_mav_msg->catch_y = ardrone_msg.catch_y; 
        
        receiver_pub.publish(ardrone_mav_msg); //publish msg
    }         
};
};
PLUGINLIB_EXPORT_CLASS(mavplugin::ReceiverPlugin, mavplugin::MavRosPlugin)
