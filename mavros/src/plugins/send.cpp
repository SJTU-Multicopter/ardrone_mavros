#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <mavros/Ardrone.h>
#include <ros/console.h>

namespace mavplugin{

class SendPlugin : public MavRosPlugin{

public:
	 SendPlugin():
	 send_nh("~send"),
	 uas(nullptr)
    { };

    void initialize(UAS &uas_){
    	uas = &uas_;  
    	send_nh.param<std::string>("frame_id", frame_id, "send");
    	data_sub = send_nh.subscribe("/xxxxx",5,&SendPlugin::send_cb,this);
    }
    
    std::string get_name() {
		return "send";
	}

     const message_map get_rx_handlers() {
		return {
			     //MESSAGE_HANDLER(212, &SendPlugin::send_cb)
		};
	}


private:
	ros::NodeHandle send_nh;
	ros::Subscriber data_sub;
	UAS *uas;

	std::string frame_id;

    void send(bool a, float b, float c, float d, float e){
    	mavlink_message_t msg;
    	mavlink_msg_ardrone_position_pack_chan(UAS_PACK_CHAN(uas),&msg,a,b,c,d,e); //pack
    	UAS_FCU(uas)->send_message(&msg); //send
        ROS_INFO("Send!\nrobot_x:%f\nrobot_y:%f\ncatch_x:%f\ncatch_y:%f\n", b, c, d, e);
    }
    
    //callbacks
    void send_cb(const mavros::Ardrone &msg){
        send(msg.moving , msg.robot_x , msg.robot_y , msg.catch_x , msg.catch_y);
    }
};

};

PLUGINLIB_EXPORT_CLASS(mavplugin::SendPlugin, mavplugin::MavRosPlugin)
