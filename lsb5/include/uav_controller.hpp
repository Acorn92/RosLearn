#ifndef uavController_HPP
#define uavController_HPP

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include "rotationTransform.hpp"

#include <dynamic_reconfigure/server.h>
#include <lsb5/uav_controller_cfgConfig.h>
// Обьявим класс для контроллера БЛА
// этот класс включает методы для управления аппаратом
//

namespace uav_controller
{
	class UavController
	{
		public:
		UavController(ros::NodeHandle &n, const std::string &uavName="mavros");

		/**
			* @brief Метод переводит аппарат в состояние arm/disarm
			* Состояние arm - аппарат готов к движению при получении комманд
			* управления начинает движение
			*
			* Состояние disarm - аппарат не готов к движению при поступлении
			* комманд управления не начинает движение
			*
			* @param cmd - смена состояния
			* True - перевод аппарата в состояние arm
			* False - перевод аппарата в состояние disarm
			*/
		void arm(bool cmd);
		/**
		   * @brief метод производит рассчет желаемых управляющих воздействий
		   *  и пересылает сообщение типа mavros_msgs::PositionTarget
		   * в топик <имя_ла>/setpoint_raw/local (как правило mavros/setpoint_raw/local)
		   */
		void calculateAndSendSetpoint();

		bool connected();

		private:

		ros::NodeHandle &n_;
		std::string uavName_;
		geometry_msgs::PoseStamped setVel_;
		mavros_msgs::PositionTarget setPoint_;       // объект сообщения для задающего воздействия
		mavros_msgs::State currentState_;            // объект сообщения о состоянии аппарата
		geometry_msgs::PoseStamped currentPoseLocal_;// объект сообщения о положении и ориентации
		geometry_msgs::PoseStamped readDesPose;      // объект сообщения принятого положения
		ros::Publisher setPointPub_;
		ros::Subscriber localPositionSub_;
		ros::Subscriber desPoseSub_;
		ros::Subscriber stateSub_;
		ros::ServiceClient setModeClient_;
		mavros_msgs::SetMode setModeName_;

		ros::Timer offboard_timer_;
		ros::AsyncSpinner spinner_;

		dynamic_reconfigure::Server<uav_controller_config::uav_controller_cfgConfig> server;
    	dynamic_reconfigure::Server<uav_controller_config::uav_controller_cfgConfig>::CallbackType f;

		double KpVx = 1;
		double KpVy = 1;
		double KpVz = 1;
		double KpYaw = 1;

		const double VxMax = 2, VxMin = -2;
		const double VyMax = 2, VyMin = -2;
		const double VzMax = 2, VzMin = -2;
		const double VyawMax = 1, VyawMin = -1;
		void rosNodeInit();
		ros::Rate rate = ros::Rate(20.0);
		void setPointTypeInit();
		void uavStateCallback(const mavros_msgs::State::ConstPtr &msg);
		void desiredPositionCallback(const geometry_msgs::PoseStamped desPose);
		void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
		void set_position(float x, float y, float z, float yaw);
		bool change_mode(const std::string& mode);
		void offboard_timer_cb(const ros::TimerEvent&);
		void config_param_callback(uav_controller_config::uav_controller_cfgConfig &config, uint32_t level);
		double saturation(double data, double max, double min);
		
	};
};// namespace uav_controller

#endif