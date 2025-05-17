#ifndef uavController_HPP
#define uavController_HPP

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
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

		private:

		ros::NodeHandle &n_;
		std::string uavName_;
		mavros_msgs::PositionTarget setPoint_;       // объект сообщения для задающего воздействия
		mavros_msgs::State currentState_;            // объект сообщения о состоянии аппарата
		geometry_msgs::PoseStamped currentPoseLocal_;// объект сообщения о положении и ориентации
		ros::Publisher setPointPub_;
		ros::Subscriber localPositionSub_;
		ros::Subscriber desPoseSub_;
		ros::Subscriber stateSub_;
		ros::ServiceClient setModeClient_;
		mavros_msgs::SetMode setModeName_;

		void rosNodeInit();
		void setPointTypeInit();
		void uavStateCallback(const mavros_msgs::State::ConstPtr &msg);
		void desiredPositionCallback(const geometry_msgs::PoseStamped desPose);
	};
};// namespace uav_controller

#endif