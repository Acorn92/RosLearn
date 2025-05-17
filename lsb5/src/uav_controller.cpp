#include "uav_controller.hpp"

namespace uav_controller
{

	UavController::UavController(ros::NodeHandle &n, const std::string &uavName)
	    : n_(n), uavName_(uavName)
	{
		rosNodeInit();
	}

	void UavController::arm(bool cmd)
	{
		// Реализуйте переход аппарата в режим ARM
	}

	void UavController::rosNodeInit()
	{
		// здесь должна быть инициализация объектов издателя и подписчика для получения информации из необходимых топиков
		// пример инициализации подписки на топик желаемого положения ЛА
		// desPoseSub_ = n_.subscribe<geometry_msgs::PoseStamped>("vehicle/des_pose", 1, &UavController::desiredPositionCallback, this);
		desPoseSub_ = n_.subscribe<geometry_msgs::PoseStamped>("vehicle/desPose", 1, &UavController::desiredPositionCallback, this);
	}

	void UavController::uavStateCallback(const mavros_msgs::State::ConstPtr &msg)
	{
		// Реализуйте обработку состояния ЛА
		currentState_ = *msg;
	}

	void UavController::desiredPositionCallback(const geometry_msgs::PoseStamped desPose)
	{
		// Реализуйте обработку желаемого положения ЛА
		ROS_INFO("Message received:");

		setPoint_.position.x = desPose.pose.position.x;
		setPoint_.position.y = desPose.pose.position.y;
		setPoint_.position.z = desPose.pose.position.z;
	}

	void UavController::calculateAndSendSetpoint()
	{
		// здесь необходимо выполнить рассчет желаемой линейной скорости ЛА
		// можно пользоваться алгоритмами систем управления которые мы изучили ранее (например П ПД или ПИД регуляторами)
		setPoint_.velocity.x = 0;// вместо заданного значения должен быть рассчет скорости
		setPoint_.velocity.y = 0;// вместо заданного значения должен быть рассчет скорости
		// при публикации такой скорости аппарат будет лететь вдоль оси Z
		// стартовой СК(по направлению вверх) со скоростью 1 м/сек
		setPoint_.velocity.z = 1;// вместо заданного значения должен быть рассчет скорости
		// здесь необходимо выполнить рассчет желаемой угловой скорости ЛА
		setPoint_.yaw_rate = 0;// вместо заданного значения должен быть рассчет угловой скорости
		// отправка
		setPointPub_.publish(setPoint_);
	}

	void UavController::setPointTypeInit()
	{
		// задаем тип используемого нами сообщения для желаемых параметров управления аппаратом
		// приведенная ниже конфигурация соответствует управлению линейной скоростью ЛА
		// и угловой скоростью аппарата в канале рыскания(yaw)
		uint16_t setpointTypeMask = mavros_msgs::PositionTarget::IGNORE_PX + mavros_msgs::PositionTarget::IGNORE_PY + mavros_msgs::PositionTarget::IGNORE_PX + mavros_msgs::PositionTarget::IGNORE_AFX + mavros_msgs::PositionTarget::IGNORE_AFY + mavros_msgs::PositionTarget::IGNORE_AFZ + mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
		// при помощи конфигурации вышеприведенным образом переменной setpointTypeMask
		// можно настроить управление аппаратом посредством передачи(положения аппарата и углового положения в канале рыскания)

		// Конфигурация системы координат в соответствии с которой задаются параметры управления ЛА
        // при setpointCoordinateFrame = 1 управление происходит в неподвижной СК (локальная неподвижная СК инициализируется при работе навигационной системы)
        // при использовании ГНСС или optical flow является стартовой, при использовании других НС начало координат соответствует таковому у выбранной
        //  навигационной системы(например оси выходят из центра реперного маркера).
        // setpointCoordinateFrame = 8 соответствует управлению аппаратом в связных нормальных осях (подвижная СК центр которой находится в центре масс ЛА)
        // в действительности без какой либо настройки, совпадает с системой координат инерциальной навигационной системы.
		uint16_t setpointCoordinateFrame = 1;

		// Присваиваем наши параметры задающего воздействия полям класса нашего сообщения
		setPoint_.type_mask = setpointTypeMask;
		setPoint_.coordinate_frame = setpointCoordinateFrame;
	}

}// namespace uav_controller