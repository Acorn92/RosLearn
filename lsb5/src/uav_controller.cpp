#include "uav_controller.hpp"

namespace uav_controller
{

	UavController::UavController(ros::NodeHandle &n, const std::string &uavName)
	    : n_(n), uavName_(uavName), spinner_(2) 
	{
		rosNodeInit();

		f = boost::bind(&UavController::config_param_callback, this, _1, _2);
		server.setCallback(f);
	
		ROS_INFO("Spinning node");

		spinner_.start();
	}

	void UavController::arm(bool cmd)
	{
		if (!cmd) {
            offboard_timer_.stop();
            return;
        }
        // Установка начального положения.
        set_position(0, 0, 0, 0);
        // Запускаем таймер отправки целевого положения
        offboard_timer_.start();
        // Немного ждем пока сообщения начнут приходить на автопилот
        ros::Rate rate(1.0);
        rate.sleep();
        // Переключение режима на OFFBOARD
		setModeName_.request.custom_mode = "OFFBOARD";
		setModeClient_ = n_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
		setModeClient_.call(setModeName_);
		setModeName_.response.mode_sent;
		// Реализуйте переход аппарата в режим ARM
		mavros_msgs::CommandBool arm_cmd;
		arm_cmd.request.value = cmd;

		ros::ServiceClient arming_client = n_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
		arming_client.call(arm_cmd);
		arm_cmd.response.success;
		while (ros::ok())
		{

		}

		
	}

	void UavController::rosNodeInit()
	{
		// здесь должна быть инициализация объектов издателя и подписчика для получения информации из необходимых топиков
		// пример инициализации подписки на топик желаемого положения ЛА
		// desPoseSub_ = n_.subscribe<geometry_msgs::PoseStamped>("vehicle/des_pose", 1, &UavController::desiredPositionCallback, this);
		stateSub_ = n_.subscribe<mavros_msgs::State>("/mavros/state",10, &UavController::uavStateCallback, this);
		localPositionSub_ = n_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &UavController::uavPoseCallback, this);
		desPoseSub_ = n_.subscribe<geometry_msgs::PoseStamped>("vehicle/desPose", 1, &UavController::desiredPositionCallback, this);

		setPointPub_ = n_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);

		offboard_timer_ = n_.createTimer(ros::Duration(0.05), &UavController::offboard_timer_cb, this); // 20 Hz
        offboard_timer_.stop(); // Останавливаем таймер до перехода в режим OFFBOARD
		n_.param<double>("KpVx", this->KpVx, 0.1);
		n_.param<double>("KpVy", this->KpVy, 0.1);
		n_.param<double>("KpVz", this->KpVz, 0.1);
		n_.param<double>("KpYaw", this->KpYaw, 0.1);
	}

	void UavController::uavStateCallback(const mavros_msgs::State::ConstPtr &msg)
	{
		// Реализуйте обработку состояния ЛА
		currentState_ = *msg;
	}

	void UavController::uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
	{
		// Реализуйте обработку состояния ЛА
		currentPoseLocal_ = *msg;
	}


	void UavController::desiredPositionCallback(const geometry_msgs::PoseStamped desPose)
	{
		// Реализуйте обработку желаемого положения ЛА
		// ROS_INFO("Message received:");
		readDesPose = desPose;

		// setPoint_.position.x = desPose.pose.position.x;
		// setPoint_.position.y = desPose.pose.position.y;
		// setPoint_.position.z = desPose.pose.position.z;
		// setPoint_.yaw = desPose.pose.orientation.w;
	}

	void UavController::calculateAndSendSetpoint()
	{

		


		setPointTypeInit();
		// здесь необходимо выполнить рассчет желаемой линейной скорости ЛА
		// можно пользоваться алгоритмами систем управления которые мы изучили ранее (например П ПД или ПИД регуляторами)
		// setPoint_.velocity.x = 0;// вместо заданного значения должен быть рассчет скорости
		
		// setPoint_.velocity.y = 0;// вместо заданного значения должен быть рассчет скорости
		// // при публикации такой скорости аппарат будет лететь вдоль оси Z
		// // стартовой СК(по направлению вверх) со скоростью 1 м/сек
		// setPoint_.velocity.z = 1;// вместо заданного значения должен быть рассчет скорости
		// // здесь необходимо выполнить рассчет желаемой угловой скорости ЛА
		// setPoint_.yaw_rate = 0;// вместо заданного значения должен быть рассчет угловой скорости
		
		// отправка
		// setPointPub_.publish(setPoint_);

			double des_yaw = atan2(readDesPose.pose.position.y - currentPoseLocal_.pose.position.y,
									readDesPose.pose.position.x - currentPoseLocal_.pose.position.x);
			double curren_yaw = tf2::getYaw(currentPoseLocal_.pose.orientation);
			setPoint_.yaw_rate = this->KpYaw*(des_yaw - curren_yaw);
			if (setPoint_.yaw_rate  >= this->KpYaw*M_PI) 
			{
				setPoint_.yaw_rate -= this->KpYaw*(2*M_PI);        
			}
			else if (setPoint_.yaw_rate  <= this->KpYaw*(-M_PI)) 
			{
				setPoint_.yaw_rate += this->KpYaw*(2*M_PI);	
			}
			setPoint_.velocity.x = this->KpVx * (readDesPose.pose.position.x - currentPoseLocal_.pose.position.x);
			setPoint_.velocity.y = this->KpVy * (readDesPose.pose.position.y - currentPoseLocal_.pose.position.y);
			setPoint_.velocity.z = this->KpVz * (readDesPose.pose.position.z - currentPoseLocal_.pose.position.z);
			ROS_INFO("des.yaw: %f cur.yaw: %f:", des_yaw, curren_yaw);
			
			ROS_INFO("Sended yaw_rate %f:", setPoint_.yaw_rate );
			setPointPub_.publish(setPoint_);
		
	}

	void UavController::setPointTypeInit()
	{
		// задаем тип используемого нами сообщения для желаемых параметров управления аппаратом
		// приведенная ниже конфигурация соответствует управлению линейной скоростью ЛА
		// и угловой скоростью аппарата в канале рыскания(yaw)
		uint16_t setpointTypeMask = mavros_msgs::PositionTarget::IGNORE_PX + mavros_msgs::PositionTarget::IGNORE_PY + mavros_msgs::PositionTarget::IGNORE_PZ + mavros_msgs::PositionTarget::IGNORE_AFX + mavros_msgs::PositionTarget::IGNORE_AFY + mavros_msgs::PositionTarget::IGNORE_AFZ + mavros_msgs::PositionTarget::IGNORE_YAW;
		// при помощи конфигурации вышеприведенным образом переменной setpointTypeMask
		// можно настроить управление аппаратом посредством передачи(положения аппарата и углового положения в канале рыскания)

		// Конфигурация системы координат в соответствии с которой задаются параметры управления ЛА
        // при setpointCoordinateFrame = 1 управление происходит в неподвижной СК (локальная неподвижная СК инициализируется при работе навигационной системы)
        // при использовании ГНСС или optical flow является стартовой, при использовании других НС начало координат соответствует таковому у выбранной
        //  навигационной системы(например оси выходят из центра реперного маркера).
        // setpointCoordinateFrame = 8 соответствует управлению аппаратом в связных нормальных осях (подвижная СК центр которой находится в центре масс ЛА)
        // в действительности без какой либо настройки, совпадает с системой координат инерциальной навигационной системы.
		uint16_t setpointCoordinateFrame = 1;
		// setPoint_.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ
		// | mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ
		// | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
		// Присваиваем наши параметры задающего воздействия полям класса нашего сообщения
		setPoint_.type_mask = setpointTypeMask;
		setPoint_.coordinate_frame = setpointCoordinateFrame;
	}

	void UavController::set_position(float x, float y, float z, float yaw) {
        // Устанавливаем систему координат в рамках которой будет отправляться команда
        setPoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        // Установим битовую маску где покажем что должен выполнить автопилот, полет в точку или набор заданной скорости, ускорения, угла угловой скорости.
        setPoint_.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ
            | mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ
            | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        
        // В этом примере летим в точку с заданным углом.        
        setPoint_.position.x = x;
        setPoint_.position.y = y;
        setPoint_.position.z = z;
        setPoint_.yaw = yaw;
    }

    
	// Таймер отправки целевого положения
    void UavController::offboard_timer_cb(const ros::TimerEvent&) {
        setPoint_.header.stamp = ros::Time::now();
        // setPointPub_.publish(setPoint_);
		this->calculateAndSendSetpoint();
    }

	bool UavController::connected()
	{
		return currentState_.connected;
	}

	void UavController::config_param_callback(uav_controller_config::uav_controller_cfgConfig &config, uint32_t level)
	{
    	ROS_INFO("Reconfigure Request: %f",
        config.KpVx);
		this->KpVx = config.KpVx;
		this->KpVy = config.KpVy;
		this->KpVz = config.KpVz;
		this->KpYaw = config.KpYaw;
	}	

	double UavController::saturation(double data, double max, double min)
	{
		if (data > max)
			return max;
		else if (data < min)
			return min;
		else
			return data;    
	}


}// namespace uav_controller