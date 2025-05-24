#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <thread>
#include <chrono>

class FlightCommander {
public:
    FlightCommander(): spinner_(2) {
        // Подписываемся на состояние и положение ЛА
        state_sub_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &FlightCommander::state_cb, this);
        pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &FlightCommander::pose_cb, this);
        // Инициализируем publisher для целевого состояния ЛА
        local_pos_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
        // Инициализируем таймер для отправки целевого состояния
        offboard_timer_ = nh_.createTimer(ros::Duration(0.05), &FlightCommander::offboard_timer_cb, this); // 20 Hz
        offboard_timer_.stop(); // Останавливаем таймер до перехода в режим OFFBOARD
        // (AsyncSpinner - это класс, который используется для асинхронного выполнения коллбеков (callback) из очереди сообщений
        // аналогично ros::spin() или ros::spinOnce()
        // Запускаем асинхронный спинер
        spinner_.start();
    }

    void run() {
        // Ожидаем подключения к автопилоту
        while (ros::ok() && !current_state_.connected) {
            ros::spinOnce();
            rate.sleep();
        }

        // Запускаем режим offboard
        offboard_enable(true);
        do_takeoff(2.0);  // Взлетаем на 2 метра....

        // Методы для следования в точку и  посадки можно сделать по аналогии.
        
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher local_pos_pub_;
    ros::Timer offboard_timer_;
    
    ros::Rate rate = ros::Rate(20.0);
    mavros_msgs::State current_state_;
    mavros_msgs::PositionTarget setpoint_;
    geometry_msgs::PoseStamped current_pose_;
    ros::AsyncSpinner spinner_;

    void state_cb(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
    }

    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose_ = *msg;
    }

    void do_takeoff(double altitude) {
        if (!current_state_.armed)
            arm_vehicle(true);

        set_position(0, 0, altitude, 0);
        
        double ascent_speed = 1.0;  // Установим скорость взлета 1 м\с
        double timeout = altitude / ascent_speed + 10.0;  // установим таймаут исходя из скорости взлета с небольшим запасом
        auto start_time = ros::Time::now();
        bool altitude_reached = false;

        while (ros::ok() && !altitude_reached) {
            rate.sleep();

            // Проверим достижение заданной высоты с точностью в 10 см
            if (std::abs(current_pose_.pose.position.z - altitude) < 0.1) {
                altitude_reached = true;
                ROS_INFO("Target altitude reached.");
            }

            // Проверяем не вышел ли взлет за таймаут
            if ((ros::Time::now() - start_time).toSec() > timeout) {
                ROS_WARN("Timeout reached without achieving target altitude.");
                break;
            }
        }
    }
    
    // Установка положения
    void set_position(float x, float y, float z, float yaw) {
        // Устанавливаем систему координат в рамках которой будет отправляться команда
        setpoint_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        // Установим битовую маску где покажем что должен выполнить автопилот, полет в точку или набор заданной скорости, ускорения, угла угловой скорости.
        setpoint_.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ
            | mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ
            | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        
        // В этом примере летим в точку с заданным углом.        
        setpoint_.position.x = x;
        setpoint_.position.y = y;
        setpoint_.position.z = z;
        setpoint_.yaw = yaw;
    }
    
    // Арм аппарата
    bool arm_vehicle(bool arm = true) {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = arm;
        
        ros::ServiceClient arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        return arming_client.call(arm_cmd) && arm_cmd.response.success;
    }

    // Смена режима полета
    bool change_mode(const std::string& mode) {
        mavros_msgs::SetMode sm;
        sm.request.custom_mode = mode;

        ros::ServiceClient client = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        return client.call(sm) && sm.response.mode_sent;
    }
    
    // Активация автоматического режима полета
    void offboard_enable(bool enable) {
        if (!enable) {
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
        change_mode("OFFBOARD");
    }

    // Таймер отправки целевого положения
    void offboard_timer_cb(const ros::TimerEvent&) {
        setpoint_.header.stamp = ros::Time::now();
        local_pos_pub_.publish(setpoint_);
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "flight_commander_node");
    FlightCommander commander;
    commander.run();
    return 0;
}

