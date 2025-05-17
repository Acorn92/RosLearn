#include 	<ros/ros.h>
#include 	<geometry_msgs/PoseStamped.h>
#include	"uav_controller.hpp"


// Объявим точку входа в исполняемом файле
int main(int argc, char **argv)
{
    // Инициализация ноды и регистрция ее в master node
    // с именем uav_controller_node
	ros::init(argc, argv, "uav_controller_node");
    // cоздаем экземпляр класса ноды
	ros::NodeHandle n;
	geometry_msgs::PoseStamped		desPose;
	ros::Rate 				rate(30);

	// создаем экземпляр класса системы управления БЛА
	uav_controller::UavController controller(n);

	while(ros::ok())
	{
		// Делаем шаг системы
        // После вызова данной комманды если в очереди были сообщения
        // произойдет чтение сообщение из очереди и вызов callback
        ros::spinOnce();
        // Ожидание для поддержания частоты работы системы
        // ожидание проходит с учетом времени между текущей и предыдущей итерациями
        // что позволяет более точно регулировать частоту работы системы
        rate.sleep();

	}

}