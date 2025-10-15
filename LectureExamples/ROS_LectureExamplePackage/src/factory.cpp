/*An example of the Factory Design pattern in a ROS 2 context.
The Factory Design pattern simplifies how an object gets created
The following example polymorphically creates a motor for a robot based on a ROS param.

A factory pattern has the followng classes
    - An abstract producible class
    - Concrete producible classes
    - A factory class that creates concrete classes
    - A client class that consumes the factory class

std::cout is used to demonstrate the motors operating but these should be avoided
in a real application

May appear similar to the strategy pattern.
Factory = how the object gets made.
Strategy = how the object behaves.

To test:
    - ros2 run mtrx3760 factory --ros-args -p motor_type:=dc (or)
    - ros2 run mtrx3760 factory --ros-args -p motor_type:=servo

By Khit Nay 2025
*/

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>

class Motor
{
    public:
        virtual void move() = 0;
        virtual ~Motor() = default;

        enum class MotorType
        {
            DC,
            SERVO
        };
};

class DCMotor : public Motor
{
    public:
        void move() override;
};

class ServoMotor : public Motor
{
    public:
        void move() override;
};

class MotorFactory
{
    public:
        static std::unique_ptr<Motor> createMotor(const Motor::MotorType motorType);
};

class MotorNode : public rclcpp::Node
{
    public:
        MotorNode();

    private:
        void timerCallback();
        std::unique_ptr<Motor> motor;
        rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorNode>());
    rclcpp::shutdown();
    return 0;
}

void DCMotor::move()
{
    std::cout << "DC Motor Moving" << std::endl;
}

void ServoMotor::move()
{
    std::cout << "Servo Motor Moving" << std::endl;
}

std::unique_ptr<Motor> MotorFactory::createMotor(const Motor::MotorType motorType)
{
    if (motorType == Motor::MotorType::DC)
    {
        return std::make_unique<DCMotor>();
    }
    else if (motorType == Motor::MotorType::SERVO)
    {
        return std::make_unique<ServoMotor>();
    }
    else
    {
        throw std::runtime_error("Unknown motor type");
    }
}

MotorNode::MotorNode()
    : Node("motor_node")
{
    // Parameter to pick motor type
    std::string motor_param = declare_parameter<std::string>("motor_type", "dc");

    if (motor_param == "dc")
    {
        motor = MotorFactory::createMotor(Motor::MotorType::DC);
    }
    else if (motor_param == "servo")
    {
        motor = MotorFactory::createMotor(Motor::MotorType::SERVO);
    }
    else
    {
        throw std::runtime_error("Invalid motor type parameter");
    }

    // Call move() every 1 second
    timer = create_wall_timer(std::chrono::seconds(1), std::bind(&MotorNode::timerCallback, this));
}

void MotorNode::timerCallback()
{
    motor->move();
}