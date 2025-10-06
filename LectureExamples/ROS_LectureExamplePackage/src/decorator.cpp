/*An example of the Decorator Design pattern in a ROS 2 context.
The Decorator Design pattern extends the functionality of a class
The following example polymorphically extends a publisher based on ROS params

A decorator pattern has the following class
    - An abstract component class
    - A concrete component class
    - An abstract decorator class that wraps around a component class
    - A concrete decorator class extending functionality of the concrete component

The base publisher simply publishes from -5 to 5 at a regular interval
A safe velocity decorator clamps the velocity at 3
A forward velocity decorator ensures no negative velocities are publishes
The two decorators can be both applied or one at a time or none at all

To test:
    - ros2 run mtrx3760 decorator in one terminal (intended velocity)
    - ros2 topic echo linear_velocity in another terminal (published velocity)
    - set / unset parameters through the following command in another terminal
        - ros2 param set /decorator_node <forward_vel/safe_vel> <true/false>

By Khit Nay 2025
*/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

// An interface for a publisher
class VelocityPublisher
{
    public:
        virtual void publish(double linearX) = 0;
};

// A minimal publisher that just publishes intended velocities
class BasicVelocityPublisher : public VelocityPublisher
{
    public:
        BasicVelocityPublisher(rclcpp::Node* node);
        void publish(double linearX) override;

    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
};

// Base decorator that wraps around the interface for a publisher
class VelocityPublisherDecorator : public VelocityPublisher
{
    public:
        VelocityPublisherDecorator(const std::shared_ptr<VelocityPublisher>& publisher);

    protected:
        std::shared_ptr<VelocityPublisher> publisher;
};

// A decorator that ensures velocities are not negative
class ForwardVelocityDecorator : public VelocityPublisherDecorator
{
    public:
        ForwardVelocityDecorator(const std::shared_ptr<VelocityPublisher>& publisher);
        void publish(double linearX) override;
};

// A decorator that ensures velocities are not too high
class SafeVelocityDecorator : public VelocityPublisherDecorator
{
    public:
        SafeVelocityDecorator(const std::shared_ptr<VelocityPublisher>& publisher);
        void publish(double linearX) override;
};

// Main node that owns the publisher that can be decorated and listens to param changes
class DecoratorNode : public rclcpp::Node
{
    public:
        DecoratorNode();

    private:
        void rebuildChain(); // Restart decorator process from scratch
        void callback();

        rclcpp::TimerBase::SharedPtr timer;
        std::shared_ptr<VelocityPublisher> publisher;
        double velocity;
        int increment;

        std::shared_ptr<rclcpp::ParameterEventHandler> paramEventHandler;
        rclcpp::ParameterCallbackHandle::SharedPtr safeHandle;
        rclcpp::ParameterCallbackHandle::SharedPtr forwardHandle;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DecoratorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

BasicVelocityPublisher::BasicVelocityPublisher(rclcpp::Node* node)
{
    publisher = node->create_publisher<geometry_msgs::msg::Twist>("linear_velocity", 10);
}

void BasicVelocityPublisher::publish(double linearX)
{
    geometry_msgs::msg::Twist msg;
    msg.linear.x = linearX;
    publisher->publish(msg);
}

VelocityPublisherDecorator::VelocityPublisherDecorator(const std::shared_ptr<VelocityPublisher>& publisher)
    : publisher(publisher)
{}

ForwardVelocityDecorator::ForwardVelocityDecorator(const std::shared_ptr<VelocityPublisher>& publisher)
    : VelocityPublisherDecorator(publisher)
{}

void ForwardVelocityDecorator::publish(double linearX)
{
    if (linearX >= 0)
    {
        publisher->publish(linearX);
    }
}

SafeVelocityDecorator::SafeVelocityDecorator(const std::shared_ptr<VelocityPublisher>& publisher)
    : VelocityPublisherDecorator(publisher)
{}

void SafeVelocityDecorator::publish(double linearX)
{
    if (linearX > 3)
    {
        linearX = 3;
    }
    publisher->publish(linearX);
}

DecoratorNode::DecoratorNode()
    : Node("decorator_node"), velocity(0), increment(1)
{
    declare_parameter<bool>("safe_vel", false);
    declare_parameter<bool>("forward_vel", false);

    paramEventHandler = std::make_shared<rclcpp::ParameterEventHandler>(this);
    safeHandle = paramEventHandler->add_parameter_callback(
        "safe_vel",
        [this](const rclcpp::Parameter&){ rebuildChain(); });

    forwardHandle = paramEventHandler->add_parameter_callback(
        "forward_vel",
        [this](const rclcpp::Parameter&){ rebuildChain(); });

    rebuildChain();

    timer = create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&DecoratorNode::callback, this));
}

void DecoratorNode::rebuildChain()
{
    publisher = std::make_shared<BasicVelocityPublisher>(this);

    bool safe_enabled = get_parameter("safe_vel").as_bool();
    bool forward_enabled = get_parameter("forward_vel").as_bool();

    if (forward_enabled)
    {
        publisher = std::make_shared<ForwardVelocityDecorator>(publisher);
    }
    if (safe_enabled)
    {
        publisher = std::make_shared<SafeVelocityDecorator>(publisher);
    }

    RCLCPP_INFO(get_logger(), "Rebuilt publisher: forward=%d safe=%d",
                forward_enabled, safe_enabled);
}

void DecoratorNode::callback()
{
    // Simulate velocity goes from -5 to 5 and back down
    if (velocity == 5.0)
    {
        increment = -1;
    }
    else if (velocity == -5.0)
    {
        increment = 1;
    }

    velocity += increment;
    RCLCPP_INFO(get_logger(), "Intended velocity = %f", velocity);
    publisher->publish(velocity);
}