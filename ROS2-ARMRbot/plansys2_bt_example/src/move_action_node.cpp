#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "tu_int/srv/execute_actions.hpp"
#include "tu_int/srv/posi_obj.hpp"
#include "tu_int/msg/pos_obj.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"



using namespace std::chrono_literals;


class MoveAction : public plansys2::ActionExecutorClient
{
public:
  MoveAction()
  : plansys2::ActionExecutorClient("move", 250ms)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  {
    std::vector<std::string> something = current_arguments_;
    std::cout << "Arguments: ";
	for (size_t i = 0; i < something.size(); ++i) {
    		std::cout << something[i] << " ";
	}
    std::cout << std::flush;   

    // Call the server with the arguments
    bool success = callServer(something);

    if (success) {
      finish(true, 1.0, "Move completed successfully");
    } else {
      finish(false, 0.0, "Move failed");
      
    }

    progress_ = 0.0;
    std::cout << std::endl;
  }

  float progress_;
  // Function to call the server with the arguments
  bool callServer(const std::vector<std::string>& args)
  {
    auto node = std::make_shared<MoveAction>();

    double randx, randy, randz;

    auto client1 = node->create_client<tu_int::srv::PosiObj>("posi_obj");
    // Wait for the server to start
    if (!client1->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_INFO(node->get_logger(), "Server not available");
      return false;
    }

    // Create the request
    auto request1 = std::make_shared<tu_int::srv::PosiObj::Request>();
    request1->pos = args[2];

    // Call the server
    auto result1 = client1->async_send_request(request1);
    rclcpp::spin_until_future_complete(node, result1);

    
    // Access the first position in the vector (assuming it's present)
   try {
    auto result2 = result1.get();
    randy = result2->position.y;
    randx = result2->position.x;
    randz = result2->position.z;
} catch (const std::future_error &e) {
    // Handle the future error
    RCLCPP_ERROR(node->get_logger(), "std::future_error: %s", e.what());
}

    auto client = node->create_client<tu_int::srv::ExecuteActions>("execute_actions");

    // Wait for the server to start
    if (!client->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_INFO(node->get_logger(), "Server not available");
      return false;
    }

    // Create the request
    auto request = std::make_shared<tu_int::srv::ExecuteActions::Request>();
    request->posit.x = randx;
    request->posit.y = randy;
    request->posit.z = randz;
    request->pos = args[2];
    RCLCPP_INFO(node->get_logger(), "move ----%f",randx);
    RCLCPP_INFO(node->get_logger(), "move ----%f",randy);

    // Call the server
    auto result = client->async_send_request(request);
    rclcpp::spin_until_future_complete(node, result);

    if (result.get()->success) {
      RCLCPP_INFO(node->get_logger(), "Server response: Success");
      return true;
    } else {
      RCLCPP_ERROR(node->get_logger(), "Server response: Failure");
      return false;
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "move"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}





 
  