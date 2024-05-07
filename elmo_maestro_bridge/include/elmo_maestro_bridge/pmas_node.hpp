#pragma once

#include "robot_config.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <ctime>

#include "elmo_motor_bridge/msg/motor_feedback.hpp" // No error, IDE include path problem.
#include "elmo_motor_bridge/msg/motor_cmd.hpp" // No error, IDE include path problem.

// Connection IP and Port.
auto host_ip = (char *)"192.168.1.250";
auto target_ip = (char *)"192.168.1.3";
int  tcp_port = 5000;


class pmas_node : public rclcpp::Node
{
  public:
    pmas_node() : Node("pmas_node_node")
    {
      // Rigest the subscriber.
      subscription_ = this->create_subscription<elmo_motor_bridge::msg::MotorCmd>("motor_cmd", 10, std::bind(&pmas_node::input_callback, this, std::placeholders::_1));
      
      // Rigest the publisher.
      publisher_ = this->create_publisher<elmo_motor_bridge::msg::MotorFeedback>("motor_read", 10);
      timer_ = this->create_wall_timer(std::chrono::microseconds(1000), std::bind(&pmas_node::timer_callback, this));

    }

  private:
    // Define the subscriber. For control the motors.
    rclcpp::Subscription<elmo_motor_bridge::msg::MotorCmd>::SharedPtr subscription_;

    // Define the publisher. Publish the state of motors.
    rclcpp::Publisher<elmo_motor_bridge::msg::MotorFeedback>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Define the server. The switch of the motors.
    //rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;

    // Cyclic Position Operation Mode
    void input_callback(const elmo_motor_bridge::msg::MotorCmd::SharedPtr motor_cmd) const{
      if(motor_cmd->op_mode == OPM402_CYCLIC_SYNC_POSITION_MODE){
        std::cout << "Position (rad) Cmd Receive:" << motor_cmd->position << std::endl;
        if(cAxis[0].GetOpMode() != OPM402_CYCLIC_SYNC_POSITION_MODE){
          int rc = cAxis[0].SetOpMode(OPM402_CYCLIC_SYNC_POSITION_MODE);
          if(rc != 0){
            cout << "Set Cyclic Position Operation Mode failed, error=" << rc << endl;
          }
          // Waits until the operation mode will changed to "Cyclic Position".
          while (cAxis[0].GetOpMode() != OPM402_CYCLIC_SYNC_POSITION_MODE);        
        }
        int pos_ctrl = cAxis[0].MoveAbsolute(
                              motor_cmd->position * RAD2ENCODER,   /*position. TZ90_ENCODER = 2 rad.*/
                              2000  * RAD2ENCODER,                 /*velocity.*/
                              1000 * RAD2ENCODER,                  /*acceleration*/
                              1000 * RAD2ENCODER,                  /*deceleration*/
                              1000 * RAD2ENCODER,                  /*jerk*/
                              MC_BUFFERED_MODE                     /*Buffer mode*/
                              ); 
        if (pos_ctrl != 0) {
          cout << "Move Absolute failed, error=" << pos_ctrl << endl;
        }
      }

      // Cyclic Torque Operation Mode
      else if(motor_cmd->op_mode == OPM402_CYCLIC_SYNC_TORQUE_MODE){
        std::cout << "Torque Cmd Receive:" << motor_cmd->current << std::endl;
        if(cAxis[0].GetOpMode() != OPM402_CYCLIC_SYNC_TORQUE_MODE){
          int rc = cAxis[0].SetOpMode(OPM402_CYCLIC_SYNC_TORQUE_MODE);
          if(rc != 0){
            cout << "Set Cyclic Position Operation Mode failed, error=" << rc << endl;
          }
          // Waits until the operation mode will changed to "Cyclic Position".
          while (cAxis[0].GetOpMode() != OPM402_CYCLIC_SYNC_TORQUE_MODE);        
        }
        cAxis[0].MoveTorque(
                            motor_cmd->current,            /*target torque*/
                            10  * TZ90_ENCODER,            /*torque velocity*/
                            100 * TZ90_ENCODER,            /*torque acceleration*/
                            MC_BUFFERED_MODE               /*Buffer mode*/
			                      );
      }

      // Cyclic Velocity Operation Mode.
      else if(motor_cmd->op_mode == OPM402_CYCLIC_SYNC_VELOCITY_MODE){
        std::cout << "Velocity (rad) Cmd Receive:" << motor_cmd->velocity << std::endl;
        if(cAxis[0].GetOpMode() != OPM402_CYCLIC_SYNC_VELOCITY_MODE){
          int rc = cAxis[0].SetOpMode(OPM402_CYCLIC_SYNC_VELOCITY_MODE);
          if(rc != 0){
            cout << "Set Cyclic Velocity Operation Mode failed, error=" << rc << endl;
          }
          // Waits until the operation mode will changed to "Cyclic Velocity".
          while (cAxis[0].GetOpMode() != OPM402_CYCLIC_SYNC_VELOCITY_MODE);        
        }
        int vel_ctrl = cAxis[0].MoveVelocity(motor_cmd->velocity * RAD2ENCODER, MC_BUFFERED_MODE);
        // Error print.
        if (vel_ctrl != 0) {
          cout << "Move Velocity failed, error=" << vel_ctrl << endl;
        }
      }

    }

    void timer_callback() {
      elmo_motor_bridge::msg::MotorFeedback mfb;

      // The data of std_msgs::Header.
      mfb.header.stamp = rclcpp::Clock().now();
      mfb.header.frame_id = "TZ90";

      // Motor feedback data.
      mfb.position = cAxis[0].GetActualPosition();
      mfb.velocity = cAxis[0].GetActualVelocity();
      mfb.current = static_cast<float>(cAxis[0].GetActualTorque());
      mfb.op_mode = cAxis[0].GetOpMode();
      //std::cout << "PCurrent Feedback:" << cAxis[0].GetActualTorque() << std::endl;
      publisher_->publish(mfb); // Publish the state from feedback.
     
    }
};

// init
class EOE
{
  public:
    void MainInit();
    int CloseConnection();

  private:
       
};
void StopAllMotor(int signum);
void Emergency_Received(unsigned short usAxisRef, short sEmcyCode);

/**
 * @brief Initialize the system, including axes, communication, etc.
 * 
 * @return int 
 */
void EOE::MainInit()
{
  // Create connection
  printf("init connection\n");
  conn_param.uiTcpPort = tcp_port;
  strcpy((char *)conn_param.ucIp, target_ip);
  g_conn_hndl = cConn.ConnectRPCEx(host_ip, target_ip, 0x7fffffff, NULL);  /*PMAS connection.*/

  printf("Connection process finish!\n");

  // Register the callback function for Emergency:
  cConn.RegisterEventCallback(MMCPP_EMCY, (void *)Emergency_Received);

  // Set Try-Catch flag Enable\Disable
  CMMCPPGlobal::Instance()->SetThrowFlag(false);
  CMMCPPGlobal::Instance()->SetThrowWarningFlag(false);

  try {
    cAxis[0].InitAxisData("a01", g_conn_hndl);

    for (int i = 0; i < MAX_AXES; i++) {
      auto Status = cAxis[i].ReadStatus();

      if (Status & NC_AXIS_ERROR_STOP_MASK) {
        std::cout << std::hex << Status << " " << std::hex << NC_AXIS_STOPPING_MASK << std::endl;
        cAxis[i].ResetAsync();
        Status = cAxis[i].ReadStatus();
        if (Status & NC_AXIS_ERROR_STOP_MASK) {
          throw runtime_error(to_string(Status));
        }
      }
    }
  } catch (CMMCException exp) {
    cout << "init failed!!" << exp.what() << endl;
    exit(1);
  }
  std::cout << std::endl;
}

/**
 * @brief Close connection
 * 
 * @return int 
 */
int EOE::CloseConnection()
{
  int retval;
  printf("close connection\n");

  retval = MMC_CloseConnection(g_conn_hndl);

  if (retval != 0) {
    printf("ERROR CloseConnection: MMC_CloseConnection fail %d\n", retval);
    return -1;
  }

  return 0;
}

void StopAllMotor(int signum)
{
  std::cout << "caught signal: " << signum << std::endl;
  for (int i = 0; i < MAX_AXES; i++) {
    cAxis[i].Stop();
  }
  exit(1);
}

/**
 * @brief Call back function if emergency received
 * 
 * @param usAxisRef 
 * @param sEmcyCode 
 */
void Emergency_Received(unsigned short usAxisRef, short sEmcyCode)
{
  printf("Emergency Message Received on Axis %d. Code: %x\n", usAxisRef, sEmcyCode);
}

