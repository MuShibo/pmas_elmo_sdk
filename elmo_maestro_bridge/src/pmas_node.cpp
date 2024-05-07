/**
 * @file pmas_node.cpp
 * @program FreeMan
 * @author Shibo Mu
 * @brief Single motor test.
 * @version 1.0
 * @date 2024
 */

#include "elmo_maestro_bridge/pmas_node.hpp"
#include "elmo_maestro_bridge/function_test.hpp"

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <stdint.h>
#include <sys/time.h>  // For time structure
#include <csignal>     // For Timer mechanism



/**
 * @brief the main function of this project
 * 
 * @return int 
 */
int main(int argc, char *argv[]) {

  EOE eoe;
  eoe.MainInit();

  // Init mode is set to be position mode.
  if(cAxis[0].SetOpMode(OPM402_CYCLIC_SYNC_POSITION_MODE) != 0){
    cout << "Set Operation Mode failed, error=" << endl;
  }
  // Waits until the operation mode will changed to "Cyclic Position".
  while (cAxis[0].GetOpMode() != OPM402_CYCLIC_SYNC_POSITION_MODE);        
  // Calibration.
  cAxis[0].SetPosition(
          1000, /*actual position*/
          0     /*mode. abs mode = 0. relative mode = 1.*/
          );

  sleep(1); 

  try {
    // enable axis and wait till axis enable is done
    std::cout << "status before power on: 0x" << std::hex << cAxis[0].ReadStatus() << std::endl;
    if (cAxis[0].ReadStatus() & NC_AXIS_DISABLED_MASK) {
      cAxis[0].PowerOn(MC_BUFFERED_MODE);
      // Waits for stand still status
      while (!(cAxis[0].ReadStatus() & NC_AXIS_STAND_STILL_MASK));
    }
    std::cout << "status after power on: 0x" << std::hex << cAxis[0].ReadStatus() << std::endl;
    std::cout << "If shutdown please use ctrl+c." << std::endl;

  } catch (CMMCException exp) {
    cout << "main function exception!!" << exp.what() << "error" << exp.error() << endl;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pmas_node>());
  rclcpp::shutdown();

  // Calls Stop command.
	cAxis[0].Stop(
				        10  * TZ90_ENCODER,           /*deceleration*/
				        10  * TZ90_ENCODER,           /*jerk*/
				        MC_ABORTING_MODE              /*buffer mode*/
			          );
	// Waits stand still status and stop status
	while ((cAxis[0].ReadStatus() & NC_AXIS_STAND_STILL_MASK) == 0 && (cAxis[0].ReadStatus() & NC_AXIS_ERROR_STOP_MASK) == 0);

	cAxis[0].PowerOff(MC_BUFFERED_MODE);
	// Waits for disabled status
	while ((cAxis[0].ReadStatus() & NC_AXIS_DISABLED_MASK)==0);
  // Disconnect from the Maestro.
	cConn.CloseConnection();

  return 0;
}
