#ifndef ATC_SLIP_HOPPING_HPP
#define ATC_SLIP_HOPPING_HPP

/**
  * @file ATC_SLIP_HOPPING.hpp
  * @author Mikhail Jones
  * @brief This implements a SLIP based template controller.
  */

// Top-level controllers are components, so we need to include this.
#include <rtt/Component.hpp>

// Include the ATC class
#include <atrias_control_lib/ATC.hpp>
// No logging helper is needed -- one log port is automatically produced.

// Our logging data type.
#include "atc_slip_hopping/controller_log_data.h"

// The type transmitted from the GUI to the controller
#include "atc_slip_hopping/controller_input.h"

// The type transmitted from the controller to the GUI
#include "atc_slip_hopping/controller_status.h"

// Our subcontroller types
#include <asc_common_toolkit/ASCCommonToolkit.hpp>
#include <asc_slip_model/ASCSlipModel.hpp>
#include <asc_leg_force/ASCLegForce.hpp>
#include <asc_hip_boom_kinematics/ASCHipBoomKinematics.hpp>
#include <asc_pd/ASCPD.hpp>
#include <asc_rate_limit/ASCRateLimit.hpp>

// Datatypes
#include <robot_invariant_defs.h>
#include <atrias_msgs/robot_state.h>
#include <atrias_shared/controller_structs.h>
#include <atrias_shared/atrias_parameters.h>

// Namespaces we're using
using namespace std;
using namespace atc_slip_hopping;

// Our namespaces
namespace atrias {
namespace controller {

/* Our class definition. We subclass ATC for a top-level controller.
 * If we don't need a data type (such as the controller-to-gui message),
 * we simply leave that spot in the template blank. The following example
 * shows the necessary definition if this controller were not to transmit
 * data to the GUI:
 *     class ATC : public ATC<log_data, gui_to_controller,>
 *
 * Here, we don't need any log data, but we do communicate both ways w/ the GUI
 */
class ATCSlipHopping : public ATC<atc_slip_hopping::controller_log_data, controller_input, controller_status> {
	public:
		/**
		  * @brief The constructor for this controller.
		  * @param name The name of this component.
		  * Every top-level controller will have this name parameter,
		  * just like current controllers.
		  */
		ATCSlipHopping(string name);

	private:
		/**
		  * @brief This is the main function for the top-level controller.
		  * The ATC class automatically handles startup and shutdown,
		  * if they are not disabled.
		  */
		void controller();

		/**
		  * @brief Gets values from GUI and updates all relavent states.
		  */
		void updateState();
		int controllerState, hoppingState;
		int stanceControlType, hoppingType, forceControlType, springType;
		bool isLeftStance, isRightStance;

		/**
		  * @brief A kinematically driven hip controller to limit knee forces.
		  */
		void hipController();
		double qLh, qRh;
		LeftRight toePosition;
		
		/**
		  * @brief A simple two leg standing controller
		  */
		void standingController();
		double qLl, rLl, qRl, rRl, qLmA, qLmB, qRmA, qRmB;
		double legRateLimit;
		
		/**
		  * @brief A SLIP based force tracking stance phase controller.
		  */
		void forceStancePhaseController();
		double ql, rl, h;
		SlipState slipState;
		LegForce legForce, fTemp;
		
		/**
		  * @brief A simple stance phase controller allowing only leg length 
		  * forces with zero leg angle torques.
		  */
		void passiveStancePhaseController();
		
		/**
		  * @brief A simple constant leg position flight phase controller.
		  */
		void flightPhaseController();
		
		/**
		  * @brief Applies virtual dampers to all motors.
		  */
		void shutdownController();
		
		/**
		  * @brief These are sub controllers used by the top level controller.
		  */
  		ASCCommonToolkit ascCommonToolkit;
		ASCSlipModel ascSlipModel;
		ASCLegForce ascLegForceLl;
		ASCLegForce ascLegForceRl;
		ASCHipBoomKinematics ascHipBoomKinematics;
		ASCPD ascPDLmA;
		ASCPD ascPDLmB;
		ASCPD ascPDRmA;
		ASCPD ascPDRmB;
		ASCPD ascPDLh;
		ASCPD ascPDRh;
		ASCRateLimit ascRateLimitLmA;
		ASCRateLimit ascRateLimitLmB;
		ASCRateLimit ascRateLimitRmA;
		ASCRateLimit ascRateLimitRmB;

};

}
}

#endif // ATC_SLIP_HOPPING_HPP