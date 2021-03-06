#include "atc_slip_hopping/ATCSlipHopping.hpp"

// The namespaces this controller resides in
namespace atrias {
namespace controller {

ATCSlipHopping::ATCSlipHopping(string name) :
	ATC(name),
	ascCommonToolkit(this, "ascCommonToolkit"),
	ascSlipModel(this, "ascSlipModel"),
	ascLegForceLl(this, "ascLegForceLl"),
	ascLegForceRl(this, "ascLegForceRl"),
	ascHipBoomKinematics(this, "ascHipBoomKinematics"),
	ascPDLmA(this, "ascPDLmA"),
	ascPDLmB(this, "ascPDLmB"),
	ascPDRmA(this, "ascPDRmA"),
	ascPDRmB(this, "ascPDRmB"),
	ascPDLh(this, "ascPDLh"),
	ascPDRh(this, "ascPDRh"),
	ascRateLimitLmA(this, "ascRateLimitLmA"),
	ascRateLimitLmB(this, "ascRateLimitLmB"),
	ascRateLimitRmA(this, "ascRateLimitRmA"),
	ascRateLimitRmB(this, "ascRateLimitRmB")
{
	// Set leg motor rate limit
	legRateLimit = 1.0;
	
	// Set hip controller toe positions
	toePosition.left = 2.15;
	toePosition.right = 2.45;
}


void ATCSlipHopping::controller() {

	/* Additionally, the following functions are available to command the robot state:
	 * commandHalt();    // Trigger a Halt
	 * commandEStop();   // Trigger an EStop
	 * commandDisable(); // Disable the robot
	 * setStartupEnabled(true/false) // Enable or disable the default startup controller
	 * setShutdownEnabled(true/false) // Enable or disable the shutdown controller
	 */

	// Startup is handled by the ATC class
	setStartupEnabled(true);

	// Update current robot state
	updateState();

	// Run hip controller
	hipController();

	// Main controller state machine
	switch (controllerState) {
		// Standing
		case 0:
			// Standing in place
			standingController();
			break;

		// Vertical hopping
		case 1:
			// SLIP hopping controller state machine
			switch (hoppingState) {
				// Stance phase
				case 0:
					// Stance control type
					if (stanceControlType == 0) {
						// Run passive stance phase - zero torque at the hip
						passiveStancePhaseController();
					} else if (stanceControlType == 1) {
						// Run force control stance phase - track SLIP forces
						slipForceStancePhaseController();
					} else if (stanceControlType == 2) {
						// Run force control stance phase - apply SLIP reaction forces
						virtualSpringStancePhaseController();
					}
					break;

				// Flight phase
				case 1:
					// Run flight phase controller
					flightPhaseController();
					break;
			}
			break;

		// Shutdown
		case 2:
			// Call shutdown controller
			shutdownController();
			break;
	}

	// Copy over positions to the GUI output data
	guiOut.isEnabled = isEnabled();

}


void ATCSlipHopping::updateState() {

	// Reset rate limiters if needed
	if (!isEnabled()) {
		ascRateLimitLmA.reset(rs.lLeg.halfA.motorAngle);
		ascRateLimitLmB.reset(rs.lLeg.halfB.motorAngle);
		ascRateLimitRmA.reset(rs.rLeg.halfA.motorAngle);
		ascRateLimitRmB.reset(rs.rLeg.halfB.motorAngle);
	}

	// Get GUI values
	controllerState = guiIn.main_controller;
	hoppingType = guiIn.hop_type;
	stanceControlType = guiIn.stance_controller;
	forceControlType = guiIn.force_type;
	springType = guiIn.spring_type;
	ascSlipModel.r0 = guiIn.slip_leg;
	h = guiIn.hop_height;

	// Set leg motor position control PD gains
	ascPDLmA.P = ascPDLmB.P = ascPDRmA.P = ascPDRmB.P = guiIn.leg_pos_kp;
	ascPDLmA.D = ascPDLmB.D = ascPDRmA.D = ascPDRmB.D = guiIn.leg_pos_kd;

	// Set hip motors position control PD gains
	ascPDLh.P = ascPDRh.P = guiIn.hip_pos_kp;
	ascPDLh.D = ascPDRh.D = guiIn.hip_pos_kd;

	// Set leg motor force control PID gains
	ascLegForceLl.kp = ascLegForceRl.kp = guiIn.leg_for_kp;
	ascLegForceLl.ki = ascLegForceRl.ki = 0.0;
	ascLegForceLl.kd = ascLegForceRl.kd = guiIn.leg_for_kd;

	// Compute actual leg force from spring deflection
	ascLegForceLl.compute(rs.lLeg, rs.position);// fix
	ascLegForceRl.compute(rs.rLeg, rs.position);// fix

	// Check for stance phase and set hopping state
	if (rs.position.zPosition < ascSlipModel.r0) {
		// Stance phase
		hoppingState = 0;
	} else {
		// Flight phase
		hoppingState = 1;
	}

	// Check hopping type and set stance leg(s)
	switch (hoppingType) {
		// Left leg hopping
		case 0:
			isLeftStance = true;
			isRightStance = false;
			break;

		// Right leg hopping
		case 1:
			isLeftStance = false;
			isRightStance = true;
			break;

		// Two leg hopping
		case 2:
			isLeftStance = true;
			isRightStance = true;
			break;
	}

}


void ATCSlipHopping::hipController() {

	// Compute inverse kinematics
	std::tie(qLh, qRh) = ascHipBoomKinematics.iKine(toePosition, rs.lLeg, rs.rLeg, rs.position);

	// Compute and set motor currents
	co.lLeg.motorCurrentHip = ascPDLh(qLh, rs.lLeg.hip.legBodyAngle, 0.0, rs.lLeg.hip.legBodyVelocity);
	co.rLeg.motorCurrentHip = ascPDRh(qRh, rs.rLeg.hip.legBodyAngle, 0.0, rs.rLeg.hip.legBodyVelocity);

}


void ATCSlipHopping::standingController() {

	// Set leg angles
	qLl = qRl = PI/2.0;
	rLl = rRl = guiIn.standing_leg;

	// Compute motor angles
	std::tie(qLmA, qLmB) = ascCommonToolkit.legPos2MotorPos(qLl, rLl);
	std::tie(qRmA, qRmB) = ascCommonToolkit.legPos2MotorPos(qRl, rRl);

	// Rate limit motor velocities
	qLmA = ascRateLimitLmA(qLmA, legRateLimit);
	qLmB = ascRateLimitLmB(qLmB, legRateLimit);
	qRmA = ascRateLimitRmA(qRmA, legRateLimit);
	qRmB = ascRateLimitRmB(qRmB, legRateLimit);

	// Compute and set motor currents
	co.lLeg.motorCurrentA = ascPDLmA(qLmA, rs.lLeg.halfA.motorAngle, 0.0, rs.lLeg.halfA.motorVelocity);
	co.lLeg.motorCurrentB = ascPDLmB(qLmB, rs.lLeg.halfB.motorAngle, 0.0, rs.lLeg.halfB.motorVelocity);
	co.rLeg.motorCurrentA = ascPDRmA(qRmA, rs.rLeg.halfA.motorAngle, 0.0, rs.rLeg.halfA.motorVelocity);
	co.rLeg.motorCurrentB = ascPDRmB(qRmB, rs.rLeg.halfB.motorAngle, 0.0, rs.rLeg.halfB.motorVelocity);

}


void ATCSlipHopping::shutdownController() {

	// Compute and set motor currents (applies virtual dampers to all actuators)
	co.lLeg.motorCurrentA = ascPDLmA(0.0, 0.0, 0.0, rs.lLeg.halfA.motorVelocity);
	co.lLeg.motorCurrentB = ascPDLmB(0.0, 0.0, 0.0, rs.lLeg.halfB.motorVelocity);
	co.lLeg.motorCurrentHip = ascPDLh(0.0, 0.0, 0.0, rs.lLeg.hip.legBodyVelocity);
	co.rLeg.motorCurrentA = ascPDRmA(0.0, 0.0, 0.0, rs.rLeg.halfA.motorVelocity);
	co.rLeg.motorCurrentB = ascPDRmB(0.0, 0.0, 0.0, rs.rLeg.halfB.motorVelocity);
	co.rLeg.motorCurrentHip = ascPDRh(0.0, 0.0, 0.0, rs.rLeg.hip.legBodyVelocity);

}


void ATCSlipHopping::slipForceStancePhaseController() {

	// Spring type
	if (springType == 0) {
		std::tie(ascSlipModel.k, ascSlipModel.dk) = ascCommonToolkit.legStiffness(slipState.r, slipState.dr, ascSlipModel.r0);
	} else if (springType == 1) {
		ascSlipModel.k = guiIn.slip_spring;
		ascSlipModel.dk = 0.0;
	}

	// Set SLIP model parameters, double stiffness if two leg hopping
	if (hoppingType == 2) {
		ascSlipModel.k = 2.0*ascSlipModel.k;
		ascSlipModel.dk = 2.0*ascSlipModel.dk;
	}

	// Compute SLIP force profile
	slipState = ascSlipModel.advanceRK5(slipState);
	legForce = ascSlipModel.force(slipState);

	// Halve the force if two leg hopping
	if (hoppingType == 2) {
		legForce.fx = legForce.fx/2.0;
		legForce.fz = legForce.fz/2.0;
		legForce.dfx = legForce.dfx/2.0;
		legForce.dfz = legForce.dfz/2.0;
	}

	// Left leg controller
	if (isLeftStance) {
		// If SLIP model says we should be in flight...
		if (slipState.isFlight) {
			// Use last know leg position from stance
			co.lLeg.motorCurrentA = ascPDLmA(qLmA, rs.lLeg.halfA.motorAngle, 0.0, rs.lLeg.halfA.motorVelocity);
			co.lLeg.motorCurrentB = ascPDLmB(qLmB, rs.lLeg.halfB.motorAngle, 0.0, rs.lLeg.halfB.motorVelocity);

		// If SLIP model says we should be in stance...
		} else {
			// Store last known leg position
			qLmA = rs.lLeg.halfA.legAngle;
			qLmB = rs.lLeg.halfB.legAngle;

			// Compute and set motor currents
			std::tie(co.lLeg.motorCurrentA, co.lLeg.motorCurrentB) = ascLegForceLl.control(legForce, rs.lLeg, rs.position);

		}

	} else {
		// Set motor angles
		std::tie(qLmA, qLmB) = ascCommonToolkit.legPos2MotorPos(PI/2.0, ascSlipModel.r0*0.85);

		// Compute and set motor currents
		co.lLeg.motorCurrentA = ascPDLmA(qLmA, rs.lLeg.halfA.motorAngle, 0.0, rs.lLeg.halfA.motorVelocity);
		co.lLeg.motorCurrentB = ascPDLmB(qLmB, rs.lLeg.halfB.motorAngle, 0.0, rs.lLeg.halfB.motorVelocity);
	}

	// Right leg controller
	if (isRightStance) {

		// If SLIP model says we should be in flight...
		if (slipState.isFlight) {
			// Use last know leg position from stance
			co.rLeg.motorCurrentA = ascPDRmA(qRmA, rs.rLeg.halfA.motorAngle, 0.0, rs.rLeg.halfA.motorVelocity);
			co.rLeg.motorCurrentB = ascPDRmB(qRmB, rs.rLeg.halfB.motorAngle, 0.0, rs.rLeg.halfB.motorVelocity);

		// If SLIP model says we should be in stance...
		} else {
			// Store last known leg position
			qRmA = rs.rLeg.halfA.legAngle;
			qRmB = rs.rLeg.halfB.legAngle;

			// Compute and set motor currents
			std::tie(co.rLeg.motorCurrentA, co.rLeg.motorCurrentB) = ascLegForceRl.control(legForce, rs.rLeg, rs.position);

		}

	} else {
		// Set motor angles
		std::tie(qRmA, qRmB) = ascCommonToolkit.legPos2MotorPos(PI/2.0, ascSlipModel.r0*0.85);

		// Compute and set motor currents
		co.rLeg.motorCurrentA = ascPDRmA(qRmA, rs.rLeg.halfA.motorAngle, 0.0, rs.rLeg.halfA.motorVelocity);
		co.rLeg.motorCurrentB = ascPDRmB(qRmB, rs.rLeg.halfB.motorAngle, 0.0, rs.rLeg.halfB.motorVelocity);
	}
}


void ATCSlipHopping::passiveStancePhaseController() {

	// Left leg controller
	if (isLeftStance) {
		// Compute current leg angle and length
		std::tie(qLl, rLl) = ascCommonToolkit.motorPos2LegPos(rs.lLeg.halfA.legAngle, rs.lLeg.halfB.legAngle);
			
		// Set motor angles
		std::tie(qLmA, qLmB) = ascCommonToolkit.legPos2MotorPos(qLl, ascSlipModel.r0);
	} else {
		// Set motor angles
		std::tie(qLmA, qLmB) = ascCommonToolkit.legPos2MotorPos(PI/2.0, ascSlipModel.r0*0.85);
	}

	// Right leg controller
	if (isRightStance) {
		// Compute current leg angle and length
		std::tie(qRl, rRl) = ascCommonToolkit.motorPos2LegPos(rs.rLeg.halfA.legAngle, rs.rLeg.halfB.legAngle);
			
		// Set motor angles
		std::tie(qRmA, qRmB) = ascCommonToolkit.legPos2MotorPos(qRl, ascSlipModel.r0);
	} else {
		// Set motor angles
		std::tie(qRmA, qRmB) = ascCommonToolkit.legPos2MotorPos(PI/2.0, ascSlipModel.r0*0.85);
	}

	// Compute and set motor currents
	co.lLeg.motorCurrentA = ascPDLmA(qLmA, rs.lLeg.halfA.motorAngle, 0.0, rs.lLeg.halfA.motorVelocity);
	co.lLeg.motorCurrentB = ascPDLmB(qLmB, rs.lLeg.halfB.motorAngle, 0.0, rs.lLeg.halfB.motorVelocity);
	co.rLeg.motorCurrentA = ascPDRmA(qRmA, rs.rLeg.halfA.motorAngle, 0.0, rs.rLeg.halfA.motorVelocity);
	co.rLeg.motorCurrentB = ascPDRmB(qRmB, rs.rLeg.halfB.motorAngle, 0.0, rs.rLeg.halfB.motorVelocity);
}


void ATCSlipHopping::virtualSpringStancePhaseController() {

	// Spring type and stiffness
	if (springType == 0) {
		// ATRIAS non-linear spring constant (per leg)
		std::tie(k, dk) = ascCommonToolkit.legStiffness(slipState.r, slipState.dr, ascSlipModel.r0);
		
	} else if (springType == 1) {
		// Desired linear stiffness (per leg)
		k = guiIn.slip_spring;
		dk = 0.0;

	}

	// Left leg controller
	if (isLeftStance) {
		// Compute curent leg angle
		std::tie(qLl, rLl) = ascCommonToolkit.motorPos2LegPos(rs.lLeg.halfA.legAngle, rs.lLeg.halfB.legAngle);
		std::tie(dqLl, drLl) = ascCommonToolkit.motorVel2LegVel(rs.lLeg.halfA.legAngle, rs.lLeg.halfB.legAngle, rs.lLeg.halfA.legVelocity, rs.lLeg.halfB.legVelocity);

		// Define component forces
		legForce.fx = -k*(rLl - ascSlipModel.r0)*cos(qLl);
		legForce.dfx = dk*cos(qLl)*(ascSlipModel.r0 - rLl) - drLl*cos(qLl)*k + dqLl*sin(qLl)*k*(rLl - ascSlipModel.r0);
		legForce.fz = k*(rLl - ascSlipModel.r0)*sin(qLl);
		legForce.dfz = drLl*sin(qLl)*k - dk*sin(qLl)*(ascSlipModel.r0 - rLl) + dqLl*cos(qLl)*k*(rLl - ascSlipModel.r0);
		
		// Apply the force
		std::tie(co.lLeg.motorCurrentA, co.lLeg.motorCurrentB) = ascLegForceLl.control(legForce, rs.lLeg, rs.position);

	} else {
		// Set motor angles
		std::tie(qLmA, qLmB) = ascCommonToolkit.legPos2MotorPos(PI/2.0, ascSlipModel.r0*0.85);

		// Compute and set motor currents
		co.lLeg.motorCurrentA = ascPDLmA(qLmA, rs.lLeg.halfA.motorAngle, 0.0, rs.lLeg.halfA.motorVelocity);
		co.lLeg.motorCurrentB = ascPDLmB(qLmB, rs.lLeg.halfB.motorAngle, 0.0, rs.lLeg.halfB.motorVelocity);
	}

	// Right leg controller
	if (isRightStance) {
		// Compute curent leg angle
		std::tie(qRl, rRl) = ascCommonToolkit.motorPos2LegPos(rs.rLeg.halfA.legAngle, rs.rLeg.halfB.legAngle);
		std::tie(dqRl, drRl) = ascCommonToolkit.motorVel2LegVel(rs.rLeg.halfA.legAngle, rs.rLeg.halfB.legAngle, rs.rLeg.halfA.legVelocity, rs.rLeg.halfB.legVelocity);
	
		// Define component forces
		legForce.fx = -k*(rRl - ascSlipModel.r0)*cos(qRl);
		legForce.dfx = dk*cos(qRl)*(ascSlipModel.r0 - rRl) - drRl*cos(qRl)*k + dqRl*sin(qRl)*k*(rRl - ascSlipModel.r0);
		legForce.fz = k*(rRl - ascSlipModel.r0)*sin(qRl);
		legForce.dfz = drRl*sin(qRl)*k - dk*sin(qRl)*(ascSlipModel.r0 - rRl) + dqRl*cos(qRl)*k*(rRl - ascSlipModel.r0);
		
		// Apply the force
		std::tie(co.rLeg.motorCurrentA, co.rLeg.motorCurrentB) = ascLegForceRl.control(legForce, rs.rLeg, rs.position);

	} else {
		// Set motor angles
		std::tie(qRmA, qRmB) = ascCommonToolkit.legPos2MotorPos(PI/2.0, ascSlipModel.r0*0.85);

		// Compute and set motor currents
		co.rLeg.motorCurrentA = ascPDRmA(qRmA, rs.rLeg.halfA.motorAngle, 0.0, rs.rLeg.halfA.motorVelocity);
		co.rLeg.motorCurrentB = ascPDRmB(qRmB, rs.rLeg.halfB.motorAngle, 0.0, rs.rLeg.halfB.motorVelocity);
	}
}


void ATCSlipHopping::flightPhaseController() {

	// Redefine slip initial conditions incase we go into stance next time step
	switch (forceControlType) {
		// Updated initial conditions (apex tracking)
		case 0:
			std::tie(slipState.q, slipState.r) = ascCommonToolkit.motorPos2LegPos(rs.rLeg.halfA.legAngle, rs.rLeg.halfB.legAngle);
			std::tie(slipState.dq, slipState.dr) = ascCommonToolkit.cartVel2PolVel(slipState.q, slipState.r, rs.position.xVelocity, rs.position.zVelocity);
			break;

		// Non-updated intitial conditions (terrain following)
		case 1:
			slipState.r = ascSlipModel.r0;
			slipState.q = PI/2.0;
			slipState.dq = 0.0;
			slipState.dr = -sqrt(2.0*9.81*h);
			break;
	}

	// Left leg controller
	if (isLeftStance) {
		// Set upcoming stance leg motor angles
		std::tie(qLmA, qLmB) = ascCommonToolkit.legPos2MotorPos(PI/2.0, ascSlipModel.r0);
	} else {
		// Set upcoming flight leg motor angles
		std::tie(qLmA, qLmB) = ascCommonToolkit.legPos2MotorPos(PI/2.0, ascSlipModel.r0*0.85);
	}

	// Right leg controller
	if (isRightStance) {
		// Set upcoming stance leg motor angles
		std::tie(qRmA, qRmB) = ascCommonToolkit.legPos2MotorPos(PI/2.0, ascSlipModel.r0);
	} else {
		// Set upcoming flight leg motor angles
		std::tie(qRmA, qRmB) = ascCommonToolkit.legPos2MotorPos(PI/2.0, ascSlipModel.r0*0.85);
	}

	// Rate limit motor velocities
	qLmA = ascRateLimitLmA(qLmA, legRateLimit);
	qLmB = ascRateLimitLmB(qLmB, legRateLimit);
	qRmA = ascRateLimitRmA(qRmA, legRateLimit);
	qRmB = ascRateLimitRmB(qRmB, legRateLimit);

	// Compute and set motor currents
	co.lLeg.motorCurrentA = ascPDLmA(qLmA, rs.lLeg.halfA.motorAngle, 0.0, rs.lLeg.halfA.motorVelocity);
	co.lLeg.motorCurrentB = ascPDLmB(qLmB, rs.lLeg.halfB.motorAngle, 0.0, rs.lLeg.halfB.motorVelocity);
	co.rLeg.motorCurrentA = ascPDRmA(qRmA, rs.rLeg.halfA.motorAngle, 0.0, rs.rLeg.halfA.motorVelocity);
	co.rLeg.motorCurrentB = ascPDRmB(qRmB, rs.rLeg.halfB.motorAngle, 0.0, rs.rLeg.halfB.motorVelocity);
}


// We need to make top-level controllers components
ORO_CREATE_COMPONENT(ATCSlipHopping)

}
}

// vim: noexpandtab
