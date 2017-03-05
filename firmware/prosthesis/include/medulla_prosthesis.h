#ifndef MEDULLA_PROS_H
#define MEDULLA_PROS_H

#include <stdio.h>

// Include the robot definitions
#include "robot_invariant_defs.h"
#include "robot_variant_defs.h"

// Include medulla_lib headers
#include "ethercat.h"
#include "limit_switch.h"
#include "adc.h"
#include "biss_encoder.h"
#include "biss_encoder_netzer.h"

// Defines for which systems to check for error state
#define ERROR_CHECK_LIMIT_SWITCH
#define ERROR_CHECK_LOGIC_VOLTAGE

void pros_initialize(uint8_t id, ecat_slave_t *ecat_slave, uint8_t *tx_sm_buffer, uint8_t *rx_sm_buffer, medulla_state_t **commanded_state, medulla_state_t **current_state, uint8_t **packet_counter, TC0_t *timestamp_timer, uint16_t **master_watchdog);

void pros_enable_outputs(void);
void pros_disable_outputs(void);
void pros_update_inputs(uint8_t id);  /**< Function called to read all the sensors */
bool pros_run_halt(uint8_t id);       /**< Runs the halt controller. Returns true if the controller wants to continue running */ 
void pros_update_outputs(uint8_t id); /**< Called to update motor outputs */
void pros_estop(void);                /**< Called to when estop happens, should send 0 torques to motor */
bool pros_check_error(uint8_t id);    /**< Checks sensor readings to decide if estop should happen, returns true if there should be an error */
bool pros_check_halt(uint8_t id);     /**< Checks sensors to determine if medulla should go into halt state, returns true if medulla should go into halt */
void pros_reset_error();
void pros_wait_loop();

#endif //MEDULLA_PROS_H
