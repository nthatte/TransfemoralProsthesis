#include "medulla_prosthesis.h"
#include <stdio.h>

//--- Define ethercat PDO entries ---//

// RxPDO entries
medulla_state_t *pros_command_state_pdo;
uint16_t *pros_counter_pdo;

// TxPDO entries
uint8_t *pros_medulla_id_pdo;
medulla_state_t *pros_current_state_pdo;
uint8_t *pros_medulla_counter_pdo;
uint8_t *pros_error_flags_pdo;
uint8_t *pros_limit_switch_pdo;

uint32_t *knee_encoder_pdo;
uint16_t *knee_encoder_timestamp_pdo;

uint32_t *ankle_encoder_pdo;
uint16_t *ankle_encoder_timestamp_pdo;

uint16_t *hall_pdo;
uint16_t *logic_voltage_pdo;

ecat_pdo_entry_t pros_rx_pdos[] = {{((void**)(&pros_command_state_pdo)),1},
                              {((void**)(&pros_counter_pdo)),2}};

ecat_pdo_entry_t pros_tx_pdos[] = {{((void**)(&pros_medulla_id_pdo)),1},
                              {((void**)(&pros_current_state_pdo)),1},
                              {((void**)(&pros_medulla_counter_pdo)),1},
                              {((void**)(&pros_error_flags_pdo)),1},
                              {((void**)(&pros_limit_switch_pdo)),1},
                              {((void**)(&knee_encoder_pdo)),4},
                              {((void**)(&knee_encoder_timestamp_pdo)),2},
                              {((void**)(&ankle_encoder_pdo)),4},
                              {((void**)(&ankle_encoder_timestamp_pdo)),2},
                              {((void**)(&hall_pdo)),4},
                              {((void**)(&logic_voltage_pdo)),2}};

// Structs for the medulla library
limit_sw_port_t limit_sw_port;
biss_encoder_t knee_encoder;
netzer_encoder_t ankle_encoder;
adc_port_t adc_port_a, adc_port_b;

// variables for filtering voltage values
uint8_t limit_switch_counter;
uint8_t pros_damping_cnt;
uint8_t logic_voltage_counter;
uint16_t hall_counter;
uint8_t knee_encoder_error_counter;
uint8_t ankle_encoder_error_counter;
bool pros_send_current_read;
TC0_t *pros_timestamp_timer;

void pros_initialize(uint8_t id, ecat_slave_t *ecat_slave, uint8_t *tx_sm_buffer, uint8_t *rx_sm_buffer, medulla_state_t **commanded_state, medulla_state_t **current_state, uint8_t **packet_counter, TC0_t *timestamp_timer, uint16_t **master_watchdog) {

	hall_counter = 0;
	logic_voltage_counter = 0;
	pros_timestamp_timer = timestamp_timer;
	*pros_error_flags_pdo = 0;
	pros_damping_cnt = 0;

	#if defined DEBUG_LOW || defined DEBUG_HIGH
	printf("[Medulla pros] Initializing pros with ID: %04x\n",id);
	#endif
	
	#ifdef DEBUG_HIGH
	printf("[Medulla pros] Initializing sync managers\n");
	#endif
	ecat_init_sync_managers(ecat_slave, rx_sm_buffer, MEDULLA_PROS_OUTPUTS_SIZE,
        0x1000, tx_sm_buffer, MEDULLA_PROS_INPUTS_SIZE, 0x2000);

	#ifdef DEBUG_HIGH
	printf("[Medulla pros] Initializing PDO entries\n");
	#endif
	ecat_configure_pdo_entries(ecat_slave, pros_rx_pdos, 
        MEDULLA_PROS_RX_PDO_COUNT, pros_tx_pdos, MEDULLA_PROS_TX_PDO_COUNT); 

	#ifdef DEUBG_HIGH
	printf("[Medulla pros] Initializing limit switches\n");
	#endif
	limit_sw_port = limit_sw_init_port(&PORTK,MEDULLA_PROS_LSW_MASK,&TCF0,pros_estop);
	limit_switch_counter = 0;

	#ifdef DEBUG_HIGH
	printf("[Medulla pros] Initializing ADC ports\n");
	#endif
	adc_port_a = adc_init_port(&ADCA);
	adc_port_b = adc_init_port(&ADCB);

	#ifdef DEBUG_HIGH
	printf("[Medulla pros] Initializing voltage monitoring pins\n");
	#endif
	adc_init_pin(&adc_port_b,6,logic_voltage_pdo);
	adc_init_pin(&adc_port_b,0,hall_pdo+0);
	adc_init_pin(&adc_port_b,2,hall_pdo+1);

	#ifdef DEBUG_HIGH
	printf("[Medulla pros] Initializing knee encoder\n");
	#endif
	knee_encoder = biss_encoder_init(&PORTC,&SPIC,timestamp_timer,32,
        knee_encoder_pdo,knee_encoder_timestamp_pdo);

	#ifdef DEBUG_HIGH
	printf("[Medulla pros] Initializing ankle encoder\n");
	#endif
	ankle_encoder = netzer_encoder_init(&PORTD,&SPID,timestamp_timer,32,
        ankle_encoder_pdo,ankle_encoder_timestamp_pdo);

	// Start reading the ADCs
	adc_start_read(&adc_port_a);
	adc_start_read(&adc_port_b);

	while (!adc_read_complete(&adc_port_a));
	while (!adc_read_complete(&adc_port_b));

	*master_watchdog = pros_counter_pdo;
	*packet_counter = pros_medulla_counter_pdo;
	*pros_medulla_id_pdo = id;
	*commanded_state = pros_command_state_pdo;
	*current_state = pros_current_state_pdo;
}
inline void pros_enable_outputs(void) {
	limit_sw_enable_port(&limit_sw_port);
}

inline void pros_disable_outputs(void) {
	limit_sw_disable_port(&limit_sw_port);
}

void pros_update_inputs(uint8_t id) {
	// Start reading the ADCs
	adc_start_read(&adc_port_a);
	adc_start_read(&adc_port_b);
	
	// Start reading from the encoders
	biss_encoder_start_reading(&knee_encoder);
	netzer_encoder_start_reading(&ankle_encoder);

	// while we are waiting for things to complete, get the limit switch state
	*pros_limit_switch_pdo = limit_sw_get_port(&limit_sw_port);

	// now wait for things to complete
	while (!adc_read_complete(&adc_port_a));
	while (!adc_read_complete(&adc_port_b));
 	while (!biss_encoder_read_complete(&knee_encoder));
	while (!netzer_encoder_read_complete(&ankle_encoder));

    // make sure our encoder data is accurate, if it is, then update, if it's
    // not, then increment the error coutner.
	if (biss_encoder_data_valid(&knee_encoder)) {
		biss_encoder_process_data(&knee_encoder);
	}
	else {
		*pros_error_flags_pdo |= medulla_error_encoder;
		knee_encoder_error_counter++;
	}
	
	if (netzer_encoder_data_valid(&ankle_encoder)) {
		netzer_encoder_process_data(&ankle_encoder);
	}
	else {
		*pros_error_flags_pdo |= medulla_error_encoder;
		ankle_encoder_error_counter++;
	}
}

bool pros_run_halt(uint8_t id) {
	pros_damping_cnt += 1;
	if (pros_damping_cnt > 100)
		return false;
	return true;
}

inline void pros_update_outputs(uint8_t id) {}

inline void pros_estop(void) {}

void pros_wait_loop() {}

bool pros_check_error(uint8_t id) {
	#ifdef ERROR_CHECK_LIMIT_SWITCH
	if (limit_sw_get_port(&limit_sw_port)) {
		limit_switch_counter ++;
	}
	else if (limit_switch_counter > 0)
		limit_switch_counter --;
	
	if (limit_switch_counter > 50) {
		#if defined DEBUG_LOW || DEBUG_HIGH
		printf("[Medulla pros] Limit switch error: %d\n",limit_sw_get_port(&limit_sw_port));
		#endif
		*pros_error_flags_pdo |= medulla_error_limit_switch;
		return true;
	}
	#endif

	#ifdef ERROR_CHECK_LOGIC_VOLTAGE
	// Do filter on logic voltage
	if (*logic_voltage_pdo < LOGIC_VOLTAGE_MIN)
		logic_voltage_counter++;
	else if (logic_voltage_counter > 0)
		logic_voltage_counter--;

	// Check if we are in the logic voltage danger range
	if (logic_voltage_counter > 500) {
		#if defined DEBUG_LOW || DEBUG_HIGH	
		printf("[Medulla pros] Logic voltage error.\n");
		#endif
		*pros_error_flags_pdo |= medulla_error_logic_voltage;
		return true;
	}
	#endif

	#ifdef ERROR_CHECK_ENCODER
	// Check the encoder error counters
	if ((motor_encoder_error_counter > 10) || (pros_encoder_error_counter > 10)) {
		#if defined DEBUG_LOW || DEBUG_HIGH	
		printf("[Medulla pros] Encoder read error\n");
		#endif
		*pros_error_flags_pdo |= medulla_error_encoder;
		return true;
	}
	#endif

    // If none of the above caused us to return true, then there are no errors
    // and we return false
	return false;
}

bool pros_check_halt(uint8_t id) {
	return false;
}

void pros_reset_error() {
	*pros_error_flags_pdo = 0;
	logic_voltage_counter = 0;
	hall_counter = 0;
	knee_encoder_error_counter = 0;
	ankle_encoder_error_counter = 0;
	pros_damping_cnt = 0;
}
