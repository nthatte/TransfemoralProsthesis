#include "medulla_knee.h"

//--- Define ethercat PDO entries ---//

// RxPDO entries
medulla_state_t *knee_command_state_pdo;
uint16_t *knee_counter_pdo;

// TxPDO entries
uint8_t *knee_medulla_id_pdo;
medulla_state_t *knee_current_state_pdo;
uint8_t *knee_medulla_counter_pdo;
uint8_t *knee_error_flags_pdo;
uint8_t *knee_limit_switch_pdo;

uint32_t *motor_encoder_pdo;
uint16_t *motor_encoder_timestamp_pdo;

uint32_t *knee_encoder_pdo;
uint16_t *knee_encoder_timestamp_pdo;

uint16_t *logic_voltage_pdo;

uint16_t *thermistor_pdo; // Pointer to all the thermistors, you can access them as an array

ecat_pdo_entry_t knee_rx_pdos[] = {{((void**)(&knee_command_state_pdo)),1},
                              {((void**)(&knee_counter_pdo)),2}};

ecat_pdo_entry_t knee_tx_pdos[] = {{((void**)(&knee_medulla_id_pdo)),1},
                              {((void**)(&knee_current_state_pdo)),1},
                              {((void**)(&knee_medulla_counter_pdo)),1},
                              {((void**)(&knee_error_flags_pdo)),1},
                              {((void**)(&knee_limit_switch_pdo)),1},
                              {((void**)(&motor_encoder_pdo)),4},
                              {((void**)(&motor_encoder_timestamp_pdo)),2},
                              {((void**)(&knee_encoder_pdo)),4},
                              {((void**)(&knee_encoder_timestamp_pdo)),2},
                              {((void**)(&logic_voltage_pdo)),2},
                              {((void**)(&thermistor_pdo)),12}};


// Structs for the medulla library
limit_sw_port_t limit_sw_port;
biss_encoder_t knee_encoder, motor_encoder;
adc_port_t adc_port_a, adc_port_b;

// variables for filtering thermistor and voltage values
uint8_t limit_switch_counter;
uint8_t knee_damping_cnt;
uint8_t thermistor_counter;
uint16_t logic_voltage_counter;
uint8_t motor_encoder_error_counter;
uint8_t knee_encoder_error_counter;
bool knee_send_current_read;
TC0_t *knee_timestamp_timer;
uint16_t knee_knee_adc_aux4;
uint16_t knee_therm_prev_val[6];

void knee_initialize(uint8_t id, ecat_slave_t *ecat_slave, uint8_t *tx_sm_buffer, uint8_t *rx_sm_buffer, medulla_state_t **commanded_state, medulla_state_t **current_state, uint8_t **packet_counter, TC0_t *timestamp_timer, uint16_t **master_watchdog) {

	thermistor_counter = 0;
	logic_voltage_counter = 0;
	knee_timestamp_timer = timestamp_timer;
	*knee_error_flags_pdo = 0;
	knee_damping_cnt = 0;

	#if defined DEBUG_LOW || defined DEBUG_HIGH
	printf("[Medulla Knee] Initializing knee with ID: %04x\n",id);
	#endif
	
	#ifdef DEBUG_HIGH
	printf("[Medulla Knee] Initializing sync managers\n");
	#endif
	ecat_init_sync_managers(ecat_slave, rx_sm_buffer, MEDULLA_KNEE_OUTPUTS_SIZE, 0x1000, tx_sm_buffer, MEDULLA_KNEE_INPUTS_SIZE, 0x2000);

	#ifdef DEBUG_HIGH
	printf("[Medulla Knee] Initializing PDO entries\n");
	#endif
	ecat_configure_pdo_entries(ecat_slave, knee_rx_pdos, MEDULLA_KNEE_RX_PDO_COUNT, knee_tx_pdos, MEDULLA_KNEE_TX_PDO_COUNT-5); 

	#ifdef DEUBG_HIGH
	printf("[Medulla knee] Initializing limit switches\n");
	#endif
	limit_sw_port = limit_sw_init_port(&PORTK,MEDULLA_KNEE_LSW_MASK,&TCF0,knee_estop);
	limit_switch_counter = 0;

	#ifdef DEBUG_HIGH
	printf("[Medulla knee] Initializing ADC ports\n");
	#endif
	adc_port_a = adc_init_port(&ADCA);
	adc_port_b = adc_init_port(&ADCB);

	#ifdef DEBUG_HIGH
	printf("[Medulla knee] Initializing Thermistor ADC pins\n");
	#endif
	adc_init_pin(&adc_port_a,1,thermistor_pdo+0);
	adc_init_pin(&adc_port_a,2,thermistor_pdo+1);
	adc_init_pin(&adc_port_a,3,thermistor_pdo+2);
	adc_init_pin(&adc_port_a,4,thermistor_pdo+3);
	adc_init_pin(&adc_port_a,5,thermistor_pdo+4);
	adc_init_pin(&adc_port_a,6,thermistor_pdo+5);
	
	#ifdef DEBUG_HIGH
	printf("[Medulla knee] Initializing voltage monitoring pins\n");
	#endif
	adc_init_pin(&adc_port_b,6,logic_voltage_pdo);

	#ifdef DEBUG_HIGH
	printf("[Medulla knee] Initializing motor encoder\n");
	#endif
	motor_encoder = biss_encoder_init(&PORTC,&SPIC,timestamp_timer,32,motor_encoder_pdo,motor_encoder_timestamp_pdo);

	#ifdef DEBUG_HIGH
	printf("[Medulla knee] Initializing knee encoder\n");
	#endif
	knee_encoder = biss_encoder_init(&PORTD,&SPID,timestamp_timer,32,knee_encoder_pdo,knee_encoder_timestamp_pdo);

	// Start reading the ADCs
	adc_start_read(&adc_port_a);
	adc_start_read(&adc_port_b);

	while (!adc_read_complete(&adc_port_a));
	while (!adc_read_complete(&adc_port_b));

	knee_therm_prev_val[0] = thermistor_pdo[0];
	knee_therm_prev_val[1] = thermistor_pdo[1];
	knee_therm_prev_val[2] = thermistor_pdo[2];
	knee_therm_prev_val[3] = thermistor_pdo[3];
	knee_therm_prev_val[4] = thermistor_pdo[4];
	knee_therm_prev_val[5] = thermistor_pdo[5];

	*master_watchdog = knee_counter_pdo;
	*packet_counter = knee_medulla_counter_pdo;
	*knee_medulla_id_pdo = id;
	*commanded_state = knee_command_state_pdo;
	*current_state = knee_current_state_pdo;
}
inline void knee_enable_outputs(void) {
	limit_sw_enable_port(&limit_sw_port);
}

inline void knee_disable_outputs(void) {
	limit_sw_disable_port(&limit_sw_port);
}

void knee_update_inputs(uint8_t id) {
	// Start reading the ADCs
	adc_start_read(&adc_port_a);
	adc_start_read(&adc_port_b);
	
	// Start reading from the encoders
	biss_encoder_start_reading(&motor_encoder);
	biss_encoder_start_reading(&knee_encoder);

	// while we are waiting for things to complete, get the limit switch state
	*knee_limit_switch_pdo = limit_sw_get_port(&limit_sw_port);

	// now wait for things to complete
	while (!adc_read_complete(&adc_port_a));
	while (!adc_read_complete(&adc_port_b));
 	while (!biss_encoder_read_complete(&motor_encoder));
	while (!biss_encoder_read_complete(&knee_encoder));

	// make sure our encoder data is accurate, if it is, then update, if it's not, then increment the error coutner.
	if (biss_encoder_data_valid(&motor_encoder)) {
		biss_encoder_process_data(&motor_encoder);
	}
	else {
		*knee_error_flags_pdo |= medulla_error_encoder;
		motor_encoder_error_counter++;
	}
	
	if (biss_encoder_data_valid(&knee_encoder)) {
		biss_encoder_process_data(&knee_encoder);
	}
	else {
		*knee_error_flags_pdo |= medulla_error_encoder;
		knee_encoder_error_counter++;
	}

	if (((knee_therm_prev_val[0]>thermistor_pdo[0]) && (knee_therm_prev_val[0]-thermistor_pdo[0] < 50)) ||
	    ((knee_therm_prev_val[0]<thermistor_pdo[0]) && (thermistor_pdo[0]-knee_therm_prev_val[0] < 50)))
		knee_therm_prev_val[0] = thermistor_pdo[0];

	if (((knee_therm_prev_val[1]>thermistor_pdo[1]) && (knee_therm_prev_val[1]-thermistor_pdo[1] < 50)) ||
	    ((knee_therm_prev_val[1]<thermistor_pdo[1]) && (thermistor_pdo[1]-knee_therm_prev_val[1] < 50)))
		knee_therm_prev_val[1] = thermistor_pdo[1];

	if (((knee_therm_prev_val[2]>thermistor_pdo[2]) && (knee_therm_prev_val[2]-thermistor_pdo[2] < 50)) ||
	    ((knee_therm_prev_val[2]<thermistor_pdo[2]) && (thermistor_pdo[2]-knee_therm_prev_val[2] < 50)))
		knee_therm_prev_val[2] = thermistor_pdo[2];

	if (((knee_therm_prev_val[3]>thermistor_pdo[3]) && (knee_therm_prev_val[3]-thermistor_pdo[3] < 1000)) ||
	    ((knee_therm_prev_val[3]<thermistor_pdo[3]) && (thermistor_pdo[3]-knee_therm_prev_val[3] < 1000)))
		knee_therm_prev_val[3] = thermistor_pdo[3];

	if (((knee_therm_prev_val[4]>thermistor_pdo[4]) && (knee_therm_prev_val[4]-thermistor_pdo[4] < 50)) ||
	    ((knee_therm_prev_val[4]<thermistor_pdo[4]) && (thermistor_pdo[4]-knee_therm_prev_val[4] < 50)))
		knee_therm_prev_val[4] = thermistor_pdo[4];

	if (((knee_therm_prev_val[5]>thermistor_pdo[5]) && (knee_therm_prev_val[5]-thermistor_pdo[5] < 50)) ||
	    ((knee_therm_prev_val[5]<thermistor_pdo[5]) && (thermistor_pdo[5]-knee_therm_prev_val[5] < 50)))
		knee_therm_prev_val[5] = thermistor_pdo[5];

	//knee_send_current_read = true;
}

bool knee_run_halt(uint8_t id) {
	knee_damping_cnt += 1;
	if (knee_damping_cnt > 100)
		return false;
	return true;
}

inline void knee_update_outputs(uint8_t id) {}

inline void knee_estop(void) {}

void knee_wait_loop() {}

bool knee_check_error(uint8_t id) {
	#ifdef ERROR_CHECK_LIMIT_SWITCH
	if (limit_sw_get_port(&limit_sw_port)) {
		limit_switch_counter ++;
	}
	else if (limit_switch_counter > 0)
		limit_switch_counter --;
	
	if (limit_switch_counter > 50) {
		#if defined DEBUG_LOW || DEBUG_HIGH
		printf("[Medulla knee] Limit switch error: %d\n",limit_sw_get_port(&limit_sw_port));
		#endif
		*knee_error_flags_pdo |= medulla_error_limit_switch;
		return true;
	}
	#endif

	#ifdef ERROR_CHECK_THERMISTORS
	// Do filtering on thermistor values 
	if ((knee_therm_prev_val[0] < THERMISTOR_MAX_VAL) ||
        (knee_therm_prev_val[1] < THERMISTOR_MAX_VAL) ||
        (knee_therm_prev_val[2] < THERMISTOR_MAX_VAL) ||
        (knee_therm_prev_val[3] < THERMISTOR_MAX_VAL) ||
        (knee_therm_prev_val[4] < THERMISTOR_MAX_VAL) ||
        (knee_therm_prev_val[5] < THERMISTOR_MAX_VAL)) {
		thermistor_counter++;
	}
	else if (thermistor_counter > 0)
		thermistor_counter--;
	if (thermistor_counter > 100) {
		#if defined DEBUG_LOW || DEBUG_HIGH	
		printf("[Medulla knee] Thermistor error.\n");
		#endif
		*knee_error_flags_pdo |= medulla_error_thermistor;
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
		printf("[Medulla knee] Logic voltage error.\n");
		#endif
		*knee_error_flags_pdo |= medulla_error_logic_voltage;
		return true;
	}
	#endif

	#ifdef ERROR_CHECK_ENCODER
	// Check the encoder error counters
	if ((motor_encoder_error_counter > 10) || (knee_encoder_error_counter > 10)) {
		#if defined DEBUG_LOW || DEBUG_HIGH	
		printf("[Medulla knee] Encoder read error\n");
		#endif
		*knee_error_flags_pdo |= medulla_error_encoder;
		return true;
	}
	#endif

	// If none of the above caused us to return true, then there are no errors and we return false
	return false;
}

bool knee_check_halt(uint8_t id) {
	return false;
}

void knee_reset_error() {
	*knee_error_flags_pdo = 0;
	thermistor_counter = 0;
	logic_voltage_counter = 0;
	motor_encoder_error_counter = 0;
	knee_encoder_error_counter = 0;
	knee_damping_cnt = 0;
	knee_therm_prev_val[0] = thermistor_pdo[0];
	knee_therm_prev_val[1] = thermistor_pdo[1];
	knee_therm_prev_val[2] = thermistor_pdo[2];
	knee_therm_prev_val[3] = thermistor_pdo[3];
	knee_therm_prev_val[4] = thermistor_pdo[4];
	knee_therm_prev_val[5] = thermistor_pdo[5];
}
