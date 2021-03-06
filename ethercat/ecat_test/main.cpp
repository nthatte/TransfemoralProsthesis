#include "main.h"

void sig_handler(int signum) {
	done = true;
}

int main(int argc, char ** argv) {
	signal(SIGINT, sig_handler);
	ec_master_t* ec_master = ecrt_request_master(0);
	if (ec_master == NULL) {
		perror("ecrt_request_master failed.");
		return 1;
	}
	ec_domain_t* domain = ecrt_master_create_domain (ec_master);
	if (domain == NULL) {
		perror("ecrt_master_create_domain failed.");
		ecrt_release_master(ec_master);
		return 2;
	}
	
	ec_slave_config_t* sc0 = ecrt_master_slave_config(ec_master, 0, 0, VENDOR_ID, PRODUCT_CODE);
	ec_slave_config_t* sc1 = ecrt_master_slave_config(ec_master, 0, 1, VENDOR_ID, PRODUCT_CODE);
	
	if (ecrt_slave_config_pdos(sc0, 4, slave_0_syncs) || ecrt_slave_config_pdos(sc1, 4, slave_1_syncs)) {
		perror("ecrt_slave_config_pdos() failed.");
		ecrt_release_master(ec_master);
		return 3;
	}
	
	unsigned int command_off0;
	unsigned int current_off0;
	unsigned int timestep_off0;
	unsigned int encoder_off0;
	unsigned int command_off1;
	unsigned int current_off1;
	unsigned int timestep_off1;
	unsigned int encoder_off1;
	ec_pdo_entry_reg_t entry_regs[] = { 
		{0, 0, VENDOR_ID, PRODUCT_CODE, 0x5, 0x1,  &command_off0, NULL},
		//{0, 0, VENDOR_ID, PRODUCT_CODE, 0x3, 0x2,  &current_off0, NULL},
		//{0, 0, VENDOR_ID, PRODUCT_CODE, 0x6, 0x1, &timestep_off0, NULL},
		//{0, 0, VENDOR_ID, PRODUCT_CODE, 0x7, 0x2,  &encoder_off0, NULL},
		{0, 1, VENDOR_ID, PRODUCT_CODE, 0x5, 0x1,  &command_off1, NULL},
		{0, 1, VENDOR_ID, PRODUCT_CODE, 0x3, 0x2,  &current_off1, NULL},
		//{0, 1, VENDOR_ID, PRODUCT_CODE, 0x6, 0x1, &timestep_off1, NULL},
		//{0, 1, VENDOR_ID, PRODUCT_CODE, 0x7, 0x2,  &encoder_off1, NULL},
		{0, 0,      0x00,         0x00, 0x0, 0x0,           NULL, NULL}};
	if (ecrt_domain_reg_pdo_entry_list(domain, entry_regs)) {
		perror("ecrt_domain_reg_pdo_entry_list() failed");
		ecrt_release_master(ec_master);
		return 4;
	}
	if (ecrt_master_activate(ec_master)) {
	perror("ecrt_master_activate() failed");
	ecrt_release_master(ec_master);
	return 5;
}	

	uint8_t* domain_pd = ecrt_domain_data(domain);
	
	unsigned char* command0  = (unsigned char*) (domain_pd + command_off0);
	uint16_t*      current0  = (uint16_t*)      (domain_pd + current_off0);
	uint16_t*      timestep0 = (uint16_t*)      (domain_pd + timestep_off0);
	uint32_t*      encoder0  = (uint32_t*)      (domain_pd + encoder_off0);
	unsigned char* command1  = (unsigned char*) (domain_pd + command_off1);
	uint16_t*      current1  = (uint16_t*)      (domain_pd + current_off1);
	uint16_t*      timestep1 = (uint16_t*)      (domain_pd + timestep_off1);
	uint32_t*      encoder1  = (uint32_t*)      (domain_pd + encoder_off1);
	
	(*command0) = 0;
	char input;
		timespec cur_time;
	clock_gettime(CLOCK_REALTIME, &cur_time);
	ecrt_master_application_time(ec_master, EC_NEWTIMEVAL2NANO(cur_time));
	ecrt_slave_config_dc(sc0, 0x0300, LOOP_PERIOD_NS, LOOP_PERIOD_NS - (cur_time.tv_nsec % LOOP_PERIOD_NS), 0, 0);
	ecrt_slave_config_dc(sc1, 0x0300, LOOP_PERIOD_NS, LOOP_PERIOD_NS - (cur_time.tv_nsec % LOOP_PERIOD_NS), 0, 0);
	
	long tgt_time = cur_time.tv_nsec;
	timespec wait_time = {
		0,
		0
	};
	ec_master_state_t master_state;
	


	while (!done) {
		*command0 = 2;
//		if (*counter < 20000)
		//*counter0 += 1;
		printf("command: %d\n",*command0);
		printf("command0: %3u, timestep0: %3u, encoder0: %8u\n", (unsigned int) *command0, *timestep0, (unsigned int) *encoder0);
		
		clock_gettime(CLOCK_REALTIME, &cur_time);
		ecrt_master_application_time(ec_master, EC_NEWTIMEVAL2NANO(cur_time));
		ecrt_master_sync_reference_clock(ec_master);
		ecrt_master_sync_slave_clocks(ec_master);
		ecrt_domain_queue(domain);
		ecrt_master_send(ec_master);
		usleep(300);
		ecrt_master_receive(ec_master);
		ecrt_domain_process(domain);
		wait_time.tv_nsec = LOOP_PERIOD_NS - (cur_time.tv_nsec % LOOP_PERIOD_NS);
		nanosleep(&wait_time, NULL);
	}
	
	ecrt_release_master(ec_master);
	return 0;
}
