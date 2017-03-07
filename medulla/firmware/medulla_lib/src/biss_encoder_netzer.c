#include "biss_encoder_netzer.h"

#define _BISS_ENCODER_SEND_CLOCK \
encoder->spi_port.spi_port->OUTCLR = (1<<7); \
_delay_us(0.125);        /* Half of the 2 Mhz period*/ \
encoder->spi_port.spi_port->OUTSET = (1<<7); \
_delay_us(0.125);        /* Other half of the 2 Mhz period*/

netzer_encoder_t netzer_encoder_init(PORT_t *spi_port, SPI_t *spi_register, void *timestamp_timer, uint16_t cnt_per_us, uint32_t *data_pointer, uint16_t *timestamp_pointer) {

	netzer_encoder_t encoder;

	// setup the encoder struct
	encoder.spi_port = spi_init_port(spi_port, spi_register, spi_div16, false);
//	encoder.spi_port.spi_register->CTRL = SPI_ENABLE_bm | SPI_MASTER_bm | SPI_MODE_0_gc | SPI_PRESCALER_DIV16_gc;
	encoder.timestamp_timer = (TC0_t*)timestamp_timer;
	encoder.cnt_per_us = cnt_per_us;
	encoder.data_pointer = data_pointer;
	encoder.timestamp_pointer = timestamp_pointer;

	// The spi driver defaults to the wrong SPI mode, so we switch it now
	spi_register->CTRL = (spi_register->CTRL & ~SPI_MODE_gm) | SPI_MODE_2_gc;
	spi_port->DIRSET = 1<<7;
	spi_port->OUTSET = 1<<7;
	// Then just return the encoder
	return encoder;
}

int netzer_encoder_start_reading(netzer_encoder_t *encoder) {
	// first check that we are not already reading
	if (netzer_encoder_read_complete(encoder) == false)
		return -1;

	// Check that the encoder is ready to be read from. MISO has be high
	if ((encoder->spi_port.spi_port->IN & (1<<6)) == 0)
		return -2;

	// clear out the data buffer
	encoder->input_buffer[0] = 0;
	encoder->input_buffer[1] = 0;
	encoder->input_buffer[2] = 0;
	encoder->input_buffer[3] = 0;

	// Now we need to manually send out the first part of the BISS packet
	// so we have to first disable the SPI driver
	encoder->spi_port.spi_register->CTRL &= ~SPI_ENABLE_bm;

	// Send the two start bits. Since we need to record the time of the first rising edge of the clock we can't use the send clock macro on the first clock.
	encoder->spi_port.spi_port->OUTCLR = (1<<7);
	_delay_us(0.125);
	cli();
	*(encoder->timestamp_pointer) = encoder->timestamp_timer->CNT; // Record the time of the rising edge
	encoder->spi_port.spi_port->OUTSET = (1<<7);
	sei();
	_delay_us(0.125);
	_BISS_ENCODER_SEND_CLOCK // Now send the second clock
	
	encoder->spi_port.spi_port->OUTCLR = (1<<7);                                                             
	_delay_us(0.0625);
	while ((encoder->spi_port.spi_port->IN & (1<<6)) == 0) {
		encoder->spi_port.spi_port->OUTSET = (1<<7);
		_delay_us(0.0125);
		encoder->spi_port.spi_port->OUTCLR = (1<<7);
		_delay_us(0.0125);
	}
	encoder->spi_port.spi_port->OUTSET = (1<<7);
        _delay_us(0.125);
	
	_BISS_ENCODER_SEND_CLOCK

	encoder->spi_port.spi_register->CTRL |= SPI_ENABLE_bm;
	spi_start_receive(&(encoder->spi_port), encoder->input_buffer,4);

	// At this point the clock generator will run. When an Ack is received by the pin interrupt the ISR will start the SPI transfer
	return 0;
}

uint8_t netzer_encoder_process_data(netzer_encoder_t *encoder) {
	// Fill the data pointer. Since the xMega is little-endian, we have to swap the byte order, by shiffing the data in one byte at a time.
	*(encoder->data_pointer) =(((uint32_t)encoder->input_buffer[0]) << 10) |
                              (((uint32_t)encoder->input_buffer[1]) <<  2) |
                              (((uint32_t)encoder->input_buffer[2]) >>  6);

	*(encoder->timestamp_pointer) += 4*encoder->cnt_per_us; // Add 4 microseconds to the timestamp because the actual measurement is taken 4 microseconds after the first rising edge of the clock.

	// The last byte is the status byte, so we can just return that
	return encoder->input_buffer[2];
}

bool netzer_encoder_read_complete(netzer_encoder_t *encoder) {
	return !(encoder->spi_port.transaction_underway);
}

bool netzer_encoder_data_valid(netzer_encoder_t *encoder) {
	return netzer_encoder_read_complete(encoder) && ((encoder->input_buffer[2] & (1<<5)) != 0);
}


