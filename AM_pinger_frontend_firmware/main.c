/*
 * Pinger Front End firmware rev2
 * to be used with Pinger Driver Board rev1
 *
 * Created: 24.03.2021
 * Author : Lars Weinstock, Christoph Guenther
 */ 

// TODO/Bugs:
// use feedback voltage ADC
// turn charge off at voltage (=set max voltage)
// receiver interlock? (only be able to connect receiver if ... is fullfilled, e.g. V<..)
// trigger interrupt from gpio
// emitter is disconnected after sending one waveform

// CPU clock
#define F_CPU 12000000UL // 12 MHz external crystal, full swing max delay

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// status register
#define STAT_REG	0x10			// status register
#define S_TRG		0		        // software trigger
#define S_IO		1				// connect emitter (1) or receiver (0)
#define S_DIS		2		        // discharge
#define S_ADIS		3		        // discharge and disconnect automatically after sending
#define S_CHRG		4				// charge
#define S_DONE		5				// done charging
volatile uint8_t	status_reg;		// status register content

// waveform registers
#define WVFRM_REG		0x11		// waveform value register
#define WVFRM_MAX_LEN	1500		// 500 bytes frei fuer Variablen [Lars], increase to 2000?

// Default waveform
uint8_t wvfrm[WVFRM_MAX_LEN] = {0x09, 0x05, 0x06, 0x05, 0x09, 0x05, 0x06, 0x05, 0x09, 0x05}; // waveform modeuint8_t wvfrm_cont[4] = {0x09, 0x05, 0x06, 0x05}; // continuous sine mode
#define LENL_REG	0x12			// waveform length register, low, high, 16 bit
#define LENH_REG	0x13
volatile uint16_t	wvfrm_len;		// waveform length
#define TICK_REG	0x14			// number of ticks per state register
volatile uint8_t	tick_reg;		// number of ticks per state
#define POSL_REG	0x22			// waveform position register, low, high, 16 bit
#define POSH_REG	0x23
volatile uint16_t	wvfrm_pos;		// waveform position (ith sample)#define N_SEND_REG 0x24				// how often the waveform should be sentvolatile uint8_t n_send = 1;		// default one time sending#define SEND_DELAY_REG 0x25			// delay after each sendvolatile uint8_t send_delay=1;		// ms delay after sending a waveform#define SEND_MODE_REG 0x28			// control which send mode, continuous sine or arbitraryvolatile uint8_t send_mode=0;		// send mode, 0=continuous, 1=arbitrary#define SEND_DURATION_REG 0x29		// waveform duration in continuous modevolatile uint8_t send_duration=10;	// ms, waveform duration in continuous mode// capacitor charger registers#define CHECK_DONE_REG 0x26			// initiate check if done charging#define EPS_VOLT_REG 0x27			// read output voltage#define CRC_REG		0x20			// calculate CRC checksum from waveform, register
// test/debug register
#define TEST_REG	0x21			// debug
volatile uint8_t echo;
/* Bridge controller definitions */
// MOSFET Driver output port
#define OUT_PORT	PORTD
#define OUT_DDR		DDRD
#define A1			  PORTD1 // aka lo HB A
#define B1			  PORTD0 // aka lo HB B
#define A2			  PORTD3 // aka hi HB A
#define B2			  PORTD2 // aka hi HB B
// Output states
#define OUT_NEG_HV	0x06 // A1=0, B1=1, A2=1, B2=0
#define OUT_NEUTRAL	0x05 // A1=1, B1=0, A2=1, B2=0
#define OUT_POS_HV	0x09 // A1=1, B1=0, A2=0, B2=1
/* Control definitions */
// Control port (en-/disable, (dis)connect, charge, ..)
#define CTRL_PORT	PORTC
#define CTRL_DDR	DDRC
#define DIS			PORTC1 // discharge, 0 = discharge, output
#define CHRG		PORTC2 // turn on charge, output
#define DONE		PORTC3 // charge done, input
#define EPS_FB		PORTC4 // feedback ADC, input
#define SW_IO		PORTC5 // connect piezo to emitter (1) / receiver (0), output

/* trigger definitions */
// trigger/ time signal
#define CAL_TIME PORTC0  // time signal to ICM, output
#define GPIO_TRG PORTB0  // trigger from MMB, input
#define CAL_TRIG PORTB1  // trigger from ICm, input

// (init) function prototypes
void init_SPI();		// initialize SPI (slave mode)
void init_GPIO();		// initialize GPIOs
void init_INT();		// initialize interlock
void init_TCTN0();		// initialize timer
void init_ADC();		// initialize ADC
void wvfrm_out();		// send a waveform
uint8_t crc8(uint8_t *addr, uint16_t len); // calculate crc8 check sum
volatile uint8_t crc8_check = 0xAB, calc_crc8 = 0, send_wvfm = 0;

// helper function prototypes
void charge_on();
void charge_off();
void discharge_on();
void discharge_off();
void connect_driver();
void connect_receiver();
uint8_t is_charge_done();
uint8_t read_voltage();

// communication registers / values
volatile uint8_t spi_buf, addr_ptr = 0x00, send_spi=0, data_rdy=0;
#define READ_REQ   0x0F		// read request
#define WRITE_REQ  0xF0		// write request
#define RW_UNSET   0		// read write undefined
#define SPI_ACK    0xD0		// acknowledge, 208
#define SPI_STOP   0xAA		// end of communication
#define NO_REG     0x00		// no register addressed
volatile uint8_t rw_request = 0;


// main function
int main (void) {
	_delay_ms (1);
	// Setup
	cli ();	init_GPIO ();	init_SPI ();
	
	// initialize full bridge GPIOS, moved to GPIO init
	// OUT_DDR = 0xFF; // set as output
	// OUT_PORT = OUT_NEUTRAL; // set to neutral
	
	// Set default register values
	spi_buf=0, addr_ptr=0x00, rw_request=0x00, send_spi=0;
	status_reg = (1 << S_ADIS); // default auto discharge enabled
	wvfrm_len = 10;	// Default 10 sample sine
	tick_reg = 21;	// f = 8.4 kHz, check this again
	wvfrm_pos = 0;
	send_wvfm = 0;
	
	// Enable all interrupts
	sei ();
	
    /* Main loop */
    while (1) {
			if (send_spi==1) {
				
				// Send waveform if flag is set
				if (send_wvfm==1) {
					wvfrm_out();
					send_wvfm=0;
				}
				
				// Calculate CRC8, if flag is set
				if (calc_crc8==1) {
					crc8_check = crc8(wvfrm, wvfrm_len);
					calc_crc8=0;
				}
				
				// send SPI data, move to top?
				SPDR=spi_buf;
				send_spi=0;
			}
    } // end while
} // end main

// initialize GPIOs
void init_GPIO () {
	// full bridge port
	OUT_DDR  = 0xFF;				// define as output
	OUT_PORT = OUT_NEUTRAL;			// set to neutral state
	 
	// control port
	CTRL_DDR = 0x27;				// 0010 0111
	CTRL_PORT &= ~(1 << CAL_TIME);	// time signal low
	CTRL_PORT &= ~(1 << DIS);		// default discharging
	CTRL_PORT &= ~(1 << CHRG);		// charge off
	CTRL_PORT |= (1 << SW_IO);		// connect to emitter
	
	// TRG port
	DDRB &= ~(1 << PINB0); // GPIO_TRG as input
	DDRB &= ~(1 << PINB1); // CAL_TRIG as input
}

// initialize spi as slave
void init_SPI (void) {
	// Pin I/O
	DDRB |= (1 << PINB4);	// MISO, output
	DDRB &= ~(1 << PINB3);	// MOSI, input
	DDRB &= ~(1 << PINB2);	// CS, input
	DDRB &= ~(1 << PINB5);	// SCK, input
	
	SPCR = 0b11000000;		// Enable SPI && interrupt enable bit
	SPDR = 0xAA;			// clear spi data register
}

// initialize ADC for EPS feedback
void init_ADC () {
	// Select Vref = AVcc, left-adjusted
	ADMUX |= (1<<REFS0) | (1<<ADLAR);
	//set prescaler to 128 and enable ADC, clock = 12 MHz / 128 = 93.75 kHz
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0) | (1<<ADEN);
}

// reading a voltage value
uint8_t read_voltage() {
	//select ADC channel ADC4 with safety mask
	ADMUX = (ADMUX & 0xF0) | (0x04 & 0x0F);
	//single conversion mode
	ADCSRA |= (1<<ADSC);
	// wait until ADC conversion is complete
	while( ADCSRA & (1<<ADSC) );
	return ADCH; // return high byte (8 bit resolution)
}

// checksum calculation
uint8_t crc8(uint8_t* addr, uint16_t len) {	// !!! Calculating CRC8 takes some time to complete !!!
	uint8_t crc= 0x00;
	for (uint16_t i=0; i<len;i++){
		uint8_t inbyte = addr[i];
		for (uint8_t j=0; j<8; j++){
			uint8_t mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) crc ^= 0x8C;
				inbyte >>= 1;
		}
	}
	return crc;
}

// waveform sending
void wvfrm_out() {
	uint8_t dt; // ticks per state
	uint8_t dt_delay = send_delay; // delay after one waveform
	uint16_t len = wvfrm_len;
	
	// uint8_t is_connected = 0;
	// if (status_reg & (1<<S_ENA)) is_connected = 1; ??
	// else is_connected = 0;
	
	// continuous mode
	if (send_mode == 0x00) {
		// Send waveform n times
		uint16_t n_ticks_continuous = ( (send_duration&0x00FF) * 1000) / (1.25 * (tick_reg&0x00FF) + 0.72); // compute how many ticks are in wf duration, use calibration coefficients, double check this!
		uint8_t j = 0; // jth waveform state
		
		// loop over n waveforms
		for (uint8_t n = 0; n < n_send; n++) {
			dt_delay = send_delay; // set delay
			PORTC |= (1 << CAL_TIME); // set cal time pin high during send
			// loop over waveform states
			for (uint16_t i=0; i<n_ticks_continuous; i++) {
				dt = tick_reg; // Set number of ticks
				if (j>3) j=0;
				OUT_PORT = wvfrm_cont[j]; // apply output state to port
				j++;
				while (dt--) // hold state for dt microsec
				_delay_us(1); // alternatively asm nop? [Lars]
			}
			PORTC &= ~(1 << CAL_TIME); // set cal time pin low after sending a waveform
			OUT_PORT = OUT_NEUTRAL; // set port to neutral again
			while(dt_delay--) // delay after waveform
			_delay_ms(1);
		}
	}
	
	// arbitrary waveform mode
	else if(send_mode == 0x01) {
		// loop over n waveforms
		for (uint8_t n = 0; n < n_send; n++) {
			dt_delay = send_delay; // set delay
			PORTC |= (1 << CAL_TIME); // set cal time pin high during send
			for (uint16_t i = 0; i < len; i++) {
				dt = tick_reg;			// set number of ticks
				OUT_PORT = wvfrm[i];	// apply output state to port
				while (dt--)			// hold state for dt microsec
				_delay_us(1);		// alternatively asm nop? [Lars]
			}
			PORTC &= ~(1 << CAL_TIME); // set cal time low after sending a waveform
			OUT_PORT = OUT_NEUTRAL; // set port to neutral again
			while(dt_delay--) // delay after waveform
			_delay_ms(1);
		}
	}
	
	// set output neutral again
	// if (status_reg & (1<<S_ENA)) OUT_PORT = (OUT_NEUTRAL | (1 << SW_ENA)); ??
	// else OUT_PORT = OUT_NEUTRAL;
	OUT_PORT = OUT_NEUTRAL;
	
	// auto disconnect drivers if requested
	if ( status_reg & (1 << S_ADIS) ) {
		charge_off ();
		discharge_on ();
		connect_driver ();
		//CTRL_PORT &= ~(1 << SW_ENA); // disconnect drivers
		// status_reg |= (1 << S_DIS);
		status_reg &= ~(1 << S_CHRG);
		status_reg |= (1 << S_DIS);
		status_reg |= (1 << S_IO);
		status_reg &= ~(1 << S_DONE);
	}

	// clear trigger bit
	status_reg &= ~(1<<S_TRG);
	return;
}

// help functions for readability
void charge_on () {
	CTRL_PORT |= (1 << CHRG);
	return;
}

void charge_off () {
	CTRL_PORT &= ~(1 << CHRG);
	return;
}

void discharge_on () {
	CTRL_PORT &= ~(1 << DIS);
	return;
}
void discharge_off () {
	CTRL_PORT |= (1 << DIS);
	return;
}

void connect_driver () {
	CTRL_PORT |= (1 << SW_IO);
	return;
}

void connect_receiver () {
	CTRL_PORT &= ~(1 << SW_IO);
	return;
}

uint8_t is_charge_done () {
	if ( (PINC & (1 << CHRG)) == (1 << CHRG) ) {
		return 0x01;
	} else {
		return 0x00;
	}
}
// uint8_t read_voltage(); TODO

// spi communication to MMB
ISR (SPI_STC_vect) {
	
	// communication takes 3 steps:
	//  - detect if read or write mode is requested
	//  - which register should be read/written to?
	//  - requested data are sent or received
	
	// wait until transfer complete
	while ( ( SPSR & (1 << SPIF) ) );
	spi_buf = SPDR;
	
	// check if read or write was requested
	switch (rw_request) {
		
		// if RW mode is unset read request from buffer
		case RW_UNSET:
		switch(spi_buf) {
			case READ_REQ: // read request
			rw_request = spi_buf;
			spi_buf = 250;
			addr_ptr = NO_REG;
			send_spi = 1;
			break;
			
			case WRITE_REQ: // write request
			rw_request = spi_buf;
			spi_buf = 251;
			addr_ptr = NO_REG;
			send_spi = 1;
			break;
			
			default: // invalid request
			rw_request = RW_UNSET;
			spi_buf = 255;
			send_spi = 1;
			break;
		}
		break;
		
		// read request
		case READ_REQ:
		// which address?
		switch (addr_ptr) {
			case NO_REG: // if no address selected yet read it from spi
			addr_ptr = spi_buf;
			spi_buf = SPI_ACK;
			send_spi = 1;
			break;
			
			case STAT_REG: // return status
			spi_buf = status_reg;
			rw_request = RW_UNSET;
			addr_ptr = NO_REG;
			send_spi = 1;
			break;
			
			case WVFRM_REG: // return ith waveform element
			spi_buf = wvfrm[0];
			rw_request = RW_UNSET;
			addr_ptr = NO_REG;
			send_spi = 1;
			break;
			
			case LENL_REG: // return lower length byte
			spi_buf = (uint8_t)(wvfrm_len & 0x00FF);
			rw_request = RW_UNSET;
			addr_ptr = NO_REG;
			send_spi = 1;
			break;
			
			case LENH_REG: // return high length byte
			spi_buf = (uint8_t)((wvfrm_len & 0xFF00) >> 8);
			rw_request = RW_UNSET;
			addr_ptr = NO_REG;
			send_spi = 1;
			break;
			
			case TICK_REG: // return tick length (us)
			spi_buf = tick_reg;
			rw_request = RW_UNSET;
			addr_ptr = NO_REG;
			send_spi = 1;
			break;
			
			case CRC_REG: // return crc8 checksum, need to initiate calculation first!
			spi_buf = crc8_check;
			rw_request = RW_UNSET;
			addr_ptr = NO_REG;
			send_spi = 1;
			break;
			
			case POSL_REG: // return lower position byte
			spi_buf = (uint8_t)(wvfrm_pos & 0x00FF);
			rw_request = RW_UNSET;
			addr_ptr = NO_REG;
			send_spi = 1;
			break;
			
			case POSH_REG: // return high position byte
			spi_buf = (uint8_t)((wvfrm_pos & 0xFF00) >> 8);
			rw_request = RW_UNSET;
			addr_ptr = NO_REG;
			send_spi = 1;
			break;
			
			case N_SEND_REG: // return number of sends
			spi_buf = n_send;
			rw_request = RW_UNSET;
			addr_ptr = NO_REG;
			send_spi = 1;
			break;
			
			case SEND_DELAY_REG: // return delay after sending
			spi_buf = send_delay;
			rw_request = RW_UNSET;
			addr_ptr = NO_REG;
			send_spi = 1;
			break;
			
			case SEND_MODE_REG: // return send mode
			spi_buf = send_mode;
			rw_request = RW_UNSET;
			addr_ptr = NO_REG;
			send_spi = 1;
			break;
			
			case SEND_DURATION_REG: // return send duration of continuous mode
			spi_buf = send_duration;
			rw_request = RW_UNSET;
			addr_ptr = NO_REG;
			send_spi = 1;
			break;
			
			default: // default = no valid address selected
			spi_buf = 254;
			rw_request = RW_UNSET;
			addr_ptr = NO_REG;
			send_spi = 1;
			break;
		}
		break;
		
		// write request
		case WRITE_REQ:
		// which address?
		switch (addr_ptr) {
			case NO_REG:
			addr_ptr = spi_buf;
			spi_buf = SPI_ACK;
			send_spi = 1;
			break;
			
			case STAT_REG: // set status
			status_reg = 0x1F & spi_buf; // clear read only bits
			// status_reg = 0x0D & spi_buf;
			// Connect drivers
			if ( status_reg & (1 << S_IO) )
			connect_driver();
			else
			connect_receiver();
			
			// Send waveform as soon as possible
			//   -> Dont send during interrupt!!
			if ( status_reg & (1 << S_TRG) )
			send_wvfm = 1;
			
			if ( status_reg & (1 << S_DIS))
			discharge_on();
			else
			discharge_off();
			
			if ( status_reg & (1 << S_CHRG))
			charge_on();
			else
			charge_off();
			
			spi_buf = SPI_ACK;
			rw_request = RW_UNSET;
			addr_ptr = NO_REG;
			send_spi = 1;
			break;
			
			case WVFRM_REG: // write waveform data
			// Save state to waveform array (check validity first)
			switch (spi_buf) {
				case OUT_NEG_HV:
				case OUT_NEUTRAL:
				case OUT_POS_HV:
				wvfrm[wvfrm_pos] = spi_buf;
				break;
				
				case SPI_STOP:
				wvfrm_pos = 0;
				spi_buf = SPI_ACK;
				rw_request = RW_UNSET;
				addr_ptr = NO_REG;
				break;
				
				default:
				wvfrm[wvfrm_pos] = OUT_NEUTRAL;
			}
			
			wvfrm_pos++; // increase wf pos so next value is written to the next position
			if (wvfrm_pos == WVFRM_MAX_LEN) {
				wvfrm_pos = 0;
				spi_buf = SPI_ACK;
				rw_request = RW_UNSET;
				addr_ptr = NO_REG;
			}
			else {
				spi_buf = wvfrm[wvfrm_pos];
			}
			send_spi = 1;
			break;
			
			case LENL_REG: // write low length byte
			// Reset waveform length -> first write lower, then upper byte
			wvfrm_len = 0x0000;
			wvfrm_len = (wvfrm_len & 0xFF00) | (spi_buf << 0);	// Lower byte
			spi_buf = SPI_ACK;
			rw_request = RW_UNSET;
			addr_ptr = NO_REG;
			send_spi = 1;
			break;
			
			case LENH_REG: // write high length byte
			wvfrm_len = (wvfrm_len & 0x00FF) | (spi_buf << 8);	// Upper byte
			// Check for maximum length
			if (wvfrm_len > WVFRM_MAX_LEN)
			wvfrm_len = WVFRM_MAX_LEN;
			spi_buf = SPI_ACK;
			rw_request = RW_UNSET;
			addr_ptr = NO_REG;
			send_spi = 1;
			break;
			
			case TICK_REG: // write tick length
			tick_reg = spi_buf;
			spi_buf = SPI_ACK;
			rw_request = RW_UNSET;
			addr_ptr = NO_REG;
			send_spi = 1;
			break;
			
			case CRC_REG:	// Start crc8 calculation as soon as possible
			calc_crc8 = 1;	//   -> Dont calculate it during interrupt!!
			spi_buf = SPI_ACK;
			rw_request = RW_UNSET;
			addr_ptr = NO_REG;
			send_spi = 1;
			break;
			
			case POSL_REG: // On write access reset the waveform position to zero
			case POSH_REG:
			wvfrm_pos = 0;
			spi_buf = SPI_ACK;
			rw_request = RW_UNSET;
			addr_ptr = NO_REG;
			send_spi = 1;
			break;
			
			case CHECK_DONE_REG: // check if done charging and update status register accordingly
			if (is_charge_done() == 0x01)
			status_reg |= (1 << S_DONE);
			else
			status_reg &= ~(1 << S_DONE);
			spi_buf = SPI_ACK;
			rw_request = RW_UNSET;
			addr_ptr = NO_REG;
			send_spi = 1;
			break;
			
			case N_SEND_REG: // set number of sends
			n_send = spi_buf;
			spi_buf = SPI_ACK;
			rw_request = RW_UNSET;
			addr_ptr = NO_REG;
			send_spi = 1;
			break;
			
			case SEND_DELAY_REG: // set delay after each sending
			send_delay = spi_buf;
			spi_buf = SPI_ACK;
			rw_request = RW_UNSET;
			addr_ptr = NO_REG;
			send_spi = 1;
			break;
			
			case SEND_MODE_REG: // set send mode, 0=cont, 1=arb
			send_mode = spi_buf;
			spi_buf = SPI_ACK;
			rw_request = RW_UNSET;
			addr_ptr = NO_REG;
			send_spi = 1;
			break;
			
			case SEND_DURATION_REG: // set continuous mode send duration
			send_duration = spi_buf;
			spi_buf = SPI_ACK;
			rw_request = RW_UNSET;
			addr_ptr = NO_REG;
			send_spi = 1;
			break;
			
			default: // invalid address
			spi_buf = 253;
			rw_request = RW_UNSET;
			addr_ptr = NO_REG;
			send_spi = 1;
			break;
		}
		break;
	}

}