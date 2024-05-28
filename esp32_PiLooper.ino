//-----------------------------------------------------
// esp32_PiLooper.ino
//-----------------------------------------------------
// Based on teensyPiLooper, the ESP32 will allow me
// to possibly implement a Wifi WebUI and/or telnet
// interface to the TE3 foot pedal.

#include <myDebug.h>
#include "driver/uart.h"

#define WITH_SERIAL1    	1
#define WITH_SERIAL2    	0

#define WITH_MIDI			1
#define DEBUG_MIDI			1


#define ESP32_PI_LOOPER_VERSION  "v1.0"

#define SERIAL0_BAUD_RATE		961200
#define SERIAL1_BAUD_RATE		460800	// 115200
#define SERIAL2_BAUD_RATE		115200

//---------------
// pins
//---------------
// Grumble. The effing default pins for UART1 (GPIO9 and GPIO10)
// apparently don't actually work "because they are internally
// connected to flash".  I then tried GPIO12 and GPIO13 and
// still had problems.  I finally decided to switch to 26 and 27.

#define SERIAL1_RX   26
#define SERIAL1_TX   27
#define SERIAL2_RX   16     // These ARE the default Serial2 pins
#define SERIAL2_TX   17     // They are untested at this time.

#define SENSE_RPI_RUN       35      // sense rpi RUN (REBBOOT) pin, HIGH == rpi has voltage
#define SENSE_RPI_READY     33      // sense rpi GPIO25, HIGH == my program has initialized
#define PIN_PI_REBOOT       32      // HIGH == REBOOT (to base of transistor)

// I tried to use 6,7,8 for these three LED pins and the ESP32
// kept crashing. I need to review what pins are ACTUALLY
// available on the ESP32

#define LED_ALIVE           2       // 2 is the pin for DEV0 onboard led
#define LED_RPI_RUN         21      // show state of RPI_RUN sense
#define LED_RPI_READY       22      // show state of RPI_READY sense
#define ALIVE_ON_TIME       50
#define ALIVE_OFF_TIME      950

// optional serial activity leds

#define LED_SERIAL0			0
#define LED_SERIAL1			0
#define LED_SERIAL2			0
#define LED_SERIAL_TIME		100		// ms


//---------------------
// other contants
//---------------------

#define UART_BUF_SIZE 		1024
  
#define FILE_SERVER_TIMEOUT 16000
    // How long to wait for the text file protocol

#define MAX_LINE_BUFFER     512
    // largest single line of text



//-------------------
// vars
//-------------------


volatile bool file_server_mode;
volatile uint32_t file_server_time;



//----------------
// rebootPi()
//----------------

void rebootPi()
{
    display(0,"esp32_PiLooper::rebootPi() called",0);
    digitalWrite(LED_RPI_RUN,0);
    digitalWrite(LED_RPI_READY,0);
    digitalWrite(PIN_PI_REBOOT,1);
    delay(900);
    digitalWrite(PIN_PI_REBOOT,0);
}



//-------------------------------------------------
// uart handlers
//-------------------------------------------------


static void uartTask0(void *param)
	// Must handle ctrl-A and ctrl-B from
{
	#if LED_SERIAL0
		uint32_t led_on = 0;
	#endif

	static uint8_t buf[UART_BUF_SIZE];

    while (1)
	{
		int len = uart_read_bytes(UART_NUM_0, buf, UART_BUF_SIZE, 1);
		if (len > 0)
	    {
			#if LED_SERIAL0
				digitalWrite(LED_SERIAL0,1);
				led_on = millis();;
			#endif

			if (len == 1 && (buf[0] == 0x1 || buf[0] == 0x2))
			{
				if (buf[0] == 0x2)
					rebootPi();
				// else if (buf[0] == 0x1)
				// {
				// 	file_server_mode = !file_server_mode;
				// 	file_server_time = file_server_mode ? millis() : 0;
				// }
			}
			else
			{
				#if WITH_SERIAL2
					if (file_server_mode)
					{
						file_server_time = millis();;
						uart_write_bytes(UART_NUM_2, (const char*) buf, len);
					}
					#if WITH_SERIAL1
						else
					#endif
				#endif

				#if WITH_SERIAL1
					uart_write_bytes(UART_NUM_1, (const char*) buf, len);
				#endif
			}
		}

		#if LED_SERIAL0
			else if (led_on && (millis(); - led_on > LED_SERIAL_TIME))
			{
				led_on = 0;
				digitalWrite(LED_RPI_READY,0);
			}
		#endif
	}
}


#if WITH_SERIAL1
	static void uartTask1(void *param)
	{
		#if LED_SERIAL1
			uint32_t led_on = 0;
		#endif

		static int in_midi = 0;
		static int out_len = 0;
		static uint8_t midi_buf[4];
		static uint8_t in_buf[UART_BUF_SIZE];
		static uint8_t out_buf[UART_BUF_SIZE];

		while (1)
		{
			int len = uart_read_bytes(UART_NUM_1, in_buf, UART_BUF_SIZE, 1);
			if (len > 0)
			{
				#if LED_SERIAL1
					digitalWrite(LED_SERIAL1,1);
					led_on = millis();
				#endif

				int in_len = 0;
				out_len = 0;

				while (in_len < len)
				{
					uint8_t c = in_buf[in_len++];
					if (in_midi == 4)
					{
						#if DEBUG_MIDI
							display(0,"MIDI_END(1->2) 0x%08x",*(uint32_t *)midi_buf);
						#endif
						#if WITH_SERIAL2
							uart_write_bytes(UART_NUM_2, (const char*) midi_buf, 4);
						#endif
						in_midi = 0;
					}

					if (in_midi)
					{
						#if DEBUG_MIDI > 1
							display(0,"MIDI_ADD(1->2) 0x%02x",c);
						#endif
						midi_buf[in_midi++] = c;
					}
					else if (c == 0x0B)
					{
						#if DEBUG_MIDI > 1
							display(0,"MIDI_START(1->2) 0x%02x",c);
						#endif
						in_midi = 1;
						midi_buf[0] = c;
					}
					else	// cannot overflow
					{
						out_buf[out_len++] = c;
					}
				}

				if (out_len > 0)
					uart_write_bytes(UART_NUM_0, (const char*) out_buf, out_len);
			}
			#if LED_SERIAL1
				else if (led_on && (millis() - led_on > LED_SERIAL_TIME))
				{
					led_on = 0;
					digitalWrite(LED_SERIAL1,0);
				}
			#endif
		}
	}
#endif



#if WITH_SERIAL2
	// uartTask2, when not in file_mode, double buffers lines of text
	// until 0x0A so that we can precede the output with "TE:"

	static void uartTask2(void *param)
	{
		#if LED_SERIAL2
			uint32_t led_on = 0;
		#endif

		static int in_midi = 0;
		static int out_len = 0;
		static int line_len = 0;
		static uint8_t midi_buf[4];
		static uint8_t in_buf[UART_BUF_SIZE];
		static uint8_t out_buf[UART_BUF_SIZE];
		static uint8_t line_buf[UART_BUF_SIZE];

		while (1)
		{
			int len = uart_read_bytes(UART_NUM_2, in_buf, UART_BUF_SIZE, 1);
			if (len > 0)
			{
				#if LED_SERIAL2
					digitalWrite(LED_SERIAL2,1);
					led_on = millis();
				#endif

				int in_len = 0;
				out_len = 0;

				while (in_len < len)
				{
					uint8_t c = in_buf[in_len++];
					if (in_midi == 4)
					{
						#if DEBUG_MIDI
							display(0,"MIDI_END(2->1) 0x%08x",(uint32_t *)midi_buf);
						#endif

						#if WITH_SERIAL1
							uart_write_bytes(UART_NUM_1, (const char*) midi_buf, 4);
						#endif

						in_midi = 0;
					}

					if (in_midi)
					{
						#if DEBUG_MIDI > 1
							display(0,"MIDI_ADD(2->1) 0x%02x",c);
						#endif
						midi_buf[in_midi++] = c;
					}
					else if (c == 0x0B)
					{
						#if DEBUG_MIDI > 1
							display(0,"MIDI_START(2->1) 0x%02x",c);
						#endif
						in_midi = 1;
						midi_buf[0] = c;
					}
					else
					{
						out_buf[out_len++] = c;
					}
				}


				if (out_len > 0)
				{
					if (file_server_mode)
					{
						uart_write_bytes(UART_NUM_0, (const char*) out_buf, out_len);
						file_server_time = millis();
					}
					else	// double buffer into text lines preceded by "TE:"
					{
						// prh - this will need some work when TE is really
						// hooked up.  Currently tested with circle/05-TSTest
						// and reversed pin junk, that kernel is outputtig_
						// a color code (white) and full \r\n lines, which
						// get interpreted a little weirdly by this code.

						display(0,"double buffering %d bytes",out_len);
						
						in_len = 0;
						while (in_len < out_len)
						{
							int c = out_buf[in_len++];

							// I was originally thinking to tack the TE into the buffer
							// and then explicitly send it to uart0.   Current (test) code
							// uses display() instead ..

							//	if (!line_len)
							//	{
							//		display(0,"new TE line",0);
							//		strcpy((char *)line_buf,"TE:");
							//		line_len = 3;
							//	}

							if (c == 0xD || c == 0xA || line_len >= UART_BUF_SIZE-3)
							{
								line_buf[line_len] = 0;
								display(0,"end TE 0x%02x %s",c,line_buf);

								// line_buf[line_len++] = c;
								// uart_write_bytes(UART_NUM_0, (const char*) line_buf, line_len);

								line_len = 0;
							}
							else
							{
								line_buf[line_len++] = c;
							}
						}
					}
				}
			}
			#if LED_SERIAL2
				else if (led_on && (millis() - led_on > LED_SERIAL_TIME))
				{
					led_on = 0;
					digitalWrite(LED_SERIAL2,0);
				}
			#endif
		}
	}


#endif


//-----------------------------------------
// setup()
//-----------------------------------------


static void initUart(uart_port_t uart_num, int baud, int pin_tx, int pin_rx)
    // Install UART drivers without TX buffer or event queue
	// and turn off any software flow control
	// IMPORTANT: flush the RX buffers before proceeding
{
    uart_config_t uart_config = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,    //UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = 122,
    };

    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, pin_tx, pin_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	uart_driver_install(uart_num, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
	uart_set_sw_flow_ctrl(uart_num,0,0,0);
	uart_flush(uart_num);
}


void setup()
{
	initUart(UART_NUM_0, SERIAL0_BAUD_RATE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	#if WITH_SERIAL1
		initUart(UART_NUM_1, SERIAL1_BAUD_RATE, SERIAL1_TX, SERIAL1_RX);
	#endif
	#if WITH_SERIAL2
		initUart(UART_NUM_2, SERIAL2_BAUD_RATE, SERIAL2_TX, SERIAL2_RX);
	#endif

	delay(500);
	Serial.begin(SERIAL0_BAUD_RATE);
		// in keeping with ESP32 fast upload time and issues
		// with being unable to reset to the serial port to
		// 115200 after an upload

	delay(2000);
	setColorString(COLOR_CONST_DEFAULT, "\033[94m");       // bright blue
	display(0,"esp32_PiLooper " ESP32_PI_LOOPER_VERSION " setup() started",0);

    pinMode(SENSE_RPI_RUN,INPUT_PULLUP);
    pinMode(SENSE_RPI_READY,INPUT_PULLDOWN);

    pinMode(LED_ALIVE,OUTPUT);
    pinMode(LED_RPI_RUN,OUTPUT);
    pinMode(LED_RPI_READY,OUTPUT);
    pinMode(PIN_PI_REBOOT,OUTPUT);

    digitalWrite(PIN_PI_REBOOT,0);
    digitalWrite(LED_RPI_RUN,0);
    digitalWrite(LED_RPI_READY,0);

	uart_flush(UART_NUM_0);
	#if WITH_SERIAL1
		uart_flush(UART_NUM_1);
	#endif
	#if WITH_SERIAL2
		uart_flush(UART_NUM_2);
	#endif

	// ESP32_CORE_ARDUINO==1
	// ESP32_CORE_OTHER==0

	xTaskCreatePinnedToCore(
		uartTask0,
		"uartTask0",
		4096,	// stack,
		NULL,	// param
		1,  	// priority
		NULL,   // handle
		0);		// core

	#if WITH_SERIAL1
		xTaskCreatePinnedToCore(
			uartTask1,
			"uartTask1",
			4096,	// stack,
			NULL,	// param
			1,  	// priority
			NULL,   // handle
			0);		// core
	#endif
	
	#if WITH_SERIAL2
		xTaskCreatePinnedToCore(
			uartTask2,
			"uartTask2",
			4096,	// stack,
			NULL,	// param
			1,  	// priority
			NULL,   // handle
			0);		// core
	#endif

	#if 0
		rebootPi();
			// if both are started at the same time,
			// the teensy supresses the pi bootstrap
	#endif

    display(0,"esp32_PiLooper setup() finished",0);
}




void loop()
{
    uint32_t now = millis();

	// RPI RUNNING LED
	
	static bool rpi_running = 0;
	if (digitalRead(SENSE_RPI_RUN) != rpi_running)
	{
		rpi_running = !rpi_running;
		digitalWrite(LED_RPI_RUN,rpi_running);
		display(0,"rpi %s",(rpi_running ? "RUNNING" : "NOT RUNNING"));
	}

	// RPI READY LED

	static bool rpi_ready = 0;
	if (digitalRead(SENSE_RPI_READY) != rpi_ready)
	{
		rpi_ready = !rpi_ready;
		digitalWrite(LED_RPI_READY,rpi_ready);
		display(0,"rpi %s",(rpi_ready ? "READY" : "NOT READY"));
	}

	// FILE_SERVER_MODE timeout and state change
	
	static bool last_file_server_mode = 0;
	if (file_server_mode && (now - file_server_time > FILE_SERVER_TIMEOUT))
	{
		file_server_mode = 0;
		file_server_time = 0;
	}
	if (last_file_server_mode != file_server_mode)
	{
		last_file_server_mode = file_server_mode;
		display(0,"FILE SERVER MODE %s",(file_server_mode ? "ON" : "OFF"));
	}

	// ALIVE LED blink

	static bool blink_state = 0;
	static uint32_t blink_time = 0;
    if ((now - blink_time) > (blink_state ? ALIVE_ON_TIME : ALIVE_OFF_TIME))
    {
        blink_time = now;
        blink_state = !blink_state;
        digitalWrite(LED_ALIVE,blink_state);
    }

}	// loop


// end of esp32_PiLooper.ino
