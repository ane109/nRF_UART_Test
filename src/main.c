/*
 * Serial UART sample program.
 *
 * The receiver is set up a bit differently than the UART used
 * elsewhere in Nordic Semiconductor sample applications. Instead
 * of using continuous reception for the receiver, this uses a
 * fixed 100 char buffer with some extra control flow logic to 
 * handle errors and overflows.
 * 
 * The attached terminal needs to be set up with either a
 * CR, LR, or CRLF line termination. Once detected, this will
 * disable the UART and send the received data to a worker thread,
 * which will re-enable the UART once finished.
 * 
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include <zephyr/logging/log.h>

#include <zephyr/drivers/uart.h>



/* ========== System Macros ========== */

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/* Uncomment to enable logging */
// LOG_MODULE_REGISTER(nrf_uart_test, LOG_LEVEL_INF);



/* ========== Workqueue Macros ========== */

/* Workqueue stack size */
#define WORKQUEUE_STACK_SIZE 512
/* Workqueue thread priority */
#define WORKQUEUE_PRIORITY 5
/* Worker submission return values */
#define KWORK_ALREADY_SUBMITTED 0
#define KWORK_QUEUED 1
#define KWORK_RUNNING_REQUEUED 2


/* ========== UART Macros ========== */

/* Length of the receive buffer */
#define UART_BUFF_SIZE 100
/* UART receive timeout period in microseconds */
#define UART_RECEIVE_TIMEOUT 250



/* ========== Workqueue Objects ========== */

/* Queue structure */
static struct k_work_q offloaded_work_queue = {0};
/* Stack area used by workqueue thread */
static K_THREAD_STACK_DEFINE(work_queue_stack_area, WORKQUEUE_STACK_SIZE);
/* Work item */
struct k_work uart_rx_work;



/* ========== UART Objects ========== */

/* Device pointer of the UART hardware */
const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
/* Basic UART transmit buffer */
static uint8_t tx_uart_buf[] = {"UART online!\r\n"};
/* Basic UART Receive buffer */
static uint8_t rx_uart_buf[UART_BUFF_SIZE] = {0};
/* Buffer to be used by the rest of the worker thread */
static uint8_t rx_app_buf[UART_BUFF_SIZE] = {0};
/* Flag to track when the UART Receive buffer needs flushed */
static bool uart_rx_buf_clear_req = false;



/* ========== Workqueue Functions ========== */

void uart_work_handler(struct k_work *work_term)
{

	LOG_INF("Work queue item started");
	LOG_DBG("Data received: %s", rx_app_buf);

	/* TODO: check for correct keyword at beginning of rx_uart_buf, AT+CNA= */

	/*
	 *
	 *
	 *	Do something here
	 * 
	 * 
	 */

	/* Uncomment to emulate a few seconds of work, for testing purposes*/
	// for(volatile int count_out = 0; count_out < 30000000; count_out ++);

	/* Set flag to clear buffer of any data receieved while thread was processing */
	uart_rx_buf_clear_req = true;
	uart_rx_enable(uart, rx_uart_buf, sizeof(rx_uart_buf), UART_RECEIVE_TIMEOUT);
	LOG_INF("Work finished");
}



/* ========== UART Functions ========== */

/* Callback function for UART */
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{

	int kwork_ret;
	int copied_rx_data_count;

	switch (evt->type) {
	/* Event types: UART_RX_RDY, UART_RX_BUF_REQUEST, UART_RX_BUF_RELEASED, UART_RX_DISABLED, UART_RX_STOPPED
					UART_TX_DONE, UART_TX_ABORTED */
	
	case UART_RX_RDY:

		LOG_DBG("Data received on UART");

		/* Make sure the received data doesn't fill the buffer */
		if (evt->data.rx.len == UART_BUFF_SIZE) {
			LOG_WRN("Buffer full, please try again");
			uart_rx_buf_clear_req = true;
			uart_rx_disable(uart);
			return;
		}

		/* Disable the RX buffer when data is received during a disable request */
		if (uart_rx_buf_clear_req) {
			LOG_WRN("Data received during RX disable request, discarding");
			uart_rx_disable(uart);
			return;
		}

		/* The offset will be greater than zero under a few conditions:
		 * 1) The terminal is in char mode
		 * 2) Data was received after the uart_rx_disable function was called,
		 *    but before uart_cb processed the UART_RX_DISABLED condition.
		 * This can cause the termination character check to fail,
		 * so clear the buffer and send a warning message.
		 */
		if (evt->data.rx.offset) {
			LOG_WRN("UART de-sync! Is terminal in line mode? (evt->data.rx.offset > 0)");
			uart_rx_buf_clear_req = true;
			uart_rx_disable(uart);
			return;
		}

		/* Check for line termination character(s), then copy to another buffer used by the worker thread */
		if ((evt->data.rx.buf[evt->data.rx.len - 1] == '\n') || (evt->data.rx.buf[evt->data.rx.len - 1] == '\r')) {
			if ((evt->data.rx.buf[evt->data.rx.len - 2] == '\n') || (evt->data.rx.buf[evt->data.rx.len - 2] == '\r')) {
				copied_rx_data_count = snprintk(rx_app_buf, evt->data.rx.len - 1, "%s", rx_uart_buf);
			}
			else {
				copied_rx_data_count = snprintk(rx_app_buf, evt->data.rx.len, "%s", rx_uart_buf);
			}
			if (copied_rx_data_count < 0 ) {
				LOG_ERR("snprintk encoding error occured, returned %d", copied_rx_data_count);
			}
			/* Stop the UART receiver. It will be re-enabled once the uart worker thread is complete */
			uart_rx_disable(uart);
		}

		break;

	case UART_RX_DISABLED:
	 	
		LOG_DBG("UART RX Disabled");
		
		/* If the receiver was disabled due to a clear buffer flag, it needs re-enabled */
		if (uart_rx_buf_clear_req) {
			LOG_DBG("Clearing RX Buffer");
			uart_rx_buf_clear_req = false;
			uart_rx_enable(uart, rx_uart_buf, sizeof(rx_uart_buf), UART_RECEIVE_TIMEOUT);
		}
		/* If the receiver was disabled because a line-terminated string was sent, start the worker thread */
		else {
			kwork_ret = k_work_submit_to_queue(&offloaded_work_queue, &uart_rx_work);
			switch(kwork_ret) {
				case KWORK_ALREADY_SUBMITTED:
				LOG_DBG("Work already submitted to queue");
				break;
				
				case KWORK_QUEUED:
				LOG_DBG("Work submitted to queue");
				break;
				
				case KWORK_RUNNING_REQUEUED:
				LOG_DBG("Work already running, submitted to queue again");
				break;

				default:
				break;
			}
		}
		break;
	
	case UART_RX_BUF_REQUEST:
		LOG_DBG("UART RX Buffer Request");
		/* When the buffer is enabled, if the clear flag is still set, disable the UART to clear it */
		if (uart_rx_buf_clear_req) {
			uart_rx_disable(uart);
		}
		break;

	case UART_RX_BUF_RELEASED:
		LOG_DBG("UART RX Buffer Released");
		break;

	case UART_RX_STOPPED:

		LOG_DBG("UART Stopped");

		uart_rx_buf_clear_req = true;

		switch (evt->data.rx_stop.reason) {
		/* Stop reasons: UART_ERROR_OVERRUN, UART_ERROR_PARITY, UART_ERROR_FRAMING, UART_BREAK,
						 UART_ERROR_COLLISION, UART_ERROR_NOISE */

		case UART_ERROR_OVERRUN:
			LOG_WRN("Buffer Overrun! Receiver stopped");
			break;

		case UART_ERROR_PARITY:
			LOG_WRN("Parity Error! Receiver stopped");
			break;

		case UART_ERROR_FRAMING:
			LOG_WRN("Framing Error! Receiver stopped");
			break;

		case UART_BREAK:
			LOG_WRN("Break Interrupt! Receiver stopped");
			break;

		case UART_ERROR_COLLISION:
			LOG_WRN("Collision Error! Receiver stopped");
			break;

		case UART_ERROR_NOISE:
			LOG_WRN("Noise Error! Receiver stopped");
			break;
		
		default:
			break;
		}
		break;

	default:
		break;
	}

}



int main(void)
{
	int ret;


	/* ========== Workqueue Initializations ========== */

	/* Start the workqueue */
	k_work_queue_start(&offloaded_work_queue, work_queue_stack_area,
		K_THREAD_STACK_SIZEOF(work_queue_stack_area),
		WORKQUEUE_PRIORITY, NULL);

	/* Initialize works items and connect them to their handler functions */
	k_work_init(&uart_rx_work, uart_work_handler);


	/* ========== UART Initializations ========== */

	/* Register UART callback function */
	ret = uart_callback_set(uart, uart_cb, NULL);
	if (ret) {
		return 1;
	}

	/* Send the data over UART by calling uart_tx() */
	ret = uart_tx(uart, tx_uart_buf, sizeof(tx_uart_buf), SYS_FOREVER_MS);
	if (ret) {
		return 1;
	}

	/* Start receiving by calling uart_rx_enable() and pass it the address of the receive buffer */
	ret = uart_rx_enable(uart, rx_uart_buf, sizeof(rx_uart_buf), UART_RECEIVE_TIMEOUT);
	if (ret) {
		return 1;
	}

	while (1) {
		k_msleep(SLEEP_TIME_MS);
	}
}