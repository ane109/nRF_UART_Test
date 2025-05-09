# nRF_UART_Test

 A program to demonstrate and test Async UART behavior in Zephyr RTOS.

The UART receiver is set up a bit differently than the receiver in the [Multi-NUS](https://github.com/NordicMatt/multi-NUS/blob/master/src/main.c) Bluetooth example. Instead of using continuous reception for the receiver, this uses a fixed 100 char buffer with some extra control flow logic to handle errors and overflows.

The attached terminal needs to be set up with either a CR, LR, or CRLF line termination. Once detected, this will disable the UART and send the received data to a worker thread, which will re-enable the UART once finished.

If logging is enabled, there are various warnings (such as overflow) that will be sent to the terminal for troubleshooting purposes. A case structure with all the available UART receiver STOP reasons is also included.

---

#### The receiver control flow can be broken down as follows:

1. A string is sent to the receiver with a standard line termination character.
2. If the string does not exceed the buffer length, the receiver checks for a line terminator. Otherwise the buffer is cleared and a warning is shown if logging is enabled.
3. If a line terminator is found, then the received string is passed to a worker thread. Otherwise the buffer is cleared and a warning is shown if logging is enabled. 
4. The UART receiver shuts down until the worker thread has completed its task.
5. Once the worker thread is finished, the UART is re-enabled.
6. The buffer is cleared of any data that may have been receieved during the worker thread processing, and is ready to accept a new string.
