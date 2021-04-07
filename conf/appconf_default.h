#ifndef APPCONF_DEFAULT_H_
#define APPCONF_DEFAULT_H_

// Default app configuration
#ifndef APPCONF_CONTROLLER_ID
#define APPCONF_CONTROLLER_ID				1
#endif
#ifndef APPCONF_TIMEOUT_MSEC
#define APPCONF_TIMEOUT_MSEC				1000
#endif
#ifndef APPCONF_TIMEOUT_BRAKE_CURRENT
#define APPCONF_TIMEOUT_BRAKE_CURRENT		0.0f
#endif
#ifndef APPCONF_SEND_CAN_STATUS
#define APPCONF_SEND_CAN_STATUS				true
#endif
#ifndef APPCONF_SEND_CAN_STATUS_RATE_HZ
#define APPCONF_SEND_CAN_STATUS_RATE_HZ		1000
#endif

// UART app
#ifndef APPCONF_UART_BAUDRATE
#define APPCONF_UART_BAUDRATE				115200
#endif

#endif /* APPCONF_DEFAULT_H_ */
