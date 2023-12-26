#ifndef AK_MOTOR_BSP_H
#define AK_MOTOR_BSP_H

#ifdef __cplusplus
extern "C" {
#endif


/* USER Settings -------------------------------------------------------------*/
#define STM32F105

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

typedef uint8_t (*AK_motor_tx_ptr)(void *, uint32_t , uint8_t[]);


typedef enum
{
	AK_MOTOR_BSP_OK,
	AK_MOTOR_BSP_ERROR,
	
	AK_MOTOR_BSP_WRONG_PARAM,
}AK_MOTOR_BSP_RETURN_T;

typedef enum
{
	CONTROL_MODE_SET_DUTY			= 0x00U,	// 占空比模式
	CONTROL_MODE_SET_CURRENT		= 0x01U,	// 电流环模式
	CONTROL_MODE_SET_CURRENT_BRAKE	= 0x02U,	// 电流刹车模式
	CONTROL_MODE_SET_RPM			= 0x03U,	// 转速模式
	CONTROL_MODE_SET_POS			= 0x04U,	// 位置模式
	CONTROL_MODE_SET_ZERO_POSITION	= 0x05U,	// 设置原点模式
	CONTROL_MODET_POS_SPD			= 0x06U,	// 位置速度环模式
}AK_MOTOR_SERVO_CONTROL_MODE_T;

typedef enum
{
	TEMPORARY_ZERO_POSITION	= 0x00U,	// 设置临时原点(断电消除)
	ETERNAL_ZERO_POSITION,				// 设置永久零点(参数自动保存)
	RESTORE_DEFAULT_ZEROPOSITION,		// 恢复默认零点(参数自动保存)
}AK_MOTOR_ZERO_POSITION_TYPE_T;

typedef struct
{
	AK_MOTOR_SERVO_CONTROL_MODE_T servo_control_mode;
	int32_t	current;
	int32_t position;
	int16_t	speed;
	int16_t	acceleration;
	AK_MOTOR_ZERO_POSITION_TYPE_T zero_position_type;
}AK_motor_command_t;

typedef struct
{
	void*		peripheral_handle;
	uint8_t		polor_num;
	float		reduction_rate;
	uint16_t	CANID;
}AK_motor_parameter_t;

typedef struct
{
	float	output_multi_round_position_degree;
	float	output_speed_rpm;
	float	current_ampire;
	int8_t	tempreture_celsius;
}AK_motor_status_t;

typedef struct
{
	int16_t multi_round_position_lsb;
	int16_t	eletrical_speed_rpm;
	int16_t	current_lsb;
	int8_t	tempreture_celsius;
	uint8_t	err_code;
}AK_motor_feedback_t;


typedef struct
{
	AK_motor_tx_ptr	tx_cmd;
	void *peripheral_handle;
} AK_motor_ctx_t;

typedef struct
{
	AK_motor_parameter_t	parameter;
	AK_motor_command_t		command;
	AK_motor_feedback_t		feedback;
	AK_motor_status_t		status;
					
} AK_motor_t;




AK_MOTOR_BSP_RETURN_T AK_motor_FeedbackProcess(AK_motor_t *motor, uint32_t CAN_ID, uint8_t data[]);
AK_MOTOR_BSP_RETURN_T AK_motor_command_transmit(AK_motor_ctx_t *ctx, AK_motor_t *motor);

#ifdef __cplusplus
}
#endif
#endif
