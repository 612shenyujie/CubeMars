/* Includes ------------------------------------------------------------------*/
#include "AK_motor.h"

#if defined(STM32F105) | (STM32F407)
	#include "can.h"
#endif



/* CAN Bus Settings ----------------------------------------------------------*/
#if defined(STM32F105) | (STM32F407)
    CAN_TxHeaderTypeDef AK_motor_CAN_TxHeaderStruct;
    uint32_t  AK_motor_pTxMailbox;
#endif


uint8_t platform_trans(void *peripheral_handle, uint32_t CANID, uint8_t data[]);


/* Motor Handle Create -------------------------------------------------------*/
AK_motor_t AK_motor_LF, AK_motor_LB, AK_motor_RF, AK_motor_RB;
AK_motor_ctx_t AK_motor_ctx;




void AK_motor_SetCurrent(AK_motor_t *motor, float current_ampire)
{
	motor->command.servo_control_mode	= CONTROL_MODE_SET_CURRENT;
	motor->command.current				= (int32_t) (current_ampire * 100); // 标准单位->协议LSB
	AK_motor_ctx.peripheral_handle		= motor->parameter.peripheral_handle;
	AK_motor_command_transmit(&AK_motor_ctx, motor);

}

void AK_motor_SetMultiPosition_SI(AK_motor_t *motor, float position_degree)
{
	motor->command.servo_control_mode	= CONTROL_MODE_SET_POS;
	motor->command.position				= (int32_t) (position_degree * 10000); // 标准单位->协议LSB
	AK_motor_ctx.peripheral_handle		= motor->parameter.peripheral_handle;
	AK_motor_command_transmit(&AK_motor_ctx, motor);
}

void AK_motor_SetZeroPosition(AK_motor_t *motor, AK_MOTOR_ZERO_POSITION_TYPE_T type)
{
	motor->command.servo_control_mode	= CONTROL_MODE_SET_ZERO_POSITION;
	motor->command.zero_position_type	= type;
	AK_motor_ctx.peripheral_handle		= motor->parameter.peripheral_handle;
	AK_motor_command_transmit(&AK_motor_ctx, motor);
}

void AK_motor_SetMultiPositionSpeedAcceleration_SI(AK_motor_t *motor, int32_t position_degree, float output_speed_limit_SI, float output_acc_limit_SI)
{
	motor->command.servo_control_mode	= CONTROL_MODET_POS_SPD;
	motor->command.position				= (int32_t) (position_degree * 10000); // 标准单位->协议LSB
	motor->command.speed				= (int16_t) (output_speed_limit_SI * motor->parameter.polor_num * motor->parameter.reduction_rate);
	motor->command.acceleration			= (int16_t) (output_acc_limit_SI * motor->parameter.polor_num * motor->parameter.reduction_rate);
	AK_motor_ctx.peripheral_handle		= motor->parameter.peripheral_handle;
	AK_motor_command_transmit(&AK_motor_ctx, motor);
}


	
AK_motor_handle_list_unit_t AK_motor_handle_list[MAXIMUM_AK_MOTOR_HANDLE_NUM];
AK_MOTOR_RETURN_T AK_motor_HandleListAdd(AK_motor_t *motor)
{
	for (int i=0; i<MAXIMUM_AK_MOTOR_HANDLE_NUM; i++) // 从表头开始找有没有空位
		if (AK_motor_handle_list[i].motor_handle == 0)
		{
			AK_motor_handle_list[i].motor_handle		= motor;
			AK_motor_handle_list[i].CANID				= motor->parameter.CANID;
			AK_motor_handle_list[i].peripheral_handle	= motor->parameter.peripheral_handle;
			return AK_MOTOR_OK;
		}
	return AK_MOTOR_ERROR;
}
/**
 * @brief 在AK电机列表中确认是否有某个AK电机的句柄，确保句柄合法性
 * @param motor 电机的句柄
 * @return 如果成功返回 STEERING_WHEEL_OK。
 */
AK_MOTOR_RETURN_T AK_motor_CheckHandleLegitimacy(AK_motor_t *motor)
{
	for (int i=0; i<MAXIMUM_AK_MOTOR_HANDLE_NUM; i++) // 从表头开始找
		if (AK_motor_handle_list[i].motor_handle == motor)
			return AK_MOTOR_OK;
		return AK_MOTOR_ILLEGAL_HANDLE; // 句柄找不到，不合法

}
/**
 * @brief 通过CANID和外设在AK电机列表中查询电机的句柄
 * @param motor 电机的句柄
 * @return 如果成功返回电机的句柄，否则返回空句柄
 */
AK_motor_t *AK_motor_FindSteeringHandle_via_CANIDandPeripheral(uint8_t CANID, void *peripheral_handle)
{
	for (int i=0; i<MAXIMUM_AK_MOTOR_HANDLE_NUM; i++)
		if (AK_motor_handle_list[i].CANID == CANID && AK_motor_handle_list[i].peripheral_handle == peripheral_handle) // 在记录的同一个外设，以及同一个CANID
			return AK_motor_handle_list[i].motor_handle;
	return AK_MOTOR_NULL_HANDLE;
}


void AK_motor_HandleInit(AK_motor_t *motor, AK_motor_parameter_t param)
{
	memcpy(&motor->parameter, &param, sizeof(param));
	AK_motor_HandleListAdd(motor);
}


void AK_motor_Init(void)
{
	// 初始化电机列表
	memset(&AK_motor_handle_list, 0, sizeof(AK_motor_handle_list));
	
	// 初始化电机
	AK_motor_parameter_t init_struct;
	// 初始化左前电机
	init_struct.reduction_rate		= 6;
	init_struct.polor_num			= 21;
	init_struct.CANID				= 0x0A;
	init_struct.peripheral_handle	= &MOTOR_ON_BUS_2;
	AK_motor_HandleInit(&AK_motor_LF, init_struct);
	// 初始化右前电机
	init_struct.CANID			= 0x0B;
	AK_motor_HandleInit(&AK_motor_RF, init_struct);
	// 初始化右后电机
	init_struct.CANID			= 0x0C;
	AK_motor_HandleInit(&AK_motor_RB, init_struct);
	// 初始化左后电机
	init_struct.CANID			= 0x0D;
	AK_motor_HandleInit(&AK_motor_LB, init_struct);
	
	// 初始化发送函数
	#if defined(STM32F105) | defined(STM32F407)
		// 指定发送的CAN函数
		AK_motor_ctx.tx_cmd = platform_trans;
		// CAN发送时用到了的结构体
		AK_motor_CAN_TxHeaderStruct.ExtId = 0;
        AK_motor_CAN_TxHeaderStruct.DLC = 8;
        AK_motor_CAN_TxHeaderStruct.IDE = CAN_ID_EXT; // 使用拓展帧
        AK_motor_CAN_TxHeaderStruct.RTR = CAN_RTR_DATA;
        AK_motor_CAN_TxHeaderStruct.TransmitGlobalTime = DISABLE;
	#endif
	HAL_Delay(4000);
	AK_motor_SetZeroPosition(&AK_motor_RF, TEMPORARY_ZERO_POSITION);
	AK_motor_SetZeroPosition(&AK_motor_RB, TEMPORARY_ZERO_POSITION);
}

void AK_motor_feedback_handler(AK_motor_t *motor, uint32_t CAN_ID, uint8_t data[])
{
	AK_motor_FeedbackProcess(motor, CAN_ID, data);
}

/**
 * @brief 供用户自定义的发送函数
 * @param peripheral_handle 电机发送时外设的句柄
 * @param CANID 发送消息的拓展帧
 * @param data[] 发送消息的数据帧
 * @return 发送是否成功
 */
uint8_t platform_trans(void *peripheral_handle, uint32_t CANID, uint8_t data[])
{
	#if defined(STM32F105) | defined(STM32F103)
		AK_motor_CAN_TxHeaderStruct.ExtId = CANID;
        return HAL_CAN_AddTxMessage(peripheral_handle, &AK_motor_CAN_TxHeaderStruct, data, &AK_motor_pTxMailbox);
	#endif

}