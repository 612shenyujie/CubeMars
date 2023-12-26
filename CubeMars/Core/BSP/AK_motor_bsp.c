#include "AK_motor_bsp.h"




static float AK_motor_From_MultiRoundPositionLsb_To_OutputMultiRoundPositionDegree(int16_t input);
static float AK_motor_From_EletricalSpeedRpm_To_OutputSpeedRpm(int16_t input, uint8_t polor_num, float reduction_rate);
static float AK_motor_From_CurrentLsb_To_CurrentAmpire(int16_t input);

AK_MOTOR_BSP_RETURN_T AK_motor_FeedbackProcess(AK_motor_t *motor, uint32_t CAN_ID, uint8_t data[])
{
	// 先把数据存起来
	// 这个电机和大疆的一样烦，居然是LSB在后 (╯‵□′)╯︵┻━┻
	motor->feedback.multi_round_position_lsb	= data[0]<<8 | data[1];
	motor->feedback.eletrical_speed_rpm			= data[2]<<8 | data[3];
	motor->feedback.current_lsb					= data[4]<<8 | data[5];
	motor->feedback.tempreture_celsius			= data[6];
	motor->feedback.err_code					= data[7];
	// 把原始数据进行转化
	motor->status.tempreture_celsius					= motor->feedback.tempreture_celsius;
	motor->status.current_ampire						= AK_motor_From_CurrentLsb_To_CurrentAmpire(motor->feedback.current_lsb);
	motor->status.output_speed_rpm						= AK_motor_From_EletricalSpeedRpm_To_OutputSpeedRpm(motor->feedback.eletrical_speed_rpm, motor->parameter.polor_num, motor->parameter.reduction_rate);
	motor->status.output_multi_round_position_degree	= AK_motor_From_MultiRoundPositionLsb_To_OutputMultiRoundPositionDegree(motor->feedback.multi_round_position_lsb);
	return AK_MOTOR_BSP_OK;
}

AK_MOTOR_BSP_RETURN_T AK_motor_command_transmit(AK_motor_ctx_t *ctx, AK_motor_t *motor)
{
	uint32_t	extid = 0;
	uint8_t		data[8];
	memset(&data, 0, sizeof(data));
	// 先填充拓展帧
	memcpy((uint8_t*)&extid,	&motor->parameter.CANID,			1);
	memcpy((uint8_t*)&extid+1,	&motor->command.servo_control_mode, 1);
	switch (motor->command.servo_control_mode)
	{
		
		case CONTROL_MODE_SET_DUTY:
			break;
		case CONTROL_MODE_SET_CURRENT:
			data[0] = motor->command.current >> 24;
			data[1] = motor->command.current >> 16	& 0xFF;
			data[2] = motor->command.current >> 8	& 0xFF;
			data[3] = motor->command.current & 0xFF;
			break;
		case CONTROL_MODE_SET_CURRENT_BRAKE:
			break;
		case CONTROL_MODE_SET_RPM:
			break;
		case CONTROL_MODE_SET_POS:
			data[0] = motor->command.position >> 24;
			data[1] = motor->command.position >> 16 & 0xFF;
			data[2] = motor->command.position >> 8	& 0xFF;
			data[3] = motor->command.position & 0xFF;
			break;
		case CONTROL_MODE_SET_ZERO_POSITION:
			data[0]	= motor->command.zero_position_type;
			break;
		case CONTROL_MODET_POS_SPD:
			data[0] = motor->command.position >> 24;
			data[1] = motor->command.position >> 16 & 0xFF;
			data[2] = motor->command.position >> 8	& 0xFF;
			data[3] = motor->command.position & 0xFF;
			data[4]	= motor->command.speed >> 8;
			data[5]	= motor->command.speed & 0xFF;
			data[6]	= motor->command.acceleration >> 8;
			data[7]	= motor->command.acceleration & 0xFF;
			break;
		default:
			return AK_MOTOR_BSP_WRONG_PARAM;
			break;
	}
	return ctx->tx_cmd(ctx->peripheral_handle, extid, data);
}



static float AK_motor_From_MultiRoundPositionLsb_To_OutputMultiRoundPositionDegree(int16_t input)
{
	return input/10.0;
}

static float AK_motor_From_EletricalSpeedRpm_To_OutputSpeedRpm(int16_t input, uint8_t polor_num, float reduction_rate)
{
	return input / polor_num / reduction_rate;
}

static float AK_motor_From_CurrentLsb_To_CurrentAmpire(int16_t input)
{
	return input/100.00;
}

