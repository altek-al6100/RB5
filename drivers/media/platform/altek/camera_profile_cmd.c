/*
 * File: camera_profile_cmd.c
 * Description: Mini ISP sample codes
 *
 * Copyright 2019-2030  Altek Semiconductor Corporation
 *
 *  2013/10/14; Bruce Chung; Initial version
 *  2013/12/05; Bruce Chung; 2nd version
 */

/*
 * This file is part of al6100.
 *
 * al6100 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2, as published by
 * the Free Software Foundation.
 *
 * al6100 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTIBILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License version 2 for
 * more details.
 *
 * You should have received a copy of the General Public License version 2
 * along with al6100. If not, see https://www.gnu.org/licenses/gpl-2.0.html.
 */



/******Include File******/

#include <linux/string.h>

#include "include/mtype.h"
#include "include/error.h"
#include "include/miniisp.h"
#include "include/isp_camera_cmd.h"
#include "include/ispctrl_if_master.h"
#include "include/ispctrl_if_master_local.h"

/******Private Constant Definition******/
#define MINI_ISP_LOG_TAG "[[miniisp]camera_profile_cmd]"

/******Private Type Declaration******/

/******Private Function Prototype******/

/******Private Global Variable******/

static struct isp_cmd_get_sensor_mode mast_sensors_info;

/******Public Function******/

/*
 *\brief Camera profile parameters init
 *\return None
 */
void isp_mast_camera_profile_para_init(void)
{
	/*Reset Camera profile parameters*/
	memset(&mast_sensors_info, 0x0,
		sizeof(struct isp_cmd_get_sensor_mode));
}

/*
 *\brief Set Sensor Mode
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_sensor_mode(
					void *devdata, u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size;
	struct isp_cmd_set_sensor_mode *set_sensor_mode_param;

	set_sensor_mode_param = (struct isp_cmd_set_sensor_mode *)param;
	misp_info("%s - enter", __func__);
	misp_info("[on/off]: %d, [id]:%d, [txskew]: %d",
		set_sensor_mode_param->sensor_on_off,
		set_sensor_mode_param->scenario_id,
		set_sensor_mode_param->mipi_tx_skew);

	misp_info("[w_tbl_idx]: %d, [merge_enable]:%d",
		set_sensor_mode_param->ae_weighting_table_index,
		set_sensor_mode_param->merge_mode_enable);

	para_size = sizeof(struct isp_cmd_set_sensor_mode);

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
				param, para_size);


	if ((((struct isp_cmd_set_sensor_mode *)param)->sensor_on_off)
			&& (err == ERR_SUCCESS))
		err = mini_isp_wait_for_event(MINI_ISP_RCV_SETSENSORMODE);

	return err;
}

/*
 *\brief Get Sensor Mode
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_get_sensor_mode(
					void *devdata, u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size;

	misp_info("%s - enter", __func__);

	para_size = sizeof(struct isp_cmd_get_sensor_mode);

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode, NULL, 0);
	if (err  != ERR_SUCCESS)
		goto mast_camera_profile_cmd_get_sensor_mode_end;

	/* Get data from slave*/
	err = ispctrl_mast_recv_response_from_slave(devdata,
			(u8 *)&mast_sensors_info, para_size, true);
	if (err  != ERR_SUCCESS)
		goto mast_camera_profile_cmd_get_sensor_mode_end;

	/* copy to usr defined target addr*/
	memcpy(param, &mast_sensors_info, para_size);

mast_camera_profile_cmd_get_sensor_mode_end:
	return err;
}

/*
 *\brief Set Output Format
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_output_format(
					void *devdata, u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size;
	struct isp_cmd_set_output_format *set_output_format_param;

	set_output_format_param = (struct isp_cmd_set_output_format *)param;

	misp_info("%s - enter", __func__);
	misp_info("[DP_size]: 0x%x, [DP_Type_LV]: 0x%x, [InvRect_bypass]: 0x%x",
		set_output_format_param->depth_size,
		set_output_format_param->reserve[0],
		set_output_format_param->reserve[1]);

	para_size = sizeof(struct isp_cmd_set_output_format);

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode, param,
						para_size);
	return err;
}

/*
 *\brief Set CP Mode
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_cp_mode(
					void *devdata, u16 opcode, u8 *param)
{
	int err = ERR_SUCCESS;
	u32 para_size = 0;
	struct misp_global_variable *dev_global_variable;

	misp_info("%s - enter", __func__);
	dev_global_variable = get_mini_isp_global_variable();
	mini_isp_e_to_a();
	dev_global_variable->spi_low_speed_mode = 1;

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
				param, para_size);
	if (err != ERR_SUCCESS)
		goto EXIT;

	err = mini_isp_wait_for_event(MINI_ISP_RCV_CPCHANGE);

	if (err != 0)
		goto EXIT;
EXIT:
	mini_isp_a_to_e();
	/*enter code here*/
	mini_isp_cp_mode_suspend_flow();
	return err;
}

/*
 *\brief Set AE statistics
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_basic_setting_cmd_set_ae_statistics(
					void *devdata, u16 opcode, u8 *param)
{
	/*Error Code */
	errcode err = ERR_SUCCESS;
	u32 para_size = sizeof(struct isp_cmd_ae_statistics);

	misp_info("%s - enter", __func__);
	/*Send command to slave */
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
					param, para_size);
	return err;
}

/*
 *\brief Preview stream on off
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_preview_stream_on_off(
					void *devdata, u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size;
	struct isp_cmd_preview_stream_on_off *preview_stream_on_off_param;

	preview_stream_on_off_param =
		(struct isp_cmd_preview_stream_on_off *)param;

	misp_info("%s - enter", __func__);
	misp_info("[tx0]: %d, [tx1]: %d, [reserve]: %d",
		preview_stream_on_off_param->tx0_stream_on_off,
		preview_stream_on_off_param->tx1_stream_on_off,
		preview_stream_on_off_param->reserve);

	para_size = sizeof(struct isp_cmd_preview_stream_on_off);

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
			param, para_size);
	return err;
}

/*
 *\brief dual PD Y calculation weightings
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode
mast_camera_profile_cmd_dual_pd_y_cauculation_weightings(
					void *devdata, u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size;

	misp_info("%s - enter", __func__);
	para_size = sizeof(struct isp_cmd_dual_pd_y_calculation_weightings);

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
			param, para_size);
	return err;
}

/*
 *\brief LED power control
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_led_power_control(
					void *devdata, u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size;
	struct isp_cmd_led_power_control *led_power_control_param;

	led_power_control_param = (struct isp_cmd_led_power_control *)param;
	misp_info("%s - enter", __func__);
	misp_info("[led_onoff]: %d, [led_lv]: %d, [led_id]: %d",
		led_power_control_param->led_on_off,
		led_power_control_param->led_power_level,
		led_power_control_param->control_projector_id);

	misp_info("[delay_after_sof]: %d, [pulse_time]: %d",
		led_power_control_param->delay_after_sof,
		led_power_control_param->pulse_time);

	misp_info("[control_mode]: %d, [reserved]: %d, [rolling_shutter]: %d",
		led_power_control_param->control_mode,
		led_power_control_param->reserved,
		led_power_control_param->rolling_shutter);

	para_size = sizeof(struct isp_cmd_led_power_control);

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
			param, para_size);
	return err;
}

/*
 *\brief Active AE
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_active_ae(
					void *devdata, u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size;
	struct isp_cmd_active_ae *active_ae_param;

	active_ae_param = (struct isp_cmd_active_ae *) param;
	misp_info("%s - enter", __func__);
	misp_info("[active ae]: %d, [f_number]: %d",
		active_ae_param->active_ae, active_ae_param->f_number_x1000);

	para_size = sizeof(struct isp_cmd_active_ae);

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
			param, para_size);
	return err;
}

/*
 *\brief Control AE on/off
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_isp_ae_control_on_off(
					void *devdata, u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size;
	struct isp_cmd_isp_ae_control_on_off *ae_control_on_off_param;

	ae_control_on_off_param = (struct isp_cmd_isp_ae_control_on_off *)param;
	misp_info("%s - enter", __func__);
	misp_info("[ae control]: %d",
		ae_control_on_off_param->isp_ae_control_mode_on_off);

	para_size = sizeof(struct isp_cmd_isp_ae_control_on_off);

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
			param, para_size);
	return err;
}

/*
 *\brief Set Frame Rate Limits
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_frame_rate_limits(
					void *devdata, u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size;

	misp_info("%s - enter", __func__);
	para_size = sizeof(struct isp_cmd_frame_rate_limits);

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
			param, para_size);
	return err;
}

/*
 *\brief Set period drop frame
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_period_drop_frame(
					void *devdata, u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size;
	struct isp_cmd_period_drop_frame *period_drop_frame_param;

	period_drop_frame_param = (struct isp_cmd_period_drop_frame *)param;
	misp_info("%s - enter", __func__);
	misp_info("[period_drop_type]: %d",
		period_drop_frame_param->period_drop_type);

	para_size = sizeof(struct isp_cmd_period_drop_frame);

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
			param, para_size);
	return err;
}

/*
 *\brief Set Max exposure
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_max_exposure(
					void *devdata, u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size;

	misp_info("%s - enter", __func__);
	para_size = sizeof(struct isp_cmd_exposure_param);

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
			param, para_size);
	return err;
}

/*
 *\brief Set target mean
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_target_mean(
					void *devdata, u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size;

	misp_info("%s - enter", __func__);
	para_size = sizeof(struct isp_cmd_target_mean);

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
			param, para_size);
	return err;
}

/*
 *\brief Frame sync control
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_frame_sync_control(
					void *devdata, u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size;
	struct isp_cmd_frame_sync_control *frame_sync_control_param;

	frame_sync_control_param = (struct isp_cmd_frame_sync_control *)param;
	misp_info("%s - enter", __func__);
	misp_info("[deviceId]: %d, [delay_frame]: %d, [active_frame]: %d,",
		frame_sync_control_param->control_deviceID,
		frame_sync_control_param->delay_framephase,
		frame_sync_control_param->active_framephase);

	misp_info("[deactive_frame]: %d, [active_timeLv]: %d",
		frame_sync_control_param->deactive_framephase,
		frame_sync_control_param->active_timelevel);

	para_size = sizeof(struct isp_cmd_frame_sync_control);

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
			param, para_size);
	return err;
}

/*
 *\brief Set shot mode
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_shot_mode(
					void *devdata, u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size;
	struct isp_cmd_set_shot_mode *set_shot_mode_param;

	set_shot_mode_param = (struct isp_cmd_set_shot_mode *)param;
	misp_info("%s - enter", __func__);
	misp_info("[shot_mode]: %d, [frame_rate]: %d",
		set_shot_mode_param->shot_mode,
		set_shot_mode_param->frame_rate);
	para_size = sizeof(struct isp_cmd_set_shot_mode);

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
			param, para_size);
	return err;
}

/*
 *\brief Lighting control
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_lighting_ctrl(
					void *devdata, u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size, i;
	struct isp_cmd_lighting_ctrl *lighting_ctrl_param;

	lighting_ctrl_param = (struct isp_cmd_lighting_ctrl *)param;
	misp_info("%s - enter", __func__);
	misp_info("[cycle_len]: %d", lighting_ctrl_param->cycle_len);
	for (i = 0; i < lighting_ctrl_param->cycle_len; i++) {
		misp_info("cycle[%d]: 0x%x, 0x%x, %d", i,
			lighting_ctrl_param->cycle[i].source,
			lighting_ctrl_param->cycle[i].TxDrop,
			lighting_ctrl_param->cycle[i].co_frame_rate);
	}
	para_size = sizeof(struct isp_cmd_lighting_ctrl);

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
			param, para_size);
	return err;
}

/*
 *\brief Set Min exposure
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_min_exposure(
					void *devdata, u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size;

	misp_info("%s - enter", __func__);
	para_size = sizeof(struct isp_cmd_exposure_param);

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
			param, para_size);
	return err;
}

/*
 *\brief Set Max exposure slope
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_max_exposure_slope(
					void *devdata, u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size;

	misp_info("%s - enter", __func__);
	para_size = sizeof(struct isp_cmd_max_exposure_slope);

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
			param, para_size);
	return err;
}

/*
 *\brief Set Depth Compensation
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_depth_compensation(
					void *devdata, u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size;
	struct isp_cmd_depth_compensation_param *depth_compensation_param;

	depth_compensation_param =
		(struct isp_cmd_depth_compensation_param *)param;

	misp_info("%s - enter", __func__);
	misp_info("[en_updated]: 0x%x, [short_dist]: %d, [compensation]: %d",
		depth_compensation_param->en_updated,
		depth_compensation_param->short_distance_value,
		depth_compensation_param->compensation);

	para_size = sizeof(struct isp_cmd_depth_compensation_param);

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
			param, para_size);
	return err;
}

/*
 *\brief Set cycle trigger depth process
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_cycle_trigger_depth(
					void *devdata, u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size;
	struct isp_cmd_cycle_trigger_depth_process
		*cycle_trigger_depth_process_param;

	cycle_trigger_depth_process_param =
		(struct isp_cmd_cycle_trigger_depth_process *)param;

	misp_info("%s - enter", __func__);
	misp_info("[Cycle_len]: %d, [DP_trigBitField]: 0x%x",
		cycle_trigger_depth_process_param->cycleLen,
		cycle_trigger_depth_process_param->depth_triggerBitField);

	misp_info("[DPOut_trigBitField]: 0x%x",
		cycle_trigger_depth_process_param->depthoutput_triggerBitField);

	para_size = sizeof(struct isp_cmd_cycle_trigger_depth_process);

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
			param, para_size);
	return err;
}
/*
 *\brief Set led active delay time
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_led_active_delay(
					void *devdata, u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size;

	misp_info("%s - enter", __func__);
	misp_info("[delay]: %d", *(u32 *)param);
	para_size = sizeof(u32);
	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
			param, para_size);
	return err;
}
/*
 *\brief Set isp control led level
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_isp_smart_ae_control(
					void *devdata, u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size;

	misp_info("%s - enter", __func__);
	misp_info("[isp control led level]: %d", *(u8 *)param);
	para_size = sizeof(u8);
	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
			param, para_size);
	return err;
}

/*
 *\brief Set group led level
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_group_led_level(
					void *devdata, u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size;
	struct ISPCMD_LED_LEVEL *Set_group_led_level_process_param;

	Set_group_led_level_process_param = (struct ISPCMD_LED_LEVEL *)param;
	misp_info("%s - enter", __func__);
	misp_info("[type]:%d,[Group0Ds]:0x%x,[Group0LV]:0x%x,[Group1Ds]:0x%x,[Group1LV]: 0x%x",
		Set_group_led_level_process_param->ucType,
		Set_group_led_level_process_param->ucGroup0IDs,
		Set_group_led_level_process_param->ucGroup0Level,
		Set_group_led_level_process_param->ucGroup1IDs,
		Set_group_led_level_process_param->ucGroup1Level);
	para_size = sizeof(struct ISPCMD_LED_LEVEL);
	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
			param, para_size);
	return err;
}

/*
 *\brief Set ctrl led dequence
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_ctrl_led_seq(
					void *devdata, u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size;
	struct ISPCMD_CTRL_LED_SEQ *Set_ctrl_led_seq_process_param;

	Set_ctrl_led_seq_process_param = (struct ISPCMD_CTRL_LED_SEQ *)param;
	misp_info("%s - enter", __func__);
	misp_info("[cycle]:%d, [Led_1]:%d,[Led_2]:%d,[Led_3]:%d,[Led_4]:%d\n",
		Set_ctrl_led_seq_process_param->ucCycle,
		Set_ctrl_led_seq_process_param->ucCycleLightCtrl[0],
		Set_ctrl_led_seq_process_param->ucCycleLightCtrl[1],
		Set_ctrl_led_seq_process_param->ucCycleLightCtrl[2],
		Set_ctrl_led_seq_process_param->ucCycleLightCtrl[3],
		Set_ctrl_led_seq_process_param->ucCycleLightCtrl[4]);
	misp_info("[Led_5]:%d,[Led_6]:%d,[Led_7]:%d,[Led_8]:%d\n",
		Set_ctrl_led_seq_process_param->ucCycleLightCtrl[5],
		Set_ctrl_led_seq_process_param->ucCycleLightCtrl[6],
		Set_ctrl_led_seq_process_param->ucCycleLightCtrl[7],
		Set_ctrl_led_seq_process_param->ucCycleLightCtrl[8]);
	misp_info("[Led_9]:%d,[Led_10]:%d,[Led_11]:%d,[Led_12]:%d\n",
		Set_ctrl_led_seq_process_param->ucCycleLightCtrl[9],
		Set_ctrl_led_seq_process_param->ucCycleLightCtrl[10],
		Set_ctrl_led_seq_process_param->ucCycleLightCtrl[11]);
	para_size = sizeof(struct ISPCMD_CTRL_LED_SEQ);
	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
			param, para_size);
	return err;
}

/*
 *\brief Host ctrl led onoff
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_host_ctrl_led_onoff(
					void *devdata, u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size;
	struct ISPCMD_CTRL_GPO *host_ctrl_led_onoff;

	host_ctrl_led_onoff = (struct ISPCMD_CTRL_GPO *)param;
	misp_info("%s - enter", __func__);
	misp_info("[GPO_0]:%d,[GPO_1]:%d,[GPO_2]:%d, [GPO_3]:%d,[GPO_4]:%d\n",
		host_ctrl_led_onoff->uwGPO_HL_BitField&0x0001,
		host_ctrl_led_onoff->uwGPO_HL_BitField&0x0002,
		host_ctrl_led_onoff->uwGPO_HL_BitField&0x0004,
		host_ctrl_led_onoff->uwGPO_HL_BitField&0x0008,
		host_ctrl_led_onoff->uwGPO_HL_BitField&0x0010);
	misp_info("[GPO_5]:%d,[GPO_6]:%d,[GPO_7]:%d,[GPO_8]:%d,[GPO_9]:%d\n",
		host_ctrl_led_onoff->uwGPO_HL_BitField&0x0020,
		host_ctrl_led_onoff->uwGPO_HL_BitField&0x0040,
		host_ctrl_led_onoff->uwGPO_HL_BitField&0x0080,
		host_ctrl_led_onoff->uwGPO_HL_BitField&0x0100,
		host_ctrl_led_onoff->uwGPO_HL_BitField&0x0200);
	misp_info("[GPO_10]:%d",
		host_ctrl_led_onoff->uwGPO_HL_BitField&0x0400);
	para_size = sizeof(struct ISPCMD_CTRL_GPO);
	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
			param, para_size);
	return err;
}

/*
 *\brief Extra RGB sensor sync
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_extra_sensor_sync(
					void *devdata, u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size = sizeof(u8);

	misp_info("%s - enter", __func__);
	misp_info("[ExtraSensorSync]: %d", *(u8 *) param);

	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
			param, para_size);
	return err;
}

errcode mast_camera_profile_cmd_set_depth_type(
					void *devdata, u16 opcode, u8 *param)
{
	/* Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size;
	struct ISPCMD_DEPTH_TYPE *set_depth_type;

	set_depth_type = (struct ISPCMD_DEPTH_TYPE *)param;
	misp_info("%s - enter", __func__);
	misp_info("[DepthType]:%d, [GroundType]:%d",
		set_depth_type->ucDepthType,
		set_depth_type->ucGroundType);

	para_size = sizeof(struct ISPCMD_DEPTH_TYPE);
	/* Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata, opcode,
			param, para_size);

	return err;
}


/************************** End Of File *******************************/
