/*
 * File: ispctrl_if_master_local.h
 * Description: The structure and API definition ISP Ctrl IF Master Local
 * It,s a header file that define structure and API for ISP Ctrl IF Master Local
 *
 * Copyright 2019-2030  Altek Semiconductor Corporation
 *
 *  2013/10/14; Aaron Chuang; Initial version
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


/**
 * \file	 ispctrl_if_master_local.h
 * \brief	ISP Ctrl IF Master Local and API define
 * \version  0.01
 * \author   Aaron Chuang
 * \date	 2013/09/23
 * \see	  ispctrl_if_master_local.h
 */

#ifndef _ISPCTRLIF_MASTER_LOCAL_H_
#define _ISPCTRLIF_MASTER_LOCAL_H_

/**
 *@addtogroup ispctrl_if_master_local
 *@{
 */


/******Include File******/

#include "mtype.h"
#include "miniisp.h"


/******Public Constant Definition******/
#define ISP_REGISTER_PARA_SIZE  8
#define ISP_REGISTER_VALUE  4


/******Public Function Prototype******/

/*********************** System manage command ***********************/
/**
 *\brief Get status of last executed command
 *\param devdata [In], misp device data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_sys_manage_cmd_get_status_of_last_exec_command(
						void *devdata,
						u16 opcode, u8 *param);

/**
 *\brief Get error code CMD
 *\param devdata [In], misp device data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_sys_manage_cmd_get_error_code_command(void *devdata,
								u16 opcode,
								u8 *param);

/**
 *\brief Set ISP register
 *\param devdata [In], misp device data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_sys_manage_cmd_set_isp_register(void *devdata,
								u16 opcode,
								u8 *param);

/**
 *\brief Get ISP register
 *\param devdata [In], misp device data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_sys_manage_cmd_get_isp_register(void *devdata,
								u16 opcode,
								u8 *param);

/**
 *\brief Set common log level
 *\param devdata [In], misp device data
 *\param opcode [In], Operation code
 *\param param [In], CMD param

 *\return Error code
 */
errcode mast_sys_manage_cmd_set_comomn_log_level(void *devdata,
								u16 opcode,
								u8 *param);

/**
 *\brief Get chip test report
 *\param  devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_sys_manage_cmd_get_chip_test_report(void *devdata,
								u16 opcode,
								u8 *param);

/**
 *\brief Get chip isp thermal
 *\param  devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_sys_manage_cmd_get_chip_thermal(void *devdata,
								u16 opcode,
								u8 *param);

/**
 *\brief Get peripheral device
 *\param  devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_sys_manage_cmd_get_peripheral_device(void *devdata,
								u16 opcode,
								u8 *param);
/**
 *\brief Set peripheral device
 *\param  devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_sys_manage_cmd_set_peripheral_device(void *devdata,
								u16 opcode,
								u8 *param);
/**
 *\brief Get HDR, IRP0, IRP1, Depth bin file version
 *\param  devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_sys_manage_cmd_get_binfile_ver(void *devdata,
								u16 opcode,
								u8 *param);

/**
 *\brief turn on Ocram 7
 *\param  devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_sys_manage_cmd_turnon_Ocram7(void *devdata,
								u16 opcode,
								u8 *param);


/*********************** Camera profile command ***********************/
/**
 *\brief Set Sensor Mode
 *\param devdata [In], misp device data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_sensor_mode(void *devdata,
								u16 opcode,
								u8 *param);

/**
 *\brief Get Sensor Mode
 *\param devdata [In], misp device data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_get_sensor_mode(void *devdata,
								u16 opcode,
								u8 *param);

/**
 *\brief Set Output Format
 *\param devdata [In], misp device data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_output_format(void *devdata,
								u16 opcode,
								u8 *param);

/*
 *\brief Set CP Mode
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_cp_mode(void *devdata,
						u16 opcode, u8 *param);

/*
 *\brief Set AE statistics
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_basic_setting_cmd_set_ae_statistics(void *devdata,
							u16 opcode, u8 *param);

/**
 *\brief Preview stream on
 *\param devdata [In], misp device data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_preview_stream_on_off(
								void *devdata,
								u16 opcode,
								u8 *param);

/*
 *\brief dual PD Y calculation weightings
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_dual_pd_y_cauculation_weightings(
						void *devdata,
						u16 opcode, u8 *param);


/*
 *\brief led power control
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_led_power_control(
						void *devdata,
						u16 opcode, u8 *param);


/*
 *\brief Active AE
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_active_ae(
						void *devdata,
						u16 opcode, u8 *param);

/*
 *\brief ISP AE control on off
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_isp_ae_control_on_off(
						void *devdata,
						u16 opcode, u8 *param);

/*
 *\brief Set Frame Rate Limits
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_frame_rate_limits(
						void *devdata,
						u16 opcode, u8 *param);

/*
 *\brief Set Period Drop Frame
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_period_drop_frame(
						void *devdata,
						u16 opcode, u8 *param);

/*
 *\brief Set max exposure
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_max_exposure(
						void *devdata,
						u16 opcode, u8 *param);

/*
 *\brief Set target mean
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_target_mean(
						void *devdata,
						u16 opcode, u8 *param);

/*
 *\brief Frame sync control
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_frame_sync_control(
						void *devdata,
						u16 opcode, u8 *param);

/*
 *\brief Set shot mode
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_shot_mode(
						void *devdata,
						u16 opcode, u8 *param);

/*
 *\brief Lighting control
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_lighting_ctrl(
						void *devdata,
						u16 opcode, u8 *param);

/*
 *\brief Set min exposure
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_min_exposure(
						void *devdata,
						u16 opcode, u8 *param);

/*
 *\brief Set Max exposure slope
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_max_exposure_slope(
						void *devdata,
						u16 opcode, u8 *param);

/*
 *\brief Set depth compensation
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_depth_compensation(
						void *devdata,
						u16 opcode, u8 *param);

/*
 *\brief Set cycle trigger depth process
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_cycle_trigger_depth(
						void *devdata,
						u16 opcode, u8 *param);

/*
 *\brief Set led active delay time
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_led_active_delay(
						void *devdata,
						u16 opcode, u8 *param);


/*
 *\brief Set isp control led level
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_isp_smart_ae_control(
						void *devdata,
						u16 opcode, u8 *param);

/*
 *\brief Set isp control led level
 *\param ucType  [In],      0: Projector 1: Flood
 *\param ucGroup0IDs {In],	bit 0: Set Light 0, 1: change Light 0 level
 *\bit 1: Set Light 1, 1: change Light 1 level
 *\param ucGroup0Level{in],	Light level for Group 0, 0~255
 *\param ucGroup0IDs {In],	bit 0: Set Light 0, 1: change Light 0 level
 *\bit 1: Set Light 1, 1: change Light 1 level
 *\param ucGroup0Level{in],	Light level for Group 0, 0~255
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_group_led_level(
						void *devdata,
						u16 opcode, u8 *param);

/**
 *\brief Set control led sequence	0x3023
 *\param ucCycle [In], 0: disable , 1~12: cycle, 13~255: 12 cycles
 *\param ucCycleLightCtrl[12] {In],	bit 0: Projector, bit 1: Flood,
 *\array [0] for 1st frame of Cycle N, array[M] (M+1)th frame of Cycle N
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_ctrl_led_seq(
						void *devdata,
						u16 opcode, u8 *param);

/**
 *\brief Set control led sequence	0x3026
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_host_ctrl_led_onoff(
						void *devdata,
						u16 opcode, u8 *param);

/**
 *\brief extra sensor sync	0x3027
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_extra_sensor_sync(
						void *devdata,
						u16 opcode, u8 *param);

/**
 *\brief Set depth type	0x3028
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_camera_profile_cmd_set_depth_type(
						void *devdata,
						u16 opcode, u8 *param);

/*
 *\brief Get isp thermal
 *\param None
 *\return Error code
 */
errcode mast_camera_profile_cmd_get_chip_thermal(
						void *devdata,
						u16 opcode, u8 *param);


/* Bulk data command*/
/**
 *\brief Write Boot Code
 *\param devdata [In], misp device data
 *\param param [In], CMD param
 *\param filp [In], boot file
 *\return Error code
 */
errcode mast_bulk_data_cmd_write_boot_code(void *devdata,
							u8 *param);


/**
 *\brief Write Boot Code (Short SPI CMD)
 *\param devdata [In], misp device data
 *\param param [In], CMD param
 *\param filp [In], boot file
 *\return Error code
 */
errcode mast_bulk_data_cmd_write_boot_code_shortlen(
				void *devdata, u8 *param);

/**
 *\brief Write Basic Code
 *\param devdata [In], misp device data
 *\param param [In], CMD param
 *\param filp [In], basic file
 *\return Error code
 */
errcode mast_bulk_data_cmd_write_basic_code(void *devdata,
					u8 *param);

/**
 *\brief Write Basic Code
 *\param devdata [In], misp device data
 *\param param [In], CMD param
 *\param filp [In], basic file
 *\return Error code
 */
errcode mast_bulk_data_cmd_write_basic_code_shortlen(void *devdata,
						u8 *param);

/**
 *\brief Write Calibration Data
 *\param devdata [In], misp device data
 *\param param [In], CMD param
 *\param filp [In], sc table file
 *\return Error code
 */
errcode mast_bulk_data_cmd_write_calibration_data(void *devdata,
						u8 *param);

/**
 *\brief Write Qmerge Data
 *\param devdata [In], misp device data
 *\param param [In], CMD param
 *\param filp [In], sc table file
 *\return Error code
 */
errcode mast_bulk_data_cmd_write_qmerge_data(void *devdata,
								u8 *param,
							struct file *filp);

/**
 *\brief Read Calibration Data
 *\param devdata [In], misp device data
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_bulk_data_cmd_read_calibration_data(void *devdata,
							u8 *param);
/**
 *\brief Write Memory Data
 *\param devdata [In], misp device data
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_bulk_data_cmd_write_memory_data(void *devdata,
							u8 *param);

/**
 *\brief Read Memory Data
 *\param devdata [In], misp device data
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_bulk_data_cmd_read_memory_data(void *devdata,
							u8 *param);

/**
 *\brief Read common log data
 *\param devdata [In], misp device data
 *\param param [In], CMD param
 *\return Error code
 */
errcode bulk_data_cmd_read_common_log(void *devdata,
							u8 *param);

/*
 *\brief Read depth Rect A, B, Invrect
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_bulk_data_cmd_write_depth_rectab_invrect(
	void *devdata, u16 opcode, u8 *param);

/**
 *\brief Write Spinor Data
 *\param devdata [In], misp device data
 *\param param [In], CMD param
 *\param filp [In], source file
 *\return Error code
 */
errcode mast_bulk_data_cmd_write_spinor_data(void *devdata,
					u8 *param);

/**
 *\brief get binfile version
 *\param devdata [In], misp device data
 *\param param [In], CMD param
 *\param filp [In], source file
 *\return Error code
 */
errcode mast_bulk_data_cmd_get_binfile_version(void *devdata, u8 *param);

/*********************** Basic setting command ************************/
/**
 *\brief Set Depth 3A Info
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_basic_setting_cmd_set_depth_3a_info(void *devdata,
						u16 opcode,
						u8 *param);


/*
 *\brief Set Depth auto interleave mode
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_basic_setting_cmd_set_depth_auto_interleave_mode(
	void *devdata, u16 opcode, u8 *param);


/*
 *\brief Set projector interleave mode with depth type
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_basic_setting_cmd_set_interleave_mode_depth_type(
	void *devdata, u16 opcode, u8 *param);

/*
 *\brief Set depth polish level
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_basic_setting_cmd_set_depth_polish_level(
	void *devdata, u16 opcode, u8 *param);

/*
 *\brief Set exposure param
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_basic_setting_cmd_set_exposure_param(
	void *devdata, u16 opcode, u8 *param);

/*
 *\brief Set depth stream size
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_basic_setting_cmd_set_depth_stream_size(
	void *devdata, u16 opcode, u8 *param);
/*********************** operation command ***********************/
/**
 *\brief Mini ISP open
 *\param devdata [In], misp device data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_operation_cmd_miniisp_open(
	void *devdata, u16 opcode, char *FileNametbl[]);


/**
 *\brief calculate check sum
 *\param input_buffer_addr [In], input buffer address
 *\param input_buffer_size [In], input buffer size
 *\return Error code
 */
u16 calculate_check_sum(const u8 *input_buffer_addr, u32 input_buffer_size);
/****************************** End of File *********************************/

/**
 *@}
 */

#endif /* _ISPCTRLIF_MASTER_LOCAL_H_*/
