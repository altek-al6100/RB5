/*
 * File: miniisp_spi.c
 * Description: Mini ISP sample codes
 *
 * Copyright 2019-2030  Altek Semiconductor Corporation
 *
 * 2017/04/11; LouisWang; Initial version
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



/************************************************************
 *\          Include File               *
 *\************************************************************/
/* Linux headers*/
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/buffer_head.h>
#include <linux/of_gpio.h>
#include <linux/firmware.h>
#include <linux/semaphore.h>

#include "include/miniisp.h"
#include "include/miniisp_ctrl.h"
#include "include/miniisp_customer_define.h"
#include "include/miniisp_chip_base_define.h"
#include "include/altek_statefsm.h"
#include "include/ispctrl_if_master.h"
#include "include/isp_camera_cmd.h"
#include "include/error/miniisp_err.h"

#if EN_REGRESSION_TEST
#include "../al6100_regression/miniisp_regression_test.h"
#endif

#if EN_605_IOCTRL_INTF
/* ALTEK_AL6100_ECHO >>> */
#include "include/miniisp_ctrl_intf.h"
/* ALTEK_AL6100_ECHO <<< */
#endif

#ifdef ALTEK_TEST
#include "include/altek_test.h"
#endif

/****************************************************************************
 *\		 Private Constant Definition				*
 *\***************************************************************************/
#define DEBUG_NODE 0
/*#define DEBUG_ALERT*/
#define MINI_ISP_LOG_TAG "[miniisp_isp]"
/*drv debug defination*/
#define _SPI_DEBUG

/****************************************************************************
 *\        Private Global Variable          *
 *\***************************************************************************/
static struct misp_global_variable *misp_drv_global_variable;
static struct class *mini_isp_class;
static struct device *mini_isp_dev;
struct altek_statefsm *altek_state;
bool IsMiniispSetupInitialed;

#if EN_605_IOCTRL_INTF
/* ALTEK_AL6100_ECHO >>> */
struct file *l_internal_file[ECHO_OTHER_MAX];
/* ALTEK_AL6100_ECHO <<< */
#endif

/************************************************************
 *		Public Global Variable
 *************************************************************/

/************************************************************
 *		Private Macro Definition
 *************************************************************/

/************************************************************
 *		Public Function Prototype
 *************************************************************/


/************************************************************
 *		Private Function
 *************************************************************/
#if EN_605_IOCTRL_INTF
void mini_isp_other_drv_open_l(char *file_name, u8 type)
{

	/* Error Code*/
	errcode err = ERR_SUCCESS;

	misp_info("%s filepath : %s", __func__, file_name);

#if ENABLE_FILP_OPEN_API
	/*l_internal_file[type] = filp_open(file_name, O_RDONLY, 0644);*/
#else
	misp_info("Error! Currently not support file open api");
	misp_info("See define ENABLE_FILP_OPEN_API");
	return;
#endif
	if (IS_ERR(l_internal_file[type])) {
		err = PTR_ERR(l_internal_file[type]);
		misp_err("%s open file failed. err: %x", __func__, err);
	} else {
		misp_info("%s open file success!", __func__);
	}
}
#endif
static ssize_t mini_isp_mode_config_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/*misp_info("%s - mini_isp_spi_send return %d", __func__, ret);*/
	return snprintf(buf, 32, "load fw:0 e_to_a:1 a_to_e:2\n");
}

static ssize_t mini_isp_mode_config_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	u8 buf_chip_id_use[4];

	if ('0' == buf[0]) {
		mini_isp_chip_init();
		mini_isp_e_to_a();
		mini_isp_drv_load_fw();
	} else if ('1' == buf[0]) {
		mini_isp_chip_init();
		mini_isp_e_to_a();
	} else if ('2' == buf[0]) {
		mini_isp_a_to_e();
	} else if ('4' == buf[0]) {
		mini_isp_get_chip_id();
	} else if ('7' == buf[0]) {
		*(u32 *)buf_chip_id_use = 0;
		mini_isp_debug_dump_img();
		mini_isp_a_to_e();
		mini_isp_chip_base_dump_irp_and_depth_based_register();
		mini_isp_memory_write(0x10, buf_chip_id_use, 4);
		mini_isp_e_to_a();
	}  else {
		mini_isp_poweron();
		mini_isp_drv_set_bypass_mode(1);
	}
	return size;
}

static DEVICE_ATTR(mini_isp_mode_config, 0660, mini_isp_mode_config_show,
		mini_isp_mode_config_store);

static ssize_t mini_isp_reset_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret = -EINVAL;

	ret = gpio_get_value(misp_drv_global_variable->reset_gpio);
	misp_info("%s - reset_gpio is %d", __func__, ret);

	return snprintf(buf, 32, "%d", ret);
}

static ssize_t mini_isp_reset_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	if ('0' == buf[0]) {
		gpio_direction_output(misp_drv_global_variable->reset_gpio, 0);
		gpio_set_value(misp_drv_global_variable->reset_gpio, 0);
	} else {
		gpio_direction_output(misp_drv_global_variable->reset_gpio, 1);
		gpio_set_value(misp_drv_global_variable->reset_gpio, 1);
	}
	misp_info("%s - ", __func__);

	return size;
}

static DEVICE_ATTR(mini_isp_reset, 0660,
					mini_isp_reset_show,
					mini_isp_reset_store);


static ssize_t mini_isp_rectab_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 32, "Set rectab Param!!\n");
}

static ssize_t mini_isp_rectab_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	u8 trans_mode;
	u32 block_size;
	struct depth_rectab_invrect_param rect_param[3];

	misp_info("Set rectab start!!");

	/* fill test pattern */
	memset((u8 *)&rect_param[0], 0xa,
		3*sizeof(struct depth_rectab_invrect_param));

	trans_mode = 0;
	block_size = 64;

	mini_isp_drv_write_depth_rectab_invrect(
		&rect_param[0], trans_mode, block_size);

	misp_info("Set rectab end!!");
	return size;
}

static DEVICE_ATTR(mini_isp_rectab, 0660,
					mini_isp_rectab_show,
					mini_isp_rectab_store);

ssize_t echo_mini_isp_drv_get_peripheral_device(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret[2];
	int t_ret;
	char cmd_name[20];
	char str_param[2][20];
	struct isp_cmd_peripheral_info peripheral_info;

	memset(&peripheral_info, 0, sizeof(struct isp_cmd_peripheral_info));
	misp_info("%s S!!", __func__);

	t_ret = sscanf(cmd_buf, "%s %s %s",
	cmd_name, &str_param[0][0], &str_param[1][0]);
	ret[0] = t_ret;
	if (ret != 3) {
		errcode = -EINVAL;
		return errcode;
	}

	/* str to u32*/
	ret[0] = kstrtou8(&str_param[0][0], 0, &peripheral_info.ucDevId);
	ret[1] = kstrtou16(&str_param[1][0], 0, &peripheral_info.uwRegAddr);
	if (ret[0] != ERR_SUCCESS)
		return ret[0];
	else if (ret[1] != ERR_SUCCESS)
		return ret[1];

	peripheral_info.uwRegData = 0; //(u16)param[2];

	misp_info("%d 0x%x",
		peripheral_info.ucDevId,
		peripheral_info.uwRegAddr);

	errcode = mini_isp_drv_get_peripheral_device(&peripheral_info);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_set_peripheral_device(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret[4];
	int t_ret;
	char cmd_name[20];
	char str_param[4][20];
	u8 verify;
	struct isp_cmd_peripheral_info peripheral_info;
	struct isp_cmd_peripheral_info peripheral_info_verify;

	memset(&peripheral_info, 0, sizeof(struct isp_cmd_peripheral_info));
	misp_info("%s S!!", __func__);

	t_ret = sscanf(cmd_buf, "%s %s %s %s %s", cmd_name,
		&str_param[0][0], &str_param[1][0], &str_param[2][0],
		&str_param[3][0]);
	ret[0] = t_ret;
	if (ret != 5) {
		errcode = -EINVAL;
		misp_info("The number of parameters is not correct(%d)"
			, ret[0]);
		return errcode;
	}

	/* str to u32*/
	ret[0] = kstrtou8(&str_param[0][0], 0, &peripheral_info.ucDevId);
	ret[1] = kstrtou16(&str_param[1][0], 0, &peripheral_info.uwRegAddr);
	ret[2] = kstrtou16(&str_param[2][0], 0, &peripheral_info.uwRegData);
	ret[3] = kstrtou8(&str_param[3][0], 0, &verify);
	if (ret[0] != ERR_SUCCESS)
		return ret[0];
	else if (ret[1] != ERR_SUCCESS)
		return ret[1];
	else if (ret[2] != ERR_SUCCESS)
		return ret[2];
	else if (ret[3] != ERR_SUCCESS)
		return ret[3];

	misp_info("%d 0x%x 0x%x",
		peripheral_info.ucDevId,
		peripheral_info.uwRegAddr,
		peripheral_info.uwRegData);

	errcode = mini_isp_drv_set_peripheral_device(&peripheral_info);

	if (verify && ERR_SUCCESS == errcode) {
		peripheral_info_verify.ucDevId = peripheral_info.ucDevId;
		peripheral_info_verify.uwRegAddr = peripheral_info.uwRegAddr;
		peripheral_info_verify.uwRegData = ~peripheral_info.uwRegData;
		errcode = mini_isp_drv_get_peripheral_device(
			&peripheral_info_verify);
		if (errcode == ERR_SUCCESS) {
			if (peripheral_info.uwRegData ==
				peripheral_info_verify.uwRegData)
				misp_info("verify OK");
			else {
				misp_info("verify NG");
				errcode = -EINVAL;
			}
		}
	}
	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_set_depth_3a_info(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret[26];
	int t_ret;
	char cmd_name[20];
	char str_param[26][20];
	u32 i;
	struct isp_cmd_depth_3a_info depth_3a_info;

	memset(&depth_3a_info, 0, sizeof(struct isp_cmd_depth_3a_info));
	misp_info("%s S!!", __func__);

	t_ret = sscanf(cmd_buf, "%s %s %s %s %s %s %s %s %s %s \
		%s%s %s %s %s %s %s %s %s %s %s%s %s %s %s %s %s",
	cmd_name,
	&str_param[0][0], &str_param[1][0], &str_param[2][0],
	&str_param[3][0], &str_param[4][0], &str_param[5][0],
	&str_param[6][0], &str_param[7][0], &str_param[8][0],
	&str_param[9][0], &str_param[10][0], &str_param[11][0],
	&str_param[12][0], &str_param[13][0], &str_param[14][0],
	&str_param[15][0], &str_param[16][0], &str_param[17][0],
	&str_param[18][0], &str_param[19][0], &str_param[20][0],
	&str_param[21][0], &str_param[22][0], &str_param[23][0],
	&str_param[24][0], &str_param[25][0]);
	ret[0] = t_ret;
	if (t_ret != 27) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u32*/
	ret[0] = kstrtou16(&str_param[0][0], 0,
		&depth_3a_info.hdr_ratio);
	ret[1] = kstrtou32(&str_param[1][0], 0,
		&depth_3a_info.main_cam_exp_time);
	ret[2] = kstrtou16(&str_param[2][0], 0,
		&depth_3a_info.main_cam_exp_gain);
	ret[3] = kstrtou16(&str_param[3][0], 0,
		&depth_3a_info.main_cam_amb_r_gain);
	ret[4] = kstrtou16(&str_param[4][0], 0,
		&depth_3a_info.main_cam_amb_g_gain);

	ret[5] = kstrtou16(&str_param[5][0], 0,
		&depth_3a_info.main_cam_amb_b_gain);
	ret[6] = kstrtou16(&str_param[6][0], 0,
		&depth_3a_info.main_cam_iso);
	ret[7] = kstrtou16(&str_param[7][0], 0,
		&depth_3a_info.main_cam_bv);
	ret[8] = kstrtos16(&str_param[8][0], 0,
		&depth_3a_info.main_cam_vcm_position);
	ret[9] = kstrtou8(&str_param[9][0], 0,
		&depth_3a_info.main_cam_vcm_status);

	ret[10] = kstrtou32(&str_param[10][0], 0,
		&depth_3a_info.sub_cam_exp_time);
	ret[11] = kstrtou16(&str_param[11][0], 0,
		&depth_3a_info.sub_cam_exp_gain);
	ret[12] = kstrtou16(&str_param[12][0], 0,
		&depth_3a_info.sub_cam_amb_r_gain);
	ret[13] = kstrtou16(&str_param[13][0], 0,
		&depth_3a_info.sub_cam_amb_g_gain);
	ret[14] = kstrtou16(&str_param[14][0], 0,
		&depth_3a_info.sub_cam_amb_b_gain);

	ret[15] = kstrtou16(&str_param[15][0], 0,
		&depth_3a_info.sub_cam_iso);
	ret[16] = kstrtou16(&str_param[16][0], 0,
		&depth_3a_info.sub_cam_bv);
	ret[17] = kstrtos16(&str_param[17][0], 0,
		&depth_3a_info.sub_cam_vcm_position);
	ret[18] = kstrtou8(&str_param[18][0], 0,
		&depth_3a_info.sub_cam_vcm_status);
	ret[19] = kstrtou16(&str_param[19][0], 0,
		&depth_3a_info.main_cam_isp_d_gain);

	ret[20] = kstrtou16(&str_param[20][0], 0,
		&depth_3a_info.sub_cam_isp_d_gain);
	ret[21] = kstrtos16(&str_param[21][0], 0,
		&depth_3a_info.hdr_long_exp_ev_x1000);
	ret[22] = kstrtos16(&str_param[22][0], 0,
		&depth_3a_info.hdr_short_exp_ev_x1000);
	ret[23] = kstrtou16(&str_param[23][0], 0,
		&depth_3a_info.ghost_prevent_low);
	ret[24] = kstrtou16(&str_param[24][0], 0,
		&depth_3a_info.ghost_prevent_high);
	ret[25] = kstrtou8(&str_param[25][0], 0,
		&depth_3a_info.depth_proc_mode);

	for (i = 0; i < 26; i++)
		if (ret[i] != ERR_SUCCESS)
			return ret[i];

	misp_info("%hu %u %hu %hu %hu %hu %hu %hu %hd %hhu %u %hu %hu %hu %hu %hu %hu %hd %hhu %hu %hu %hd %hd %hu %hu %hhu",
		depth_3a_info.hdr_ratio,
		depth_3a_info.main_cam_exp_time,
		depth_3a_info.main_cam_exp_gain,
		depth_3a_info.main_cam_amb_r_gain,
		depth_3a_info.main_cam_amb_g_gain,
		depth_3a_info.main_cam_amb_b_gain,
		depth_3a_info.main_cam_iso,
		depth_3a_info.main_cam_bv,
		depth_3a_info.main_cam_vcm_position,
		depth_3a_info.main_cam_vcm_status,
		depth_3a_info.sub_cam_exp_time,
		depth_3a_info.sub_cam_exp_gain,
		depth_3a_info.sub_cam_amb_r_gain,
		depth_3a_info.sub_cam_amb_g_gain,
		depth_3a_info.sub_cam_amb_b_gain,
		depth_3a_info.sub_cam_iso,
		depth_3a_info.sub_cam_bv,
		depth_3a_info.sub_cam_vcm_position,
		depth_3a_info.sub_cam_vcm_status,
		depth_3a_info.main_cam_isp_d_gain,
		depth_3a_info.sub_cam_isp_d_gain,
		depth_3a_info.hdr_long_exp_ev_x1000,
		depth_3a_info.hdr_short_exp_ev_x1000,
		depth_3a_info.ghost_prevent_low,
		depth_3a_info.ghost_prevent_high,
		depth_3a_info.depth_proc_mode);

	errcode = mini_isp_drv_set_depth_3a_info(&depth_3a_info);
	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_get_last_exec_cmd(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;

	misp_info("%s S!!", __func__);

	errcode = mini_isp_drv_get_last_exec_cmd();

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_get_binfile_ver(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;

	misp_info("%s S!!", __func__);

	errcode = mini_isp_drv_get_binfile_ver();

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_set_depth_auto_interleave_mode(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret[4];
	int t_ret;
	char cmd_name[20];
	char str_param[4][20];
	struct isp_cmd_depth_auto_interleave_param depth_auto_interleave_param;

	memset(&depth_auto_interleave_param, 0,
		sizeof(struct isp_cmd_depth_auto_interleave_param));
	misp_info("%s S!!", __func__);

	t_ret = sscanf(cmd_buf, "%s %s %s %s %s", cmd_name,
		&str_param[0][0], &str_param[1][0],
		&str_param[2][0], &str_param[3][0]);
	ret[0] = t_ret;
	if (ret != 5) {
		errcode = -EINVAL;
		return errcode;
	}

	/* str to u8*/
	ret[0] = kstrtou8(&str_param[0][0], 0,
		&depth_auto_interleave_param.depth_interleave_mode_on_off);
	ret[1] = kstrtou8(&str_param[1][0], 0,
	&depth_auto_interleave_param.skip_frame_num_after_illuminator_pulse);
	ret[2] = kstrtou8(&str_param[2][0], 0,
		&depth_auto_interleave_param.projector_power_level);
	ret[3] = kstrtou8(&str_param[3][0], 0,
		&depth_auto_interleave_param.illuminator_power_level);
	if (ret[0] != ERR_SUCCESS)
		return ret[0];
	else if (ret[1] != ERR_SUCCESS)
		return ret[1];
	else if (ret[2] != ERR_SUCCESS)
		return ret[2];
	else if (ret[3] != ERR_SUCCESS)
		return ret[3];

	misp_info("%d, %d, %d, %d",
	depth_auto_interleave_param.depth_interleave_mode_on_off,
	depth_auto_interleave_param.skip_frame_num_after_illuminator_pulse,
	depth_auto_interleave_param.projector_power_level,
	depth_auto_interleave_param.illuminator_power_level);

	errcode = mini_isp_drv_set_depth_auto_interleave_mode(
		&depth_auto_interleave_param);
	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_set_exposure_param(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret[3];
	int t_ret;
	char cmd_name[20];
	char str_param[3][20];

	struct isp_cmd_exposure_param set_exposure_param;

	memset(&set_exposure_param, 0, sizeof(struct isp_cmd_exposure_param));
	misp_info("%s S!!", __func__);

	t_ret = sscanf(cmd_buf, "%s %s %s %s", cmd_name,
		&str_param[0][0], &str_param[1][0], &str_param[2][0]);
	ret[0] = t_ret;
	if (ret != 4) {
		errcode = -EINVAL;
		return errcode;
	}

	/* str to u32*/
	ret[0] = kstrtou32(&str_param[0][0], 0,
		&set_exposure_param.udExpTime);
	ret[1] = kstrtou16(&str_param[1][0], 0,
		&set_exposure_param.uwISO);
	ret[2] = kstrtou8(&str_param[2][0], 0,
		&set_exposure_param.ucActiveDevice);
	if (ret[0] != ERR_SUCCESS)
		return ret[0];
	else if (ret[1] != ERR_SUCCESS)
		return ret[1];
	else if (ret[2] != ERR_SUCCESS)
		return ret[2];

	misp_info("menu exposure param: %d %d %d",
		set_exposure_param.udExpTime,
		set_exposure_param.uwISO,
		set_exposure_param.ucActiveDevice);

	errcode = mini_isp_drv_set_exposure_param(&set_exposure_param);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_set_depth_stream_size(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret;
	char cmd_name[20];
	char str_param[20];

	struct isp_cmd_depth_stream_size depth_stream_size_param;

	memset(&depth_stream_size_param, 0,
		sizeof(struct isp_cmd_depth_stream_size));
	misp_info("%s S!!", __func__);

	ret = sscanf(cmd_buf, "%s %s", cmd_name, &str_param[0]);
	if (ret != 2) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8*/
	errcode = kstrtou8(&str_param[0], 0,
		&depth_stream_size_param.depth_stream_size);
	if (errcode != ERR_SUCCESS)
		return errcode;

	misp_info("depth stream size: %d",
		depth_stream_size_param.depth_stream_size);

	errcode = mini_isp_drv_set_depth_stream_size(
		&depth_stream_size_param);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_bulk_get_binfile_ver(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;

	misp_info("%s S!!", __func__);
	errcode = mini_isp_drv_bulk_get_binfile_ver();
	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_get_sensor_mode(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;

	misp_info("%s S!!", __func__);

	errcode = mini_isp_drv_get_sensor_mode();

	misp_info("%s E!!", __func__);
	return errcode;
}



ssize_t echo_mini_isp_drv_set_sensor_mode(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret[5];
	int t_ret;
	char cmd_name[20];
	char str_param[5][20];
	u8 sensor_on_off = 0;
	u8 scenario_id = 0;
	u8 mipi_tx_skew_enable = 0;
	u8 ae_weighting_table_index = 0;
	u8 merge_mode_enable = 0;

	misp_info("%s S!!", __func__);

	t_ret = sscanf(cmd_buf, "%s %s %s %s %s %s", cmd_name,
		&str_param[0][0], &str_param[1][0], &str_param[2][0],
		&str_param[3][0], &str_param[4][0]);
	ret[0] = t_ret;
	if (ret != 6) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8*/
	ret[0] = kstrtou8(&str_param[0][0], 0, &sensor_on_off);
	ret[1] = kstrtou8(&str_param[1][0], 0, &scenario_id);
	ret[2] = kstrtou8(&str_param[2][0], 0, &mipi_tx_skew_enable);
	ret[3] = kstrtou8(&str_param[3][0], 0, &ae_weighting_table_index);
	ret[4] = kstrtou8(&str_param[4][0], 0, &merge_mode_enable);
	if (ret[0] != ERR_SUCCESS)
		return ret[0];
	else if (ret[1] != ERR_SUCCESS)
		return ret[1];
	else if (ret[2] != ERR_SUCCESS)
		return ret[2];
	else if (ret[3] != ERR_SUCCESS)
		return ret[3];
	else if (ret[4] != ERR_SUCCESS)
		return ret[4];

	misp_info("0x%x, 0x%x, 0x%x, 0x%x, 0x%x",
		sensor_on_off, scenario_id, mipi_tx_skew_enable,
		ae_weighting_table_index, merge_mode_enable);

	errcode = mini_isp_drv_set_sensor_mode(sensor_on_off, scenario_id,
		mipi_tx_skew_enable, ae_weighting_table_index,
		merge_mode_enable);
	misp_info("%s E!!", __func__);
	return errcode;
}


ssize_t echo_mini_isp_drv_set_output_format(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret[3];
	int t_ret;
	char cmd_name[20];
	char str_param[3][20];
	struct isp_cmd_set_output_format output_format_param;

	memset(&output_format_param, 0,
		sizeof(struct isp_cmd_set_output_format));
	misp_info("%s S!!", __func__);

	t_ret = sscanf(cmd_buf, "%s %s %s %s", cmd_name,
		&str_param[0][0], &str_param[1][0], &str_param[2][0]);
	ret[0] = t_ret;
	if (ret != 4) {
		errcode = -EINVAL;
		return errcode;
	}

	/* str to u8*/
	ret[0] = kstrtou8(&str_param[0][0], 0,
		&output_format_param.depth_size);
	ret[1] = kstrtou8(&str_param[1][0], 0,
		&output_format_param.reserve[0]);
	ret[2] = kstrtou8(&str_param[2][0], 0,
		&output_format_param.reserve[1]);
	if (ret[0] != ERR_SUCCESS)
		return ret[0];
	else if (ret[1] != ERR_SUCCESS)
		return ret[1];
	else if (ret[2] != ERR_SUCCESS)
		return ret[2];

	misp_info("0x%x, 0x%x, 0x%x",
		output_format_param.depth_size,
		output_format_param.reserve[0],
		output_format_param.reserve[1]);

	errcode = mini_isp_drv_set_output_format(&output_format_param);
	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_set_cp_mode(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;

	misp_info("%s S!!", __func__);
	errcode = mini_isp_drv_set_cp_mode();
	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_leave_cp_mode(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret[5];
	int t_ret;
	char cmd_name[20];
	char str_param[5][20];
	u8 sensor_on_off;
	u8 scenario_id;
	u8 mipi_tx_skew_enable;
	u8 ae_weighting_table_index;
	u8 merge_mode_enable;

	misp_info("%s S!!", __func__);
	t_ret = sscanf(cmd_buf, "%s %s %s %s %s %s",
			cmd_name, &str_param[0][0], &str_param[1][0],
			&str_param[2][0], &str_param[3][0], &str_param[4][0]);
	ret[0] = t_ret;
	if (ret != 6) {
		errcode = -EINVAL;
		return errcode;
	}

	/* str to u8*/
	ret[0] = kstrtou8(&str_param[0][0], 0, &sensor_on_off);
	ret[1] = kstrtou8(&str_param[1][0], 0, &scenario_id);
	ret[2] = kstrtou8(&str_param[2][0], 0, &mipi_tx_skew_enable);
	ret[3] = kstrtou8(&str_param[3][0], 0, &ae_weighting_table_index);
	ret[4] = kstrtou8(&str_param[4][0], 0, &merge_mode_enable);
	if (ret[0] != ERR_SUCCESS)
		return ret[0];
	else if (ret[1] != ERR_SUCCESS)
		return ret[1];
	else if (ret[2] != ERR_SUCCESS)
		return ret[2];
	else if (ret[3] != ERR_SUCCESS)
		return ret[3];
	else if (ret[4] != ERR_SUCCESS)
		return ret[4];

	misp_info("0x%x, 0x%x, 0x%x, 0x%x, 0x%x",
		sensor_on_off, scenario_id, mipi_tx_skew_enable,
		ae_weighting_table_index, merge_mode_enable);

	errcode = mini_isp_drv_leave_cp_mode(
		sensor_on_off, scenario_id, mipi_tx_skew_enable,
		ae_weighting_table_index, merge_mode_enable);
	misp_info("%s E!!", __func__);
	return errcode;
}
ssize_t echo_mini_isp_drv_preview_stream_on_off(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int param_num;
	int ret[10];
	char cmd_name[20];
	char str_param[10][20];
	u8 tx0_stream_on_off;
	u8 tx1_stream_on_off;
	u8 reserve;
	u16 timedelay_ms;
	u16 tx_a0_count;
	u16 tx_d0_count;
	u16 tx_c0_count;
	u32 tx_a0_reg;
	u32 tx_c0_reg;
	u32 tx_d0_reg;
	u32 reg_val;

	misp_info("%s S!!", __func__);

	param_num = sscanf(cmd_buf, "%s %s %s %s %s %s %s %s %s %s %s",
		cmd_name,
		&str_param[0][0], &str_param[1][0], &str_param[2][0],
		&str_param[3][0], &str_param[4][0], &str_param[5][0],
		&str_param[6][0], &str_param[7][0], &str_param[8][0],
		&str_param[9][0]);
	if (param_num != 11) {
		param_num = sscanf(cmd_buf, "%s %s %s %s", cmd_name,
			&str_param[0][0], &str_param[1][0], &str_param[2][0]);
		if (param_num != 4) {
			errcode = -EINVAL;
			return errcode;
		}
	}
	/* str to u8*/
	ret[0] = kstrtou8(&str_param[0][0], 0, &tx0_stream_on_off);
	ret[1] = kstrtou8(&str_param[1][0], 0, &tx1_stream_on_off);
	ret[2] = kstrtou8(&str_param[2][0], 0, &reserve);
	if (ret[0] != ERR_SUCCESS)
		return ret[0];
	else if (ret[1] != ERR_SUCCESS)
		return ret[1];
	else if (ret[2] != ERR_SUCCESS)
		return ret[2];

	misp_info("0x%x, 0x%x, 0x%x",
		tx0_stream_on_off, tx1_stream_on_off, reserve);
	errcode = mini_isp_drv_preview_stream_on_off(
		tx0_stream_on_off, tx1_stream_on_off, reserve);
	/* Confirm AL6100 tx a0,tx c0,tx d0 frame count*/
	if (param_num == 11 && ERR_SUCCESS == errcode) {
		/* str to u16*/
		ret[3] = kstrtou16(&str_param[3][0], 0, &timedelay_ms);
		ret[4] = kstrtou16(&str_param[4][0], 0, &tx_a0_count);
		ret[5] = kstrtou16(&str_param[5][0], 0, &tx_c0_count);
		ret[6] = kstrtou16(&str_param[6][0], 0, &tx_d0_count);
		ret[7] = kstrtou32(&str_param[7][0], 0, &tx_a0_reg);
		ret[8] = kstrtou32(&str_param[8][0], 0, &tx_c0_reg);
		ret[9] = kstrtou32(&str_param[9][0], 0, &tx_d0_reg);
		if (ret[3] != ERR_SUCCESS)
			return ret[3];
		else if (ret[4] != ERR_SUCCESS)
			return ret[4];
		else if (ret[5] != ERR_SUCCESS)
			return ret[5];
		else if (ret[6] != ERR_SUCCESS)
			return ret[6];
		else if (ret[7] != ERR_SUCCESS)
			return ret[7];
		else if (ret[8] != ERR_SUCCESS)
			return ret[8];
		else if (ret[9] != ERR_SUCCESS)
			return ret[9];
		misp_info("Criterion: Delay= %d ms, tx a0 count= %d ,tx c0 count= %d",
				timedelay_ms, tx_a0_count, tx_c0_count);
		misp_info("tx d0 count= %d,tx a0 reg= %d tx c0 reg= %d,tx_d0_req= %d",
				tx_d0_count, tx_a0_reg, tx_c0_reg, tx_d0_reg);
		if (timedelay_ms)
			msleep(timedelay_ms);

		// switch to E mode
		mini_isp_a_to_e();
		// tx a0
		mini_isp_register_read(tx_a0_reg, &reg_val);
		misp_info("tx a0 count = %d", reg_val);
		if (tx_a0_count > reg_val) {
			misp_info("tx a0 count is out of criterion");
			errcode = -EINVAL;
			// switch to A mode
			mini_isp_e_to_a();
			goto ERR;
		}
		// tx c0
		mini_isp_register_read(tx_c0_reg, &reg_val);
		misp_info("tx c0 count = %d", reg_val);
		if (tx_c0_count > reg_val) {
			misp_info("tx c0 count is out of criterion");
			errcode = -EINVAL;
			// switch to A mode
			mini_isp_e_to_a();
			goto ERR;
		}
		// tx d0
		mini_isp_register_read(tx_d0_reg, &reg_val);
		misp_info("tx d0 count = %d", reg_val);
		if (tx_d0_count > reg_val) {
			misp_info("tx d0 count is out of criterion");
			errcode = -EINVAL;
			// switch to A mode
			mini_isp_e_to_a();
			goto ERR;
		}
		// switch to A mode
		mini_isp_e_to_a();

	}
ERR:
	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_led_power_control(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret[8];
	int t_ret;
	char cmd_name[20];
	char str_param[8][20];
	struct isp_cmd_led_power_control projector_control_param;

	memset(&projector_control_param, 0,
		sizeof(struct isp_cmd_led_power_control));
	misp_info("%s S!!", __func__);

	t_ret = sscanf(cmd_buf, "%s %s %s %s %s %s %s %s %s",
		cmd_name, &str_param[0][0], &str_param[1][0], &str_param[2][0],
		&str_param[3][0], &str_param[4][0], &str_param[5][0],
		&str_param[6][0], &str_param[7][0]);
	ret[0] = t_ret;
	if (ret != 9) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8*/
	ret[0] = kstrtou8(&str_param[0][0], 0,
		&projector_control_param.led_on_off);
	ret[1] = kstrtou8(&str_param[1][0], 0,
		&projector_control_param.led_power_level);
	ret[2] = kstrtou8(&str_param[2][0], 0,
		&projector_control_param.control_projector_id);
	ret[3] = kstrtou32(&str_param[3][0], 0,
		&projector_control_param.delay_after_sof);
	ret[4] = kstrtou32(&str_param[4][0], 0,
		&projector_control_param.pulse_time);
	ret[5] = kstrtou8(&str_param[5][0], 0,
		&projector_control_param.control_mode);
	ret[6] = kstrtou8(&str_param[6][0], 0,
		&projector_control_param.reserved);
	ret[7] = kstrtou8(&str_param[7][0], 0,
		&projector_control_param.rolling_shutter);

	if (ret[0] != ERR_SUCCESS)
		return ret[0];
	else if (ret[1] != ERR_SUCCESS)
		return ret[1];
	else if (ret[2] != ERR_SUCCESS)
		return ret[2];
	else if (ret[3] != ERR_SUCCESS)
		return ret[3];
	else if (ret[4] != ERR_SUCCESS)
		return ret[4];
	else if (ret[5] != ERR_SUCCESS)
		return ret[5];
	else if (ret[6] != ERR_SUCCESS)
		return ret[6];
	else if (ret[7] != ERR_SUCCESS)
		return ret[7];

	misp_info("%d, %d, %d, %d, %d, %d, %d, %d",
		projector_control_param.led_on_off,
		projector_control_param.led_power_level,
		projector_control_param.control_projector_id,
		projector_control_param.delay_after_sof,
		projector_control_param.pulse_time,
		projector_control_param.control_mode,
		projector_control_param.reserved,
		projector_control_param.rolling_shutter);

	errcode = mini_isp_drv_led_power_control(&projector_control_param);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_active_ae(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret[2];
	int t_ret;
	char cmd_name[20];
	char str_param[2][20];
	struct isp_cmd_active_ae active_ae_param;

	memset(&active_ae_param, 0,
		sizeof(struct isp_cmd_active_ae));
	misp_info("%s S!!", __func__);

	t_ret = sscanf(cmd_buf, "%s %s %s",
		cmd_name, &str_param[0][0], &str_param[1][0]);
	ret[0] = t_ret;
	if (ret != 3) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8*/
	ret[0] = kstrtou8(&str_param[0][0], 0,
		&active_ae_param.active_ae);
	ret[1] = kstrtou16(&str_param[1][0], 0,
		&active_ae_param.f_number_x1000);
	if (ret[0] != ERR_SUCCESS)
		return ret[0];
	else if (ret[1] != ERR_SUCCESS)
		return ret[1];

	misp_info("%d, %d",
		active_ae_param.active_ae,
		active_ae_param.f_number_x1000);

	errcode = mini_isp_drv_active_ae(&active_ae_param);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_isp_ae_control_mode_on_off(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret;
	char cmd_name[20];
	char str_param[20];
	u8 param;

	misp_info("%s S!!", __func__);

	ret = sscanf(cmd_buf, "%s %s", cmd_name, &str_param[0]);
	if (ret != 2) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8*/
	ret = kstrtou8(&str_param[0], 0, &param);
	if (ret != ERR_SUCCESS)
		return ret;

	misp_info("%d", param);

	errcode = mini_isp_drv_isp_ae_control_mode_on_off(param);
	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_set_frame_rate_limits(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret[4];
	int t_ret;
	char cmd_name[20];
	char str_param[4][20];
	struct isp_cmd_frame_rate_limits frame_rate_param;

	memset(&frame_rate_param, 0,
		sizeof(struct isp_cmd_frame_rate_limits));
	misp_info("%s S!!", __func__);

	t_ret = sscanf(cmd_buf, "%s %s %s %s %s", cmd_name,
		&str_param[0][0], &str_param[1][0],
		&str_param[2][0], &str_param[3][0]);
	ret[0] = t_ret;
	if (t_ret != 5) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8*/
	ret[0] = kstrtou16(&str_param[0][0], 0,
		&frame_rate_param.main_min_framerate_x100);
	ret[1] = kstrtou16(&str_param[1][0], 0,
		&frame_rate_param.main_max_framerate_x100);
	ret[2] = kstrtou16(&str_param[2][0], 0,
		&frame_rate_param.sub_min_framerate_x100);
	ret[3] = kstrtou16(&str_param[3][0], 0,
		&frame_rate_param.sub_max_framerate_x100);
	if (ret[0] != ERR_SUCCESS)
		return ret[0];
	else if (ret[1] != ERR_SUCCESS)
		return ret[1];
	else if (ret[2] != ERR_SUCCESS)
		return ret[2];
	else if (ret[3] != ERR_SUCCESS)
		return ret[3];

	misp_info("%d %d %d %d",
		frame_rate_param.main_min_framerate_x100,
		frame_rate_param.main_max_framerate_x100,
		frame_rate_param.sub_min_framerate_x100,
		frame_rate_param.sub_max_framerate_x100);

	errcode = mini_isp_drv_set_frame_rate_limits(&frame_rate_param);
	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_set_max_exposure(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret[3];
	int t_ret;
	char cmd_name[20];
	char str_param[3][20];

	struct isp_cmd_exposure_param set_max_exposure;

	memset(&set_max_exposure, 0, sizeof(struct isp_cmd_exposure_param));
	misp_info("%s S!!", __func__);

	t_ret = sscanf(cmd_buf, "%s %s %s %s",
		cmd_name, &str_param[0][0], &str_param[1][0], &str_param[2][0]);
	ret[0] = t_ret;
	if (ret != 4) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8*/
	ret[0] = kstrtou32(&str_param[0][0], 0,
		&set_max_exposure.udExpTime);
	ret[1] = kstrtou16(&str_param[1][0], 0,
		&set_max_exposure.uwISO);
	ret[2] = kstrtou8(&str_param[2][0], 0,
		&set_max_exposure.ucActiveDevice);
	if (ret[0] != ERR_SUCCESS)
		return ret[0];
	else if (ret[1] != ERR_SUCCESS)
		return ret[1];
	else if (ret[2] != ERR_SUCCESS)
		return ret[2];

	misp_info("max exposure param: %d %d %d",
		set_max_exposure.udExpTime,
		set_max_exposure.uwISO,
		set_max_exposure.ucActiveDevice);

	errcode = mini_isp_drv_set_max_exposure(&set_max_exposure);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_frame_sync_control(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret[5];
	int t_ret;
	char cmd_name[20];
	char str_param[5][20];
	struct isp_cmd_frame_sync_control frame_sync_control_param;

	memset(&frame_sync_control_param, 0,
		sizeof(struct isp_cmd_frame_sync_control));
	misp_info("%s S!!", __func__);

	t_ret = sscanf(cmd_buf, "%s %s %s %s %s %s", cmd_name,
		&str_param[0][0], &str_param[1][0], &str_param[2][0],
		&str_param[3][0], &str_param[4][0]);
	ret[0] = t_ret;
	if (ret != 6) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8*/
	ret[0] = kstrtou8(&str_param[0][0], 0,
		&frame_sync_control_param.control_deviceID);
	ret[1] = kstrtou8(&str_param[1][0], 0,
		&frame_sync_control_param.delay_framephase);
	ret[2] = kstrtou8(&str_param[2][0], 0,
		&frame_sync_control_param.active_framephase);
	ret[3] = kstrtou8(&str_param[3][0], 0,
		&frame_sync_control_param.deactive_framephase);
	ret[4] = kstrtou8(&str_param[4][0], 0,
		&frame_sync_control_param.active_timelevel);
	if (ret[0] != ERR_SUCCESS)
		return ret[0];
	else if (ret[1] != ERR_SUCCESS)
		return ret[1];
	else if (ret[2] != ERR_SUCCESS)
		return ret[2];
	else if (ret[3] != ERR_SUCCESS)
		return ret[3];
	else if (ret[4] != ERR_SUCCESS)
		return ret[4];

	misp_info("%d %d %d %d %d",
		frame_sync_control_param.control_deviceID,
		frame_sync_control_param.delay_framephase,
		frame_sync_control_param.active_framephase,
		frame_sync_control_param.deactive_framephase,
		frame_sync_control_param.active_timelevel);

	errcode = mini_isp_drv_frame_sync_control(&frame_sync_control_param);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_set_shot_mode(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret[2];
	int t_ret;
	char cmd_name[20];
	char str_param[2][20];
	struct isp_cmd_set_shot_mode set_shot_mode_param;

	memset(&set_shot_mode_param, 0, sizeof(struct isp_cmd_set_shot_mode));
	misp_info("%s S!!", __func__);

	t_ret = sscanf(cmd_buf, "%s %s %s", cmd_name,
		&str_param[0][0], &str_param[1][0]);
	ret[0] = t_ret;
	if (ret != 3) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8*/
	ret[0] = kstrtou8(&str_param[0][0], 0,
		&set_shot_mode_param.shot_mode);
	ret[1] = kstrtou16(&str_param[1][0], 0,
		&set_shot_mode_param.frame_rate);
	if (ret[0] != ERR_SUCCESS)
		return ret[0];
	else if (ret[1] != ERR_SUCCESS)
		return ret[1];

	misp_info("%d %d",
		set_shot_mode_param.shot_mode,
		set_shot_mode_param.frame_rate);

	errcode = mini_isp_drv_set_shot_mode(&set_shot_mode_param);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_lighting_ctrl(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret[25];
	int t_ret;
	char cmd_name[20];
	char str_param[25][20];
	u8 i = 0;
	struct isp_cmd_lighting_ctrl lighting_ctrl;

	memset(&lighting_ctrl, 0, sizeof(struct isp_cmd_lighting_ctrl));
	misp_info("%s S!!", __func__);

	t_ret = sscanf(cmd_buf, "%s %s %s %s %s %s %s %s \
		%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s",
		cmd_name, &str_param[0][0], &str_param[1][0], &str_param[2][0],
		&str_param[3][0], &str_param[4][0], &str_param[5][0],
		&str_param[6][0], &str_param[7][0], &str_param[8][0],
		&str_param[9][0], &str_param[10][0], &str_param[11][0],
		&str_param[12][0], &str_param[13][0], &str_param[14][0],
		&str_param[15][0], &str_param[16][0], &str_param[17][0],
		&str_param[18][0], &str_param[19][0], &str_param[20][0],
		&str_param[21][0], &str_param[22][0], &str_param[23][0],
		&str_param[24][0]);
	ret[0] = t_ret;
	if (ret != 26) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8*/
	ret[0] = kstrtou32(&str_param[0][0], 0, &lighting_ctrl.cycle_len);
	if (ret[0] != ERR_SUCCESS)
		return ret[0];

	for (i = 0; i < lighting_ctrl.cycle_len; i++) {
		ret[i*3+1] = kstrtou8(&str_param[i*3+1][0], 0,
			&lighting_ctrl.cycle[i].source);
		ret[i*3+2] = kstrtou8(&str_param[i*3+2][0], 0,
			&lighting_ctrl.cycle[i].TxDrop);
		ret[i*3+3] = kstrtou16(&str_param[i*3+3][0], 0,
			&lighting_ctrl.cycle[i].co_frame_rate);

		if (ret[i*3+1] != ERR_SUCCESS)
			return ret[i*3+1];
		else if (ret[i*3+2] != ERR_SUCCESS)
			return ret[i*3+2];
		else if (ret[i*3+3] != ERR_SUCCESS)
			return ret[i*3+3];
	}

	misp_info("cycle_len: %d", lighting_ctrl.cycle_len);
	for (i = 0; i < lighting_ctrl.cycle_len; i++) {
		misp_info("cycle[%d]: %d, %d, %d", i,
			lighting_ctrl.cycle[i].source,
			lighting_ctrl.cycle[i].TxDrop,
			lighting_ctrl.cycle[i].co_frame_rate);
	}

	errcode = mini_isp_drv_lighting_ctrl(&lighting_ctrl);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_depth_compensation(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret[4];
	int t_ret;
	char cmd_name[20];
	char str_param[4][20];
	u8 param[2];
	u8 en_update = 0;
	struct isp_cmd_depth_compensation_param depth_compensation_param;

	memset(&depth_compensation_param, 0,
		sizeof(struct isp_cmd_depth_compensation_param));
	misp_info("%s S!!", __func__);

	t_ret = sscanf(cmd_buf, "%s %s %s %s %s", cmd_name,
		&str_param[0][0], &str_param[1][0],
		&str_param[2][0], &str_param[3][0]);
	ret[0] = t_ret;
	if (ret != 5) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8*/
	ret[0] = kstrtou8(&str_param[0][0], 0, &param[0]);
	ret[1] = kstrtou8(&str_param[1][0], 0, &param[1]);
	ret[2] = kstrtou16(&str_param[2][0], 0,
		&depth_compensation_param.short_distance_value);
	ret[3] = kstrtos8(&str_param[3][0], 0,
		&depth_compensation_param.compensation);

	en_update = (param[0] << 4) | param[1];
	depth_compensation_param.en_updated = (u8)en_update;

	if (ret[0] != ERR_SUCCESS)
		return ret[0];
	else if (ret[1] != ERR_SUCCESS)
		return ret[1];
	else if (ret[2] != ERR_SUCCESS)
		return ret[2];
	else if (ret[3] != ERR_SUCCESS)
		return ret[3];

	misp_info("0x%x %d %d",
		depth_compensation_param.en_updated,
		depth_compensation_param.short_distance_value,
		depth_compensation_param.compensation);

	errcode = mini_isp_drv_depth_compensation(&depth_compensation_param);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_cycle_trigger_depth_process(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret[3];
	int t_ret;
	char cmd_name[20];
	char str_param[3][20];

	struct isp_cmd_cycle_trigger_depth_process depth_cycle_param;

	memset(&depth_cycle_param, 0,
		sizeof(struct isp_cmd_cycle_trigger_depth_process));
	misp_info("%s S!!", __func__);

	t_ret = sscanf(cmd_buf, "%s %s %s %s", cmd_name,
		&str_param[0][0], &str_param[1][0], &str_param[2][0]);
	ret[0] = t_ret;
	if (ret != 4) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8*/
	ret[0] = kstrtou8(&str_param[0][0], 0,
		&depth_cycle_param.cycleLen);
	ret[1] = kstrtou16(&str_param[1][0], 0,
		&depth_cycle_param.depth_triggerBitField);
	ret[2] = kstrtou16(&str_param[2][0], 0,
		&depth_cycle_param.depthoutput_triggerBitField);
	if (ret[0] != ERR_SUCCESS)
		return ret[0];
	else if (ret[1] != ERR_SUCCESS)
		return ret[1];
	else if (ret[2] != ERR_SUCCESS)
		return ret[2];

	misp_info("depth cycle len: %d %d %d",
		depth_cycle_param.cycleLen,
		depth_cycle_param.depth_triggerBitField,
		depth_cycle_param.depthoutput_triggerBitField);

	errcode = mini_isp_drv_cycle_trigger_depth_process(
		&depth_cycle_param);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_set_min_exposure(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret[3];
	int t_ret;
	char cmd_name[20];
	char str_param[3][20];

	struct isp_cmd_exposure_param set_min_exposure;

	memset(&set_min_exposure, 0,
		sizeof(struct isp_cmd_exposure_param));
	misp_info("%s S!!", __func__);

	t_ret = sscanf(cmd_buf, "%s %s %s %s", cmd_name,
		&str_param[0][0], &str_param[1][0], &str_param[2][0]);
	ret[0] = t_ret;
	if (ret != 4) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8*/
	ret[0] = kstrtou32(&str_param[0][0], 0,
		&set_min_exposure.udExpTime);
	ret[1] = kstrtou16(&str_param[1][0], 0,
		&set_min_exposure.uwISO);
	ret[2] = kstrtou8(&str_param[2][0], 0,
		&set_min_exposure.ucActiveDevice);
	if (ret[0] != ERR_SUCCESS)
		return ret[0];
	else if (ret[1] != ERR_SUCCESS)
		return ret[1];
	else if (ret[2] != ERR_SUCCESS)
		return ret[2];

	misp_info("min exposure param: %d %d %d",
		set_min_exposure.udExpTime,
		set_min_exposure.uwISO,
		set_min_exposure.ucActiveDevice);

	errcode = mini_isp_drv_set_min_exposure(&set_min_exposure);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_set_max_exposure_slope(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret[2];
	int t_ret;
	char cmd_name[20];
	char str_param[2][20];

	struct isp_cmd_max_exposure_slope max_exposure_slope;

	memset(&max_exposure_slope, 0,
		sizeof(struct isp_cmd_max_exposure_slope));
	misp_info("%s S!!", __func__);

	t_ret = sscanf(cmd_buf, "%s %s %s", cmd_name,
		&str_param[0][0], &str_param[1][0]);
	ret[0] = t_ret;
	if (ret != 3) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8*/
	ret[0] = kstrtou32(&str_param[0][0], 0,
		&max_exposure_slope.max_exposure_slope);
	ret[1] = kstrtou8(&str_param[1][0], 0,
		&max_exposure_slope.ucActiveDevice);
	if (ret[0] != ERR_SUCCESS)
		return ret[0];
	else if (ret[1] != ERR_SUCCESS)
		return ret[1];

	misp_info("max exposure slope: %d %d",
		max_exposure_slope.max_exposure_slope,
		max_exposure_slope.ucActiveDevice);

	errcode = mini_isp_drv_set_max_exposure_slope(&max_exposure_slope);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_led_active_delay(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret;
	char cmd_name[20];
	char str_param[20];
	u32 delay_ms;

	misp_info("%s S!!", __func__);

	ret = sscanf(cmd_buf, "%s %s", cmd_name, &str_param[0]);
	if (ret != 2) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8*/
	ret = kstrtou32(&str_param[0], 0, &delay_ms);
	if (ret != ERR_SUCCESS)
		return ret;

	misp_info("[delay]: %d", delay_ms);

	errcode = mini_isp_drv_led_active_delay(delay_ms);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_isp_smart_ae_control(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret;
	char cmd_name[20];
	char str_param[20];
	u8 on;

	misp_info("%s S!!", __func__);

	ret = sscanf(cmd_buf, "%s %s", cmd_name, &str_param[0]);
	if (ret != 2) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8*/
	ret = kstrtou8(&str_param[0], 0, &on);
	if (ret != ERR_SUCCESS)
		return ret;

	misp_info("[isp smart ae control]: %d", on);

	errcode = mini_isp_drv_isp_smart_ae_control(on);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_set_group_led_level(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret[5];
	int t_ret;
	char cmd_name[20];
	char str_param[5][20];
	struct ISPCMD_LED_LEVEL set_led_level;

	memset(&set_led_level, 0, sizeof(struct ISPCMD_LED_LEVEL));

	misp_info("%s S!!", __func__);

	t_ret = sscanf(cmd_buf, "%s %s %s %s %s %s",
		cmd_name, &str_param[0][0], &str_param[1][0], &str_param[2][0],
		&str_param[3][0], &str_param[4][0]);
	ret[0] = t_ret;
	if (ret != 6) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8*/
	ret[0] = kstrtou8(&str_param[0][0], 0,
		&set_led_level.ucType);
	ret[1] = kstrtou8(&str_param[1][0], 0,
		&set_led_level.ucGroup0IDs);
	ret[2] = kstrtou8(&str_param[2][0], 0,
		&set_led_level.ucGroup0Level);
	ret[3] = kstrtou8(&str_param[3][0], 0,
		&set_led_level.ucGroup1IDs);
	ret[4] = kstrtou8(&str_param[4][0], 0,
		&set_led_level.ucGroup1Level);
	if (ret[0] != ERR_SUCCESS)
		return ret[0];
	else if (ret[1] != ERR_SUCCESS)
		return ret[1];
	else if (ret[2] != ERR_SUCCESS)
		return ret[2];
	else if (ret[3] != ERR_SUCCESS)
		return ret[3];
	else if (ret[4] != ERR_SUCCESS)
		return ret[4];

	misp_info("[isp set group led level]: %d, 0x%x, 0x%x, 0x%x, 0x%x",
		set_led_level.ucType,
		set_led_level.ucGroup0IDs,
		set_led_level.ucGroup0Level,
		set_led_level.ucGroup1IDs,
		set_led_level.ucGroup1Level);
	errcode = mini_isp_drv_set_group_led_level(&set_led_level);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_ctrl_led_seq(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret[13];
	int t_ret;
	char cmd_name[20];
	char str_param[13][20];
	u32 i = 0;
	struct ISPCMD_CTRL_LED_SEQ ctrl_led_seq;

	memset(&ctrl_led_seq, 0, sizeof(struct ISPCMD_CTRL_LED_SEQ));

	misp_info("%s S!!", __func__);

	t_ret = sscanf(cmd_buf, "%s %s %s %s %s %s %s %s \
		%s %s %s %s %s %s",
		cmd_name, &str_param[0][0], &str_param[1][0], &str_param[2][0],
		&str_param[3][0], &str_param[4][0], &str_param[5][0],
		&str_param[6][0], &str_param[7][0], &str_param[8][0],
		&str_param[9][0], &str_param[10][0], &str_param[11][0],
		&str_param[12][0]);
	ret[0] = t_ret;
	if (ret != 14) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8*/
	ret[0] = kstrtou8(&str_param[0][0], 0, &ctrl_led_seq.ucCycle);
	if (ret[0] != ERR_SUCCESS)
		return ret[0];

	for (i = 0; i < ctrl_led_seq.ucCycle; i++) {
		ret[i+1] = kstrtou8(&str_param[i+1][0], 0,
			&ctrl_led_seq.ucCycleLightCtrl[i]);
		if (ret[i+1] != ERR_SUCCESS)
			return ret[i+1];
	}

	misp_info("[Cycle]: %d", ctrl_led_seq.ucCycle);
	for (i = 0; i < ctrl_led_seq.ucCycle; i++)
		misp_info("[%d]: %d", i, ctrl_led_seq.ucCycleLightCtrl[i]);

	errcode = mini_isp_drv_set_ctrl_led_seq(&ctrl_led_seq);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_host_ctrl_led_onoff(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret[2];
	int t_ret;
	char cmd_name[20];
	char str_param[2][20];
	u32 param[2] = {0};/* param[0]: GPO, param[1]: H/L*/
	struct ISPCMD_CTRL_GPO host_led_onoff;

	memset(&host_led_onoff, 0, sizeof(struct ISPCMD_CTRL_GPO));

	misp_info("%s S!!", __func__);

	t_ret = sscanf(cmd_buf, "%s %s %s", cmd_name,
		&str_param[0][0], &str_param[1][0]);
	ret[0] = t_ret;
	if (ret != 3) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u32*/
	ret[0] = kstrtou32(&str_param[0][0], 0, &param[0]);
	ret[1] = kstrtou32(&str_param[1][0], 0, &param[1]);
	if (ret[0] != ERR_SUCCESS)
		return ret[0];
	else if (ret[1] != ERR_SUCCESS)
		return ret[1];

	misp_info("0x%x 0x%x", param[0], param[1]);
	host_led_onoff.uwGPO_HL_BitField =
		(u16)((param[1]&0x0001)<<(param[0]&0x000F));

	errcode = mini_isp_drv_host_ctrl_led_onoff(&host_led_onoff);


	return errcode;
}

ssize_t echo_mini_isp_drv_extra_sensor_sync(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret;
	char cmd_name[20];
	char str_param[20];
	u8 sync_ratio;

	misp_info("%s S!!", __func__);
	ret = sscanf(cmd_buf, "%s %s",
		cmd_name, &str_param[0]);
	if (ret != 2) {
		errcode = -EINVAL;
		return errcode;
	}

	/* str to u8*/
	ret = kstrtou8(&str_param[0], 0, &sync_ratio);
	if (ret != ERR_SUCCESS)
		return ret;

	misp_info("%d", sync_ratio);
	errcode = mini_isp_drv_extra_sensor_sync(sync_ratio);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_set_depth_type(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret[2];
	int t_ret;
	char cmd_name[20];
	char str_param[2][20];
	u32 param[2] = {0};
	struct ISPCMD_DEPTH_TYPE set_depth_type;

	misp_info("%s S!!", __func__);

	t_ret = sscanf(cmd_buf, "%s %s %s",
		cmd_name, &str_param[0][0], &str_param[1][0]);
	ret[0] = t_ret;
	if (ret != 3) {
		errcode = -EINVAL;
		return errcode;
	}

	/* str to u32*/
	ret[0] = kstrtou32(&str_param[0][0], 0, &param[0]);
	ret[1] = kstrtou32(&str_param[1][0], 0, &param[1]);
	if (ret[0] != ERR_SUCCESS)
		return ret[0];
	else if (ret[1] != ERR_SUCCESS)
		return ret[1];

	misp_info("0x%x 0x%x", param[0], param[1]);

	set_depth_type.ucDepthType = (u8)param[0];
	set_depth_type.ucGroundType = (u8)param[1];

	errcode = mini_isp_drv_set_depth_type(&set_depth_type);

	misp_info("%s E!!", __func__);
	return errcode;
}


ssize_t echo_mini_isp_drv_get_chip_thermal(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	u16 thermal_val;

	misp_info("%s S!!", __func__);

	errcode = mini_isp_drv_get_chip_thermal(&thermal_val);
	misp_info("0x%x", thermal_val);
	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_poweron(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;

	misp_info("%s S!!", __func__);
	errcode = mini_isp_poweron();
	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_poweroff(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;

	misp_info("%s S!!", __func__);
	errcode = mini_isp_poweroff();
	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_load_fw(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;

	misp_info("%s S!!", __func__);
	errcode = mini_isp_drv_load_fw();
	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_write_calibration_data(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret;
	char cmd_name[20];
	char str_param[20];
	u8 info_id;

	misp_info("%s S!!", __func__);
	ret = sscanf(cmd_buf, "%s %s", cmd_name, &str_param[0]);
	if (ret != 2) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8*/
	ret = kstrtou8(&str_param[0], 0, &info_id);
	if (ret != ERR_SUCCESS)
		return ret;

	misp_info("%d", info_id);
	if (info_id == 0 || info_id == 1) {
		misp_info("Currently not support IQ calibration and packdata!");
		goto ERR;
	}
	errcode = mini_isp_drv_write_calibration_data(info_id, NULL, 0);
ERR:

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_write_spinor_data(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret;
	char cmd_name[20];
	char str_param[20];
	u8 info_id;

	misp_info("%s S!!", __func__);
	ret = sscanf(cmd_buf, "%s %s", cmd_name, &str_param[0]);
	if (ret != 2) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8*/
	ret = kstrtou8(&str_param[0], 0, &info_id);
	if (ret != ERR_SUCCESS)
		return ret;

	misp_info("%d", info_id);
	if (info_id > SPINOR_ID_DEPTHBIN)
		goto ERR;

	errcode = mini_isp_drv_write_spinor_data(info_id);
ERR:
	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_mini_isp_get_chip_id(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;

	misp_info("%s S!!", __func__);
	errcode = mini_isp_get_chip_id();
	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_get_comlog(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	struct common_log_hdr_info info;

	memset(&info, 0, sizeof(struct common_log_hdr_info));
	misp_info("%s S!!", __func__);

	info.block_size = SPI_TX_BULK_SIZE;
	info.total_size = LEVEL_LOG_BUFFER_SIZE;
	errcode = ispctrl_if_mast_execute_cmd(ISPCMD_BULK_READ_COMLOG,
		(u8 *)&info);
	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_conti_set_register(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret[3];
	int t_ret;
	char cmd_name[20];
	char str_param[3][80];
	u8 *send_buf = NULL;
	u32 reg_start_addr;
	u32 total_size;
	char FileName[80] = {0};
	struct file *pFile = NULL;
	mm_segment_t oldfs;

	misp_info("%s S", __func__);
	// switch to E mode
	mini_isp_a_to_e();

	/** Input Format
	 *\ 1. full path file name
	 *\ 2. start address
	 *\ 3. total size
	 */
	t_ret = sscanf(cmd_buf, "%s %s %s %s", cmd_name,
			&str_param[0][0], &str_param[1][0], &str_param[2][0]);
	ret[0] = t_ret;
	if (ret != 4) {
		errcode = -EINVAL;
		return errcode;
	}

	strlcpy(FileName, &str_param[0][0], 80);
	misp_info("[Filename]: %s", FileName);

	/* str to u8*/
	ret[1] = kstrtou32(&str_param[1][0], 0, &reg_start_addr);
	ret[2] = kstrtou32(&str_param[2][0], 0, &total_size);
	if (ret[1] != ERR_SUCCESS)
		return ret[1];
	else if (ret[2] != ERR_SUCCESS)
		return ret[2];

	/* check total size must be 4 alignment */
	if (total_size != 0 && (total_size % 4) == 0) {
		send_buf = kzalloc(total_size, GFP_KERNEL);
		if (send_buf == NULL) {
			misp_err("%s, L: %d buf alloc fail!",
				__func__, __LINE__);
			goto EXIT;
		}
	} else {
		misp_err("%s, L: %d total size err!", __func__, __LINE__);
		goto EXIT;
	}

	misp_info("set register 0x%x, 0x%x", reg_start_addr, total_size);
#if ENABLE_FILP_OPEN_API
	/*pFile = filp_open(FileName, O_RDONLY, 0777);*/
#else
	misp_info("Error! Currently not support file open api");
	misp_info("See define ENABLE_FILP_OPEN_API");
	goto EXIT;
#endif

	oldfs = get_fs();
	/*Set segment descriptor associated*/
	set_fs(get_ds());

	if (IS_ERR(pFile)) {
		errcode = PTR_ERR(pFile);
		misp_err("%s L: %d open file failed. err: %zd",
			__func__, __LINE__, errcode);
		goto EXIT;
	}

	ret[0] = vfs_read(pFile, send_buf, total_size, &pFile->f_pos);
	if (ret[0] < 0) {
		misp_info("%s:L%d Error! - %d", __func__, __LINE__, ret[0]);
		errcode = ~ERR_SUCCESS;
		goto EXIT;
	}

	filp_close(pFile, NULL);

	mini_isp_register_conti_write(reg_start_addr, send_buf, total_size);

EXIT:
	set_fs(oldfs);
	if (send_buf != NULL)
		kfree(send_buf);

	// switch to A mode
	mini_isp_e_to_a();

	misp_info("%s E", __func__);
	return errcode;
}
ssize_t echo_set_register(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret[2];
	int t_ret;
	char cmd_name[20];
	char str_param[2][20];
	u32 reg_addr;
	u32 reg_new_value;
	struct misp_global_variable *dev_global_variable;

	dev_global_variable = get_mini_isp_global_variable();

	misp_info("%s S", __func__);
	// switch to E mode
	mini_isp_a_to_e();
	t_ret = sscanf(cmd_buf, "%s %s %s", cmd_name,
		&str_param[0][0], &str_param[1][0]);
	ret[0] = t_ret;
	if (ret != 3) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8*/
	ret[0] = kstrtou32(&str_param[0][0],
		0, &reg_addr);
	ret[1] = kstrtou32(&str_param[1][0],
		0, &reg_new_value);
	if (ret[0] != ERR_SUCCESS)
		return ret[0];
	else if (ret[1] != ERR_SUCCESS)
		return ret[1];

	misp_info("set register 0x%x, 0x%x", reg_addr, reg_new_value);
	mini_isp_register_write(reg_addr, reg_new_value);

	// switch to A mode
	mini_isp_e_to_a();

	misp_info("%s E", __func__);
	return errcode;
}

ssize_t echo_get_register(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret;
	char cmd_name[20];
	char str_param[20];
	u32 reg_addr;
	u32 reg_val;
	struct misp_global_variable *dev_global_variable;

	dev_global_variable = get_mini_isp_global_variable();

	misp_info("%s S", __func__);
	// switch to E mode
	mini_isp_a_to_e();
	ret = sscanf(cmd_buf, "%s %s", cmd_name, &str_param[0]);
	if (ret != 2) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u32*/
	ret = kstrtou32(&str_param[0], 0, &reg_addr);
	if (ret != ERR_SUCCESS)
		return ret;

	mini_isp_register_read(reg_addr, &reg_val);
	misp_info("get register 0x%x, 0x%x", reg_addr, reg_val);

	// switch to A mode
	mini_isp_e_to_a();

	misp_info("%s E", __func__);
	return errcode;
}

/* SPI_A mode command */
ssize_t echo_memwrite(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret[4];
	int t_ret;
	char cmd_name[20];
	char str_param[4][80];
	char FileName[80] = {0};
	struct file *pFile = NULL;
	mm_segment_t oldfs;
	struct memory_write_info Mem_W_Info = {0};
	u8 *psend_data_buf = NULL;
	u8 send_cmd_buf[T_SPI_CMD_LENGTH];

	misp_info("%s S", __func__);

	/** Input Format
	 *\1. full path file name
	 *\2. start address
	 *\3. total size
	 */
	t_ret = sscanf(cmd_buf, "%s %s %s %s", cmd_name,
			&str_param[0][0], &str_param[1][0], &str_param[2][0]);
	ret[0] = t_ret;
	if (ret != 4) {
		errcode = -EINVAL;
		return errcode;
	}

	strlcpy(FileName, &str_param[0][0], 80);
	misp_info("file name: %s", FileName);

	memset(&Mem_W_Info, 0x0, sizeof(struct memory_write_info));
	/* str to u32*/
	ret[1] = kstrtou32(&str_param[1][0], 0, &Mem_W_Info.start_addr);
	ret[2] = kstrtou32(&str_param[2][0], 0, &Mem_W_Info.total_size);
	if (ret[1] != ERR_SUCCESS)
		return ret[1];
	else if (ret[2] != ERR_SUCCESS)
		return ret[2];

	if ((Mem_W_Info.total_size % 4) != 0) {
		misp_err("%s L: %d total size must be 4 alignment",
		__func__, __LINE__);
		goto EXIT;
	}

	misp_info("0x%x %d", Mem_W_Info.start_addr, Mem_W_Info.total_size);
#if ENABLE_FILP_OPEN_API
	/*pFile = filp_open(FileName, O_RDONLY, 0777);*/
#else
	misp_info("Error! Currently not support file open api");
	misp_info("See define ENABLE_FILP_OPEN_API");
	goto EXIT;
#endif
	oldfs = get_fs();
	/*Set segment descriptor associated*/
	set_fs(get_ds());

	if (IS_ERR(pFile)) {
		errcode = PTR_ERR(pFile);
		misp_err("%s L: %d open file failed. err: %zd",
		__func__, __LINE__, errcode);
		goto EXIT;
	}

	psend_data_buf = kzalloc(Mem_W_Info.total_size, GFP_KERNEL);
	if (psend_data_buf == NULL) {
		errcode = ~ERR_SUCCESS;
		misp_info("%s:L%d interface Error!", __func__, __LINE__);
		goto EXIT;
	}

	ret[0] = vfs_read(pFile, psend_data_buf,
			Mem_W_Info.total_size, &pFile->f_pos);
	if (ret[0] < 0) {
		misp_info("%s:L%d Error! - %d", __func__, __LINE__, ret[0]);
		errcode = ~ERR_SUCCESS;
		goto EXIT;
	}

	filp_close(pFile, NULL);

	memcpy(send_cmd_buf, &Mem_W_Info.start_addr, sizeof(u32));
	memcpy(send_cmd_buf + 4, &Mem_W_Info.total_size, sizeof(u32));
	memcpy(send_cmd_buf + 8, &psend_data_buf, sizeof(psend_data_buf));


	/* write memory */
	ret[0] = ispctrl_if_mast_execute_cmd(ISPCMD_BULK_WRITE_MEMORY,
		send_cmd_buf);
	if (ret[0] != ERR_SUCCESS) {
		misp_info("%s:L%d Error!", __func__, __LINE__);
		errcode = ret[0];
		goto EXIT;
	}

EXIT:
	set_fs(oldfs);
	if (psend_data_buf != NULL)
		kfree(psend_data_buf);

	return errcode;
}

/* SPI_E mode command */
ssize_t echo_memdump(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret[2];
	int t_ret;
	char cmd_name[2];
	char str_param[2][20];
	char filename[80];
	u32 start_addr;
	u32 len;
	struct misp_global_variable *dev_global_variable;

	dev_global_variable = get_mini_isp_global_variable();

	misp_info("%s S", __func__);
	// switch to E mode
	mini_isp_a_to_e();
	t_ret = sscanf(cmd_buf, "%s %s %s", cmd_name,
			&str_param[0][0], &str_param[1][0]);
	ret[0] = t_ret;
	if (ret != 3) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u32*/
	ret[0] = kstrtou32(&str_param[0][0], 0, &start_addr);
	ret[1] = kstrtou32(&str_param[1][0], 0, &len);
	if (ret[0] != ERR_SUCCESS)
		return ret[0];
	else if (ret[1] != ERR_SUCCESS)
		return ret[1];

	misp_info("Get mem 0x%x, %d", start_addr, len);

	snprintf(filename, 80, "memdump_0x%x_%d", start_addr, len);
	errcode = mini_isp_memory_read_then_write_file(
			start_addr, len,
			MINIISP_INFO_DUMPLOCATION, filename);

	// switch to A mode
	mini_isp_e_to_a();

	misp_info("%s E", __func__);
	return errcode;
}

ssize_t echo_show_version(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	struct misp_global_variable *dev_global_variable;

	dev_global_variable = get_mini_isp_global_variable();
	misp_info("MINIISP_DRIVER_VERSION: %s",
		MINIISP_DRIVER_VERSION);
	misp_info("AL6100 project: %s, fw ver: %05d.%05d, build by %s",
		g_fw_project_name, g_fw_version_before_point,
		g_fw_version_after_point, g_fw_build_by);
	misp_info("SC table build data: %d",
		g_sc_build_date);
	misp_info("set fsm status: %d",
		dev_global_variable->now_state);
	misp_info("GENERAL_OPCODE: %d",
		GET_TRANS_SCEN_INTF(MINIISP_GENERAL_OPCODE));
	misp_info("BULK_OPCODE: %d",
		GET_TRANS_SCEN_INTF(MINIISP_BULK_OPCODE));
	misp_info("CFG_REG_MEM: %d",
		GET_TRANS_SCEN_INTF(MINIISP_CFG_REG_MEM));
	misp_info("busy_lock: %d",
		dev_global_variable->busy_lock.count.counter);
	misp_info("transfer_lock: %u",
		dev_global_variable->transfer_lock.count);
	misp_info("bulk_lock: %u",
		dev_global_variable->bulk_lock.count);
	return errcode;
}

ssize_t echo_set_fsm_status(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret;
	char cmd_name[20];
	char str_param[20];
	struct misp_global_variable *dev_global_variable;

	dev_global_variable = get_mini_isp_global_variable();

	ret = sscanf(cmd_buf, "%s %s", cmd_name, &str_param[0]);
	if (ret != 2) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to int*/
	ret = kstrtoint(&str_param[0], 0, &dev_global_variable->now_state);
	if (ret != ERR_SUCCESS)
		return ret;

	misp_info("set fsm status: %d", dev_global_variable->now_state);
	return errcode;
}

ssize_t echo_cfg_cmd_send(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret;
	char cmd_name[20];
	char str_param[20];
	u8 param;
	struct misp_global_variable *dev_global_variable;

	dev_global_variable = get_mini_isp_global_variable();

	ret = sscanf(cmd_buf, "%s %s", cmd_name, &str_param[0]);
	if (ret != 2) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8 */
	ret = kstrtou8(&str_param[0], 0, &param);
	if (ret != ERR_SUCCESS)
		return ret;

	dev_global_variable->en_cmd_send = (bool) param;
	misp_info("set en_cmd_send: %d", dev_global_variable->en_cmd_send);
	return errcode;
}

ssize_t echo_mini_isp_a_to_e(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;

	misp_info("%s, S", __func__);
	mini_isp_a_to_e();
	misp_info("%s, E", __func__);
	return errcode;
}

ssize_t echo_mini_isp_e_to_a(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;

	misp_info("%s, S", __func__);
	mini_isp_e_to_a();
	misp_info("%s, E", __func__);
	return errcode;
}

ssize_t echo_mini_isp_chip_init(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;

	misp_info("%s, S", __func__);
	mini_isp_chip_init();
	misp_info("%s, E", __func__);
	return errcode;
}

ssize_t echo_mini_isp_debug_dump_img(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;

	misp_info("%s, S", __func__);
	errcode = mini_isp_debug_dump_img();
	misp_info("%s, E", __func__);
	return errcode;
}

ssize_t echo_mini_isp_drv_set_bypass_mode(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret;
	u16 mini_isp_mode;
	char cmd_name[20];
	char str_param[20];

	misp_info("%s S", __func__);
	ret = sscanf(cmd_buf, "%s %s", cmd_name, &str_param[0]);
	if (ret != 2) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8 */
	ret = kstrtou16(&str_param[0], 0, &mini_isp_mode);
	if (ret != ERR_SUCCESS)
		return ret;

	if (mini_isp_mode == 0)
		mini_isp_mode = 1;

	misp_info("%d", mini_isp_mode);
	errcode = mini_isp_drv_set_bypass_mode(mini_isp_mode);
	misp_info("%s E", __func__);
	return errcode;
}

ssize_t echo_mini_isp_utility_read_reg_e_mode_for_bypass_use(
	const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;

	misp_info("%s S", __func__);
	errcode = mini_isp_utility_read_reg_e_mode_for_bypass_use();
	misp_info("%s E", __func__);
	return errcode;
}

ssize_t echo_mini_isp_utility_read_reg_e_mode(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;

	misp_info("%s S", __func__);
	errcode = mini_isp_utility_read_reg_e_mode();
	misp_info("%s E", __func__);
	return errcode;
}

ssize_t echo_mini_isp_debug_packdata_dump(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;

	misp_info("%s S", __func__);
	errcode = mini_isp_debug_packdata_dump();
	misp_info("%s E", __func__);
	return errcode;
}

ssize_t echo_mini_isp_debug_IQCalib_dump(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;

	misp_info("%s S", __func__);
	errcode = mini_isp_debug_IQCalib_dump();
	misp_info("%s E", __func__);
	return errcode;
}

ssize_t echo_mini_isp_debug_metadata_dump(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;

	misp_info("%s S", __func__);
	errcode = mini_isp_debug_metadata_dump();
	misp_info("%s E", __func__);
	return errcode;
}



ssize_t echo_mini_isp_debug_rect_combo_dump(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret;
	u8 is_ground_mode;
	char cmd_name[20];
	char str_param[20];

	misp_info("%s S", __func__);
	ret = sscanf(cmd_buf, "%s %s", cmd_name, &str_param[0]);
	if (ret != 2) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8 */
	ret = kstrtou8(&str_param[0], 0, &is_ground_mode);
	if (ret != ERR_SUCCESS)
		return ret;

	misp_info("dump mode %hhu", is_ground_mode);
	errcode = mini_isp_debug_depth_rect_combo_dump(is_ground_mode);
	misp_info("%s E", __func__);
	return errcode;
}

ssize_t echo_mini_isp_debug_depth_info(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;

	misp_info("%s S", __func__);
	errcode = mini_isp_debug_depth_info();
	misp_info("%s E", __func__);
	return errcode;
}


ssize_t echo_mini_isp_debug_metadata_info(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;

	misp_info("%s S", __func__);
	errcode = mini_isp_debug_metadata_info();
	misp_info("%s E", __func__);
	return errcode;
}

ssize_t echo_mini_isp_debug_sensor_info(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret;
	u8 IsLog2File;
	char cmd_name[20];
	char str_param[20];

	misp_info("%s S", __func__);
	ret = sscanf(cmd_buf, "%s %s", cmd_name, &str_param[0]);
	if (ret != 2) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8 */
	ret = kstrtou8(&str_param[0], 0, &IsLog2File);
	if (ret != ERR_SUCCESS)
		return ret;

	misp_info("IsLog2File: %d", IsLog2File);
	errcode = mini_isp_debug_sensor_info(IsLog2File);
	misp_info("%s E", __func__);
	return errcode;
}

ssize_t echo_mini_isp_debug_led_info(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret;
	u8 IsLog2File;
	char cmd_name[20];
	char str_param[20];

	misp_info("%s S", __func__);
	ret = sscanf(cmd_buf, "%s %s", cmd_name, &str_param[0]);
	if (ret != 2) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8 */
	ret = kstrtou8(&str_param[0], 0, &IsLog2File);
	if (ret != ERR_SUCCESS)
		return ret;

	misp_info("IsLog2File: %d", IsLog2File);
	errcode = mini_isp_debug_led_info(IsLog2File);
	misp_info("%s E", __func__);
	return errcode;
}

ssize_t echo_mini_isp_debug_rx_fps_info(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret[2];
	int t_ret;
	u8 taskstart;
	u8 IsLog2File;
	char cmd_name[20];
	char str_param[2][20];

	misp_info("%s S", __func__);
	t_ret = sscanf(cmd_buf, "%s %s %s", cmd_name,
			&str_param[0][0], &str_param[1][0]);
	ret[0] = t_ret;
	if (ret != 3) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8 */
	ret[0] = kstrtou8(&str_param[0][0], 0, &taskstart);
	ret[1] = kstrtou8(&str_param[1][0], 0, &IsLog2File);
	if (ret[0] != ERR_SUCCESS)
		return ret[0];
	else if (ret[1] != ERR_SUCCESS)
		return ret[1];

	if (taskstart == 0) {
		misp_info("rx_fps_info initial!");
		errcode = mini_isp_debug_mipi_rx_fps_start(IsLog2File);
	} else if (taskstart == 1) {
		misp_info("rx_fps_info exit!");
		mini_isp_debug_mipi_rx_fps_stop();
	} else
		misp_info("input err!");

	misp_info("%s E", __func__);
	return errcode;
}
ssize_t echo_mini_isp_debug_GPIO_Status(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret;
	u8 IsLog2File;
	char cmd_name[20];
	char str_param[20];

	misp_info("%s S", __func__);
	ret = sscanf(cmd_buf, "%s %s", cmd_name, &str_param[0]);
	if (ret != 2) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8 */
	ret = kstrtou8(&str_param[0], 0, &IsLog2File);
	if (ret != ERR_SUCCESS)
		return ret;

	misp_info("%d", IsLog2File);
	errcode = mini_isp_debug_GPIO_Status(IsLog2File);
	misp_info("%s E", __func__);
	return errcode;
}

ssize_t echo_mini_isp_eeprom_wp(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret;
	struct misp_global_variable *dev_global_variable;
	u8 param;
	char cmd_name[20];
	char str_param[20];

	misp_info("%s S", __func__);
	ret = sscanf(cmd_buf, "%s %s", cmd_name, &str_param[0]);
	if (ret != 2) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8 */
	ret = kstrtou8(&str_param[0], 0, &param);
	if (ret != ERR_SUCCESS)
		return ret;

	misp_info("%d", param);
	if (WP_GPIO != NULL) {
		dev_global_variable = get_mini_isp_global_variable();
		if (param)
			gpio_set_value(dev_global_variable->wp_gpio, 1);
		else
			gpio_set_value(dev_global_variable->wp_gpio, 0);
	}
	misp_info("%s E", __func__);
	return errcode;
}

ssize_t echo_mini_isp_eeprom_op(const char *cmd_buf)
{
	errcode err = ERR_SUCCESS;
	int ret;
	struct misp_global_variable *dev_global_variable;
	u8 filename[80] = "WRITE_OTP_DATA.bin";
	const struct firmware *fw;
	struct device *mini_isp_device;
	u8 *data_buf_addr;
	u32 total_size;
	u8 param;
	char cmd_name[20];
	char str_param[20];

	misp_info("%s S", __func__);
	ret = sscanf(cmd_buf, "%s %s", cmd_name, &str_param[0]);
	if (ret != 2) {
		err = -EINVAL;
		return err;
	}
	/* str to u8 */
	ret = kstrtou8(&str_param[0], 0, &param);
	if (ret != ERR_SUCCESS)
		return ret;

	misp_info("%d", param);
	if (param == 1) {
		dev_global_variable = get_mini_isp_global_variable();

		/* load bin file flow*/
		misp_info("%s, fw name: %s", __func__, filename);
		mini_isp_device = mini_isp_getdev();
		if (mini_isp_device != NULL && filename != NULL) {
			err = request_firmware(&fw,
				filename, mini_isp_device);

			if (err) {
				misp_info("%s, L: %d, err: %d",
					__func__, __LINE__, err);
				goto code_end;
			}
		}
		total_size = fw->size;
		if (total_size) {
			data_buf_addr = kzalloc(total_size, GFP_KERNEL);
			if (data_buf_addr == NULL) {
				misp_err("%s - Kzalloc data buf fail ",
						__func__);
				goto code_end;
			}
			memcpy(data_buf_addr, fw->data, total_size);
			if (WP_GPIO != NULL)
				gpio_set_value(dev_global_variable->wp_gpio, 0);

			err = mini_isp_drv_write_calibration_data(
				CALIBRATION_ID_EEPROM,
				data_buf_addr, total_size);
			misp_info("%s write otp data = %x", __func__, err);

			if (WP_GPIO != NULL)
				gpio_set_value(dev_global_variable->wp_gpio, 1);

			kfree(data_buf_addr);
		}
code_end:
		if (fw != NULL)
			release_firmware(fw);
	} else if (param == 2)
		err = mini_isp_drv_read_calibration_data(CALIBRATION_ID_EEPROM);
	else
		misp_err("%s - Not Support ", __func__);

	misp_info("%s E!!", __func__);
	return err;
}

ssize_t echo_mini_isp_calib_op(const char *cmd_buf)
{
	errcode err = ERR_SUCCESS;
	int ret;
	struct misp_global_variable *dev_global_variable;
	u8 filename[80] = "WRITE_CALIB_DATA.bin";
	const struct firmware *fw;
	struct device *mini_isp_device;
	u8 *data_buf_addr;
	u32 total_size;
	u8 param;
	char cmd_name[20];
	char str_param[20];

	misp_info("%s S", __func__);
	ret = sscanf(cmd_buf, "%s %s", cmd_name, &str_param[0]);
	if (ret != 2) {
		err = -EINVAL;
		return err;
	}
	/* str to u8 */
	ret = kstrtou8(&str_param[0], 0, &param);
	if (ret != ERR_SUCCESS)
		return ret;

	misp_info("%d", param);
	if (param == 1) {
		dev_global_variable = get_mini_isp_global_variable();

		/* load bin file flow*/
		misp_info("%s, fw name: %s", __func__, filename);
		mini_isp_device = mini_isp_getdev();
		if (mini_isp_device != NULL && filename != NULL) {
			err = request_firmware(&fw,
				filename, mini_isp_device);

			if (err) {
				misp_info("%s, L: %d, err: %d",
					__func__, __LINE__, err);
				goto code_end;
			}
		}
		total_size = fw->size;
		if (total_size) {
			data_buf_addr = kzalloc(total_size, GFP_KERNEL);
			if (data_buf_addr == NULL) {
				misp_err("%s - Kzalloc data buf fail ",
				__func__);
				goto code_end;
			}
			memcpy(data_buf_addr, fw->data, total_size);
			if (WP_GPIO != NULL)
				gpio_set_value(dev_global_variable->wp_gpio, 0);

			err = mini_isp_drv_write_calibration_data(
				CALIBRATION_ID_ALTEK,
				data_buf_addr,
				total_size);
			misp_info("%s write calib data = %x", __func__, err);

			if (WP_GPIO != NULL)
				gpio_set_value(dev_global_variable->wp_gpio, 1);

			kfree(data_buf_addr);
		}
code_end:
		if (fw != NULL)
			release_firmware(fw);
	} else if (param == 2)
		err = mini_isp_drv_read_calibration_data(CALIBRATION_ID_ALTEK);
	else
		misp_err("%s - Not Support ", __func__);

	misp_info("%s E!!", __func__);
	return err;
}

ssize_t echo_mini_isp_packdata_op(const char *cmd_buf)
{
	errcode errcode = ERR_SUCCESS;
	int ret;
	struct misp_global_variable *dev_global_variable;
	u8 filename[80] = "/data/firmware/";
	struct file *filp;
	mm_segment_t oldfs;
	off_t currpos;
	u8 *data_buf_addr = NULL;
	u32 total_size;
	u8 param;
	char cmd_name[20];
	char str_param[20];

	misp_info("%s S", __func__);
	ret = sscanf(cmd_buf, "%s %s", cmd_name, &str_param[0]);
	if (ret != 2) {
		errcode = -EINVAL;
		return errcode;
	}
	/* str to u8 */
	ret = kstrtou8(&str_param[0], 0, &param);
	if (ret != ERR_SUCCESS)
		return ret;

	misp_info("%d", param);
	if (param == 1) {
		dev_global_variable = get_mini_isp_global_variable();

		strlcat(filename, "/WRITE_PACK_DATA.bin", sizeof(filename));

		oldfs = get_fs();
		set_fs(KERNEL_DS);

		/*open the file*/
#if ENABLE_FILP_OPEN_API
		/*filp = filp_open(filename, O_RDONLY, 0644);*/
#else
		misp_info("Error! Currently not support file open api");
		misp_info("See define ENABLE_FILP_OPEN_API");
		goto EXIT;
#endif
		if (IS_ERR(filp)) {
			misp_err("%s  Open File Failure", __func__);
			errcode = ~ERR_SUCCESS;
			set_fs(oldfs);
			goto code_end;
		}
		/*get the file size*/
		currpos = vfs_llseek(filp, 0L, SEEK_END);
		if (currpos == -1) {
			misp_err("%s  llseek failed", __func__);
			errcode = ~ERR_SUCCESS;
			set_fs(oldfs);
			goto code_end;
		}
		total_size = (u32)currpos;
		misp_info("%s  filesize : %d", __func__, total_size);
		vfs_llseek(filp, 0L, SEEK_SET);

		if (total_size) {
			data_buf_addr = kzalloc(total_size, GFP_KERNEL);
			if (data_buf_addr == NULL) {
			misp_err("%s - Kzalloc data buf fail ", __func__);
			set_fs(oldfs);
			goto code_end;
			}
			errcode = vfs_read(filp, data_buf_addr, total_size,
				&filp->f_pos);

			if (errcode == -1) {
				misp_err("%s - Read file failed.", __func__);
			} else {
				if (WP_GPIO != NULL)
				gpio_set_value(dev_global_variable->wp_gpio, 0);

				errcode = mini_isp_drv_write_calibration_data(
					CALIBRATION_ID_DEPTH,
					data_buf_addr, total_size);
				misp_info("%s write pack data = %x",
				__func__, errcode);

				if (WP_GPIO != NULL)
					gpio_set_value(
					dev_global_variable->wp_gpio, 1);
			}

	}

		set_fs(oldfs);
		/*close the file*/
		filp_close(filp, NULL);
	} else if (param == 2) {
		errcode = mini_isp_drv_read_calibration_data(
			CALIBRATION_ID_DEPTH);
	} else
		misp_err("%s - Not Support ", __func__);

code_end:
	if (data_buf_addr != NULL)
		kfree(data_buf_addr);

	misp_info("%s E!!", __func__);
	return errcode;
}

ssize_t echo_regression_test(const char *cmd_buf)
{
#if EN_REGRESSION_TEST
	size_t errcode = ERR_SUCCESS;
	int ret[2];
	int t_ret;
	char cmd_name[20];
	char str_param[2][20];
	u32 TestID_Start;
	u32 TestID_End;

	misp_info("%s S", __func__);

	t_ret = sscanf(cmd_buf, "%s %s %s", cmd_name,
			&str_param[0][0], &str_param[1][0]);
	ret[0] = t_ret;
	if (ret != 3) {
		errcode = -EINVAL;
		return errcode;
	}

	/* str to u32*/
	ret[0] = kstrtou32(&str_param[0][0], 0, &TestID_Start);
	ret[1] = kstrtou32(&str_param[1][0], 0, &TestID_End);
	if (ret[0] != ERR_SUCCESS)
		return ret[0];
	else if (ret[1] != ERR_SUCCESS)
		return ret[1];

	misp_info("[TestID_Start]: %d, [TestID_End]: %d",
		TestID_Start, TestID_End);
	miniisp_regression_test(TestID_Start, TestID_End);

	misp_info("%s E", __func__);
	return errcode;
#else
	size_t errcode = ERR_SUCCESS;

	misp_info("not enable regression test!");
	return errcode;
#endif
}

ssize_t echo_turnon_Ocram7(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;

	misp_info("%s S", __func__);
	errcode = ispctrl_if_mast_execute_cmd(
		ISPCMD_SYSTEM_OCRAM7_TURNON, NULL);

	misp_info("%s E", __func__);
	return errcode;
}

ssize_t echo_test(const char *cmd_buf)
{
	size_t errcode = ERR_SUCCESS;
	int ret;
	char cmd_name[20];
	char str_param[20];
	u8 cmd_param_buf[T_SPI_CMD_LENGTH];
	u32 param;

	misp_info("%s S", __func__);
	ret = sscanf(cmd_buf, "%s %s", cmd_name, &str_param[0]);
	if (ret != 2) {
		errcode = -EINVAL;
		return errcode;
	}

	/* str to u32*/
	ret = kstrtou32(&str_param[0], 0, &param);
	if (ret != ERR_SUCCESS)
		return ret;

	misp_info("param: %d", param);
	memset(cmd_param_buf, 0x0, T_SPI_CMD_LENGTH);

	if (param == 0)
		ispctrl_if_mast_execute_cmd(ISPCMD_BULK_WRITE_BOOTCODE,
		cmd_param_buf);
	else if (param == 1)
		ispctrl_if_mast_execute_cmd(ISPCMD_BULK_WRITE_BASICCODE,
	cmd_param_buf);
	else if (param == 2) {
		ispctrl_if_mast_execute_cmd(ISPCMD_BULK_WRITE_BOOTCODE,
		cmd_param_buf);
		msleep(500);
		memset(cmd_param_buf, 0x0, T_SPI_CMD_LENGTH);
		ispctrl_if_mast_execute_cmd(ISPCMD_BULK_WRITE_BASICCODE,
		cmd_param_buf);

	}

	if (ret != ERR_SUCCESS) {
		misp_info("%s:L%d Error!", __func__, __LINE__);
		goto EXIT;
	}

EXIT:
	misp_info("%s, E", __func__);
	return errcode;
}

static errcode mini_isp_cmd_write_err_file(errcode a_ErrCode)
{
	int ret = ERR_SUCCESS;
	struct file *pfile;
	mm_segment_t fs;
	char FileName[80] = "/data/firmware/miniisp_cmd_err";

	misp_info("Write err code into %s", FileName);
#if ENABLE_FILP_OPEN_API
	/*pfile = filp_open(FileName, O_CREAT | O_RDWR, 0777);*/
#else
	misp_info("Error! Currently not support file open api");
	misp_info("See define ENABLE_FILP_OPEN_API");
	goto EXIT;
#endif
	if (IS_ERR(pfile)) {
		ret = PTR_ERR(pfile);
		misp_err("%s open file failed. err: %d", __func__, ret);
		set_fs(fs);
		goto EXIT;
	}

	/*Get current segment descriptor*/
	fs = get_fs();
	/*Set segment descriptor associated*/
	set_fs(get_ds());

	vfs_write(pfile, (char *)&a_ErrCode, sizeof(errcode),
		&pfile->f_pos);
	filp_close(pfile, NULL);
EXIT:
	return ret;
}

struct echo_cmd_format {
	u32 opcode;
	char cmd_name[30];
	ssize_t (*pfunc)(const char *cmd_buf);
};

struct echo_cmd_format echo_cmd_list[] = {
	{.opcode = 0x0116,	.cmd_name = "get_peripheral_device",
		.pfunc = echo_mini_isp_drv_get_peripheral_device},
	{.opcode = 0x0117,	.cmd_name = "set_peripheral_device",
		.pfunc = echo_mini_isp_drv_set_peripheral_device},
	{.opcode = 0x0015,	.cmd_name = "get_last_cmd",
		.pfunc = echo_mini_isp_drv_get_last_exec_cmd},
	{.opcode = 0x0119,	.cmd_name = "get_binfile_ver",
		.pfunc = echo_mini_isp_drv_get_binfile_ver},
	{.opcode = 0x10B9,	.cmd_name = "depth_3a_info",
		.pfunc = echo_mini_isp_drv_set_depth_3a_info},
	{.opcode = 0x10BC,	.cmd_name = "interleave",
		.pfunc = echo_mini_isp_drv_set_depth_auto_interleave_mode},
	{.opcode = 0x10BF,	.cmd_name = "set_menu_exposure",
		.pfunc = echo_mini_isp_drv_set_exposure_param},
	{.opcode = 0x10C0,	.cmd_name = "set_depth_stream_size",
		.pfunc = echo_mini_isp_drv_set_depth_stream_size},
	{.opcode = 0x210F,	.cmd_name = "bulk_get_binfile_ver",
		.pfunc = echo_mini_isp_drv_bulk_get_binfile_ver},
	{.opcode = 0x3008,	.cmd_name = "get_sensor",
		.pfunc = echo_mini_isp_drv_get_sensor_mode},
	{.opcode = 0x300A,	.cmd_name = "set_sensor",
		.pfunc = echo_mini_isp_drv_set_sensor_mode},
	{.opcode = 0x300D,	.cmd_name = "output_format",
		.pfunc = echo_mini_isp_drv_set_output_format},
	{.opcode = 0x300E,	.cmd_name = "enter_cp",
		.pfunc = echo_mini_isp_drv_set_cp_mode},
	{.opcode = 0x3010,	.cmd_name = "streamon",
		.pfunc = echo_mini_isp_drv_preview_stream_on_off},
	{.opcode = 0x3012,	.cmd_name = "led_power",
		.pfunc = echo_mini_isp_drv_led_power_control},
	{.opcode = 0x3013,	.cmd_name = "active_ae",
		.pfunc = echo_mini_isp_drv_active_ae},
	{.opcode = 0x3014,	.cmd_name = "ae_onoff",
		.pfunc = echo_mini_isp_drv_isp_ae_control_mode_on_off},
	{.opcode = 0x3015,	.cmd_name = "framerate",
		.pfunc = echo_mini_isp_drv_set_frame_rate_limits},
	{.opcode = 0x3017,	.cmd_name = "set_max_exposure",
		.pfunc = echo_mini_isp_drv_set_max_exposure},
	{.opcode = 0x3019,	.cmd_name = "frame_sync",
		.pfunc = echo_mini_isp_drv_frame_sync_control},
	{.opcode = 0x301A,	.cmd_name = "shot_mode",
		.pfunc = echo_mini_isp_drv_set_shot_mode},
	{.opcode = 0x301B,	.cmd_name = "lighting_ctrl",
		.pfunc = echo_mini_isp_drv_lighting_ctrl},
	{.opcode = 0x301C,	.cmd_name = "depth_compensation",
		.pfunc = echo_mini_isp_drv_depth_compensation},
	{.opcode = 0x301D,	.cmd_name = "cycle_trigger_depth",
		.pfunc = echo_mini_isp_drv_cycle_trigger_depth_process},
	{.opcode = 0x301E,	.cmd_name = "set_min_exposure",
		.pfunc = echo_mini_isp_drv_set_min_exposure},
	{.opcode = 0x301F,	.cmd_name = "set_max_exposure_slop",
		.pfunc = echo_mini_isp_drv_set_max_exposure_slope},
	{.opcode = 0x3020,	.cmd_name = "led_active_delay",
		.pfunc = echo_mini_isp_drv_led_active_delay},
	{.opcode = 0x3021,	.cmd_name = "isp_smart_ae_control",
		.pfunc = echo_mini_isp_drv_isp_smart_ae_control},
	{.opcode = 0x3022,	.cmd_name = "isp_set_group_led_level",
		.pfunc = echo_mini_isp_drv_set_group_led_level},
	{.opcode = 0x3023,	.cmd_name = "isp_ctrl_led_sequence",
		.pfunc = echo_mini_isp_drv_ctrl_led_seq},
	{.opcode = 0x3026,	.cmd_name = "host_ctrl_led_onoff",
		.pfunc = echo_mini_isp_drv_host_ctrl_led_onoff},
	{.opcode = 0x3027,	.cmd_name = "extra_sensor_sync",
		.pfunc = echo_mini_isp_drv_extra_sensor_sync},
	{.opcode = 0x3028,	.cmd_name = "set_depth_type",
		.pfunc = echo_mini_isp_drv_set_depth_type},
	{.opcode = 0x0166,	.cmd_name = "get_chip_thermal",
		.pfunc = echo_mini_isp_drv_get_chip_thermal},
	{.opcode = 0xFFFF,	.cmd_name = "power_on",
		.pfunc = echo_mini_isp_poweron},
	{.opcode = 0xFFFF,	.cmd_name = "power_off",
		.pfunc = echo_mini_isp_poweroff},
	{.opcode = 0xFFFF,	.cmd_name = "load_fw",
		.pfunc = echo_mini_isp_drv_load_fw},
	{.opcode = 0xFFFF,	.cmd_name = "load_cali",
		.pfunc = echo_mini_isp_drv_write_calibration_data},
	{.opcode = 0xFFFF,	.cmd_name = "load_spinor",
		.pfunc = echo_mini_isp_drv_write_spinor_data},
	{.opcode = 0xFFFF,	.cmd_name = "get_chip_id",
		.pfunc = echo_mini_isp_get_chip_id},
	{.opcode = 0xFFFF,	.cmd_name = "leave_cp",
		.pfunc = echo_mini_isp_drv_leave_cp_mode},
	{.opcode = 0xFFFF,	.cmd_name = "comlog",
		.pfunc = echo_get_comlog},
	{.opcode = 0xFFFF,	.cmd_name = "conti_setreg",
		.pfunc = echo_conti_set_register},
	{.opcode = 0xFFFF,	.cmd_name = "setreg",
		.pfunc = echo_set_register},
	{.opcode = 0xFFFF,	.cmd_name = "getreg",
		.pfunc = echo_get_register},
	{.opcode = 0xFFFF,	.cmd_name = "memwrite",
		.pfunc = echo_memwrite},
	{.opcode = 0xFFFF,	.cmd_name = "memdump",
		.pfunc = echo_memdump},
	{.opcode = 0xFFFF,	.cmd_name = "version",
		.pfunc = echo_show_version},
	{.opcode = 0xFFFF,	.cmd_name = "set_fsm_status",
		.pfunc = echo_set_fsm_status},
	{.opcode = 0xFFFF,	.cmd_name = "cfg_cmd_send",
		.pfunc = echo_cfg_cmd_send},
	{.opcode = 0xFFFF,	.cmd_name = "a2e",
		.pfunc = echo_mini_isp_a_to_e},
	{.opcode = 0xFFFF,	.cmd_name = "e2a",
		.pfunc = echo_mini_isp_e_to_a},
	{.opcode = 0xFFFF,	.cmd_name = "chip_init",
		.pfunc = echo_mini_isp_chip_init},
	{.opcode = 0xFFFF,	.cmd_name = "pure_bypass",
		.pfunc = echo_mini_isp_drv_set_bypass_mode},
	{.opcode = 0xFFFF,	.cmd_name = "dump_bypass_reg",
		.pfunc = echo_mini_isp_utility_read_reg_e_mode_for_bypass_use},
	{.opcode = 0xFFFF,	.cmd_name = "dump_normal_reg",
		.pfunc = echo_mini_isp_utility_read_reg_e_mode},
	{.opcode = 0xFFFF,	.cmd_name = "dump_packdata",
		.pfunc = echo_mini_isp_debug_packdata_dump},
	{.opcode = 0xFFFF,	.cmd_name = "dump_IQCalib",
		.pfunc = echo_mini_isp_debug_IQCalib_dump},
	{.opcode = 0xFFFF,	.cmd_name = "dump_Meta",
		.pfunc = echo_mini_isp_debug_metadata_dump},
	{.opcode = 0xFFFF,	.cmd_name = "dump_irp_img",
		.pfunc = echo_mini_isp_debug_dump_img},
	{.opcode = 0xFFFF,	.cmd_name = "dump_depth_reg",
		.pfunc = echo_mini_isp_debug_rect_combo_dump},
	{.opcode = 0xFFFF,	.cmd_name = "debug_depth_info",
		.pfunc = echo_mini_isp_debug_depth_info},
	{.opcode = 0xFFFF,	.cmd_name = "debug_metadata_info",
		.pfunc = echo_mini_isp_debug_metadata_info},
	{.opcode = 0xFFFF,	.cmd_name = "debug_sensor_info",
		.pfunc = echo_mini_isp_debug_sensor_info},
	{.opcode = 0xFFFF,	.cmd_name = "debug_led_info",
		.pfunc = echo_mini_isp_debug_led_info},
	{.opcode = 0xFFFF,	.cmd_name = "debug_rx_fps_info",
		.pfunc = echo_mini_isp_debug_rx_fps_info},
	{.opcode = 0xFFFF,	.cmd_name = "debug_GPIO_status",
		.pfunc = echo_mini_isp_debug_GPIO_Status},
	{.opcode = 0xFFFF,	.cmd_name = "eeprom_wp",
		.pfunc = echo_mini_isp_eeprom_wp},
	{.opcode = 0xFFFF,	.cmd_name = "eeprom_op",
		.pfunc = echo_mini_isp_eeprom_op},
	{.opcode = 0xFFFF,	.cmd_name = "calib_op",
		.pfunc = echo_mini_isp_calib_op},
	{.opcode = 0xFFFF,	.cmd_name = "packdata_op",
		.pfunc = echo_mini_isp_packdata_op},
	{.opcode = 0xFFFF,	.cmd_name = "regression_test",
		.pfunc = echo_regression_test},
	{.opcode = 0xFFFF,	.cmd_name = "turnon_ocram7",
		.pfunc = echo_turnon_Ocram7},
	{.opcode = 0xFFFF,	.cmd_name = "test",
		.pfunc = echo_test},
};

#define echo_cmd_list_len \
	(sizeof(echo_cmd_list) / sizeof(struct echo_cmd_format))

static ssize_t mini_isp_cmd_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/* print cmd list */
	u32 total_len = 0, count = 0;
	u32 i = 0;

	for (i = 0; i < echo_cmd_list_len; i++) {
		count = snprintf(buf, 70, "%s\n", echo_cmd_list[i].cmd_name);
		buf += count; /* move buffer pointer */
		total_len += count;
	}

	return total_len;
}

static ssize_t mini_isp_cmd_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	char cmd_name[20];
	size_t ErrCode = ERR_SUCCESS;
	int ret;
	u32 opcode = 0x0;
	u32 loopidx = 0;

	ret = sscanf(buf, "%s", cmd_name);
	if (ret != 1) {
		ErrCode = -EINVAL;
		mini_isp_cmd_write_err_file((errcode) ErrCode);
		return size;
	}
	/* check first input is opcode or cmd_name */
	if (cmd_name[0] == '0' && cmd_name[1] == 'x') {
		ret = sscanf(cmd_name, "0x%x", &opcode);

		if (ret != 1) {
			ErrCode = -EINVAL;
			mini_isp_cmd_write_err_file((errcode) ErrCode);
			return size;
		}
	}

	for (loopidx = 0; loopidx < echo_cmd_list_len; loopidx++) {
		if (echo_cmd_list[loopidx].opcode == opcode ||
			strcmp(echo_cmd_list[loopidx].cmd_name,
			cmd_name) == 0){

			ErrCode = echo_cmd_list[loopidx].pfunc(buf);
			if (ErrCode) {
				misp_info("%s, err: 0x%zx", __func__, ErrCode);
				mini_isp_cmd_write_err_file((errcode) ErrCode);
			}
			return size;
		}
	}

	misp_info("command not find!");
	return size;
}

static
DEVICE_ATTR(mini_isp_cmd, 0660, mini_isp_cmd_show, mini_isp_cmd_store);

/************************************************************
 *\       Public Function         *
 *\************************************************************/
u32 GET_TRANS_SCEN_INTF(enum MINI_ISP_TRANS_CLASS trans_class)
{
	u32 mask = 0xF << (trans_class*4);

	if (misp_drv_global_variable->bootfromspinor)
	return (MINIISP_TRANS_SCENARIO_SPINOR & mask) >> (trans_class*4);
	else
	return (MINIISP_TRANS_SCENARIO & mask) >> (trans_class*4);
}

/**
 *\brief Get miniisp transfer data interface.
 *\param trans_class    [In] define by MINI_ISP_TRANS_CLASS.
 *\return transfer data interface driver data
 */
struct misp_data *get_mini_isp_intf(
	enum MINI_ISP_TRANS_CLASS trans_class)
{
	switch (GET_TRANS_SCEN_INTF(trans_class)) {
	case INTF_SPI:
		if (misp_drv_global_variable->intf_status & INTF_SPI)
			return get_mini_isp_intf_spi();
	break;
	case INTF_I2C_TOP:
		if (misp_drv_global_variable->intf_status & INTF_I2C_TOP)
			return get_mini_isp_intf_i2c_top_data();
	break;
	case INTF_I2C_SLAVE:
		if (misp_drv_global_variable->intf_status & INTF_I2C_SLAVE)
			return get_mini_isp_intf_i2c_slave_data();
	break;
	case INTF_CCI:
	default:
		misp_err("%s - error. trans_class: %d, intf_status: %d",
		__func__, GET_TRANS_SCEN_INTF(trans_class),
		misp_drv_global_variable->intf_status);
		return NULL;
	break;
	}
	misp_err("%s - error. trans_class: %d, intf_status: %d",
		__func__, GET_TRANS_SCEN_INTF(trans_class),
		misp_drv_global_variable->intf_status);
	return NULL;
}

void set_mini_isp_data(struct misp_data *data, int intf_type)
{
	if (!misp_drv_global_variable)
		misp_err("%s - set global_variable error", __func__);
	else
		misp_drv_global_variable->intf_status |= intf_type;
}

struct misp_global_variable *get_mini_isp_global_variable(void)
{
	if (!misp_drv_global_variable) {
		misp_err("%s - get global_variable error", __func__);
		return NULL;
	} else {
		return misp_drv_global_variable;
	}
}

struct altek_statefsm *get_mini_isp_fsm(void)
{
	if (!altek_state) {
		misp_err("%s - get fsm error", __func__);
		return NULL;
	} else {
		return altek_state;
	}
}


int mini_isp_setup_resource(struct device *dev, struct misp_data *drv_data)
{
	int status = 0;

	if (IsMiniispSetupInitialed == true) {
		misp_err("%s - already initial", __func__);
		goto setup_done;
	}
	misp_info("%s - start", __func__);
	if (misp_drv_global_variable != NULL) {
		misp_err("%s - resource already been setupped", __func__);
		goto setup_done;
	}

	/*step 1: alloc misp_drv_global_variable*/
	misp_drv_global_variable =
		kzalloc(sizeof(*misp_drv_global_variable), GFP_KERNEL);

	if (!misp_drv_global_variable) {
		misp_info("%s - Out of memory", __func__);
		status = -ENOMEM;
		goto alloc_fail;
	}
	misp_info("%s - step1 done.", __func__);

	/*step 2: init mutex and gpio resource*/
	mutex_init(&misp_drv_global_variable->busy_lock);
	sema_init(&misp_drv_global_variable->transfer_lock, 1);
	sema_init(&misp_drv_global_variable->bulk_lock, 1);

	status = mini_isp_gpio_init(dev, drv_data, misp_drv_global_variable);
	if (status < 0) {
		misp_info("%s - gpio init fail", __func__);
		goto setup_fail;
	}
	misp_info("%s - step2 done.", __func__);

	misp_drv_global_variable->before_booting = 1;
	misp_drv_global_variable->en_cmd_send = 1;
	misp_drv_global_variable->altek_spi_mode = ALTEK_SPI_MODE_E;
	/* Set boot_from_spinor to be the default boot type */
	misp_drv_global_variable->bootfromspinor = false;
	/*step 3: register to VFS as character device*/
	mini_isp_class = class_create(THIS_MODULE, "mini_isp");
	if (IS_ERR(mini_isp_class))
		misp_err("Failed to create class(mini_isp_class)!");
	mini_isp_dev = miniisp_chdev_create(mini_isp_class);

	if (IS_ERR(mini_isp_dev))
		misp_err("Failed to create device(mini_isp_dev)!");

	status = device_create_file(mini_isp_dev,
				&dev_attr_mini_isp_mode_config);

	if (status < 0)
		misp_err("Failed to create device file(%s)!",
			dev_attr_mini_isp_mode_config.attr.name);

	if (RESET_GPIO != NULL) {
		status = device_create_file(mini_isp_dev,
						&dev_attr_mini_isp_reset);

		if (status < 0)
			misp_err("Failed to create device file(%s)!",
				dev_attr_mini_isp_reset.attr.name);
	}

	status = device_create_file(mini_isp_dev,
			&dev_attr_mini_isp_rectab);
	if (status < 0)
		misp_err("Failed to create device file(%s)!",
			dev_attr_mini_isp_rectab.attr.name);

	status = device_create_file(mini_isp_dev,
		&dev_attr_mini_isp_cmd);
	if (status < 0)
		misp_err("Failed to create device file(%s)!",
			dev_attr_mini_isp_cmd.attr.name);

	misp_info("%s - step3 done.", __func__);

	misp_info("%s - success.", __func__);
	IsMiniispSetupInitialed = true;
	goto setup_done;

setup_fail:
	mutex_destroy(&misp_drv_global_variable->busy_lock);
	kfree(misp_drv_global_variable);

alloc_fail:
	misp_drv_global_variable = NULL;

setup_done:
	return status;
}

struct device *mini_isp_getdev(void)
{
	return mini_isp_dev;
}

static int __init mini_isp_init(void)
{
	int ret = 0;
	struct altek_statefsm *fsm = NULL;

	misp_info("%s - start", __func__);

	fsm = altek_statefsmcreate();
	altek_state = fsm;

	isp_mast_camera_profile_para_init();

	/* register SPI driver */
	ret = spi_register_driver(&mini_isp_intf_spi);
	if (ret) {
		misp_err("%s - regsiter failed. Errorcode:%d",
			__func__, ret);
	}

	/* register I2C driver */
	ret = i2c_add_driver(&mini_isp_intf_i2c_slave);
	if (ret)
		misp_info("%s - failed. Error:%d", __func__, ret);

	ret = i2c_add_driver(&mini_isp_intf_i2c_top);
	if (ret)
		misp_info("%s - failed. Error:%d", __func__, ret);

	misp_info("MINIISP_DRIVER_VERSION: %s", MINIISP_DRIVER_VERSION);
	misp_info("%s - success", __func__);

	return ret;
}

static void __exit mini_isp_exit(void)
{
	misp_info("%s", __func__);

	if (misp_drv_global_variable->irq_gpio)
		gpio_free(misp_drv_global_variable->irq_gpio);

	/*if (misp_drv_global_variable)*/
		kfree(misp_drv_global_variable);

	altek_statefsmdelete(altek_state);
	altek_state = NULL;

	/* unregister all driver */
	spi_unregister_driver(&mini_isp_intf_spi);
	i2c_del_driver(&mini_isp_intf_i2c_slave);
	i2c_del_driver(&mini_isp_intf_i2c_top);
}

module_init(mini_isp_init);
module_exit(mini_isp_exit);
MODULE_LICENSE("Dual BSD/GPL");
