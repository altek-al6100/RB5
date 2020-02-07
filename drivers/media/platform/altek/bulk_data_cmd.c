/*
 * File:  bulk_data_cmd.c
 * Description: Mini ISP sample codes
 *
 * Copyright 2019-2030  Altek Semiconductor Corporation
 *
 *  2013/10/14; Bruce Chung; Initial version
 *  2013/12/05; Bruce Chung; 2nd version
 *  2016/05/05; Louis Wang; Linux Coding Style
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
#include <linux/buffer_head.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>
#include <linux/firmware.h>
#include "include/isp_camera_cmd.h"
#include "include/ispctrl_if_master.h"
#include "include/error/ispctrl_if_master_err.h"
#include "include/error/miniisp_err.h"
#include "include/miniisp.h"
#include "include/ispctrl_if_master_local.h"
#include "include/miniisp_customer_define.h"
#include "include/altek_statefsm.h"

/******Private Constant Definition******/
#define LOGSIZE  (4*1024)
#define RAWBLOCKSIZE SPI_TX_BULK_SIZE_BOOT
#define MINI_ISP_LOG_TAG	"[[miniisp]bulk_data_cmd]"
#define MID_PJ_EXEBIN_BUF (1024*1024)

/*Private Type Declaration*/

/*Calibration data buffer address*/
static u8 *calibration_data_buf_addr;

u16 g_fw_version_before_point;
u16 g_fw_version_after_point;
char g_fw_build_by[9];
char g_fw_project_name[17];
u32 g_sc_build_date;

/******Private Function Prototype******/

/******Private Global Variable******/


/******Public Global Variable*******/

/******Public Function******/

const u8 *fw_data;
/**
 *\brief Write Boot Code
 *\param devdata [In], misp_data
 *\param param [In], CMD param
 *\param filp [In], boot code file pointer
 *\return Error code
 */
errcode mast_bulk_data_cmd_write_boot_code(void *devdata,
						u8 *param)
{
	errcode err = ERR_SUCCESS;
	u32 total_size;
	u32 curpos;
	const struct firmware *fw;
	struct device *mini_isp_device;
	u32 ProductId_val = 0;
	u16 miniboot_version_before_point = 0;
	u16 miniboot_version_after_point = 0;
	u8 ProductId[4];
	u8 miniboot_ver_major[2];
	u8 miniboot_ver_minor[2];
	char miniboot_build_by[9];
	char *fw_name;

	/* check status */
	if (g_load_code_status != MINI_ISP_FW_NONE) {
		err = ERR_MINIISP_REQUESTFIRMWARE_FAILED;
		misp_info("%s, L: %d, err: %d", __func__, __LINE__, err);
		goto mast_bulk_data_cmd_write_boot_code_end;
	} else
		g_load_code_status = MINI_ISP_BOOT_CODE_LOADING;

	mini_isp_check_and_leave_bypass_mode();

	/* load boot fw file */
	mini_isp_device = mini_isp_getdev();
	if (mini_isp_device != NULL) {
		fw_name = strrchr(BOOT_FILE_LOCATION, '/');
		err = request_firmware(&fw,
			fw_name, mini_isp_device);
		if (err) {
			misp_info("%s, L: %d, err: %d",
				__func__, __LINE__, err);
			goto mast_bulk_data_cmd_write_boot_code_end;
		}
	}

	total_size = fw->size;
	fw_data = fw->data;

	/*Transfer boot code*/
	/* boot code & main code can only be sent by SPI */
	err = ispctrl_if_mast_send_bulk(devdata,
		fw_data, total_size, RAWBLOCKSIZE, true);

	if (err != ERR_SUCCESS)
		goto mast_bulk_data_cmd_write_boot_code_end;

	misp_info("%s send boot code success", __func__);

	/* Get miniboot version */
	curpos = total_size - 16;
	memcpy(ProductId, &fw_data[curpos], 4);
	curpos += 4;
	memcpy(miniboot_ver_major, &fw_data[curpos], 2);
	curpos += 2;
	memcpy(miniboot_ver_minor, &fw_data[curpos], 2);
	curpos += 2;
	memcpy(miniboot_build_by, &fw_data[curpos], 8);

	if (err == -1) {
		misp_info("%s - Read file failed.", __func__);
	} else {
		err = 0;
		ProductId_val = (ProductId[3]<<24) + (ProductId[2]<<16) +
				(ProductId[1] << 8) + ProductId[0];
		miniboot_version_before_point = miniboot_ver_major[1]*256 +
						miniboot_ver_major[0];
		miniboot_version_after_point = miniboot_ver_minor[1]*256 +
						miniboot_ver_minor[0];
		miniboot_build_by[8] = '\0';
		misp_info("%s - miniboot version: %d.%d.%d, build by %s",
			__func__, ProductId_val, miniboot_version_before_point,
			miniboot_version_after_point, miniboot_build_by);
	}

mast_bulk_data_cmd_write_boot_code_end:
	if (fw != NULL)
		release_firmware(fw);

	if (err == ERR_SUCCESS)
		g_load_code_status = MINI_ISP_BOOT_CODE_FINISH;

	return err;
}

/**
 *\brief Write Boot Code (Short SPI Len)
 *\param devdata [In], misp_data
 *\param param [In], CMD param
 *\param filp [In], boot code file pointer
 *\return Error code
 */
errcode
mast_bulk_data_cmd_write_boot_code_shortlen(void *devdata,
						u8 *param)
{
	errcode err = ERR_SUCCESS;
	u32 total_size;
	u32 block_size;
	u32 curpos;
	const struct firmware *fw;
	struct device *mini_isp_device;
	u32 ProductId_val = 0;
	u16 miniboot_version_before_point = 0;
	u16 miniboot_version_after_point = 0;
	u8 ProductId[4];
	u8 miniboot_ver_major[2];
	u8 miniboot_ver_minor[2];
	char miniboot_build_by[9];
	char *fw_name;

	/* check status */
	if (g_load_code_status != MINI_ISP_FW_NONE) {
		err = ERR_MINIISP_REQUESTFIRMWARE_FAILED;
		misp_info("%s, L: %d, err: %d", __func__, __LINE__, err);
		goto mast_bulk_data_cmd_write_boot_code_end;
	} else
		g_load_code_status = MINI_ISP_BOOT_CODE_LOADING;

	mini_isp_check_and_leave_bypass_mode();

	/* load boot fw file */
	mini_isp_device = mini_isp_getdev();
	if (mini_isp_device != NULL) {
		fw_name = strrchr(BOOT_FILE_LOCATION, '/');
		err = request_firmware(&fw,
			fw_name, mini_isp_device);
		if (err) {
			misp_info("%s, err: %d", __func__, err);
			goto mast_bulk_data_cmd_write_boot_code_end;
		}
	}

	block_size = SPI_BLOCK_LEN;
	total_size = fw->size;
	fw_data = fw->data;
	/*misp_info("%s  filesize : %d", __func__, total_size);*/
	/*misp_info("block_size %d", RAWBLOCKSIZE);*/

	/*Transfer boot code*/
	/* boot code & main code can only be sent by SPI */
	err = ispctrl_if_mast_send_bulk(devdata,
		fw_data, total_size, block_size, true);

	if (err != ERR_SUCCESS)
		goto mast_bulk_data_cmd_write_boot_code_end;

	misp_info("%s send boot code success", __func__);

	/* Get miniboot version */
	curpos = total_size - 16;
	memcpy(ProductId, &fw_data[curpos], 4);
	curpos += 4;
	memcpy(miniboot_ver_major, &fw_data[curpos], 2);
	curpos += 2;
	memcpy(miniboot_ver_minor, &fw_data[curpos], 2);
	curpos += 2;
	memcpy(miniboot_build_by, &fw_data[curpos], 8);

	if (err == -1) {
		misp_info("%s - Read file failed.", __func__);
	} else {
		err = 0;
		ProductId_val = (ProductId[3]<<24) + (ProductId[2]<<16) +
				(ProductId[1] << 8) + ProductId[0];
		miniboot_version_before_point = miniboot_ver_major[1]*256 +
						miniboot_ver_major[0];
		miniboot_version_after_point = miniboot_ver_minor[1]*256 +
						miniboot_ver_minor[0];
		miniboot_build_by[8] = '\0';
		misp_info("%s - miniboot version: %d.%d.%d, build by %s",
			__func__, ProductId_val, miniboot_version_before_point,
			miniboot_version_after_point, miniboot_build_by);
	}
	/* Get miniboot version */

mast_bulk_data_cmd_write_boot_code_end:
	if (fw != NULL)
		release_firmware(fw);

	if (err == ERR_SUCCESS)
		g_load_code_status = MINI_ISP_BOOT_CODE_FINISH;

	return err;
}

/**
 *\brief Write Basic Code
 *\param devdata [In], misp_data
 *\param param [In], CMD param
 *\param filp  [In], basic code file pointer
 *\return Error code
 */
errcode mast_bulk_data_cmd_write_basic_code(void *devdata,
						u8 *param)
{
	errcode err = ERR_SUCCESS;
	u8 fw_version[4];
	u32 para_size = ISPCMD_EXEBIN_INFOBYTES;
	u32 *total_size = (u32 *)&param[ISPCMD_EXEBIN_ADDRBYTES];
	u32 file_total_size;
	u32 block_size;
	u32 currpos;
	const struct firmware *fw;
	struct device *mini_isp_device;
	char *fw_name;

	if (g_load_code_status != MINI_ISP_BOOT_CODE_FINISH) {
		err = ERR_MINIISP_REQUESTFIRMWARE_FAILED;
		misp_info("%s, L: %d, err: %d", __func__, __LINE__, err);
		goto mast_bulk_data_cmd_write_basic_code_end;
	} else
		g_load_code_status = MINI_ISP_MAIN_CODE_LOADING;

	block_size = ((struct misp_data *)devdata)->bulk_cmd_blocksize;
	/* load boot fw file */
	mini_isp_device = mini_isp_getdev();
	if (mini_isp_device != NULL) {
		fw_name = strrchr(BASIC_FILE_LOCATION, '/');
		err = request_firmware(&fw,
			fw_name, mini_isp_device);
		if (err) {
			misp_info("%s, L: %d, err: %d",
				__func__, __LINE__, err);
			goto mast_bulk_data_cmd_write_basic_code_end;
		}
	}

	file_total_size = fw->size;
	fw_data = fw->data;

	/*read the header info (first 16 bytes in the basic code)*/
	memcpy(param, fw_data, ISPCMD_EXEBIN_INFOBYTES);

	/*To copy checksum value to correct header point*/
	memcpy((u8 *)(param + ISPCMD_EXEBIN_ADDRBYTES +
		ISPCMD_EXEBIN_TOTALSIZEBYTES + ISPCMD_EXEBIN_BLOCKSIZEBYTES),
		(u8 *)(param + ISPCMD_EXEBIN_ADDRBYTES +
		ISPCMD_EXEBIN_TOTALSIZEBYTES), sizeof(u32));
	/*Assign block size to correct header point*/
	memcpy((u8 *)(param + ISPCMD_EXEBIN_ADDRBYTES +
		ISPCMD_EXEBIN_TOTALSIZEBYTES), &block_size, sizeof(u32));
	/*
	 * misp_info("%s param[0][1][2][3]: %02x %02x %02x %02x",
	 *  __func__, param[0], param[1], param[2], param[3]);
	 * misp_info("%s param[4][5][6][7]: %02x %02x %02x %02x",
	 *  __func__, param[4], param[5], param[6], param[7]);
	 * misp_info("%s param[8][9][10][11]: %02x %02x %02x %02x",
	 *  __func__, param[8], param[9], param[10], param[11]);
	 * misp_info("%s param[12][13][14][15]: %02x %02x %02x %02x",
	 *  __func__, param[12], param[13], param[14], param[15]);
	 */

	misp_info("block size: %d", block_size);
	misp_info("Total fw size: %zu", fw->size);
	misp_info("fw size: %d", ((u32 *)param)[1]);
	misp_info("0x%x, 0x%x, 0x%x, 0x%x",
		((u32 *)fw->data)[0], ((u32 *)fw->data)[1],
		((u32 *)fw->data)[2], ((u32 *)fw->data)[3]);

	/*Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata,
		ISPCMD_BULK_WRITE_BASICCODE, param, para_size);
	if (err != ERR_SUCCESS) {
		misp_info("%s, L: %d, err: %d", __func__, __LINE__, err);
		goto mast_bulk_data_cmd_write_basic_code_end;
	}

	/*misp_info("%s send leaking packet success", __func__);*/

	/*misp_info("block_size %d", BLOCKSIZE);*/
	misp_info("%s send basic code start", __func__);
	/*Transfer basic code*/
	err = ispctrl_if_mast_send_bulk(devdata,
		fw_data + ISPCMD_EXEBIN_INFOBYTES,
		*total_size, block_size, false);

	if (err != ERR_SUCCESS) {
		misp_info("%s, L: %d, err: %d", __func__, __LINE__, err);
		goto mast_bulk_data_cmd_write_basic_code_end;
	}

	/*wait for the interrupt*/
	err = mini_isp_wait_for_event(MINI_ISP_RCV_BULKDATA);

	if (err != ERR_SUCCESS) {
		misp_info("%s, L: %d, err: %d", __func__, __LINE__, err);
		goto mast_bulk_data_cmd_write_basic_code_end;
	}

	misp_info("%s - send basic code success", __func__);

	currpos = file_total_size - 32;
	memcpy(g_fw_project_name, &fw_data[currpos], 16);
	currpos = file_total_size - 12;
	memcpy(fw_version, &fw_data[currpos], 4);
	currpos += 4;
	memcpy(g_fw_build_by, &fw_data[currpos], 8);

	err = 0;
	g_fw_version_before_point = fw_version[1] * 256 + fw_version[0];
	g_fw_version_after_point = fw_version[3] * 256 + fw_version[2];
	g_fw_build_by[8] = '\0';
	g_fw_project_name[16] = '\0';
	misp_info("%s project: %s, fw version: %05d.%05d, build by %s",
		  __func__, g_fw_project_name, g_fw_version_before_point,
		  g_fw_version_after_point, g_fw_build_by);

mast_bulk_data_cmd_write_basic_code_end:
	if (fw != NULL)
		release_firmware(fw);

	if (err == ERR_SUCCESS)
		g_load_code_status = MINI_ISP_MAIN_CODE_FINISH;

	return err;
}

/**
 *\brief Write Basic Code (Short SPI Len)
 *\param devdata [In], misp_data
 *\param param [In], CMD param
 *\param filp  [In], basic code file pointer
 *\return Error code
 */
errcode mast_bulk_data_cmd_write_basic_code_shortlen(
				void *devdata, u8 *param)
{
	errcode err = ERR_SUCCESS;
	u8 fw_version[4];
	/* u32 para_size = ISPCMD_EXEBIN_INFOBYTES; */
	u32 *total_size = (u32 *)&param[ISPCMD_EXEBIN_ADDRBYTES];
	u32 file_total_size;
	u32 block_size;
	off_t currpos;
	const struct firmware *fw;
	struct device *mini_isp_device;
	char *fw_name;

	if (g_load_code_status != MINI_ISP_BOOT_CODE_FINISH) {
		err = ERR_MINIISP_REQUESTFIRMWARE_FAILED;
		misp_info("%s, L: %d, err: %d", __func__, __LINE__, err);
		goto mast_bulk_data_cmd_write_basic_code_shortlen_end;
	} else
		g_load_code_status = MINI_ISP_MAIN_CODE_LOADING;

	/* load boot fw file */
	mini_isp_device = mini_isp_getdev();
	if (mini_isp_device != NULL) {
		fw_name = strrchr(BASIC_FILE_LOCATION, '/');
		err = request_firmware(&fw,
			fw_name, mini_isp_device);
		if (err) {
			misp_info("%s, err: %d", __func__, err);
			goto mast_bulk_data_cmd_write_basic_code_shortlen_end;
		}
	}

	block_size = SPI_BLOCK_LEN;

	/*get the file size*/
	file_total_size = fw->size;
	fw_data = fw->data;
	/*misp_info("%s  filesize : %u", __func__, file_total_size);*/


	/*read the header info (first 16 bytes in the basic code)*/
	memcpy(param, fw_data, ISPCMD_EXEBIN_INFOBYTES);

	/*To copy checksum value to correct header point*/
	memcpy((u8 *)(param + ISPCMD_EXEBIN_ADDRBYTES +
		  ISPCMD_EXEBIN_TOTALSIZEBYTES + ISPCMD_EXEBIN_BLOCKSIZEBYTES),
		(u8 *)(param + ISPCMD_EXEBIN_ADDRBYTES +
		ISPCMD_EXEBIN_TOTALSIZEBYTES), sizeof(u32));
	/*Assign block size to correct header point*/
	memcpy((u8 *)(param + ISPCMD_EXEBIN_ADDRBYTES +
		ISPCMD_EXEBIN_TOTALSIZEBYTES), &block_size, sizeof(u32));


	misp_info("%s param[0][1][2][3]: %02x %02x %02x %02x",
		__func__, param[0], param[1], param[2], param[3]);
	misp_info("%s param[4][5][6][7]: %02x %02x %02x %02x",
		__func__, param[4], param[5], param[6], param[7]);
	misp_info("%s param[8][9][10][11]: %02x %02x %02x %02x",
		__func__, param[8], param[9], param[10], param[11]);
	misp_info("%s param[12][13][14][15]: %02x %02x %02x %02x",
		__func__, param[12], param[13], param[14], param[15]);



	/* misp_info("%s Main Code Address  >>>> param[0]: %04x",
	 *	__func__, *(u32 *)&param[0]);
	 * misp_info("%s Main Code Size     >>>> param[4]: %04x",
	 *	__func__, *(u32 *)&param[4]);
	 * misp_info("%s Main Code Checksum >>>> param[12]: %04x",
	 *	__func__, *(u32 *)&param[12]);
	 */

	/*Send command to slave*/
	/* Step 1: Main Code Address */
	err = ispctrl_mast_send_cmd_to_slave(devdata,
		ISPCMD_BULK_WRITE_BASICCODE_CODEADDR,
		&param[0], ISPCMD_EXEBIN_ADDRBYTES);

	if (err != ERR_SUCCESS)
		goto mast_bulk_data_cmd_write_basic_code_shortlen_end;

	msleep(20); /*mdelay(1);*/

	/* Step 2: Main Code Size */
	err = ispctrl_mast_send_cmd_to_slave(devdata,
		ISPCMD_BULK_WRITE_BASICCODE_CODESIZE,
		&param[4], ISPCMD_EXEBIN_TOTALSIZEBYTES);

	if (err != ERR_SUCCESS)
		goto mast_bulk_data_cmd_write_basic_code_shortlen_end;

	msleep(20); /*mdelay(1);*/

	/* Step 3: Main Code Checksum */
	err = ispctrl_mast_send_cmd_to_slave(devdata,
		ISPCMD_BULK_WRITE_BASICCODE_CODESUM,
		&param[12], ISPCMD_EXEBIN_CKSUMBYTES);

	if (err != ERR_SUCCESS)
		goto mast_bulk_data_cmd_write_basic_code_shortlen_end;


	/*misp_info("%s send leaking packet success", __func__);*/

	/*misp_info("block_size %d", BLOCKSIZE);*/
	misp_info("%s send basic code start", __func__);
	/*Transfer basic code*/
	err = ispctrl_if_mast_send_bulk(devdata,
		fw_data + ISPCMD_EXEBIN_INFOBYTES,
		*total_size, block_size, false);

	if (err != ERR_SUCCESS)
		goto mast_bulk_data_cmd_write_basic_code_shortlen_end;

	misp_info("%s - send basic code success", __func__);


	/* mdelay(1); */

	/* wait for the interrupt */
	err = mini_isp_wait_for_event(MINI_ISP_RCV_BULKDATA);

	if (err != ERR_SUCCESS)
		goto mast_bulk_data_cmd_write_basic_code_shortlen_end;

	currpos = file_total_size - 32;
	memcpy(g_fw_project_name, &fw_data[currpos], 16);
	currpos = file_total_size - 12;
	memcpy(fw_version, &fw_data[currpos], 4);
	currpos += 4;
	memcpy(g_fw_build_by, &fw_data[currpos], 8);

	err = 0;
	g_fw_version_before_point = fw_version[1] * 256 + fw_version[0];
	g_fw_version_after_point = fw_version[3] * 256 + fw_version[2];
	g_fw_build_by[8] = '\0';
	g_fw_project_name[16] = '\0';
	misp_info("%s project: %s, fw version: %05d.%05d, build by %s",
		  __func__, g_fw_project_name, g_fw_version_before_point,
		  g_fw_version_after_point, g_fw_build_by);

mast_bulk_data_cmd_write_basic_code_shortlen_end:
	if (fw != NULL)
		release_firmware(fw);

	if (err == ERR_SUCCESS)
		g_load_code_status = MINI_ISP_MAIN_CODE_FINISH;

	return err;
}

/**
 *\brief Write Calibration Data
 *\param devdata [In], misp_data
 *\param param [In], CMD param
 *\param filp  [In], calibration data file pointer
 *\return Error code
 */
errcode
mast_bulk_data_cmd_write_calibration_data(void *devdata,
						u8 *param)
{
	#define IS_FROM_AP_BUF (infomode == CALIBRATION_ID_OTP || \
				infomode == CALIBRATION_ID_DEPTH || \
				infomode == CALIBRATION_ID_BLENDINGTABLE || \
				infomode == CALIBRATION_ID_EEPROM || \
				infomode == CALIBRATION_ID_ALTEK)
	errcode err = ERR_SUCCESS;
	u8 infomode;
	u16 opcode;
	u16 ckecksum;
	u32 para_size = 11;
	u32 filesize;
	u32 block_size;
	const char *file_name;
	const struct firmware *fw;
	struct device *mini_isp_device;

	infomode = param[8];
	block_size = ((struct misp_data *)devdata)->bulk_cmd_blocksize;

	/* Trasfered files have been opened in kernel
	 * when mini_isp_drv_load_fw()
	 */

	/* load boot fw file */
	if (infomode == CALIBRATION_ID_SCID)
		file_name = strrchr(
		(SCENARIO_TABLE_FILE_LOCATION ?
		SCENARIO_TABLE_FILE_LOCATION : ""), '/');
	else if (infomode == CALIBRATION_ID_HDR)
		file_name = strrchr(
		(HDR_QMERGE_DATA_FILE_LOCATION ?
		HDR_QMERGE_DATA_FILE_LOCATION : ""), '/');
	else if (infomode == CALIBRATION_ID_IRP0)
		file_name = strrchr(
		(IRP0_QMERGE_DATA_FILE_LOCATION ?
		IRP0_QMERGE_DATA_FILE_LOCATION : ""), '/');

	else if (infomode == CALIBRATION_ID_IRP1)
		file_name = strrchr(
		(IRP1_QMERGE_DATA_FILE_LOCATION ?
		IRP1_QMERGE_DATA_FILE_LOCATION : ""), '/');

	else if (infomode == CALIBRATION_ID_PPMAP)
		file_name = strrchr(
		(PP_MAP_FILE_LOCATION ?
		PP_MAP_FILE_LOCATION : ""), '/');

	else if (infomode == CALIBRATION_ID_BLENDINGTABLE)
		file_name = NULL;
	else if (infomode == CALIBRATION_ID_QMERGE)
		file_name = strrchr(
		(DPETH_QMERGE_DATA_FILE_LOCATION ?
		DPETH_QMERGE_DATA_FILE_LOCATION : ""), '/');
	else if (infomode == CALIBRATION_ID_AE)
		file_name = strrchr(
		(AE_TUNNING_DATA_FILE_LOCATION ?
		AE_TUNNING_DATA_FILE_LOCATION : ""), '/');
	else
		file_name = NULL;

	/* skip char '/' */
	if (file_name != NULL)
		file_name = file_name + 1;

	/* AP provide data buffer */
	if (IS_FROM_AP_BUF) {
		misp_info("%s, buf_len: %d", __func__, *(u32 *)&param[0]);
		memcpy(&filesize, param, sizeof(u32));
		calibration_data_buf_addr = kzalloc(filesize, GFP_KERNEL);
		if (!calibration_data_buf_addr) {
			err = ~ERR_SUCCESS;
			kfree(calibration_data_buf_addr);
			misp_info("%s, L: %d, err: %d",
					__func__, __LINE__, err);
			goto cmd_write_calibration_data_end;
		}

		memcpy(calibration_data_buf_addr,
			param + T_SPI_CMD_LENGTH, filesize);

		ckecksum = calculate_check_sum(
			(const u8 *) calibration_data_buf_addr, filesize);

		fw_data = (const u8 *) calibration_data_buf_addr;
	} else {
		/* load bin file flow*/
		misp_info("%s, fw name: %s", __func__, file_name);
		mini_isp_device = mini_isp_getdev();
		if (mini_isp_device != NULL && file_name != NULL) {
			err = request_firmware(&fw,
				file_name, mini_isp_device);

			if (err) {
				misp_info("%s, L: %d, err: %d",
					__func__, __LINE__, err);
				goto cmd_write_calibration_data_end;
			}
		}
		filesize = fw->size;
		fw_data = fw->data;
		misp_info("file size: %d", filesize);

		if (infomode == CALIBRATION_ID_SCID) {
			g_sc_build_date =
				fw_data[16+3]*256*256*256 +
				fw_data[16+2]*256*256 +
				fw_data[16+1]*256 +
				fw_data[16+0];
			misp_info("%s - SC table build date %d.", __func__,
				g_sc_build_date);
		}

		ckecksum = calculate_check_sum(fw_data, filesize);
	}

	/*Assign Info ID to correct header point*/
	memcpy((u8 *)(param + 8), &infomode, sizeof(u8));
	/*To copy Total Size to correct header point*/
	memcpy((u8 *)param, &filesize, sizeof(u32));
	/*Assign block size to correct header point*/
	memcpy((u8 *)(param + 4), &block_size, sizeof(u32));
	/*Assign check sum to correct header point*/
	memcpy((u8 *)(param + 9), &ckecksum, sizeof(u16));

	if (SPI_SHORT_LEN_MODE && SPI_SHORT_LEN_MODE_WRITE_ENABLE) {
		opcode = ISPCMD_BULK_WRITE_CALIBRATION_NO_BLOCK;
		/* Total size(4byte) + data id(1byte) + checksum(2byte)*/
		para_size = 7;
		/* left shift 4byte */
		memcpy((u8 *)(param + 4), (u8 *)(param + 8),
			sizeof(u32) + sizeof(u16));
		block_size = SPI_BLOCK_LEN;
	} else
		opcode = ISPCMD_BULK_WRITE_CALIBRATION_DATA;

	/*Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata,
		opcode, param, para_size);

	if (err != ERR_SUCCESS) {
		misp_info("%s L:%d err: %d", __func__, __LINE__, err);
		goto cmd_write_calibration_data_end;
	}

	/*misp_info("%s send leaking packet success", __func__);*/
	/*misp_info("block_size %d", BLOCKSIZE);*/

	err = mini_isp_wait_for_event(MINI_ISP_RCV_CMD_READY);
	if (err) {
		misp_info("%s L:%d err: %d", __func__, __LINE__, err);
		goto cmd_write_calibration_data_end;
	}

	err = ispctrl_if_mast_send_bulk(devdata,
		fw_data, filesize, block_size, false);

	if (err != ERR_SUCCESS) {
		misp_info("%s L:%d err: %d", __func__, __LINE__, err);
		goto cmd_write_calibration_data_end;
	}

	err = mini_isp_wait_for_event(MINI_ISP_RCV_BULKDATA);
	if (err) {
		misp_info("%s L:%d err: %d", __func__, __LINE__, err);
		goto cmd_write_calibration_data_end;
	}

	if (infomode == CALIBRATION_ID_OTP)
		misp_info("%s write IQ calibration data success", __func__);
	else if (infomode == CALIBRATION_ID_DEPTH)
		misp_info("%s write depth packet data success", __func__);
	else if (infomode == CALIBRATION_ID_SCID)
		misp_info("%s write scenario table success", __func__);
	else if (infomode == CALIBRATION_ID_HDR)
		misp_info("%s write HDR Qmerge data success", __func__);
	else if (infomode == CALIBRATION_ID_IRP0)
		misp_info("%s write IRP0 Qmerge data success", __func__);
	else if (infomode == CALIBRATION_ID_IRP1)
		misp_info("%s write IRP1 Qmerge data success", __func__);
	else if (infomode == CALIBRATION_ID_PPMAP)
		misp_info("%s write PP map success", __func__);
	else if (infomode == CALIBRATION_ID_BLENDINGTABLE)
		misp_info("%s write blending table success", __func__);
	else if (infomode == CALIBRATION_ID_QMERGE)
		misp_info("%s write Depth Qmerge data success", __func__);
	else if (infomode == CALIBRATION_ID_EEPROM)
		misp_info("%s write EEPROM data success", __func__);
	else if (infomode == CALIBRATION_ID_ALTEK)
		misp_info("%s write Altek data success", __func__);
	else if (infomode == CALIBRATION_ID_AE)
		misp_info("%s write AE tuning data success", __func__);
cmd_write_calibration_data_end:
	if (IS_FROM_AP_BUF && calibration_data_buf_addr != NULL)
		kfree(calibration_data_buf_addr);
	else if (fw != NULL)
		release_firmware(fw);

	return err;

}

/*
 *\brief Read Calibration Data
 *\param devdata [In], misp_data
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_bulk_data_cmd_read_calibration_data(void *devdata,
							u8 *param)
{
	/*Error Code*/
	errcode err = ERR_SUCCESS;
	u8 *allocated_memmory = 0;
	u32 read_size = 0;
	u8 filename[80] = "/data/firmware/";
	u16 org_chk_sum;
	/*
	 *Parameter size
	 *4bytes for start addr, 4 bytes for total size, 4 bytes for block size,
	 *4 bytes for memory dump mode
	 */
	u32 para_size = sizeof(struct read_calib_info);
	/*Total size*/
	u32 total_size;
	struct read_calib_info *read_calib_config;
	struct file *f;
	mm_segment_t fs;

	read_size = MID_PJ_EXEBIN_BUF;

	/*Request memory*/
	allocated_memmory = kzalloc(read_size, GFP_KERNEL);
	if (!allocated_memmory) {
		err = ~ERR_SUCCESS;
		misp_err("%s Allocate memory failed.", __func__);
		goto allocate_memory_fail;
	}


	read_calib_config = (struct read_calib_info *)param;

	/*Assign total size*/
	total_size = read_calib_config->total_size + ISPCMD_CMDSIZEWDUMMY
					+ ISPCMD_CKSUMBYTES;
	if (total_size > read_size) {
		err = ERR_MASTERCMDSIZE_MISMATCH;
		misp_err("%s total_size error.", __func__);
		goto mast_bulk_data_cmd_read_memory_data_end;
	}

	/*Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata,
		ISPCMD_BULK_READ_CALIBRATION_DATA, param, para_size);
	if (err != ERR_SUCCESS) {
		kfree(allocated_memmory);
		misp_err("%s send command fail, 0x%x.", __func__, err);
		goto mast_bulk_data_cmd_read_memory_data_end;
	}
	/*Get memory data from slave*/
	err = ispctrl_if_mast_recv_memory_data_from_slave(devdata,
			allocated_memmory,
			&total_size,
			read_calib_config->block_size,
			true);
	if (err  != ERR_SUCCESS) {
		kfree(allocated_memmory);
		misp_err("%s get bulk fail, 0x%x.", __func__, err);
		goto mast_bulk_data_cmd_read_memory_data_end;
	}

	misp_info("%s - Read memory finished.", __func__);
	/*checksum*/
	memcpy(&org_chk_sum,
		&allocated_memmory[(total_size - ISPCMD_CKSUMBYTES)],
		ISPCMD_CKSUMBYTES);
	if (org_chk_sum != calculate_check_sum(allocated_memmory,
				(total_size - ISPCMD_CKSUMBYTES))) {
		misp_err("%s - checksum error", __func__);
		err = ERR_MASTERCMDCKSM_INVALID;
		goto mast_bulk_data_cmd_read_memory_data_end;
	}
	/*write out allocated_memmory to file here*/
	/*** add your codes here ***/
	if (read_calib_config->calib_id == CALIBRATION_ID_DEPTH)
		strlcat(filename, "/READ_PACK_DATA.bin", sizeof(filename));
	else if (read_calib_config->calib_id == CALIBRATION_ID_EEPROM)
		strlcat(filename, "/READ_OTP_DATA.bin", sizeof(filename));
	else if (read_calib_config->calib_id == CALIBRATION_ID_ALTEK)
		strlcat(filename, "/READ_CALIB_DATA.bin", sizeof(filename));
#if ENABLE_FILP_OPEN_API
	/*f = filp_open(filename, O_CREAT | O_RDWR, 0777);*/
#else
	misp_info("Error! Currently not support file open api");
	misp_info("See define ENABLE_FILP_OPEN_API");
	goto file_open_fail;
#endif
	/*Get current segment descriptor*/
	fs = get_fs();
	/*Set segment descriptor associated*/
	set_fs(get_ds());

	if (IS_ERR(f)) {
		err = PTR_ERR(f);
		misp_err("%s open file failed. err: %d", __func__, err);
		set_fs(fs);
		goto file_open_fail;
	}

	/*write the file*/
	vfs_write(f, (char *)(allocated_memmory+ISPCMD_CMDSIZEWDUMMY),
				read_calib_config->total_size,
				&f->f_pos);

	/*Restore segment descriptor*/
	set_fs(fs);
	filp_close(f, NULL);
	/*** end of the codes ***/
file_open_fail:
allocate_memory_fail:
	kfree(allocated_memmory);
mast_bulk_data_cmd_read_memory_data_end:

	return err;


}

/*
 *\brief Write Memory Data
 *\param devdata [In], misp_data
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_bulk_data_cmd_write_memory_data(void *devdata,
							u8 *param)
{
	errcode err = ERR_SUCCESS;
	u8 send_cmd_buf[T_SPI_CMD_LENGTH] = {0};
	u16 checksum;
	u8 *psend_data_buf = NULL;
	u32 param_size = 10;
	u32 block_size = SPI_TX_BULK_SIZE;
	struct memory_write_info *pMem_W_Info = NULL;

	pMem_W_Info = (struct memory_write_info *) param;
	/* copy send buffer address */
	memcpy(&psend_data_buf,
			param + sizeof(struct memory_write_info),
			sizeof(psend_data_buf));
	checksum = calculate_check_sum(psend_data_buf, pMem_W_Info->total_size);

	memcpy(send_cmd_buf, &pMem_W_Info->start_addr, sizeof(u32));
	memcpy(send_cmd_buf + 4, &pMem_W_Info->total_size, sizeof(u32));
	memcpy(send_cmd_buf + 8, &checksum, sizeof(u16));

	/* configure block size */
	if (GET_TRANS_SCEN_INTF(MINIISP_BULK_OPCODE) == INTF_SPI)
		block_size = SPI_TX_BULK_SIZE;
	else if (GET_TRANS_SCEN_INTF(MINIISP_BULK_OPCODE) == INTF_I2C_TOP)
		block_size = I2C_TX_BULK_SIZE;

	/*Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata,
		ISPCMD_BULK_WRITE_MEMORY, send_cmd_buf, param_size);

	if (err != ERR_SUCCESS) {
		misp_info("%s, L: %d err!", __func__, __LINE__);
		return err;
	}

	/*misp_info("%s send leaking packet success", __func__);*/
	/*misp_info("block_size %d", BLOCKSIZE);*/

	err = mini_isp_wait_for_event(MINI_ISP_RCV_CMD_READY);
	if (err != ERR_SUCCESS) {
		misp_info("%s, L: %d err!", __func__, __LINE__);
		return err;
	}

	err = ispctrl_if_mast_send_bulk(devdata,
		psend_data_buf, pMem_W_Info->total_size,
		block_size, false);

	if (err != ERR_SUCCESS) {
		misp_info("%s, L: %d err!", __func__, __LINE__);
		return err;
	}

	err = mini_isp_wait_for_event(MINI_ISP_RCV_BULKDATA);

	return err;
}

/*
 *\brief Read Memory Data
 *\param devdata [In], misp_data
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_bulk_data_cmd_read_memory_data(void *devdata,
							u8 *param)
{
	/*Error Code*/
	errcode err = ERR_SUCCESS;
	u8 *allocated_memmory = 0;
	u32 read_size = 0;
	u8 filename[80];
	/*
	 *Parameter size
	 *4bytes for start addr, 4 bytes for total size, 4 bytes for block size,
	 *4 bytes for memory dump mode
	 */
	u32 para_size = sizeof(struct memmory_dump_hdr_info);
	/*Total size*/
	u32 total_size;
	struct memmory_dump_hdr_info *memory_dump_hdr_config;
	struct file *f;
	mm_segment_t fs;

	read_size = MID_PJ_EXEBIN_BUF;

	/*Request memory*/
	allocated_memmory = kzalloc(read_size, GFP_KERNEL);
	if (!allocated_memmory) {
		err = ~ERR_SUCCESS;
		misp_err("%s Allocate memory failed.", __func__);
		goto allocate_memory_fail;
	}


	memory_dump_hdr_config = (struct memmory_dump_hdr_info *)param;

	/*Assign total size*/
	total_size = memory_dump_hdr_config->total_size;
	if (total_size > read_size) {
		err = ERR_MASTERCMDSIZE_MISMATCH;
		misp_err("%s total_size error.", __func__);
		goto mast_bulk_data_cmd_read_memory_data_end;
	}

	/*Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata,
		ISPCMD_BULK_READ_MEMORY, param, para_size);
	if (err != ERR_SUCCESS) {
		kfree(allocated_memmory);
		misp_err("%s send command fail, 0x%x.", __func__, err);
		goto mast_bulk_data_cmd_read_memory_data_end;
	}
	/*Get memory data from slave*/
	err = ispctrl_if_mast_recv_memory_data_from_slave(devdata,
			allocated_memmory,
			&total_size,
			memory_dump_hdr_config->block_size,
			true);
	if (err  != ERR_SUCCESS) {
		kfree(allocated_memmory);
		misp_err("%s get bulk fail, 0x%x.", __func__, err);
		goto mast_bulk_data_cmd_read_memory_data_end;
	}

	misp_info("%s - Read memory finished.", __func__);
	/*write out allocated_memmory to file here*/
	/*** add your codes here ***/
	snprintf(filename, 80, "%s/miniISP_memory.log",
			MINIISP_INFO_DUMPLOCATION);
#if ENABLE_FILP_OPEN_API
	/*f = filp_open(filename, O_CREAT | O_RDWR, 0777);*/
#else
	misp_info("Error! Currently not support file open api");
	misp_info("See define ENABLE_FILP_OPEN_API");
	goto file_open_fail;
#endif
	/*Get current segment descriptor*/
	fs = get_fs();
	/*Set segment descriptor associated*/
	set_fs(get_ds());

	if (IS_ERR(f)) {
		err = PTR_ERR(f);
		misp_err("%s open file failed. err: %d", __func__, err);
		set_fs(fs);
		goto file_open_fail;
	}

	/*write the file*/
	vfs_write(f, (char *)allocated_memmory, total_size,
		&f->f_pos);

	/*Restore segment descriptor*/
	set_fs(fs);
	filp_close(f, NULL);
	/*** end of the codes ***/
file_open_fail:
allocate_memory_fail:
	kfree(allocated_memmory);
mast_bulk_data_cmd_read_memory_data_end:

	return err;


}

/*
 *\brief Read common log data
 *\param devdata [In], misp_data
 *\param param [In], CMD param
 *\return Error code
 */
errcode bulk_data_cmd_read_common_log(void *devdata, u8 *param)
{
	/*Error Code*/
	errcode err = ERR_SUCCESS;
	u8 *allocated_memmory = 0;
	u32 read_size = LOGSIZE;
	struct file *f;
	mm_segment_t fs;
	u8 filename[80];

	/*Parameter size : 4 bytes for total size, 4 bytes for block size*/
	u32 para_size = sizeof(struct common_log_hdr_info);
	/*Total size*/
	u32 total_size;
	struct common_log_hdr_info *common_log_hdr_cfg;


	/*Request memory*/
	allocated_memmory = kzalloc(read_size, GFP_KERNEL);
	if (!allocated_memmory) {
		err = ~ERR_SUCCESS;
		goto allocate_memory_fail;
	}


	common_log_hdr_cfg = (struct common_log_hdr_info *)param;

	/*Assign total size*/
	total_size = common_log_hdr_cfg->total_size;
	if (total_size > read_size) {
		err = ERR_MASTERCMDSIZE_MISMATCH;
		kfree(allocated_memmory);
		goto bulk_data_cmd_read_common_log_end;
	}

	/*Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata,
		ISPCMD_BULK_READ_COMLOG, param, para_size);
	if (err != ERR_SUCCESS) {
		kfree(allocated_memmory);
		misp_info("%s err - 0x%x.", __func__, err);
		goto bulk_data_cmd_read_common_log_end;
	}
	misp_info("%s - Start to read log.", __func__);

	/*
	 *Get memory data from slave,don't wait INT
	 *and use polling interval to wait
	 */
	err = ispctrl_if_mast_recv_memory_data_from_slave(devdata,
			allocated_memmory,
			&total_size,
			common_log_hdr_cfg->block_size,
			true);
	if (err  != ERR_SUCCESS) {
		kfree(allocated_memmory);
		goto bulk_data_cmd_read_common_log_end;
	}
	misp_info("%s - Read log finished.", __func__);
	snprintf(filename, 80, "%s/miniISP_Common_Log.log",
		MINIISP_INFO_DUMPLOCATION);
#if ENABLE_FILP_OPEN_API
	/*f = filp_open(filename, O_CREAT | O_RDWR, 0777);*/
#else
	misp_info("Error! Currently not support file open api");
	misp_info("See define ENABLE_FILP_OPEN_API");
	goto file_open_fail;
#endif
	/*Get current segment descriptor*/
	fs = get_fs();

	/*Set segment descriptor associated*/
	set_fs(get_ds());

	if (IS_ERR(f)) {
		err = PTR_ERR(f);
		misp_err("%s open file failed. err: %d", __func__, err);
		goto file_open_fail;
	}
	/*write the file*/
	vfs_write(f, (char *)allocated_memmory, strlen(allocated_memmory),
		&f->f_pos);

file_open_fail:
	/*Restore segment descriptor*/
	set_fs(fs);
	filp_close(f, NULL);

	goto bulk_data_cmd_read_common_log_end;
allocate_memory_fail:

	misp_err("%s Allocate memory failed.", __func__);
bulk_data_cmd_read_common_log_end:
	kfree(allocated_memmory);
	return err;
}


/*
 *\brief Get depth Rect A, B, Invrect
 *\param devdata [In], misp_data
 *\param opcode [In], Operation code
 *\param param [In], CMD param
 *\return Error code
 */
errcode mast_bulk_data_cmd_write_depth_rectab_invrect(
	void *devdata, u16 opcode, u8 *param)
{
	/*Error Code */
	errcode err = ERR_SUCCESS;
	u8 *allocated_memmory = 0;
	u32 para_size = 0;
	u32 rect_set_num = 0;
	u32 send_total_bulk_size = 0;
	u8 send_cmd[T_SPI_CMD_LENGTH];
	struct isp_cmd_depth_rectab_invrect_info *depth_rect_info;

	depth_rect_info = (struct isp_cmd_depth_rectab_invrect_info *)param;
	if (depth_rect_info->trans_mode == 0)
		rect_set_num = 3;
	else
		rect_set_num = 1;

	send_total_bulk_size =
		rect_set_num*sizeof(struct depth_rectab_invrect_param);

	/* Fill command buffer for send */
	memcpy(&send_cmd[0], &(depth_rect_info->trans_mode),
		sizeof(depth_rect_info->trans_mode));
	para_size = sizeof(depth_rect_info->trans_mode);

	memcpy(&send_cmd[para_size], &(depth_rect_info->block_size),
		sizeof(depth_rect_info->block_size));
	para_size += sizeof(depth_rect_info->block_size);

	/*Request memory*/
	allocated_memmory = kzalloc(send_total_bulk_size, GFP_KERNEL);
	if (!allocated_memmory) {
		err = ~ERR_SUCCESS;
		goto allocate_memory_fail;
	}

	/*Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata,
		ISPCMD_BULK_WRITE_DEPTH_RECTAB_INVRECT,
		(u8 *)&send_cmd[0], para_size);

	if (err  != ERR_SUCCESS)
		goto mast_bulk_data_cmd_write_depth_rectab_invrect_end;

	err = mini_isp_wait_for_event(MINI_ISP_RCV_CMD_READY);
	if (err)
		goto mast_bulk_data_cmd_write_depth_rectab_invrect_end;

	/* Send bulk data to slave*/
	err = ispctrl_if_mast_send_bulk(
			devdata,
			(u8 *)&depth_rect_info->rect_param[0],
			send_total_bulk_size,
			depth_rect_info->block_size, false);

	if (err != ERR_SUCCESS)
		goto mast_bulk_data_cmd_write_depth_rectab_invrect_end;

	err = mini_isp_wait_for_event(MINI_ISP_RCV_BULKDATA);
	if (err)
		goto mast_bulk_data_cmd_write_depth_rectab_invrect_end;

mast_bulk_data_cmd_write_depth_rectab_invrect_end:
	kfree(allocated_memmory);
	allocated_memmory = NULL;

allocate_memory_fail:
	misp_err("%s Allocate memory failed.", __func__);

	return err;
}

/**
 *\brief Write Spinor Data
 *\param devdata [In], misp_data
 *\param param [In], CMD param
 *\param filp  [In], the data file pointer
 *\return Error code
 */
errcode mast_bulk_data_cmd_write_spinor_data(void *devdata,
						u8 *param)
{
	errcode err = ERR_SUCCESS;
	u8 infomode;
	u16 opcode;
	u16 ckecksum;
	u32 para_size = 11;
	u32 filesize;
	u32 block_size;
	const char *file_name;
	const struct firmware *fw = NULL;
	struct device *mini_isp_device;
	const u8 *fw_data;

	infomode = param[8];
	block_size = ((struct misp_data *)devdata)->bulk_cmd_blocksize;

	if (infomode == SPINOR_ID_BOOT)
		file_name = strrchr(
		BOOT_SPINOR_FILE_LOCATION ?
		BOOT_SPINOR_FILE_LOCATION : "", '/');
	else if (infomode == SPINOR_ID_MAIN)
		file_name = strrchr(
		BASIC_FILE_LOCATION ?
		BASIC_FILE_LOCATION : "", '/');
	else if (infomode == SPINOR_ID_SCID)
		file_name = strrchr(
		SCENARIO_TABLE_FILE_LOCATION ?
		SCENARIO_TABLE_FILE_LOCATION : "", '/');
	else if (infomode == SPINOR_ID_HDR)
		file_name = strrchr(
		HDR_QMERGE_DATA_FILE_LOCATION ?
		HDR_QMERGE_DATA_FILE_LOCATION : "", '/');
	else if (infomode == SPINOR_ID_IRP0)
		file_name = strrchr(
		IRP0_QMERGE_DATA_FILE_LOCATION ?
		IRP0_QMERGE_DATA_FILE_LOCATION : "", '/');
	else if (infomode == SPINOR_ID_IRP1)
		file_name = strrchr(
		IRP1_QMERGE_DATA_FILE_LOCATION ?
		IRP1_QMERGE_DATA_FILE_LOCATION : "", '/');
	else if (infomode == SPINOR_ID_DEPTHBIN)
		file_name = strrchr(
		DPETH_QMERGE_DATA_FILE_LOCATION ?
		DPETH_QMERGE_DATA_FILE_LOCATION : "", '/');
	else {
		misp_err("%s not support infomode: %x", __func__, infomode);
		goto mast_bulk_data_cmd_write_spinor_data_end;
	}

	mini_isp_device = mini_isp_getdev();
	if (mini_isp_device != NULL && file_name != NULL) {
		err = request_firmware(&fw,
			file_name, mini_isp_device);

		if (err) {
			misp_info("%s, L: %d, err: %d",
				__func__, __LINE__, err);
			goto mast_bulk_data_cmd_write_spinor_data_end;
		}
	}
	/*get the file size*/
	filesize = fw->size;
	fw_data = fw->data;

	ckecksum = calculate_check_sum(fw_data, filesize);
	misp_info("ckecksum %d", ckecksum);
	/*Assign Info ID to correct header point*/
	memcpy((u8 *)(param + 8), &infomode, sizeof(u8));
	/*To copy Total Size to correct header point*/
	memcpy((u8 *)param, &filesize, sizeof(u32));
	/*Assign block size to correct header point*/
	memcpy((u8 *)(param + 4), &block_size, sizeof(u32));
	/*Assign check sum to correct header point*/
	memcpy((u8 *)(param + 9), &ckecksum, sizeof(u16));

	opcode = ISPCMD_BULK_WRITE_SPINOR_DATA;

	/*Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata,
		opcode, param, para_size);
	if (err != ERR_SUCCESS) {
		misp_info("%s, L: %d, err: %d", __func__, __LINE__, err);
		goto mast_bulk_data_cmd_write_spinor_data_end;
	}

	/*misp_info("%s send leaking packet success", __func__);*/

	/*misp_info("block_size %d", BLOCKSIZE);*/

	err = mini_isp_wait_for_event(MINI_ISP_RCV_CMD_READY);
	if (err) {
		misp_info("%s, L: %d, err: %d", __func__, __LINE__, err);
		goto mast_bulk_data_cmd_write_spinor_data_end;
	}

	err = ispctrl_if_mast_send_bulk(devdata,
		fw_data, filesize, block_size, false);

	if (err != ERR_SUCCESS) {
		misp_info("%s, L: %d, err: %d", __func__, __LINE__, err);
		goto mast_bulk_data_cmd_write_spinor_data_end;
	}

	err = mini_isp_wait_for_event(MINI_ISP_RCV_BULKDATA);

	if (err) {
		misp_info("%s, L: %d, err: %d", __func__, __LINE__, err);
		goto mast_bulk_data_cmd_write_spinor_data_end;
	}

	if (infomode == 0)
		misp_info("%s write spinor boot data success", __func__);
	else if (infomode == 1)
		misp_info("%s write spinor main data success", __func__);

mast_bulk_data_cmd_write_spinor_data_end:
	if (fw != NULL)
		release_firmware(fw);

	return err;

}

errcode mast_bulk_data_cmd_get_binfile_version(void *devdata, u8 *param)
{
	/*Error Code*/
	errcode err = ERR_SUCCESS;
	u32 para_size = sizeof(struct common_log_hdr_info);
	u32 read_data_size = sizeof(struct isp_cmd_binfile_version);
	struct isp_cmd_binfile_version *ptBinFileVer = NULL;
	struct common_log_hdr_info tCmdParaInfo = {0};
	u8 *allocated_memmory = 0;
	u16 org_chk_sum;
	u32 read_total_size;
	u32 block_size = I2C_TX_BULK_SIZE;

	/* update block size */
	if (GET_TRANS_SCEN_INTF(MINIISP_BULK_OPCODE) == INTF_SPI) {
		if (SPI_SHORT_LEN_MODE_READ_ENABLE)
			block_size = SPI_BLOCK_LEN;
		else
			block_size = SPI_TX_BULK_SIZE;
	} else if (GET_TRANS_SCEN_INTF(MINIISP_BULK_OPCODE) == INTF_I2C_TOP)
		block_size = I2C_TX_BULK_SIZE;
	else
		block_size = I2C_TX_BULK_SIZE;

	/*Assign total size*/
	read_total_size = read_data_size + ISPCMD_CMDSIZEWDUMMY
						+ ISPCMD_CKSUMBYTES;
	tCmdParaInfo.total_size = read_total_size;
	tCmdParaInfo.block_size = block_size;

	/*Request memory*/
	allocated_memmory = kzalloc(read_total_size, GFP_KERNEL);
	if (!allocated_memmory) {
		err = ~ERR_SUCCESS;
		misp_err("%s Allocate memory failed.", __func__);
		goto EXIT;
	}

	/*Send command to slave*/
	err = ispctrl_mast_send_cmd_to_slave(devdata,
		ISPCMD_BULK_GET_BINFILE_VER, (u8 *)&tCmdParaInfo, para_size);
	if (err != ERR_SUCCESS) {
		misp_err("%s send command fail, 0x%x.", __func__, err);
		goto EXIT;
	}
	/*Get memory data from slave*/
	err = ispctrl_if_mast_recv_memory_data_from_slave(devdata,
			allocated_memmory,
			&read_total_size,
			block_size,
			true);
	if (err  != ERR_SUCCESS) {
		misp_err("%s get bulk fail, 0x%x.", __func__, err);
		goto EXIT;
	}

	misp_info("%s - Read memory finished.", __func__);

	/*checksum*/
	memcpy(&org_chk_sum,
		&allocated_memmory[(read_total_size - ISPCMD_CKSUMBYTES)],
		ISPCMD_CKSUMBYTES);
	if (org_chk_sum != calculate_check_sum(allocated_memmory,
				(read_total_size - ISPCMD_CKSUMBYTES))) {
		misp_err("%s - checksum error", __func__);
		err = ERR_MASTERCMDCKSM_INVALID;
		goto EXIT;
	}

	/* rxBuf => |resp_len|opcode|DummyByte|resp_data|chksum| */
	ptBinFileVer = (struct isp_cmd_binfile_version *)
				(allocated_memmory + ISPCMD_CMDSIZEWDUMMY);
	misp_info("%s [MainCode_ver]: %d.%d.%d", __func__,
		ptBinFileVer->ulMainCode_ProductID,
		ptBinFileVer->ulMainCode_MajVer,
		ptBinFileVer->ulMainCode_MinVer);

	misp_info("%s [BootCode_ver]: %d.%d.%d", __func__,
		ptBinFileVer->ulBootCode_ProductID,
		ptBinFileVer->ulBootCode_Ver >> 16, /*Major ver*/
		ptBinFileVer->ulBootCode_Ver & 0xFFFF); /*Minor ver*/

	misp_info("%s [SCTable Project]: %s, [SCTable Type]: %d, [SCTable_Ver]: %d",
		__func__,
		ptBinFileVer->aucSCTablesTag, ptBinFileVer->ucSCTableType,
		ptBinFileVer->ucSCTable_Ver);

	misp_info("%s [HDR_ver]: %d.%d, [IRP0_ver]: %d.%d", __func__,
		ptBinFileVer->ulHDR_Ver / 1000,
		ptBinFileVer->ulHDR_Ver % 1000,
		ptBinFileVer->ulIRP0_Ver / 1000,
		ptBinFileVer->ulIRP0_Ver % 1000);

	misp_info("%s [IRP1_ver]: %d.%d, [Depth_ver]: [%d][%d].%d", __func__,
		ptBinFileVer->ulIRP1_Ver / 1000,
		ptBinFileVer->ulIRP1_Ver % 1000,
		ptBinFileVer->ulDepth_Ver / 10000,
		(ptBinFileVer->ulDepth_Ver % 10000)/1000,
		ptBinFileVer->ulDepth_Ver % 1000);

	misp_info("%s [AE_Ver]: %d.%d", __func__,
		ptBinFileVer->ulAE_MajorVer, ptBinFileVer->ulAE_MinorVer);
	/*** end of the codes ***/
EXIT:
	if (allocated_memmory != NULL)
		kfree(allocated_memmory);

	return err;
}


/******Private Function******/
u16 calculate_check_sum(const u8 *input_buffer_addr, u32 input_buffer_size)
{
	u32 i;
	u32 sum = 0;
	u16 sumvalue;

	/*calculating unit is 2 bytes */
	for (i = 0; i < input_buffer_size; i++) {
		if (0 == (i%2))
			sum += input_buffer_addr[i];
		else
			sum += (input_buffer_addr[i] << 8);
	}

	/*Do 2's complement */
	sumvalue = (u16)(65536 - (sum & 0x0000FFFF));

	return sumvalue;
}

/******End Of File******/
