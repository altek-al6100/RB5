/*
 * File: miniisp_intf_i2c.c
 * Description: Mini ISP i2c sample codes
 *
 * Copyright 2019-2030  Altek Semiconductor Corporation
 *
 *  2017/03/30; Max Tseng; Initial version
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


/****************************************************************************
 *\ File: miniisp_intf_i2c.c                                                  *
 *\ Description: Mini ISP i2c sample codes                                    *
 *\                                                                           *
 *\ History:                                                                  *
 *\   2017/03/30; Max Tseng; Initial version                                  *
 *\****************************************************************************/

/****************************************************************************
 *\                               Include File                               *
 *\***************************************************************************/
/* Linux headers*/
#include <linux/module.h>
#include <linux/buffer_head.h>
#include <linux/delay.h>

#include "include/miniisp.h"
#include "include/miniisp_ctrl.h"
#include "include/error/miniisp_err.h"
#include "include/miniisp_chip_base_define.h"

/****************************************************************************
 *\			Private Constant Definition			*
 *\*************************************************************************/
#define MINI_ISP_LOG_TAG "[miniisp_intf_i2c]"
#define MINIISP_SLAVE_ADDR 0x11
#define MINIISP_TOP_ADDR   0x77

/* linux kernel use u16 to store I2C transfer length, can't exceed 0xFFFF */
#define MAX_I2C_TRANS_LEN 0xFFFF
/****************************************************************************
 *\                          Private Global Variable                        *
 *\**************************************************************************/
static struct misp_data *misp_intf_i2c_slave_data;
static struct misp_data *misp_intf_i2c_top_data;
static u8 *i2c_bulkbuffer;

/****************************************************************************
 *\                        Private Function Prototype                        *
 *\***************************************************************************/

/**********************************************************************
 *\                         Public Function                          *
 *\*********************************************************************/

static const struct i2c_device_id mini_isp_intf_i2c_id[] = {
	{ "altek_i2c_slave", MINIISP_I2C_SLAVE},
	{ "altek_i2c_top", MINIISP_I2C_TOP},
	{} /* NULL terminated*/
};
MODULE_DEVICE_TABLE(i2c, mini_isp_intf_i2c_id);

/**
 *\get mini isp data
 *\return mini isp i2c data
 */
struct misp_data *get_mini_isp_intf_i2c_top_data(void)
{
	if (!misp_intf_i2c_top_data) {
		misp_err("%s - get pdata error", __func__);
		return NULL;
	} else {
		return misp_intf_i2c_top_data;
	}
}

struct misp_data *get_mini_isp_intf_i2c_slave_data(void)
{
	if (!misp_intf_i2c_slave_data) {
		misp_err("%s - get pdata error", __func__);
		return NULL;
	} else {
		return misp_intf_i2c_slave_data;
	}
}
struct misp_data *get_mini_isp_intf_i2c(int i2c_type)
{
	struct misp_data *psmisp_data = NULL;

	if (i2c_type == MINIISP_I2C_SLAVE)
		psmisp_data = get_mini_isp_intf_i2c_slave_data();
	else if (i2c_type == MINIISP_I2C_TOP)
		psmisp_data = get_mini_isp_intf_i2c_top_data();
	else {
		misp_err("%s - error i2c type %d", __func__, i2c_type);
		psmisp_data = NULL;
	}
	return psmisp_data;
}


/****************************************************************************
 *\                             Private Function                             *
 *\***************************************************************************/
/**write command to miniISP through i2c,this function will block.
 *\return 0  successful
 *\others    fail
 */
static int mini_isp_intf_i2c_write(void *dev, u8 *tx_buf, u8 *rx_buf, u32 len)
{
	int status = 0;
	struct misp_data *devdata = (struct misp_data *)dev;
	struct misp_global_variable *misp_drv_global;

	//misp_info("%s S", __func__);
	misp_drv_global = get_mini_isp_global_variable();
	down(&misp_drv_global->transfer_lock);
	if (!devdata) {
		misp_err("%s - invalid arg devdata = %p", __func__, devdata);
		status = -EINVAL;
		goto EXIT;
	}

	if (len > MAX_I2C_TRANS_LEN) {
		misp_err("%s - I2C transfer len err", __func__);
		status = -EINVAL;
		goto EXIT;
	}

	status = i2c_master_send(devdata->cfg.i2c, tx_buf, len);

	if (status < 0)
		misp_err("%s - sync error: status=%d", __func__, status);
	else
		status = 0;

EXIT:
	up(&misp_drv_global->transfer_lock);
	//misp_info("%s E", __func__);

	return status;
}

/*read command from device ,this function will block.
 *return 0  successful
 *others	fail
 */
static int mini_isp_intf_i2c_read(
	void *dev, u8 *tx_buf, u32 tx_len, u8 *rx_buf, u32 rx_len)
{
	int status = ERR_SUCCESS;
	struct misp_data *devdata = (struct misp_data *)dev;
	struct misp_global_variable *misp_drv_global;

	//misp_info("%s S", __func__);
	misp_drv_global = get_mini_isp_global_variable();
	down(&misp_drv_global->transfer_lock);

	if (!devdata) {
		misp_err("%s - invalid arg devdata = %p", __func__, devdata);
		status = -EINVAL;
		goto EXIT;
	}

	if (tx_len > MAX_I2C_TRANS_LEN || rx_len > MAX_I2C_TRANS_LEN) {
		misp_err("%s - I2C transfer len err", __func__);
		status = -EINVAL;
		goto EXIT;
	}

	if (tx_len > 0) {
		status = i2c_master_send(devdata->cfg.i2c, tx_buf, tx_len);
		if (status != tx_len) {
			misp_err("%s - send length err", __func__);
			goto EXIT;
		}
	}

	/* read data from device through i2c */
	status = i2c_master_recv(devdata->cfg.i2c, rx_buf, rx_len);

	if (status < 0)
		misp_err("%s - sync error: status=%d", __func__, status);
	else
		status = ERR_SUCCESS;
EXIT:
	up(&misp_drv_global->transfer_lock);
	//misp_info("%s E", __func__);
	return status;
}

/**
 *write command data to miniISP through i2c
 *return 0  successful
 *others    fail
 */
static int mini_isp_intf_i2c_send(void *dev, u32 len)
{
	int status = 0;
	struct misp_data *devdata = (struct misp_data *)dev;

	misp_info("%s - enter", __func__);

	if (!devdata) {
		misp_err("%s - invalid arg devdata = %p, len= %d",
			__func__, devdata, len);
		return -EINVAL;
	}

	misp_info("%s - i2c_master_send start.", __func__);
	/* send data to miniISP through i2c */
	status = mini_isp_intf_i2c_write(
		devdata, devdata->tx_buf, devdata->rx_buf, len);

	misp_info("%s - devdata->cfg.i2c->addr = %x",
		__func__, devdata->cfg.i2c->addr);
	if (status < 0)
		misp_err("%s - sync error: status=%d", __func__, status);
	else
		status = 0;

	return status;
}

/* read miniISP using i2c ,this function will block.
 *return 0  successful
 *others    fail
 */
static int mini_isp_intf_i2c_recv(void *dev, u32 len, bool waitINT)
{
	int status = 0;
	struct misp_data *devdata = (struct misp_data *)dev;
	struct misp_global_variable *dev_global_variable;

	int original_altek_spi_mode;
	u32 altek_event_state;
	int i = 0;


	misp_info("%s - enter", __func__);
	dev_global_variable = get_mini_isp_global_variable();
	original_altek_spi_mode = dev_global_variable->altek_spi_mode;
	if (!devdata) {
		misp_err("%s - invalid arg devdata=%p,len=%d", __func__,
							devdata, len);
		return -EINVAL;
	}

	if (waitINT)
		/*wait for the interrupt*/
		status = mini_isp_wait_for_event(MINI_ISP_RCV_CMD_READY);
	else {
		mini_isp_a_to_e();
		for (i = 0; i < 200; i++) {
			status = mini_isp_get_altek_status(devdata,
				&altek_event_state);

			if (altek_event_state & COMMAND_COMPLETE) {
				altek_event_state = (altek_event_state &
					~((~0) << 1));
				mini_isp_register_write(
					INTERRUPT_STATUS_REGISTER_ADDR,
					altek_event_state);
				break;
			}
			msleep(20); /* mdelay(5); */
		}

		if (original_altek_spi_mode == ALTEK_SPI_MODE_A)
			mini_isp_e_to_a();
		if (i >= 200) {
			misp_err("%s time out.", __func__);
			status = ERR_MINIISP_GETDATA_TIMEOUT;
			return status;
		}
	}

	if (status) {
		misp_err("%s - irq error: status=%d", __func__, status);
		return status;
	}

	/* receive the data through i2c */
	status = mini_isp_intf_i2c_read(
		devdata, devdata->tx_buf, 0, devdata->rx_buf, len);

	if (status) {
		misp_err("%s - sync error: status=%d", __func__, status);
		return status;
	}

	misp_info("%s - recv buf len=%d:", __func__, len);
	rx_data_debug(devdata->rx_buf, RX_BUF_SIZE, len);
	return status;
}

/*used to send the firmware*/
static int mini_isp_intf_i2c_send_bulk(void *devdata,
	u32 total_size, u32 block_size, bool is_raw,
	const u8 *i2c_Sendbulkbuffer)
{
	int status = 0, count = 0;
	int remain_size, one_size;
	int datablock_size;
	int shift = 0;
	struct misp_global_variable *misp_drv_global;

	misp_info("%s - enter", __func__);
	misp_drv_global = get_mini_isp_global_variable();
	down(&misp_drv_global->bulk_lock);

	if (i2c_Sendbulkbuffer != NULL) {
		misp_info("%s start. Total size: %d. block_size: %d",
			__func__, total_size, block_size);

		if (total_size > I2C_TX_BULK_SIZE)
			i2c_bulkbuffer = kzalloc(I2C_TX_BULK_SIZE, GFP_DMA);
		/* Allocate boot code bulk buffer*/
		else
			i2c_bulkbuffer = kzalloc(total_size, GFP_DMA);

		if (!i2c_bulkbuffer) {
			status = -EINVAL;
			goto T_EXIT;
		}

		i2c_bulkbuffer[0] = I2C_TX_BULK_CTRL_BYTE;
		datablock_size = block_size - 1;
		for (remain_size = total_size; remain_size > 0;
			remain_size -= one_size) {
			one_size = (remain_size > datablock_size) ?
				datablock_size : remain_size;

			misp_info("remain size: %d one_size: %d.",
				remain_size, one_size);

			memcpy(&i2c_bulkbuffer[1], i2c_Sendbulkbuffer +
			shift, one_size);
			shift += one_size;

			/*send the data*/
			status = mini_isp_intf_i2c_write(devdata,
					i2c_bulkbuffer, NULL, one_size + 1);

			if (status != 0) {
				misp_err("%s failed! block:%d status:%d",
					__func__, count, status);
				break;
			}
			misp_info("%s write block %d success", __func__, count);
			count++;
		}
	}

T_EXIT:

	if (i2c_bulkbuffer != NULL) {
		kfree(i2c_bulkbuffer);
		i2c_bulkbuffer = NULL;
	}

	up(&misp_drv_global->bulk_lock);
	if (status != ERR_SUCCESS)
		misp_err("%s error: %d", __func__, status);
	else
		misp_info("%s success", __func__);

	return status;
}

struct misp_intf_fn_t intf_i2c_fn = {
	.send = mini_isp_intf_i2c_send,
	.recv = mini_isp_intf_i2c_recv,
	.read = mini_isp_intf_i2c_read,
	.write = mini_isp_intf_i2c_write,
	.send_bulk = mini_isp_intf_i2c_send_bulk,
};

static int mini_isp_intf_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int status = 0;
	struct misp_data *drv_data = NULL;

	misp_info("%s - start, addr[0x%x].", __func__, client->addr);

	/* step 0: Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		misp_err("%s - i2c_check_functionality err.", __func__);
		return -EIO;
	}
	/*step 1: alloc driver data struct*/
	drv_data = kmalloc(sizeof(struct misp_data), GFP_KERNEL);
	if (!drv_data) {
		status = -ENOMEM;
		goto alloc_misp_data_fail;
	}
	misp_info("%s - step1 done.", __func__);

	/*step 2: init driver data*/
	drv_data->cfg.i2c = client;
	drv_data->intf_fn = &intf_i2c_fn;
	drv_data->bulk_cmd_blocksize = I2C_TX_BULK_SIZE;
	misp_info("%s - step2 done.", __func__);

	/*step 3: setup recource : gpio, sem*/
	if (id->driver_data == MINIISP_I2C_SLAVE) {
		misp_intf_i2c_slave_data = drv_data;
		i2c_set_clientdata(client, misp_intf_i2c_slave_data);
		status = mini_isp_setup_resource(&client->dev, drv_data);
		if (status < 0) {
			misp_err("%s step3. probe - setup resource error",
				__func__);
			goto setup_i2c_error;
		}
		set_mini_isp_data(drv_data, INTF_I2C_SLAVE);
		misp_info("%s - misp_intf_i2c_slave_data set done.", __func__);
	} else if (id->driver_data == MINIISP_I2C_TOP) {
		misp_intf_i2c_top_data = drv_data;
		i2c_set_clientdata(client, misp_intf_i2c_top_data);
		status = mini_isp_setup_resource(&client->dev, drv_data);
		if (status < 0) {
			misp_err("%s step3. probe - setup resource error",
				__func__);
			goto setup_i2c_error;
		}
		set_mini_isp_data(drv_data, INTF_I2C_TOP);
		misp_info("%s - misp_intf_i2c_top_data set done.", __func__);
	}

	goto done;

setup_i2c_error:
	kfree(misp_intf_i2c_slave_data);
	misp_intf_i2c_slave_data = NULL;

alloc_misp_data_fail:

done:
	return status;

}
static int mini_isp_intf_i2c_remove(struct i2c_client *client)
{
	struct misp_data *misp_intf_i2c = i2c_get_clientdata(client);

	if (!misp_intf_i2c) {
		misp_err("%s: i2c data is NULL\n", __func__);
		return 0;
	}

	kfree(misp_intf_i2c);

	return 0;
}
static const struct of_device_id mini_isp_dt_i2c_slave_match[] = {
		{  .compatible  =  "altek_i2c_slave",},
		/*Compatible  node  must  match  dts*/
		{  },
		};

MODULE_DEVICE_TABLE(of, mini_isp_dt_i2c_slave_match);

static const struct of_device_id mini_isp_dt_i2c_top_match[] = {
		{  .compatible  =  "altek_i2c_top",},
		/*Compatible  node  must  match  dts*/
		{  },
		};

MODULE_DEVICE_TABLE(of, mini_isp_dt_i2c_top_match);

struct i2c_driver mini_isp_intf_i2c_slave = {
	.driver = {
		.name =         "altek_i2c_slave",
		.owner =        THIS_MODULE,
		.of_match_table  = mini_isp_dt_i2c_slave_match,
	},
	.probe =        mini_isp_intf_i2c_probe,
	.remove =       mini_isp_intf_i2c_remove,
	.id_table =     mini_isp_intf_i2c_id,
};

struct i2c_driver mini_isp_intf_i2c_top = {
	.driver = {
		.name =         "altek_i2c_top",
		.owner =        THIS_MODULE,
		.of_match_table  = mini_isp_dt_i2c_top_match,
	},
	.probe =        mini_isp_intf_i2c_probe,
	.remove =       mini_isp_intf_i2c_remove,
	.id_table =     mini_isp_intf_i2c_id,
};
