/*
 * File: miniisp_customer_define.c
 * Description: Mini ISP sample codes
 *
 * Copyright 2019-2030  Altek Semiconductor Corporation
 *
 * 2017/03/14 LouisWang; Initial version
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
/* Linux headers*/
#include <linux/delay.h>
#include  <linux/of_gpio.h>

#include "include/miniisp_customer_define.h"
#include "include/miniisp.h"
#include "include/miniisp_ctrl.h"
#include "include/altek_statefsm.h"
#include "include/altek_state.h"
#include "include/error/miniisp_err.h"
#define MINI_ISP_LOG_TAG "[miniisp_customer_define]"


int mini_isp_power_open(void)
{
	int ret = 0;
	struct misp_global_variable *dev_global_variable;
	struct misp_data *devdata = NULL;
	u32 altek_event_state = 0;

	misp_info("%s - enter", __func__);
	dev_global_variable = get_mini_isp_global_variable();

	if (RESET_GPIO != NULL) {
		gpio_direction_output(dev_global_variable->reset_gpio, 0);
		gpio_set_value(dev_global_variable->reset_gpio, 0);
		msleep(20);
	}

	if (VCC1_GPIO != NULL)
		gpio_set_value(dev_global_variable->vcc1_gpio, 0);
	if (VCC2_GPIO != NULL)
		gpio_set_value(dev_global_variable->vcc2_gpio, 0);
	if (VCC3_GPIO != NULL)
		gpio_set_value(dev_global_variable->vcc3_gpio, 0);

	msleep(20);

	if (VCC1_GPIO != NULL)
		gpio_set_value(dev_global_variable->vcc1_gpio, 1);
	if (VCC2_GPIO != NULL)
		gpio_set_value(dev_global_variable->vcc2_gpio, 1);
	if (VCC3_GPIO != NULL)
		gpio_set_value(dev_global_variable->vcc3_gpio, 1);

	msleep(20);
	if (ISP_CLK != NULL)
		if (clk_prepare_enable(dev_global_variable->isp_clk) < 0)
			misp_err("mini_isp_poweron clk_prepare_enable failed");

	if (RESET_GPIO != NULL) {
		gpio_direction_output(dev_global_variable->reset_gpio, 1);
		gpio_set_value(dev_global_variable->reset_gpio, 1);
		msleep(20);
	}

	if (dev_global_variable->bootfromspinor) {
		misp_info("SPINOR - wait main code ready...");
		/* polling main code ready INT status */
		devdata = get_mini_isp_intf(MINIISP_CFG_REG_MEM);
		if (!devdata) {
			misp_info("%s err, devdata is null ", __func__);
			return -ENODEV;
		}
		msleep(100);

		do {
			misp_info("%s - polling...", __func__);
			/* It should be SPI_E mode by default */
			mini_isp_get_altek_status(devdata, &altek_event_state);

			if (altek_event_state & SYSTEM_ERROR_LEVEL1) {
				mini_isp_e_to_a();
				/* need to port out of this ISR */
				mini_isp_drv_get_err_code_cmd_in_irq();
				mini_isp_a_to_e();
				break;
			} else if (altek_event_state & SYSTEM_ERROR_LEVEL2) {
				mini_isp_utility_read_reg_e_mode();
				break;
			} else if (altek_event_state & MINI_ISP_RCV_BULKDATA) {
				mini_isp_register_write(0xffef00b4,
				MINI_ISP_RCV_BULKDATA);
				break;
			}
			msleep(50);
		} while ((altek_event_state & MINI_ISP_RCV_BULKDATA) == 0);

		g_load_code_status = MINI_ISP_MAIN_CODE_FINISH;
		dev_global_variable->before_booting = 0;
		misp_info("SPINOR - main code done");
	} else {
		g_load_code_status = MINI_ISP_FW_NONE;
		dev_global_variable->before_booting = 1;
	}
#if (ISR_MECHANISM == INTERRUPT_METHOD)
	/* register IRQ */
	if (IRQ_GPIO != NULL) {
		misp_info("%s - probe irq_gpio = %d",
			__func__, dev_global_variable->irq_gpio);

		gpio_direction_input(dev_global_variable->irq_gpio);
		dev_global_variable->irq_num =
			gpio_to_irq(dev_global_variable->irq_gpio);

		/* use misp_drv_global_variable addr as dev_id */
		ret = request_threaded_irq(
			dev_global_variable->irq_num,
			NULL, mini_isp_irq,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, "mini_isp",
			dev_global_variable);

		if (ret)
			misp_err("%s - step4. probe - request irq error",
				__func__);
		else {
			misp_info("%s succeed, irq_gpio: %d, irg_num %d",
					__func__,
					dev_global_variable->irq_gpio,
					dev_global_variable->irq_num);
		}
	}
#endif

	if (RESET_GPIO != NULL)
		misp_err("%s -reset_gpio gpio_get_value = %d", __func__,
			gpio_get_value(dev_global_variable->reset_gpio));

	if (VCC1_GPIO != NULL)
		misp_err("%s -vcc1_gpio gpio_get_value = %d", __func__,
			gpio_get_value(dev_global_variable->vcc1_gpio));
	if (VCC2_GPIO != NULL)
		misp_err("%s -vcc2_gpio gpio_get_value = %d", __func__,
			gpio_get_value(dev_global_variable->vcc2_gpio));
	if (VCC3_GPIO != NULL)
		misp_err("%s -vcc3_gpio gpio_get_value = %d", __func__,
			gpio_get_value(dev_global_variable->vcc3_gpio));

	misp_err("%s - leave", __func__);
	return ret;
}

int mini_isp_power_close(void)
{
	struct misp_global_variable *dev_global_variable;

	misp_info("%s enter", __func__);
	dev_global_variable = get_mini_isp_global_variable();
	/* add your code hereafter... */
	if (RESET_GPIO != NULL)
		gpio_set_value(dev_global_variable->reset_gpio, 0);
	msleep(20);
	if (VCC1_GPIO != NULL)
		gpio_set_value(dev_global_variable->vcc1_gpio, 0);
	if (VCC2_GPIO != NULL)
		gpio_set_value(dev_global_variable->vcc2_gpio, 0);
	if (VCC3_GPIO != NULL)
		gpio_set_value(dev_global_variable->vcc3_gpio, 0);
	if (ISP_CLK != NULL)
		clk_disable_unprepare(dev_global_variable->isp_clk);
	msleep(20);
	dev_global_variable->be_set_to_bypass = 0;
	dev_global_variable->before_booting = 1;

#if (ISR_MECHANISM == INTERRUPT_METHOD)
	if (IRQ_GPIO != NULL) {
		free_irq(dev_global_variable->irq_num
		, dev_global_variable);
	}
#endif
	return 0;
}

extern errcode mini_isp_poweron(void)
{
	errcode ret = 0;
	int *pintf_status = NULL;
	void *devdata;
	struct misp_global_variable *dev_global_variable;
	struct altek_statefsm *fsm;

	fsm = get_mini_isp_fsm();
	dev_global_variable = get_mini_isp_global_variable();
	/*reset mini-isp keep low for at least 200us, release to high for 20ms*/
	misp_err("[miniISP]mini_isp_poweron");

	pintf_status = &dev_global_variable->intf_status;

	misp_info("GENERAL_OPCODE: %d",
			GET_TRANS_SCEN_INTF(MINIISP_GENERAL_OPCODE));
	misp_info("BULK_OPCODE: %d", GET_TRANS_SCEN_INTF(MINIISP_BULK_OPCODE));
	misp_info("CFG_REG_MEM: %d", GET_TRANS_SCEN_INTF(MINIISP_CFG_REG_MEM));
	misp_info("intf_status: 0x%x", *pintf_status);

	/* wait all transfer interface ready */
	while (!((*pintf_status &
		GET_TRANS_SCEN_INTF(MINIISP_GENERAL_OPCODE)) &&
		(*pintf_status & GET_TRANS_SCEN_INTF(MINIISP_BULK_OPCODE)) &&
		(*pintf_status & GET_TRANS_SCEN_INTF(MINIISP_CFG_REG_MEM)))) {
		misp_info("%s - Wait all driver interface ready", __func__);
		msleep(20);
	}

	devdata = get_mini_isp_intf(MINIISP_GENERAL_OPCODE);
	if (!devdata) {
		misp_info("%s err, devdata is null ", __func__);
		return ERR_MINIISP_DEVDATA_NULL;
	}

	/*state check*/
	if (IS_EN_FSM)
		ret = altek_statefsmispdrv_open(fsm, ((void *)devdata));
	else
		ret = mini_isp_power_open();

	if (ret != 0) {
		misp_err("%s err, %x", __func__, ret);
		return ret;
	}

	dev_global_variable->be_set_to_bypass = 0;
	dev_global_variable->altek_spi_mode = ALTEK_SPI_MODE_E;
	dev_global_variable->now_state = MINI_ISP_POWER_ON;

	return ret;
}
EXPORT_SYMBOL(mini_isp_poweron);

extern errcode mini_isp_poweroff(void)
{
	void *devdata;
	struct misp_global_variable *dev_global_variable;
	int ret = 0;
	struct altek_statefsm *fsm;

	fsm = get_mini_isp_fsm();
	misp_err("[miniISP]mini_isp_poweroff");
	dev_global_variable = get_mini_isp_global_variable();

	devdata = get_mini_isp_intf(MINIISP_GENERAL_OPCODE);
	if (!devdata) {
		misp_info("%s err, devdata is null ", __func__);
		return ERR_MINIISP_DEVDATA_NULL;
	}
	/*state check*/
	if (IS_EN_FSM)
		ret = altek_statefsmispdrv_close(fsm, devdata);
	else
		ret = mini_isp_power_close();

	if (ret != 0) {
		misp_err("%s err, %x", __func__, ret);
		return ret;
	}

	dev_global_variable->now_state = MINI_ISP_POWER_OFF;
	misp_info("%s - X", __func__);

	return ret;
}
EXPORT_SYMBOL(mini_isp_poweroff);

extern void mini_isp_eeprom_wpon(void)
{
	struct misp_global_variable *dev_global_variable;

	misp_err("[miniISP]mini_isp_eeprom_wpon");
	if (WP_GPIO != NULL) {
		dev_global_variable = get_mini_isp_global_variable();
		gpio_set_value(dev_global_variable->wp_gpio, 1);
	}
	misp_info("%s - X", __func__);
}
EXPORT_SYMBOL(mini_isp_eeprom_wpon);

extern void mini_isp_eeprom_wpoff(void)
{
	struct misp_global_variable *dev_global_variable;

	misp_err("[miniISP]mini_isp_eeprom_wpoff");
	if (WP_GPIO != NULL) {
		dev_global_variable = get_mini_isp_global_variable();
		gpio_set_value(dev_global_variable->wp_gpio, 0);
	}
	misp_info("%s - X", __func__);
}
EXPORT_SYMBOL(mini_isp_eeprom_wpoff);

extern void mini_isp_set_bootfromspinor(void)
{
	struct misp_global_variable *dev_global_variable;

	dev_global_variable = get_mini_isp_global_variable();
	dev_global_variable->bootfromspinor = true;
}
EXPORT_SYMBOL(mini_isp_set_bootfromspinor);

extern int mini_isp_gpio_init(struct device *dev,
			struct misp_data *drv_data,
			struct misp_global_variable *drv_global_variable)
{
	int ret = 0;

	if (VCC1_GPIO != NULL) {
		drv_global_variable->vcc1_gpio =
			of_get_named_gpio(dev->of_node, VCC1_GPIO, 0);
		misp_info("%s - probe vcc1-gpios = %d", __func__,
			drv_global_variable->vcc1_gpio);

		ret = devm_gpio_request(dev,
			drv_global_variable->vcc1_gpio, VCC1_GPIO);
		if (ret) {
			misp_err("%s -step 4. request vcc1-gpio error",
				__func__);
			goto err_gpio1_config;
		}

		gpio_direction_output(drv_global_variable->vcc1_gpio, 1);
		msleep(20);
		gpio_set_value(drv_global_variable->vcc1_gpio, 1);
		msleep(20);
	}

	if (VCC2_GPIO != NULL) {
		drv_global_variable->vcc2_gpio = of_get_named_gpio(
			dev->of_node, VCC2_GPIO, 0);
		misp_info("%s - probe vcc2-gpios = %d", __func__,
			drv_global_variable->vcc2_gpio);

		ret = devm_gpio_request(dev,
			drv_global_variable->vcc2_gpio, VCC2_GPIO);
		if (ret) {
			misp_err("%s -step 4. request vcc2-gpios error",
				__func__);
			goto err_gpio2_config;
		}

		gpio_direction_output(drv_global_variable->vcc2_gpio, 1);
		msleep(20);
		gpio_set_value(drv_global_variable->vcc2_gpio, 1);
		msleep(20);
	}

	if (VCC3_GPIO != NULL) {
		drv_global_variable->vcc3_gpio = of_get_named_gpio(
			dev->of_node, VCC3_GPIO, 0);
		misp_err("%s - probe vcc3-gpios = %d", __func__,
					drv_global_variable->vcc3_gpio);

		ret = devm_gpio_request(dev,
			drv_global_variable->vcc3_gpio, VCC3_GPIO);
		if (ret) {
			misp_err("%s -step 4. request vcc3-gpio error",
				__func__);
			goto err_gpio_config;
		}

		gpio_direction_output(drv_global_variable->vcc3_gpio, 1);
		gpio_set_value(drv_global_variable->vcc3_gpio, 1);
		msleep(20);

	}

	if (WP_GPIO != NULL) {
		drv_global_variable->wp_gpio = of_get_named_gpio(
			dev->of_node, WP_GPIO, 0);
		misp_info("%s - probe wp-gpios = %d", __func__,
					drv_global_variable->wp_gpio);

		ret = devm_gpio_request(dev,
			drv_global_variable->wp_gpio, WP_GPIO);
		if (ret) {
			misp_err("%s -step 4. request wp-gpio error",
				__func__);
			goto err_gpio_config;
		}

		gpio_direction_output(drv_global_variable->wp_gpio, 1);
		gpio_set_value(drv_global_variable->wp_gpio, 1);
		msleep(20);

	}

	if (ISP_CLK != NULL) {
		drv_global_variable->isp_clk = devm_clk_get(dev,
						ISP_CLK);
		misp_err("clk_ptr = %p", drv_global_variable->isp_clk);
		ret = clk_set_rate(drv_global_variable->isp_clk, 19200000L);
		if (ret < 0)
			misp_err("clk_set_rate failed, not fatal\n");

		misp_err("clk_get_rate %ld\n", clk_get_rate(
					drv_global_variable->isp_clk));
		ret = clk_prepare_enable(drv_global_variable->isp_clk);
		if (ret < 0) {
			misp_err("clk_prepare_enable failed\n");
			goto err_clk_config;
		}
		msleep(20);
	}

	if (RESET_GPIO != NULL) {
		drv_global_variable->reset_gpio =
			of_get_named_gpio(dev->of_node, RESET_GPIO, 0);
		misp_info("%s - probe reset_gpio = %d", __func__,
			drv_global_variable->reset_gpio);

		ret = devm_gpio_request(dev,
			drv_global_variable->reset_gpio, RESET_GPIO);
		if (ret) {
			misp_err("%s -step 4. request reset gpio error",
				__func__);
			goto err_reset_config;
		}

		gpio_direction_output(drv_global_variable->reset_gpio, 0);
		gpio_set_value(drv_global_variable->reset_gpio, 0);
		msleep(20);

	}

#if (ISR_MECHANISM == INTERRUPT_METHOD)
	if (IRQ_GPIO != NULL) {
		drv_global_variable->irq_gpio =
			of_get_named_gpio(dev->of_node, IRQ_GPIO, 0);
		misp_info("%s - probe irq_gpio = %d", __func__,
				drv_global_variable->irq_gpio);
		ret = devm_gpio_request(dev,
				drv_global_variable->irq_gpio, IRQ_GPIO);

		if (ret) {
			misp_err("%s -step 4. request irq gpio error",
				__func__);
			goto err_irq_config;
		}
	}
#endif
	misp_info("%s - step4 done", __func__);
	/*step 5:other additional config*/

	misp_info("%s - step5 done", __func__);


	return ret;

#if (ISR_MECHANISM == INTERRUPT_METHOD)

err_irq_config:
	if (IRQ_GPIO != NULL)
		gpio_free(drv_global_variable->irq_gpio);
#endif

err_reset_config:
	if (RESET_GPIO != NULL)
		gpio_free(drv_global_variable->reset_gpio);

err_clk_config:
	if (ISP_CLK != NULL)
		clk_disable_unprepare(drv_global_variable->isp_clk);

err_gpio_config:
	if (VCC2_GPIO != NULL)
		gpio_free(drv_global_variable->vcc2_gpio);
err_gpio2_config:
	if (VCC1_GPIO != NULL)
		gpio_free(drv_global_variable->vcc1_gpio);

err_gpio1_config:

	return ret;
}
