/*
 * Copyright (c) 2023 Fisher & Paykel Healthcare Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file  udc_mcux.c
 * @brief NXP MCUX USB device controller (UDC) driver
 *
 * The driver implements the interface between the NXP MCUX USBD peripheral
 * driver and UDC API.
 */
/* USB device controller access from devicetree */
#define DT_DRV_COMPAT nxp_mcux_usbd

#include <string.h>
#include <stdio.h>
#include <soc.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/usb/udc.h>

#ifdef CONFIG_USB_DC_NXP_EHCI
#include "usb_device_ehci.h"
#endif
#ifdef CONFIG_USB_DC_NXP_LPCIP3511
#include "usb_device_lpcip3511.h"
#endif
#ifdef CONFIG_HAS_MCUX_CACHE
#include <fsl_cache.h>
#endif

#include "udc_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(udc_mcux, CONFIG_UDC_DRIVER_LOG_LEVEL);

// static K_KERNEL_STACK_DEFINE(drv_stack, CONFIG_UDC_MCUX_THREAD_STACK_SIZE);
// static struct k_thread drv_stack_data;

/*
 * Endpoint absolute index calculation:
 *
 * MCUX EHCI USB device controller supports a specific
 * number of bidirectional endpoints. Bidirectional means
 * that an endpoint object is represented to the outside
 * as an OUT and an IN Endpoint with its own buffers
 * and control structures.
 *
 * EP_ABS_IDX refers to the corresponding control
 * structure, for example:
 *
 *  EP addr | ep_idx | ep_abs_idx
 * -------------------------------
 *  0x00    | 0x00   | 0x00
 *  0x80    | 0x00   | 0x01
 *  0x01    | 0x01   | 0x02
 *  0x81    | 0x01   | 0x03
 *  ....    | ....   | ....
 *
 * The NUM_OF_EP_MAX (and number of s_ep_ctrl) should be double
 * of num_bidir_endpoints.
 */
#define EP_ABS_IDX(ep)		(USB_EP_GET_IDX(ep) * 2 + \
					(USB_EP_GET_DIR(ep) >> 7))
#define NUM_OF_EP_MAX		(DT_INST_PROP(0, num_bidir_endpoints) * 2)
#define CONTROLLER_ID		(DT_INST_ENUM_IDX(0, usb_controller_index))

#define CFG_EPIN_CNT		DT_INST_PROP(0, num_bidir_endpoints)
#define CFG_EPOUT_CNT		DT_INST_PROP(0, num_bidir_endpoints)

static struct udc_ep_config ep_cfg_out[CFG_EPOUT_CNT + 1];
static struct udc_ep_config ep_cfg_in[CFG_EPIN_CNT + 1];

const static struct device *udc_mcux_dev;

struct udc_mcux_config {
	// clock_control_subsys_t clock;
	// nrfx_power_config_t pwr;
	// nrfx_power_usbevt_config_t evt;
};

static void udc_mcux_thread(const struct device *dev)
{
}

static int udc_mcux_driver_init(const struct device *dev)
{
	struct udc_data *data = dev->data;
// 	int err;

// 	LOG_INF("Preinit");
// 	udc_mcux_dev = dev;
// 	k_mutex_init(&data->mutex);
// 	k_thread_create(&drv_stack_data, drv_stack,
// 			K_KERNEL_STACK_SIZEOF(drv_stack),
// 			(k_thread_entry_t)udc_mcux_thread,
// 			(void *)dev, NULL, NULL,
// 			K_PRIO_COOP(8), 0, K_NO_WAIT);

// 	k_thread_name_set(&drv_stack_data, "udc_mcux");
    return 0;
}

static int udc_mcux_lock(const struct device *dev)
{
	return udc_lock_internal(dev, K_FOREVER);
}

static int udc_mcux_unlock(const struct device *dev)
{
	return udc_unlock_internal(dev);
}

static const struct udc_api udc_mcux_api = {
	// .lock = udc_mcux_lock,
	// .unlock = udc_mcux_unlock,
	// .init = udc_mcux_init,
	// .enable = udc_mcux_enable,
	// .disable = udc_mcux_disable,
	// .shutdown = udc_mcux_shutdown,
	// .set_address = udc_mcux_set_address,
	// .host_wakeup = udc_mcux_host_wakeup,
	// .ep_try_config = NULL,
	// .ep_enable = udc_mcux_ep_enable,
	// .ep_disable = udc_mcux_ep_disable,
	// .ep_set_halt = udc_mcux_ep_set_halt,
	// .ep_clear_halt = udc_mcux_ep_clear_halt,
	// .ep_enqueue = udc_mcux_ep_enqueue,
	// .ep_dequeue = udc_mcux_ep_dequeue,
	.lock = udc_mcux_lock,
	.unlock = udc_mcux_unlock,
	.init = NULL,
	.enable = NULL,
	.disable = NULL,
	.shutdown = NULL,
	.set_address = NULL,
	.host_wakeup = NULL,
	.ep_try_config = NULL,
	.ep_enable = NULL,
	.ep_disable = NULL,
	.ep_set_halt = NULL,
	.ep_clear_halt = NULL,
	.ep_enqueue = NULL,
	.ep_dequeue = NULL,
};

static const struct udc_mcux_config udc_mcux_cfg = {
// 	.clock = COND_CODE_1(NRF_CLOCK_HAS_HFCLK192M,
// 			     (CLOCK_CONTROL_NRF_SUBSYS_HF192M),
// 			     (CLOCK_CONTROL_NRF_SUBSYS_HF)),
// 	.pwr = {
// 		.dcdcen = IS_ENABLED(CONFIG_SOC_DCDC_NRF52X) ||
// 			  IS_ENABLED(CONFIG_SOC_DCDC_NRF53X_APP),
// #if NRFX_POWER_SUPPORTS_DCDCEN_VDDH
// 		.dcdcenhv = IS_ENABLED(CONFIG_SOC_DCDC_NRF52X_HV) ||
// 			    IS_ENABLED(CONFIG_SOC_DCDC_NRF53X_HV),
// #endif
// 	},

// 	.evt = {
// 		.handler = udc_nrf_power_handler
// 	},
};

static struct udc_data udc_mcux_data = {
	.mutex = Z_MUTEX_INITIALIZER(udc_mcux_data.mutex),
	.priv = NULL,
};

// https://docs.zephyrproject.org/latest/kernel/drivers/index.html#c.DEVICE_DT_INST_DEFINE
DEVICE_DT_INST_DEFINE(0, udc_mcux_driver_init, NULL,
		      &udc_mcux_data, &udc_mcux_cfg,
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		      &udc_mcux_api);
