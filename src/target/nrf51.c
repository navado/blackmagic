/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2014  Mike Walters <mike@flomp.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements nRF51 target specific functions for detecting
 * the device, providing the XML memory map and Flash memory programming.
 */

#include "general.h"
#include "target.h"
#include "target_internal.h"
#include "cortexm.h"

static int nrf51_flash_erase(struct target_flash *f, target_addr addr, size_t len);
static int nrf51_flash_write(struct target_flash *f,
                             target_addr dest, const void *src, size_t len);

static bool nrf51_cmd_erase_all(target *t);
static bool nrf51_cmd_read_hwid(target *t);
static bool nrf51_cmd_read_fwid(target *t);
static bool nrf51_cmd_read_deviceid(target *t);
static bool nrf51_cmd_read_deviceaddr(target *t);
static bool nrf51_cmd_read_help(target *t);
static bool nrf51_cmd_read(target *t, int argc, const char *argv[]);

const struct command_s nrf51_cmd_list[] = {
	{"erase_mass", (cmd_handler)nrf51_cmd_erase_all, "Erase entire flash memory"},
	{"read", (cmd_handler)nrf51_cmd_read, "Read device parameters"},
	{NULL, NULL, NULL}
};
const struct command_s nrf51_read_cmd_list[] = {
	{"help", (cmd_handler)nrf51_cmd_read_help, "Display help for read commands"},
	{"hwid", (cmd_handler)nrf51_cmd_read_hwid, "Read hardware identification number"},
	{"fwid", (cmd_handler)nrf51_cmd_read_fwid, "Read pre-loaded firmware ID"},
	{"deviceid", (cmd_handler)nrf51_cmd_read_deviceid, "Read unique device ID"},
	{"deviceaddr", (cmd_handler)nrf51_cmd_read_deviceaddr, "Read device address"},
	{NULL, NULL, NULL}
};

/* Non-Volatile Memory Controller (NVMC) Registers */
#define NRF51_NVMC					0x4001E000
#define NRF51_NVMC_READY			(NRF51_NVMC + 0x400)
#define NRF51_NVMC_CONFIG			(NRF51_NVMC + 0x504)
#define NRF51_NVMC_ERASEPAGE		(NRF51_NVMC + 0x508)
#define NRF51_NVMC_ERASEALL			(NRF51_NVMC + 0x50C)
#define NRF51_NVMC_ERASEUICR		(NRF51_NVMC + 0x514)

#define NRF51_NVMC_CONFIG_REN		0x0						// Read only access
#define NRF51_NVMC_CONFIG_WEN		0x1						// Write enable
#define NRF51_NVMC_CONFIG_EEN		0x2						// Erase enable

/* Factory Information Configuration Registers (FICR) */
#define NRF51_FICR				0x10000000
#define NRF51_FICR_CODEPAGESIZE			(NRF51_FICR + 0x010)
#define NRF51_FICR_CODESIZE			(NRF51_FICR + 0x014)
#define NRF51_FICR_CONFIGID			(NRF51_FICR + 0x05C)
#define NRF51_FICR_DEVICEID_LOW			(NRF51_FICR + 0x060)
#define NRF51_FICR_DEVICEID_HIGH		(NRF51_FICR + 0x064)
#define NRF51_FICR_DEVICEADDRTYPE		(NRF51_FICR + 0x0A0)
#define NRF51_FICR_DEVICEADDR_LOW		(NRF51_FICR + 0x0A4)
#define NRF51_FICR_DEVICEADDR_HIGH		(NRF51_FICR + 0x0A8)
#define NRF52_PART_INFO					(NRF51_FICR + 0x100)
#define NRF52_INFO_RAM					(NRF51_FICR + 0x10C)

/* User Information Configuration Registers (UICR) */
#define NRF51_UICR				0x10001000

#define NRF51_PAGE_SIZE 1024
#define NRF52_PAGE_SIZE 4096

#define SRAM_BASE          0x20000000
#define STUB_BUFFER_BASE   ALIGN(SRAM_BASE + sizeof(nrf51_flash_write_stub), 4)

static const uint16_t nrf51_flash_write_stub[] = {
#include "flashstub/nrf51.stub"
};

static void nrf51_add_flash(target *t,
                            uint32_t addr, size_t length, size_t erasesize)
{
	struct target_flash *f = calloc(1, sizeof(*f));
	f->start = addr;
	f->length = length;
	f->blocksize = erasesize;
	f->erase = nrf51_flash_erase;
	f->write = nrf51_flash_write;
	f->erased = 0xff;
	target_add_flash(t, f);
}

bool nrf51_probe(target *t)
{
	uint32_t page_size = target_mem_read32(t, NRF51_FICR_CODEPAGESIZE);
	uint32_t code_size = target_mem_read32(t, NRF51_FICR_CODESIZE);
	/* Check that page_size and code_size makes sense */
	if ((page_size == 0xffffffff) || (code_size == 0xffffffff) ||
		(page_size ==  0) || (code_size ==  0) ||
		(page_size > 0x10000) || (code_size > 0x10000))	   
		return false;
	/* Check that device identifier makes sense */
	uint32_t uid0 = target_mem_read32(t, NRF51_FICR_DEVICEID_LOW);
	uint32_t uid1 = target_mem_read32(t, NRF51_FICR_DEVICEID_HIGH);
	if ((uid0 == 0xffffffff) || (uid1 == 0xffffffff) ||
		(uid0 ==  0) || (uid1 ==  0))
		return false;
	/* Test for NRF52 device*/
	uint32_t info_part = target_mem_read32(t, NRF52_PART_INFO);
	if ((info_part != 0xffffffff) && (info_part != 0) &&
		((info_part & 0x00ff000) == 0x52000)) {
		uint32_t ram_size = target_mem_read32(t, NRF52_INFO_RAM);
		t->idcode = info_part;
		t->driver = "Nordic nRF52";
		target_add_ram(t, 0x20000000, ram_size * 1024);
		nrf51_add_flash(t, 0, page_size * code_size, page_size);
		target_add_commands(t, nrf51_cmd_list, "nRF52");
		return true;
	} else {
		t->driver = "Nordic nRF51";
		/* Use the biggest RAM size seen in NRF51 fammily.
		 * IDCODE is kept as '0', as deciphering is hard and
		 * there is later no usage.*/
		target_add_ram(t, 0x20000000, 0x8000);
		nrf51_add_flash(t, 0, page_size * code_size, page_size);
		target_add_commands(t, nrf51_cmd_list, "nRF51");
		return true;
	}
	return false;
}

static int nrf51_flash_erase(struct target_flash *f, target_addr addr, size_t len)
{
	target *t = f->t;
	/* Enable erase */
	target_mem_write32(t, NRF51_NVMC_CONFIG, NRF51_NVMC_CONFIG_EEN);

	/* Poll for NVMC_READY */
	while (target_mem_read32(t, NRF51_NVMC_READY) == 0)
		if(target_check_error(t))
			return -1;

	while (len) {
		if (addr == NRF51_UICR) { // Special Case
			/* Write to the ERASE_UICR register to erase */
			target_mem_write32(t, NRF51_NVMC_ERASEUICR, 0x1);

		} else { // Standard Flash Page
			/* Write address of first word in page to erase it */
			target_mem_write32(t, NRF51_NVMC_ERASEPAGE, addr);
		}

		/* Poll for NVMC_READY */
		while (target_mem_read32(t, NRF51_NVMC_READY) == 0)
			if(target_check_error(t))
				return -1;

		addr += f->blocksize;
		len -= f->blocksize;
	}

	/* Return to read-only */
	target_mem_write32(t, NRF51_NVMC_CONFIG, NRF51_NVMC_CONFIG_REN);

	/* Poll for NVMC_READY */
	while (target_mem_read32(t, NRF51_NVMC_READY) == 0)
		if(target_check_error(t))
			return -1;

	return 0;
}

static int nrf51_flash_write(struct target_flash *f,
                             target_addr dest, const void *src, size_t len)
{
	target *t = f->t;

	/* Enable write */
	target_mem_write32(t, NRF51_NVMC_CONFIG, NRF51_NVMC_CONFIG_WEN);

	/* Poll for NVMC_READY */
	while (target_mem_read32(t, NRF51_NVMC_READY) == 0)
		if(target_check_error(t))
			return -1;

	/* Write stub and data to target ram and call stub */
	target_mem_write(t, SRAM_BASE, nrf51_flash_write_stub,
	                 sizeof(nrf51_flash_write_stub));
	target_mem_write(t, STUB_BUFFER_BASE, src, len);
	int ret = cortexm_run_stub(t, SRAM_BASE, dest,
	                           STUB_BUFFER_BASE, len, 0);
	/* Return to read-only */
	target_mem_write32(t, NRF51_NVMC_CONFIG, NRF51_NVMC_CONFIG_REN);

	return ret;
}

static bool nrf51_cmd_erase_all(target *t)
{
	tc_printf(t, "erase..\n");

	/* Enable erase */
	target_mem_write32(t, NRF51_NVMC_CONFIG, NRF51_NVMC_CONFIG_EEN);

	/* Poll for NVMC_READY */
	while (target_mem_read32(t, NRF51_NVMC_READY) == 0)
		if(target_check_error(t))
			return false;

	/* Erase all */
	target_mem_write32(t, NRF51_NVMC_ERASEALL, 1);

	/* Poll for NVMC_READY */
	while (target_mem_read32(t, NRF51_NVMC_READY) == 0)
		if(target_check_error(t))
			return false;

	return true;
}

static bool nrf51_cmd_read_hwid(target *t)
{
	uint32_t hwid = target_mem_read32(t, NRF51_FICR_CONFIGID) & 0xFFFF;
	tc_printf(t, "Hardware ID: 0x%04X\n", hwid);

	return true;
}
static bool nrf51_cmd_read_fwid(target *t)
{
	uint32_t fwid = (target_mem_read32(t, NRF51_FICR_CONFIGID) >> 16) & 0xFFFF;
	tc_printf(t, "Firmware ID: 0x%04X\n", fwid);

	return true;
}
static bool nrf51_cmd_read_deviceid(target *t)
{
	uint32_t deviceid_low = target_mem_read32(t, NRF51_FICR_DEVICEID_LOW);
	uint32_t deviceid_high = target_mem_read32(t, NRF51_FICR_DEVICEID_HIGH);

	tc_printf(t, "Device ID: 0x%08X%08X\n", deviceid_high, deviceid_low);

	return true;
}
static bool nrf51_cmd_read_deviceaddr(target *t)
{
	uint32_t addr_type = target_mem_read32(t, NRF51_FICR_DEVICEADDRTYPE);
	uint32_t addr_low = target_mem_read32(t, NRF51_FICR_DEVICEADDR_LOW);
	uint32_t addr_high = target_mem_read32(t, NRF51_FICR_DEVICEADDR_HIGH) & 0xFFFF;

	if ((addr_type & 1) == 0) {
		tc_printf(t, "Publicly Listed Address: 0x%04X%08X\n", addr_high, addr_low);
	} else {
		tc_printf(t, "Randomly Assigned Address: 0x%04X%08X\n", addr_high, addr_low);
	}

	return true;
}
static bool nrf51_cmd_read_help(target *t)
{
	const struct command_s *c;

	tc_printf(t, "Read commands:\n");
	for(c = nrf51_read_cmd_list; c->cmd; c++)
		tc_printf(t, "\t%s -- %s\n", c->cmd, c->help);

	return true;
}
static bool nrf51_cmd_read(target *t, int argc, const char *argv[])
{
	const struct command_s *c;
	if (argc > 1) {
		for(c = nrf51_read_cmd_list; c->cmd; c++) {
			/* Accept a partial match as GDB does.
			 * So 'mon ver' will match 'monitor version'
			 */
			if(!strncmp(argv[1], c->cmd, strlen(argv[1])))
				return !c->handler(t, argc - 1, &argv[1]);
		}
	}
	return nrf51_cmd_read_help(t);
}

#include "adiv5.h"
#define NRF52_MDM_IDR 0x02880000

static bool nrf51_mdm_cmd_erase_mass(target *t);

const struct command_s nrf51_mdm_cmd_list[] = {
	{"erase_mass", (cmd_handler)nrf51_mdm_cmd_erase_mass, "Erase entire flash memory"},
	{NULL, NULL, NULL}
};

static bool nop_function(void)
{
	return true;
}

void nrf51_mdm_probe(ADIv5_AP_t *ap)
{
	switch(ap->idr) {
	case NRF52_MDM_IDR:
		break;
	default:
		return;
	}

	target *t = target_new();
	adiv5_ap_ref(ap);
	t->priv = ap;
	t->priv_free = (void*)adiv5_ap_unref;

	t->driver = "Nordic nRF52 Access Port";
	t->attach = (void*)nop_function;
	t->detach = (void*)nop_function;
	t->check_error = (void*)nop_function;
	t->mem_read = (void*)nop_function;
	t->mem_write = (void*)nop_function;
	t->regs_size = 4;
	t->regs_read = (void*)nop_function;
	t->regs_write = (void*)nop_function;
	t->reset = (void*)nop_function;
	t->halt_request = (void*)nop_function;
	//t->halt_poll = mdm_halt_poll;
	t->halt_resume = (void*)nop_function;

	target_add_commands(t, nrf51_mdm_cmd_list, t->driver);
}

#define MDM_POWER_EN ADIV5_DP_REG(0x01)
#define MDM_SELECT_AP ADIV5_DP_REG(0x02)
#define MDM_STATUS  ADIV5_AP_REG(0x08)
#define MDM_CONTROL ADIV5_AP_REG(0x04)
#define MDM_PROT_EN  ADIV5_AP_REG(0x0C)


static bool nrf51_mdm_cmd_erase_mass(target *t)
{
	ADIv5_AP_t *ap = t->priv;

	uint32_t status = adiv5_ap_read(ap, MDM_STATUS);

	adiv5_dp_write(ap->dp, MDM_POWER_EN, 0x50000000);

	adiv5_dp_write(ap->dp, MDM_SELECT_AP, 0x01000000);

	adiv5_ap_write(ap, MDM_CONTROL, 0x00000001);

	// Read until 0, probably should have a timeout here...
	do {
		status = adiv5_ap_read(ap, MDM_STATUS);
	} while (status);

	// The second read will provide true prot status
	status = adiv5_ap_read(ap, MDM_PROT_EN);
	status = adiv5_ap_read(ap, MDM_PROT_EN);

	// should we return the prot status here?
	return true;
}
