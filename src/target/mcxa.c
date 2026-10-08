/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2026 Black Magic Debug contributors
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

/*
 * This file implements support for the MCX A175/A176/A185/A186 and
 * A255/A256/A265/A266 devices, including their ROM flash programming API.
 *
 * Reference: MCXAP144M240F61RM, revision 2, 2025-11-17.
 */

#include "general.h"
#include "target.h"
#include "target_internal.h"
#include "adiv5.h"
#include "cortexm.h"

#define MCXA_DMAP_IDR                 0x002a0000U
#define MCXA_DMAP_AP                  1U
#define MCXA_DMAP_RESYNCH_RESET       0x21U
#define MCXA_DMAP_START_DEBUG_SESSION 0x07U

#define MCXA_DEVICE_TYPE_ADDRESS 0x40091ff4U
#define MCXA_DEVICE_TYPE_MASK    0x0000ffffU
#define MCXA_LPCAC_CTRL          0x40091824U
#define MCXA_LPCAC_MEM_REQ       (1U << 8U)

#define MCXA_FLASH_BASE             0x00000000U
#define MCXA_BOOTLOADER_TREE        0x03005fe0U
#define MCXA_FLASH_DRIVER_POINTER   (MCXA_BOOTLOADER_TREE + 4U)
#define MCXA_ROM_START              0x03000000U
#define MCXA_ROM_END                0x03010000U
#define MCXA_FLASH_CONFIG_ADDRESS   0x20000000U
#define MCXA_FLASH_CONFIG_SIZE      0x60U
#define MCXA_FLASH_CONFIG_INIT_SIZE 44U
#define MCXA_WRITE_BUFFER_ADDRESS   0x20000100U
/* RAMX1 may be allocated to LPCAC, so use RAMX0 for executable scratch and the stack. */
#define MCXA_CODE_PATCH_ADDRESS 0x04000000U
#define MCXA_IAP_MSP_ADDRESS    0x04002000U
#define MCXA_IAP_TIMEOUT_MS     1000U
#define MCXA_FLASH_ERASE_KEY    0x6b65666cU
#define MCXA_IAP_RESTORE_OFFSET 0x10U
#define MCXA_IAP_RETURN_OFFSET  0x16U
#define MCXA_IAP_MSPLIM_OFFSET  0x18U

#define MCXA_CODE_RAM_SIZE    0x00002000U
#define MCXA_SMALL_RAM_SIZE   0x0001e000U
#define MCXA_LARGE_RAM_SIZE   0x0003c000U
#define MCXA_SMALL_FLASH_SIZE 0x00080000U
#define MCXA_LARGE_FLASH_SIZE 0x00100000U

typedef enum mcxa_flash_api {
	MCXA_FLASH_API_INIT = 0U,
	MCXA_FLASH_API_ERASE_SECTOR = 4U,
	MCXA_FLASH_API_PROGRAM_PAGE = 12U,
} mcxa_flash_api_e;

typedef struct mcxa_flash_config {
	uint32_t flash_block_base;
	uint32_t flash_total_size;
	uint32_t flash_block_count;
	uint32_t flash_page_size;
	uint32_t flash_sector_size;
} mcxa_flash_config_s;

typedef struct mcxa_device {
	uint16_t part_id;
	const char *name;
	uint32_t flash_size;
	uint32_t ram_size;
} mcxa_device_s;

static const mcxa_device_s mcxa_devices[] = {
	{0x2175U, "MCXA175", MCXA_SMALL_FLASH_SIZE, MCXA_SMALL_RAM_SIZE},
	{0x2176U, "MCXA176", MCXA_LARGE_FLASH_SIZE, MCXA_LARGE_RAM_SIZE},
	{0x2185U, "MCXA185", MCXA_SMALL_FLASH_SIZE, MCXA_SMALL_RAM_SIZE},
	{0x2186U, "MCXA186", MCXA_LARGE_FLASH_SIZE, MCXA_LARGE_RAM_SIZE},
	{0x2255U, "MCXA255", MCXA_SMALL_FLASH_SIZE, MCXA_SMALL_RAM_SIZE},
	{0x2256U, "MCXA256", MCXA_LARGE_FLASH_SIZE, MCXA_LARGE_RAM_SIZE},
	{0x2265U, "MCXA265", MCXA_SMALL_FLASH_SIZE, MCXA_SMALL_RAM_SIZE},
	{0x2266U, "MCXA266", MCXA_LARGE_FLASH_SIZE, MCXA_LARGE_RAM_SIZE},
};

static bool mcxa_enter_flash_mode(target_s *target);
static bool mcxa_flash_prepare(target_flash_s *flash);
static bool mcxa_flash_erase(target_flash_s *flash, target_addr_t addr, size_t len);
static bool mcxa_flash_write(target_flash_s *flash, target_addr_t dest, const void *src, size_t len);

static bool mcxa_dmap_command(adiv5_access_port_s *const ap, const uint32_t command)
{
	platform_timeout_s timeout;
	platform_timeout_set(&timeout, MCXA_IAP_TIMEOUT_MS);
	while (adiv5_ap_read(ap, ADIV5_AP_CSW) != 0U) {
		if (platform_timeout_is_expired(&timeout))
			return false;
	}

	adiv5_ap_write(ap, ADIV5_AP_TAR_LOW, command);
	platform_timeout_set(&timeout, MCXA_IAP_TIMEOUT_MS);
	while ((adiv5_ap_read(ap, ADIV5_AP_DRW) & UINT32_C(0xffff)) != 0U) {
		if (platform_timeout_is_expired(&timeout))
			return false;
	}
	return true;
}

void mcxa_dp_prepare(adiv5_debug_port_s *const dp)
{
	/* The MCXA debug mailbox is AP1. AP0 is the Cortex-M33 AHB-AP. */
	adiv5_dp_abort(dp, ADIV5_DP_ABORT_DAPABORT);

	adiv5_access_port_s ap = {0};
	ap.dp = dp;
	ap.apsel = MCXA_DMAP_AP;
	ap.idr = adiv5_ap_read(&ap, ADIV5_AP_IDR);
	if (ap.idr != MCXA_DMAP_IDR)
		return;

	/* Preserve attach-to-running behaviour when the core AP is already visible. */
	ap.apsel = 0U;
	ap.idr = adiv5_ap_read(&ap, ADIV5_AP_IDR);
	if (ap.idr != 0U)
		return;
	/* A failed AP0 access can leave STICKYERR set, blocking the mailbox writes. */
	adiv5_dp_abort(dp, ADIV5_DP_ABORT_STKERRCLR);

	DEBUG_INFO("Running MCXA debug mailbox activation sequence\n");
	ap.apsel = MCXA_DMAP_AP;
	adiv5_ap_write(&ap, ADIV5_AP_CSW, MCXA_DMAP_RESYNCH_RESET);
	if (!mcxa_dmap_command(&ap, MCXA_DMAP_START_DEBUG_SESSION))
		DEBUG_WARN("MCXA debug mailbox activation failed\n");
}

static const mcxa_device_s *mcxa_find_device(const uint16_t part_id)
{
	for (size_t index = 0U; index < ARRAY_LENGTH(mcxa_devices); ++index) {
		if (mcxa_devices[index].part_id == part_id)
			return &mcxa_devices[index];
	}
	return NULL;
}

static bool mcxa_is_power_of_two(const uint32_t value)
{
	return value != 0U && (value & (value - 1U)) == 0U;
}

static bool mcxa_rom_address_valid(const target_addr32_t address, const bool function)
{
	const target_addr32_t code_address = function ? address & ~1U : address;
	return (!function || (address & 1U) != 0U) && code_address >= MCXA_ROM_START && code_address < MCXA_ROM_END;
}

static bool mcxa_get_iap_address(
	target_s *const target, const mcxa_flash_api_e api, target_addr32_t *const function_address)
{
	target_addr32_t flash_driver = 0U;
	if (target_mem32_read(target, &flash_driver, MCXA_FLASH_DRIVER_POINTER, sizeof(flash_driver)) ||
		!mcxa_rom_address_valid(flash_driver, false) || (flash_driver & 3U) != 0U)
		return false;

	if (target_mem32_read(target, function_address, flash_driver + (uint32_t)api, sizeof(*function_address)) ||
		!mcxa_rom_address_valid(*function_address, true))
		return false;
	return true;
}

static bool mcxa_wait_for_iap(target_s *const target, const uint32_t return_address)
{
	platform_timeout_s timeout;
	platform_timeout_set(&timeout, MCXA_IAP_TIMEOUT_MS);
	target_halt_reason_e reason = TARGET_HALT_RUNNING;
	while (reason == TARGET_HALT_RUNNING && !platform_timeout_is_expired(&timeout))
		reason = target_halt_poll(target, NULL);

	if (reason == TARGET_HALT_BREAKPOINT) {
		uint32_t pc = 0U;
		if (target_reg_read(target, CORTEX_REG_PC, &pc, sizeof(pc)) != sizeof(pc) || target_check_error(target)) {
			DEBUG_ERROR("MCXA ROM API return PC read failed\n");
			return false;
		}
		if (pc != return_address) {
			DEBUG_ERROR("MCXA ROM API stopped at %08" PRIx32 ", expected %08" PRIx32 "\n", pc, return_address);
			return false;
		}
		return true;
	}

	if (reason == TARGET_HALT_RUNNING) {
		DEBUG_ERROR("MCXA ROM API call timed out\n");
		target_halt_request(target);
		/* Do not let callers restore scratch state while the ROM is still running. */
		platform_timeout_set(&timeout, MCXA_IAP_TIMEOUT_MS);
		while (target_halt_poll(target, NULL) == TARGET_HALT_RUNNING && !platform_timeout_is_expired(&timeout))
			continue;
	} else
		DEBUG_ERROR("MCXA ROM API call halted unexpectedly (%u)\n", (unsigned int)reason);
	return false;
}

static bool mcxa_iap_call(target_s *const target, const mcxa_flash_api_e api, const uint32_t r1, const uint32_t r2,
	const uint32_t r3, uint32_t *const status)
{
	target_addr32_t function_address = 0U;
	if (!mcxa_get_iap_address(target, api, &function_address)) {
		DEBUG_ERROR("MCXA ROM flash API pointer is invalid\n");
		return false;
	}

	uint32_t regs[CORTEXM_GENERAL_REG_COUNT + CORTEX_FLOAT_REG_COUNT + CORTEXM_TRUSTZONE_REG_COUNT];
	target_regs_read(target, regs);
	if (target_check_error(target))
		return false;

	regs[CORTEX_REG_MSP] = MCXA_IAP_MSP_ADDRESS;
	regs[CORTEX_REG_SP] = MCXA_IAP_MSP_ADDRESS;
	regs[CORTEX_REG_XPSR] = CORTEXM_XPSR_THUMB;
	/* Mask application interrupts and select the privileged main stack for the ROM call. */
	regs[CORTEX_REG_SPECIAL] = 1U;
	regs[0] = MCXA_FLASH_CONFIG_ADDRESS;
	regs[1] = r1;
	regs[2] = r2;
	regs[3] = r3;
	regs[12] = function_address;
	regs[CORTEX_REG_PC] = MCXA_CODE_PATCH_ADDRESS | 1U;

	/*
	 * The application can set MSPLIM above RAMX0. Switching MSP alone then causes
	 * STKOF as soon as a ROM function uses the stack. Save MSPLIM beside the code,
	 * disable it for the ROM call, and restore it before the breakpoint. The saved
	 * word also lets us restore the limit after halting a failed or timed-out call.
	 */
	static const uint16_t trampoline[] = {
		0xf3efU,
		0x840aU, /* mrs r4, msplim */
		0xa504U, /* adr r5, saved_msplim */
		0x602cU, /* str r4, [r5] */
		0x2500U, /* movs r5, #0 */
		0xf385U,
		0x880aU, /* msr msplim, r5 */
		0x47e0U, /* blx r12 */
		0x4c01U, /* ldr r4, saved_msplim (recovery entry) */
		0xf384U,
		0x880aU, /* msr msplim, r4 */
		CORTEX_THUMB_BREAKPOINT,
		0xffffU,
		0xffffU, /* saved_msplim: invalid aligned limit until the wrapper starts */
	};
	uint16_t readback[ARRAY_LENGTH(trampoline)];
	if (target_mem32_write(target, MCXA_CODE_PATCH_ADDRESS, trampoline, sizeof(trampoline)) ||
		target_mem32_read(target, readback, MCXA_CODE_PATCH_ADDRESS, sizeof(readback)) ||
		memcmp(readback, trampoline, sizeof(trampoline)) != 0 || target_check_error(target))
		return false;
	target_regs_write(target, regs);
	if (target_check_error(target))
		return false;

	target_halt_resume(target, false);
	if (!mcxa_wait_for_iap(target, MCXA_CODE_PATCH_ADDRESS + MCXA_IAP_RETURN_OFFSET)) {
		/* Only execute the stackless recovery entry once the core is halted. */
		const uint32_t dhcsr = target_mem32_read32(target, CORTEXM_DHCSR);
		const uint32_t saved_msplim = target_mem32_read32(target, MCXA_CODE_PATCH_ADDRESS + MCXA_IAP_MSPLIM_OFFSET);
		if (!target_check_error(target) && (dhcsr & CORTEXM_DHCSR_S_HALT) && saved_msplim != UINT32_MAX) {
			regs[CORTEX_REG_PC] = (MCXA_CODE_PATCH_ADDRESS + MCXA_IAP_RESTORE_OFFSET) | 1U;
			target_regs_write(target, regs);
			if (!target_check_error(target)) {
				target_halt_resume(target, false);
				if (!mcxa_wait_for_iap(target, MCXA_CODE_PATCH_ADDRESS + MCXA_IAP_RETURN_OFFSET))
					DEBUG_ERROR("MCXA ROM API stack limit restoration failed\n");
			}
		}
		return false;
	}

	target_regs_read(target, regs);
	if (target_check_error(target))
		return false;
	*status = regs[0];
	return true;
}

static bool mcxa_prepare_flash_config(target_s *const target)
{
	const uint8_t config[MCXA_FLASH_CONFIG_INIT_SIZE] = {0};
	return !target_mem32_write(target, MCXA_FLASH_CONFIG_ADDRESS, config, sizeof(config));
}

static bool mcxa_flash_init(target_s *const target, mcxa_flash_config_s *const config)
{
	if (!mcxa_prepare_flash_config(target))
		return false;

	uint32_t status = 0U;
	if (!mcxa_iap_call(target, MCXA_FLASH_API_INIT, 0U, 0U, 0U, &status)) {
		DEBUG_ERROR("MCXA flash init call failed\n");
		return false;
	}
	if (status != 0U) {
		DEBUG_ERROR("MCXA flash init failed with status %" PRIu32 "\n", status);
		return false;
	}
	return !target_mem32_read(target, config, MCXA_FLASH_CONFIG_ADDRESS, sizeof(*config));
}

static bool mcxa_flash_geometry_valid(const mcxa_flash_config_s *const config, const uint32_t nominal_flash_size)
{
	return config->flash_block_base == MCXA_FLASH_BASE && config->flash_total_size != 0U &&
		config->flash_total_size <= nominal_flash_size && config->flash_block_count != 0U &&
		mcxa_is_power_of_two(config->flash_page_size) && mcxa_is_power_of_two(config->flash_sector_size) &&
		config->flash_page_size <= config->flash_sector_size && config->flash_sector_size <= config->flash_total_size &&
		(config->flash_total_size % config->flash_sector_size) == 0U;
}

static bool mcxa_add_flash(target_s *const target, const uint32_t nominal_flash_size)
{
	uint32_t regs[CORTEXM_GENERAL_REG_COUNT + CORTEX_FLOAT_REG_COUNT + CORTEXM_TRUSTZONE_REG_COUNT];
	uint8_t config_backup[MCXA_FLASH_CONFIG_SIZE];
	uint8_t *const code_ram_backup = malloc(MCXA_CODE_RAM_SIZE);
	if (!code_ram_backup) {
		DEBUG_ERROR("malloc: failed in %s\n", __func__);
		return false;
	}

	target_regs_read(target, regs);
	if (target_check_error(target) ||
		target_mem32_read(target, config_backup, MCXA_FLASH_CONFIG_ADDRESS, sizeof(config_backup)) ||
		target_mem32_read(target, code_ram_backup, MCXA_CODE_PATCH_ADDRESS, MCXA_CODE_RAM_SIZE)) {
		free(code_ram_backup);
		return false;
	}

	mcxa_flash_config_s config;
	bool success = mcxa_flash_init(target, &config) && mcxa_flash_geometry_valid(&config, nominal_flash_size);
	if (!success)
		DEBUG_WARN("MCXA ROM returned invalid flash geometry\n");

	/* Probe-time discovery must not disturb the application state or scratch RAM. */
	success = !target_mem32_write(target, MCXA_FLASH_CONFIG_ADDRESS, config_backup, sizeof(config_backup)) && success;
	success = !target_mem32_write(target, MCXA_CODE_PATCH_ADDRESS, code_ram_backup, MCXA_CODE_RAM_SIZE) && success;
	target_regs_write(target, regs);
	success = !target_check_error(target) && success;
	free(code_ram_backup);
	if (!success)
		return false;

	target_flash_s *const flash = calloc(1, sizeof(*flash));
	if (!flash) {
		DEBUG_ERROR("calloc: failed in %s\n", __func__);
		return false;
	}

	flash->start = config.flash_block_base;
	flash->length = config.flash_total_size;
	flash->blocksize = config.flash_sector_size;
	flash->writesize = config.flash_page_size;
	flash->prepare = mcxa_flash_prepare;
	flash->erase = mcxa_flash_erase;
	flash->write = mcxa_flash_write;
	flash->erased = 0xffU;
	target_add_flash(target, flash);
	target->enter_flash_mode = mcxa_enter_flash_mode;

	DEBUG_INFO("MCXA: detected %" PRIu32 " bytes of flash, %" PRIu32 "-byte pages, %" PRIu32 "-byte sectors\n",
		config.flash_total_size, config.flash_page_size, config.flash_sector_size);
	return true;
}

bool mcxa_probe(target_s *const target)
{
	const adiv5_access_port_s *const ap = cortex_ap(target);
	if (ap->apsel != 0U)
		return false;

	uint32_t device_type = 0U;
	if (target_mem32_read(target, &device_type, MCXA_DEVICE_TYPE_ADDRESS, sizeof(device_type)))
		return false;

	const mcxa_device_s *const device = mcxa_find_device((uint16_t)(device_type & MCXA_DEVICE_TYPE_MASK));
	if (!device)
		return false;

	target->driver = device->name;
	target->target_options |= TOPT_INHIBIT_NRST;
	target_add_ram32(target, 0x04000000U, MCXA_CODE_RAM_SIZE);
	/* RAMX1 ceases to be ordinary RAM once the application allocates it to LPCAC. */
	uint32_t lpcac_ctrl = 0U;
	if (!target_mem32_read(target, &lpcac_ctrl, MCXA_LPCAC_CTRL, sizeof(lpcac_ctrl)) &&
		!(lpcac_ctrl & MCXA_LPCAC_MEM_REQ))
		target_add_ram32(target, 0x04002000U, MCXA_CODE_RAM_SIZE);
	target_add_ram32(target, 0x20000000U, device->ram_size);

	if (!mcxa_add_flash(target, device->flash_size))
		DEBUG_WARN("MCXA: flash support unavailable, continuing with debug support only\n");
	return true;
}

static bool mcxa_enter_flash_mode(target_s *const target)
{
	const uint32_t pc = MCXA_CODE_PATCH_ADDRESS | 1U;
	if (target_mem32_write16(target, MCXA_CODE_PATCH_ADDRESS, 0xb672U) ||
		target_mem32_write16(target, MCXA_CODE_PATCH_ADDRESS + 2U, CORTEX_THUMB_BREAKPOINT) ||
		target_mem32_read16(target, MCXA_CODE_PATCH_ADDRESS) != 0xb672U ||
		target_mem32_read16(target, MCXA_CODE_PATCH_ADDRESS + 2U) != CORTEX_THUMB_BREAKPOINT ||
		target_reg_write(target, CORTEX_REG_PC, &pc, sizeof(pc)) != sizeof(pc) || target_check_error(target))
		return false;

	target_halt_resume(target, false);
	return mcxa_wait_for_iap(target, MCXA_CODE_PATCH_ADDRESS + 2U);
}

static bool mcxa_flash_prepare(target_flash_s *const flash)
{
	mcxa_flash_config_s config;
	if (!mcxa_flash_init(flash->t, &config))
		return false;

	const bool geometry_matches = config.flash_block_base == flash->start && config.flash_total_size == flash->length &&
		config.flash_page_size == flash->writesize && config.flash_sector_size == flash->blocksize;
	if (!geometry_matches)
		DEBUG_ERROR("MCXA flash geometry changed after probe\n");
	return geometry_matches;
}

static bool mcxa_flash_erase(target_flash_s *const flash, const target_addr_t addr, const size_t len)
{
	uint32_t status = 0U;
	if (!mcxa_iap_call(flash->t, MCXA_FLASH_API_ERASE_SECTOR, addr, (uint32_t)len, MCXA_FLASH_ERASE_KEY, &status)) {
		DEBUG_ERROR("MCXA flash erase call failed\n");
		return false;
	}
	if (status != 0U) {
		DEBUG_ERROR("MCXA flash erase failed with status %" PRIu32 "\n", status);
		return false;
	}
	return true;
}

static bool mcxa_flash_write(
	target_flash_s *const flash, const target_addr_t dest, const void *const src, const size_t len)
{
	if (target_mem32_write(flash->t, MCXA_WRITE_BUFFER_ADDRESS, src, len))
		return false;

	uint32_t status = 0U;
	if (!mcxa_iap_call(
			flash->t, MCXA_FLASH_API_PROGRAM_PAGE, dest, MCXA_WRITE_BUFFER_ADDRESS, (uint32_t)len, &status)) {
		DEBUG_ERROR("MCXA flash program call failed\n");
		return false;
	}
	if (status != 0U) {
		DEBUG_ERROR("MCXA flash program failed with status %" PRIu32 "\n", status);
		return false;
	}
	return true;
}
