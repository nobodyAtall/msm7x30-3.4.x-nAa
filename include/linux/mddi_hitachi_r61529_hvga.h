#ifndef __ARCH_ARM_MACH_MSM_MDDI_HITACHI_R61529_HVGA_H
#define __ARCH_ARM_MACH_MSM_MDDI_HITACHI_R61529_HVGA_H

#define MDDI_HITACH_R61529_HVGA_NAME "mddi_hitachi_r61529_hvga"

struct hitachi_hvga_platform_data {
	void (*power_on)(void);
	void (*power_off)(void);
	void (*window_adjust)(u16 x1, u16 x2, u16 y1, u16 y2);
	void (*exit_deep_standby) (void);
};

#endif
