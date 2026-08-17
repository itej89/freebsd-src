/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Innosilicon HDMI TX driver for StarFive JH7110.
 */

#ifndef _JH7110_INNO_HDMI_H_
#define _JH7110_INNO_HDMI_H_

int jh7110_hdmi_read_edid(uint8_t *buf, size_t len);
bool jh7110_hdmi_is_connected(void);
bool jh7110_hdmi_pixclock_supported(uint32_t pixclock);
struct jh7110_hdmi_mode {
	uint32_t	pixclock;	/* Hz */
	uint16_t	hdisplay;
	uint16_t	hsync_start;
	uint16_t	hsync_end;
	uint16_t	htotal;
	uint16_t	vdisplay;
	uint16_t	vsync_start;
	uint16_t	vsync_end;
	uint16_t	vtotal;
	bool		hsync_positive;
	bool		vsync_positive;
	bool		interlace;
};

void jh7110_hdmi_enable(const struct jh7110_hdmi_mode *mode);
void jh7110_hdmi_disable(void);
bool jh7110_hdmi_is_available(void);

#endif /* _JH7110_INNO_HDMI_H_ */
