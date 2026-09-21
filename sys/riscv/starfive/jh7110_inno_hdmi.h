/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Innosilicon HDMI TX driver for StarFive JH7110.
 */

#ifndef _JH7110_INNO_HDMI_H_
#define _JH7110_INNO_HDMI_H_

int jh7110_hdmi_read_edid(uint8_t *buf, size_t len);
bool jh7110_hdmi_is_connected(void);
/* 1 connected, 0 disconnected, -1 unknown (no HPD GPIO). */
int jh7110_hdmi_hpd_state(void);
/* True if a sink answers DDC with a valid EDID header. */
bool jh7110_hdmi_sink_present(void);
/* Called on every HPD edge; cb must be able to sleep. */
void jh7110_hdmi_set_hotplug_cb(void (*cb)(void *), void *arg);
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

/*
 * For jh7110_hdmi_audio.c. The audio block shares this register window, but
 * lives in a separate, sound(4)-dependent file because this driver is built
 * into every kernel.
 */
uint32_t jh7110_hdmi_tmds_rate(void);
void jh7110_hdmi_audio_write(uint32_t reg, uint32_t val);
uint32_t jh7110_hdmi_audio_read(uint32_t reg);
void jh7110_hdmi_audio_modb(uint32_t reg, uint32_t mask, uint32_t val);
void jh7110_hdmi_disable(void);
bool jh7110_hdmi_is_available(void);

#endif /* _JH7110_INNO_HDMI_H_ */
