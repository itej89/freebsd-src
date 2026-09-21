/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * StarFive JH7110 I2S TX0 -- interface for the HDMI codec.
 *
 * audio_soc(4) registers a mixer against the codec, but the gain has to be
 * applied where the samples are copied, which is the CPU DAI. These let the
 * codec drive it without a shared global.
 */

#ifndef _JH7110_I2S_H_
#define	_JH7110_I2S_H_

/* Playback attenuation, 0-100; 100 is untouched. */
void	jh7110_i2s_set_volume(int pct);
int	jh7110_i2s_get_volume(void);

#endif /* _JH7110_I2S_H_ */
