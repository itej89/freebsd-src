/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * StarFive JH7110 PWMDAC -- interface for its codec.
 *
 * audio_soc(4) registers a mixer against the codec, but the gain has to be
 * applied where the samples are written, which is the CPU DAI.
 */

#ifndef _JH7110_PWMDAC_H_
#define	_JH7110_PWMDAC_H_

/* Playback volume, 0-100; 100 is full scale. */
void	jh7110_pwmdac_set_volume(int pct);
int	jh7110_pwmdac_get_volume(void);

#endif /* _JH7110_PWMDAC_H_ */
