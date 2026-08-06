/*****************************************************************************
 * dualmic.h : dual-microphone adaptive noise canceller (NLMS engine)
 *****************************************************************************
 * Copyright © 2026 VLC authors and VideoLAN
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2.1 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston MA 02110-1301, USA.
 *****************************************************************************/

#ifndef VLC_AUDIO_FILTER_DUALMIC_H
#define VLC_AUDIO_FILTER_DUALMIC_H 1

/**
 * \file
 * Adaptive noise canceller for a two-microphone setup.
 *
 * One microphone is worn close to the mouth (the \e primary input) and picks
 * up voice plus whatever the room is doing.  A second, distant microphone (the
 * \e reference input) picks up essentially only the room.  The room noise
 * reaches the two microphones over two different acoustic paths, so it arrives
 * with a different delay, level and colouration: subtracting one recording
 * from the other sample by sample does not cancel anything, and in the worst
 * case it makes the noise 6 dB louder.
 *
 * What actually cancels is the classic Widrow adaptive noise canceller.  An
 * adaptive FIR filter \c w continuously estimates the path from the reference
 * microphone to the noise as heard by the primary microphone, and the estimate
 * is what gets subtracted:
 *
 *     y[n] = sum(w[k] * ref[n-k], k = 0..taps-1)
 *     out[n] = primary[n] - y[n]
 *
 * The taps are updated with normalised least mean squares (NLMS), which is
 * scale invariant and therefore does not need the two microphones to be gain
 * matched:
 *
 *     w[k] += mu * out[n] * ref[n-k] / (refpow + eps)
 *
 * Because \c out[n] is also the error signal that drives the adaptation, and
 * because the distant microphone hears the voice too, an unguarded NLMS slowly
 * learns to cancel the voice as well.  dualmic_process() therefore watches the
 * ratio between the two inputs and slows adaptation right down while the near
 * end is talking; see the near-end detector below.
 *
 * The engine is deliberately free of any VLC dependency so that it can be
 * exercised on its own.
 */

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* The near-end detector has to react within a fraction of the filter's
 * convergence time: a speech onset that slips through is a large, coherent
 * perturbation, and a few of them per second are enough to undo the
 * cancellation entirely. Hence a very fast attack, a slow release, and a
 * hangover on top. */
/** Attack time constant of the peak estimates the decision is made on. */
#define DUALMIC_ATTACK_TC   0.001f
/** Release time constant of those peak estimates. */
#define DUALMIC_RELEASE_TC  0.050f
/** Time constant of the averages the noise-only floor is tracked from. */
#define DUALMIC_AVG_TC      0.020f
/** Time constant of the residual scale used for clipping. */
#define DUALMIC_RES_TC      0.250f
/** How long adaptation stays frozen after the near end goes quiet. */
#define DUALMIC_HANGOVER_TC 0.120f
/** Number of minimum-statistics buckets. */
#define DUALMIC_BUCKETS     4
/** Duration of one minimum-statistics bucket, in seconds. */
#define DUALMIC_BUCKET_TC   0.5f
/** Update errors beyond this many residual sigmas are clipped. */
#define DUALMIC_ROBUST_K    3.0f
/* Adaptation is stopped outright while the near end talks, rather than merely
 * slowed. The voice reaches the reference microphone as a coherent copy of
 * itself, so any residual step size is enough for the filter to eventually
 * learn to cancel the voice -- it just takes longer. */
/** Time constant of the noise-reduction meter. */
#define DUALMIC_METER_TC    1.0f
/** Taps below this magnitude are flushed to zero, to stay clear of denormals. */
#define DUALMIC_DENORMAL    1e-18f
/** Mean power below which an input counts as silent (about -80 dBFS RMS). */
#define DUALMIC_ACTIVE_POW  1e-8f

struct dualmic_cfg
{
    unsigned rate;          /**< sample rate, in Hz */
    unsigned taps;          /**< adaptive FIR length, in samples (>= 1) */
    /* Only the difference between these two aligns the microphones. Their
     * common part is free look-ahead for the near-end detector, which reads
     * the undelayed inputs -- worth little once the update is robust, so it is
     * not worth buying with latency of its own. */
    unsigned primary_delay; /**< delay added to the primary path, in samples */
    unsigned ref_delay;     /**< delay added to the reference path, in samples */
    float mu;               /**< NLMS step size, 0 (frozen) to 2 */
    float leak;             /**< per-sample tap leakage, 0 to 1 */
    float sub_gain;         /**< reference gain, plain subtraction mode only */
    float makeup;           /**< linear output gain */
    float protect_ratio;    /**< near-end trigger, as a linear power ratio */
    bool adaptive;          /**< false: out = primary - sub_gain * reference */
    bool protect;           /**< throttle adaptation while the near end talks */
};

struct dualmic
{
    struct dualmic_cfg cfg;

    /* Adaptive filter.  taps[] is stored oldest-tap-first so that it lines up
     * with the reference window and both the convolution and the update run as
     * a single forward pass over contiguous memory. */
    float *taps;
    float *hist;            /* reference history, 2 * cfg.taps entries */
    size_t pos;             /* index of the newest sample in hist[] */
    double refpow;          /* running sum of squares over the tap window */

    /* Bulk alignment delay lines (either one may be empty). */
    float *pdelay;
    float *rdelay;
    size_t pdelay_len, pdelay_pos;
    size_t rdelay_len, rdelay_pos;

    /* Near-end detector: the primary-to-reference power ratio sits at a floor
     * whenever only the room is audible, and jumps above it as soon as the near
     * end talks.  The floor is tracked with minimum statistics so that it
     * calibrates itself instead of relying on matched microphones.
     *
     * The decision needs to react within a millisecond, while the floor needs
     * a steady estimate, so two sets of power estimates are kept.  Both ratios
     * have to sit on the same scale for the one threshold to apply to both,
     * which is why the averages are only mildly slower than the peaks rather
     * than long-term: a long-term average never settles back down between
     * words, and would put the floor above the speech it has to detect. */
    float attack_alpha, release_alpha, avg_alpha, res_alpha;
    float peak_pri, peak_ref;   /* fast attack, slow release: the decision */
    float avg_pri, avg_ref;     /* steady: the noise-only floor */
    float bucket[DUALMIC_BUCKETS];
    float ratio_floor;
    unsigned bucket_idx;
    unsigned bucket_len, bucket_left;
    unsigned hangover, hangover_len;
    bool near_end;

    /* Scale of the residual, tracking what the filter achieves when it is not
     * being disturbed.  Update errors far outside it are outliers rather than
     * gradient information, and get clipped. */
    float res_pow;

    /* Noise-reduction meter, smoothed input and output power. */
    float meter_alpha;
    float meter_pri, meter_out;

    unsigned resets;        /**< number of divergence recoveries so far */
};

/** Smoothing coefficient of a one-pole averager with the given time constant. */
static inline float dualmic_alpha(float tc, unsigned rate)
{
    float n = tc * (float)rate;
    return n > 1.f ? 1.f - expf(-1.f / n) : 1.f;
}

/** Zeroes the adaptive filter without touching the rest of the state. */
static inline void dualmic_reset_taps(struct dualmic *d)
{
    memset(d->taps, 0, d->cfg.taps * sizeof (*d->taps));
}

/**
 * Drops every buffered sample.  The learned taps are kept: they describe the
 * room, which a discontinuity in the stream does not change.
 */
static inline void dualmic_reset(struct dualmic *d)
{
    memset(d->hist, 0, 2 * d->cfg.taps * sizeof (*d->hist));
    d->pos = d->cfg.taps - 1;
    d->refpow = 0.;

    if (d->pdelay != NULL)
        memset(d->pdelay, 0, d->pdelay_len * sizeof (*d->pdelay));
    if (d->rdelay != NULL)
        memset(d->rdelay, 0, d->rdelay_len * sizeof (*d->rdelay));
    d->pdelay_pos = d->rdelay_pos = 0;

    d->peak_pri = d->peak_ref = 0.f;
    d->avg_pri = d->avg_ref = 0.f;
    for (unsigned i = 0; i < DUALMIC_BUCKETS; i++)
        d->bucket[i] = FLT_MAX;
    d->ratio_floor = FLT_MAX;
    d->bucket_idx = 0;
    d->bucket_left = d->bucket_len;
    d->hangover = 0;
    d->near_end = false;
    d->res_pow = 0.f;

    d->meter_pri = d->meter_out = 0.f;
}

/**
 * Sets an engine up.  Returns 0, or -1 if the configuration is unusable or
 * memory ran out; on failure nothing needs to be released.
 */
static inline int dualmic_init(struct dualmic *d, const struct dualmic_cfg *cfg)
{
    if (cfg->rate == 0 || cfg->taps == 0)
        return -1;
    /* hist[] is twice the tap count. Anything near this bound is already far
     * beyond usable, and refusing it keeps the allocation size in range on
     * 32-bit hosts. */
    if (cfg->taps > (1u << 24))
        return -1;

    memset(d, 0, sizeof (*d));
    d->cfg = *cfg;

    d->taps = calloc(cfg->taps, sizeof (*d->taps));
    d->hist = calloc(2 * (size_t)cfg->taps, sizeof (*d->hist));
    if (d->taps == NULL || d->hist == NULL)
        goto error;

    d->pdelay_len = cfg->primary_delay;
    if (d->pdelay_len > 0
     && (d->pdelay = calloc(d->pdelay_len, sizeof (*d->pdelay))) == NULL)
        goto error;

    d->rdelay_len = cfg->ref_delay;
    if (d->rdelay_len > 0
     && (d->rdelay = calloc(d->rdelay_len, sizeof (*d->rdelay))) == NULL)
        goto error;

    d->attack_alpha = dualmic_alpha(DUALMIC_ATTACK_TC, cfg->rate);
    d->release_alpha = dualmic_alpha(DUALMIC_RELEASE_TC, cfg->rate);
    d->avg_alpha = dualmic_alpha(DUALMIC_AVG_TC, cfg->rate);
    d->res_alpha = dualmic_alpha(DUALMIC_RES_TC, cfg->rate);
    d->meter_alpha = dualmic_alpha(DUALMIC_METER_TC, cfg->rate);

    d->bucket_len = (unsigned)(DUALMIC_BUCKET_TC * (float)cfg->rate);
    if (d->bucket_len == 0)
        d->bucket_len = 1;

    d->hangover_len = (unsigned)(DUALMIC_HANGOVER_TC * (float)cfg->rate);

    dualmic_reset(d);
    return 0;

error:
    free(d->rdelay);
    free(d->pdelay);
    free(d->hist);
    free(d->taps);
    memset(d, 0, sizeof (*d));
    return -1;
}

static inline void dualmic_destroy(struct dualmic *d)
{
    free(d->rdelay);
    free(d->pdelay);
    free(d->hist);
    free(d->taps);
    memset(d, 0, sizeof (*d));
}

/** Pushes a sample through a circular delay line and returns the oldest one. */
static inline float dualmic_delay(float *line, size_t len, size_t *pos, float v)
{
    if (len == 0)
        return v;

    float old = line[*pos];

    line[*pos] = v;
    *pos = (*pos + 1 == len) ? 0 : *pos + 1;
    return old;
}

/** Runs the minimum-statistics tracker for one sample. */
static inline void dualmic_track_floor(struct dualmic *d, float ratio)
{
    if (ratio < d->bucket[d->bucket_idx])
        d->bucket[d->bucket_idx] = ratio;

    if (--d->bucket_left > 0)
        return;

    /* Retire the oldest bucket and recompute the floor over the window. */
    d->bucket_idx = (d->bucket_idx + 1) % DUALMIC_BUCKETS;
    d->bucket[d->bucket_idx] = ratio;
    d->bucket_left = d->bucket_len;

    float floor_ = d->bucket[0];
    for (unsigned i = 1; i < DUALMIC_BUCKETS; i++)
        if (d->bucket[i] < floor_)
            floor_ = d->bucket[i];
    d->ratio_floor = floor_;
}

/**
 * Cancels the reference microphone out of the primary microphone.
 *
 * \param in interleaved stereo samples holding both microphones
 * \param out mono samples; may alias \p in, which is written back in place
 * \param samples number of frames to process
 * \param primary_ch index (0 or 1) of the close-talk microphone in \p in
 */
static inline void dualmic_process(struct dualmic *d, const float *in,
                                   float *out, size_t samples,
                                   unsigned primary_ch)
{
    const size_t taps = d->cfg.taps;
    const unsigned ref_ch = primary_ch ^ 1;
    /* Regularisation, and the reference power below which adaptation is
     * pointless.  Both scale with the tap count because refpow is a sum over
     * the whole window; the floor sits at about -80 dBFS of reference RMS. */
    const double eps = (double)taps * 1e-6;
    const double floor_pow = (double)taps * 1e-8;
    const float lambda = 1.f - d->cfg.leak;

    for (size_t i = 0; i < samples; i++)
    {
        float p = in[2 * i + primary_ch];
        float r = in[2 * i + ref_ch];

        /* A single NaN in the input would otherwise poison the tap history
         * for good. */
        if (!isfinite(p))
            p = 0.f;
        if (!isfinite(r))
            r = 0.f;

        /* The detector runs on the undelayed inputs, so the alignment delay
         * doubles as look-ahead: a speech onset is flagged primary_delay
         * samples before the sample carrying it reaches the adaptation. */
        float pp = p * p, rr = r * r;

        p = dualmic_delay(d->pdelay, d->pdelay_len, &d->pdelay_pos, p);
        r = dualmic_delay(d->rdelay, d->rdelay_len, &d->rdelay_pos, r);

        float e;

        if (!d->cfg.adaptive)
            e = p - d->cfg.sub_gain * r;
        else
        {
            /* Slide the reference window forward.  hist[] holds two windows so
             * that the live one is always contiguous; it is folded back to the
             * start whenever the write cursor runs off the end. */
            size_t pos = d->pos + 1;

            if (pos == 2 * taps)
            {
                memmove(d->hist, d->hist + taps, taps * sizeof (*d->hist));
                pos = taps;
            }
            d->hist[pos] = r;
            d->pos = pos;

            float leaving = d->hist[pos - taps];

            d->refpow += (double)r * r - (double)leaving * leaving;
            if (!(d->refpow > 0.))
                d->refpow = 0.;

            const float *x = d->hist + pos - taps + 1;
            float *w = d->taps;
            float y = 0.f;

            for (size_t k = 0; k < taps; k++)
                y += w[k] * x[k];

            if (!isfinite(y))
            {
                /* Should not happen, but a diverged filter is unlistenable and
                 * never recovers on its own. */
                dualmic_reset_taps(d);
                d->resets++;
                y = 0.f;
            }
            e = p - y;

            /* Near-end detection, on the pre-cancellation powers.  Silence is
             * excluded: its ratio carries no information about the acoustic
             * paths, and letting it reach the tracker would peg the floor at
             * an arbitrary value and throttle adaptation from then on. */
            d->peak_pri += (pp > d->peak_pri ? d->attack_alpha
                                             : d->release_alpha)
                         * (pp - d->peak_pri);
            d->peak_ref += (rr > d->peak_ref ? d->attack_alpha
                                             : d->release_alpha)
                         * (rr - d->peak_ref);
            d->avg_pri += d->avg_alpha * (pp - d->avg_pri);
            d->avg_ref += d->avg_alpha * (rr - d->avg_ref);

            /* Both estimates have to be up and running: during the first
             * instants one of them is still at zero, and the meaningless ratio
             * that comes out of that would sit in the floor for seconds. */
            if (d->avg_ref > DUALMIC_ACTIVE_POW && d->avg_pri > DUALMIC_ACTIVE_POW)
                dualmic_track_floor(d, d->avg_pri / d->avg_ref);

            bool talking = d->peak_ref > DUALMIC_ACTIVE_POW
                        && d->peak_pri > d->peak_ref * d->ratio_floor
                                       * d->cfg.protect_ratio;

            if (talking)
                d->hangover = d->hangover_len;
            else if (d->hangover > 0)
                d->hangover--;

            d->near_end = d->cfg.protect && (talking || d->hangover > 0);

            /* Leakage is applied along with the update, so the taps hold their
             * picture of the room through silences rather than bleeding away
             * (and so silence costs one pass over the taps instead of two). */
            if (d->cfg.mu > 0.f && !d->near_end && d->refpow > floor_pow)
            {
                /* Whatever the detector misses -- the first instants of a
                 * speech onset, above all -- arrives as an error far larger
                 * than the filter's own residual.  Clipping it there bounds
                 * how far one such sample can drag the taps. */
                /* Seed the scale from the smoothed input power rather than
                 * from one sample, which could land on a zero crossing and
                 * clamp the filter to a near-zero limit for a good while. */
                if (!(d->res_pow > 0.f))
                    d->res_pow = (d->avg_pri > 0.f) ? d->avg_pri : e * e;

                float lim = DUALMIC_ROBUST_K * sqrtf(d->res_pow);
                float e_upd = e > lim ? lim : (e < -lim ? -lim : e);
                float g = d->cfg.mu * e_upd / (float)(d->refpow + eps);

                /* The scale is tracked from the clipped error, not the raw
                 * one: an outlier fed back into its own rejection threshold
                 * raises the threshold and stops being rejected. */
                d->res_pow += d->res_alpha * (e_upd * e_upd - d->res_pow);

                for (size_t k = 0; k < taps; k++)
                    w[k] = w[k] * lambda + g * x[k];
            }
        }

        d->meter_pri += d->meter_alpha * (p * p - d->meter_pri);
        d->meter_out += d->meter_alpha * (e * e - d->meter_out);

        out[i] = e * d->cfg.makeup;
    }

    /* Once per block: keep leaked-away taps from decaying into denormals,
     * which are ruinously slow on some CPUs. */
    for (size_t k = 0; k < taps; k++)
        if (fabsf(d->taps[k]) < DUALMIC_DENORMAL)
            d->taps[k] = 0.f;
}

/**
 * Returns how much quieter the output is than the primary input, in dB, as a
 * rough indication that cancellation is doing something.  Note that this
 * measures total reduction, so near-end speech pushes it back towards 0 dB.
 */
static inline float dualmic_reduction_db(const struct dualmic *d)
{
    if (!(d->meter_pri > 0.f) || !(d->meter_out > 0.f))
        return 0.f;
    return 10.f * log10f(d->meter_pri / d->meter_out);
}

#endif /* !VLC_AUDIO_FILTER_DUALMIC_H */
