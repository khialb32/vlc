/*****************************************************************************
 * dualmic.c: test the dual-microphone noise canceller
 *****************************************************************************
 * Copyright © 2026 VLC authors and VideoLAN
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2.1 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston MA 02110-1301, USA.
 *****************************************************************************/

#ifdef HAVE_CONFIG_H
# include "config.h"
#endif

#undef NDEBUG
#include <assert.h>
#include <stdio.h>

#include "../../../modules/audio_filter/dualmic.h"

/* A synthetic two-microphone recording: room noise reaches the close-talk
 * microphone through an unknown acoustic path, and some of the voice leaks
 * into the distant reference microphone. */

#define RATE      16000u
#define SECONDS   20
#define NSAMP     ((size_t)RATE * SECONDS)

#define MS(x)     ((size_t)((x) * RATE / 1000))
#define PATH_LEN  MS(8)         /* room -> close-talk microphone */
#define ALIGN     MS(2)         /* the filter's default alignment */
#define TAPS      MS(16)        /* the filter's default length */
#define VOICE_LAG MS(0.2)       /* mouth -> reference microphone */

static float *room, *voice, *noise_at_primary, *interleaved, *out, *err;
static unsigned char *voiced;

static uint32_t rng_state = 0x12345678u;

static float urand(void)        /* uniform in [-1, 1) */
{
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return (float)(int32_t)rng_state / 2147483648.f;
}

static double power(const float *buf, const unsigned char *gate,
                    size_t first, size_t last)
{
    double sum = 0.;
    size_t n = 0;

    for (size_t i = first; i < last; i++)
        if (gate == NULL || gate[i])
        {
            sum += (double)buf[i] * buf[i];
            n++;
        }
    return n > 0 ? sum / n : 0.;
}

static double db(double ratio)
{
    return 10. * log10(ratio > 0. ? ratio : 1e-300);
}

static void build_scene(void)
{
    float *path = malloc(PATH_LEN * sizeof (float));
    float lp = 0.f;

    assert(path != NULL);

    /* Room noise: white noise through a one-pole lowpass, so that it is
     * coloured like real ambience rather than flat. */
    for (size_t i = 0; i < NSAMP; i++)
    {
        lp += 0.4f * (urand() * 0.3f - lp);
        room[i] = lp;
    }

    /* An arbitrary decaying impulse response, with a bulk delay in front. */
    for (size_t k = 0; k < PATH_LEN; k++)
        path[k] = (k < MS(1.5)) ? 0.f
                : urand() * expf(-(float)k / (float)MS(1.25));

    /* Voice: a few harmonics, gated on and off roughly twice a second. */
    for (size_t i = 0; i < NSAMP; i++)
    {
        double t = (double)i / RATE;
        int on = ((int)(t * 1.7) & 1) == 0;

        voiced[i] = on;
        voice[i] = on ? 0.35f * (float)(sin(2 * M_PI * 140 * t)
                                      + 0.5 * sin(2 * M_PI * 280 * t)
                                      + 0.25 * sin(2 * M_PI * 700 * t)) : 0.f;
    }

    for (size_t i = 0; i < NSAMP; i++)
    {
        float conv = 0.f;

        for (size_t k = 0; k < PATH_LEN && k <= i; k++)
            conv += path[k] * room[i - k];
        noise_at_primary[i] = conv;
    }
    free(path);
}

/** Lays out the scene, with \p leak of the voice reaching the reference. */
static void scene(float leak)
{
    for (size_t i = 0; i < NSAMP; i++)
    {
        interleaved[2 * i] = noise_at_primary[i] + voice[i];
        interleaved[2 * i + 1] = room[i]
            + ((i >= VOICE_LAG) ? leak * voice[i - VOICE_LAG] : 0.f);
    }
}

static struct dualmic_cfg base_cfg(void)
{
    struct dualmic_cfg cfg = {
        .rate = RATE, .taps = TAPS, .primary_delay = ALIGN, .ref_delay = 0,
        .mu = 0.3f, .leak = 1.f / (30.f * RATE), .sub_gain = 1.f,
        .makeup = 1.f, .protect_ratio = powf(10.f, 6.f / 10.f),
        .adaptive = true, .protect = true,
    };
    return cfg;
}

/**
 * Runs the whole scene through the engine and returns the output SNR in dB:
 * the voice power over the deviation from a perfect result, which is the voice
 * delayed by the primary path.  Residual noise and damage done to the voice
 * both land in that deviation, so one number covers both failure modes.
 */
static double run(const struct dualmic_cfg *cfg, size_t lead_silence)
{
    struct dualmic d;

    assert(dualmic_init(&d, cfg) == 0);

    for (size_t i = 0; i < lead_silence; i += 1024)
    {
        static const float quiet[2048];
        float sink[1024];
        size_t n = (lead_silence - i < 1024) ? lead_silence - i : 1024;

        dualmic_process(&d, quiet, sink, n, 0);
    }

    for (size_t i = 0; i < NSAMP; i += 1024)
    {
        size_t n = (NSAMP - i < 1024) ? NSAMP - i : 1024;

        dualmic_process(&d, interleaved + 2 * i, out + i, n, 0);
    }
    assert(d.resets == 0);

    const size_t delay = cfg->primary_delay;

    for (size_t i = 0; i < NSAMP; i++)
        err[i] = out[i] - ((i >= delay) ? voice[i - delay] : 0.f);

    /* Score the last third, once the filter has had time to converge. */
    size_t first = NSAMP * 2 / 3;
    double snr = db(power(voice, voiced, first, NSAMP)
                  / power(err, NULL, first, NSAMP));

    dualmic_destroy(&d);
    return snr;
}

int main(void)
{
    room = malloc(NSAMP * sizeof (float));
    voice = malloc(NSAMP * sizeof (float));
    noise_at_primary = malloc(NSAMP * sizeof (float));
    interleaved = malloc(2 * NSAMP * sizeof (float));
    out = malloc(NSAMP * sizeof (float));
    err = malloc(NSAMP * sizeof (float));
    voiced = malloc(NSAMP);
    assert(room && voice && noise_at_primary && interleaved && out && err
        && voiced);

    build_scene();

    const size_t first = NSAMP * 2 / 3;
    const double snr_in = db(power(voice, voiced, first, NSAMP)
                           / power(noise_at_primary, NULL, first, NSAMP));

    printf("close-talk microphone SNR: %.2f dB\n\n", snr_in);
    printf("voice reaching        output SNR, dB\n");
    printf("the reference    adaptive   unguarded   subtract\n");

    static const float leaks[] = { 0.f, 0.05f, 0.15f };

    for (size_t i = 0; i < sizeof (leaks) / sizeof (leaks[0]); i++)
    {
        struct dualmic_cfg cfg = base_cfg();
        double guarded, unguarded, plain;

        scene(leaks[i]);
        guarded = run(&cfg, 0);
        cfg.protect = false;
        unguarded = run(&cfg, 0);
        cfg = base_cfg();
        cfg.adaptive = false;
        plain = run(&cfg, 0);

        printf("%9.0f dB    %+7.2f    %+8.2f   %+8.2f\n",
               leaks[i] > 0.f ? 20. * log10(leaks[i]) : -99.,
               guarded, unguarded, plain);

        /* Adapting through speech teaches the filter to cancel the voice. */
        assert(guarded > unguarded);
        /* Subtracting two unmatched microphones is not cancellation. */
        assert(guarded > plain);
        /* And whatever else happens, do no harm. */
        assert(guarded > snr_in);
    }

    /* At a realistic level of leakage, the point of the filter is a large SNR
     * gain, so hold it to one. */
    {
        struct dualmic_cfg cfg = base_cfg();

        scene(0.05f);

        double gain = run(&cfg, 0) - snr_in;

        printf("\nSNR gain at -26 dB leakage: %+.2f dB\n", gain);
        assert(gain > 8.);
    }

    /* Starting into digital silence must not wedge the near-end detector: the
     * ratio it watches is meaningless there, and must stay out of the floor. */
    {
        struct dualmic_cfg cfg = base_cfg();

        scene(0.05f);

        double from_signal = run(&cfg, 0);
        double from_silence = run(&cfg, RATE * 3);

        printf("after 3 s of leading silence: %+.2f dB (vs %+.2f dB)\n",
               from_silence, from_signal);
        assert(from_silence > from_signal - 3.);
    }

    scene(0.05f);

    /* Mono output is written back over the stereo input it came from, so
     * in-place processing has to be identical to out-of-place. */
    {
        struct dualmic a, b;
        struct dualmic_cfg cfg = base_cfg();
        const size_t n = 20000;
        float *scratch = malloc(2 * n * sizeof (float));
        float *expected = malloc(n * sizeof (float));

        assert(scratch != NULL && expected != NULL);
        assert(dualmic_init(&a, &cfg) == 0);
        assert(dualmic_init(&b, &cfg) == 0);

        dualmic_process(&a, interleaved, expected, n, 0);
        memcpy(scratch, interleaved, 2 * n * sizeof (float));
        dualmic_process(&b, scratch, scratch, n, 0);

        assert(memcmp(expected, scratch, n * sizeof (float)) == 0);
        dualmic_destroy(&a);
        dualmic_destroy(&b);
        free(scratch);
        free(expected);
    }

    /* Selecting the other channel has to be the same as swapping them. */
    {
        struct dualmic a, b;
        struct dualmic_cfg cfg = base_cfg();
        const size_t n = 20000;
        float *swapped = malloc(2 * n * sizeof (float));
        float *left = malloc(n * sizeof (float));
        float *right = malloc(n * sizeof (float));

        assert(swapped != NULL && left != NULL && right != NULL);
        for (size_t i = 0; i < n; i++)
        {
            swapped[2 * i] = interleaved[2 * i + 1];
            swapped[2 * i + 1] = interleaved[2 * i];
        }
        assert(dualmic_init(&a, &cfg) == 0);
        assert(dualmic_init(&b, &cfg) == 0);

        dualmic_process(&a, interleaved, left, n, 0);
        dualmic_process(&b, swapped, right, n, 1);

        assert(memcmp(left, right, n * sizeof (float)) == 0);
        dualmic_destroy(&a);
        dualmic_destroy(&b);
        free(swapped);
        free(left);
        free(right);
    }

    /* Plain subtraction is exactly that. */
    {
        struct dualmic d;
        struct dualmic_cfg cfg = base_cfg();
        float in[8] = { 1.f, 0.25f, -2.f, 0.5f, 0.f, 1.f, 3.f, -1.f };
        float o[4];

        cfg.adaptive = false;
        cfg.primary_delay = cfg.ref_delay = 0;
        cfg.sub_gain = 0.5f;
        cfg.makeup = 2.f;
        assert(dualmic_init(&d, &cfg) == 0);
        dualmic_process(&d, in, o, 4, 0);

        for (size_t i = 0; i < 4; i++)
            assert(fabsf(o[i] - (in[2 * i] - 0.5f * in[2 * i + 1]) * 2.f)
                   < 1e-6f);
        dualmic_destroy(&d);
    }

    /* One bad sample from a capture device must not poison the tap history. */
    {
        struct dualmic d;
        struct dualmic_cfg cfg = base_cfg();
        const size_t n = 20000;
        float *dirty = malloc(2 * n * sizeof (float));
        float *o = malloc(n * sizeof (float));

        assert(dirty != NULL && o != NULL);
        memcpy(dirty, interleaved, 2 * n * sizeof (float));
        dirty[2 * 100] = NAN;
        dirty[2 * 500 + 1] = INFINITY;

        assert(dualmic_init(&d, &cfg) == 0);
        dualmic_process(&d, dirty, o, n, 0);

        for (size_t i = 0; i < n; i++)
            assert(isfinite(o[i]));
        dualmic_destroy(&d);
        free(dirty);
        free(o);
    }

    /* Nonsense configurations are refused, and leave nothing to release. */
    {
        struct dualmic d;
        struct dualmic_cfg cfg = base_cfg();

        cfg.taps = 0;
        assert(dualmic_init(&d, &cfg) != 0);
        cfg = base_cfg();
        cfg.rate = 0;
        assert(dualmic_init(&d, &cfg) != 0);
        cfg = base_cfg();
        cfg.taps = 1u << 25;
        assert(dualmic_init(&d, &cfg) != 0);
    }

    free(room);
    free(voice);
    free(noise_at_primary);
    free(interleaved);
    free(out);
    free(err);
    free(voiced);
    return 0;
}
