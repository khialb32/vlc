/*****************************************************************************
 * dualmic.c : dual-microphone noise cancelling virtual microphone
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

#ifdef HAVE_CONFIG_H
# include "config.h"
#endif

#include <vlc_common.h>
#include <vlc_aout.h>
#include <vlc_configuration.h>
#include <vlc_filter.h>
#include <vlc_plugin.h>

#include "dualmic.h"

/* The filter takes one stereo stream carrying two different microphones, and
 * emits the mono difference between them once the room has been cancelled out.
 * See dualmic.h for the signal processing, and doc/dual-microphone.md for how
 * to wire two capture devices into a single stereo stream and expose the
 * result as a virtual microphone. */

#define CFG_PREFIX "dualmic-"

static int Open(vlc_object_t *);
static void Close(filter_t *);
static block_t *Process(filter_t *, block_t *);
static void Flush(filter_t *);

typedef struct
{
    struct dualmic anc;
    unsigned primary_ch;
    unsigned log_period;    /* samples between two status messages */
    unsigned since_log;
    unsigned resets;        /* last reported divergence count */
} filter_sys_t;

#define HELP_TEXT N_( \
    "Cancels a room out of a close-talk microphone using a second, distant " \
    "microphone as a noise reference. The two microphones must arrive as the " \
    "two channels of one stereo stream; the filter outputs mono." )

#define PRIMARY_TEXT N_("Close-talk microphone channel")
#define PRIMARY_LONGTEXT N_( \
    "Channel carrying the microphone you speak into, for instance a Bluetooth " \
    "headset. The other channel is used as the noise reference." )

#define ADAPTIVE_TEXT N_("Adaptive cancellation")
#define ADAPTIVE_LONGTEXT N_( \
    "Learn the acoustic path between the two microphones and cancel the noise " \
    "through it. Disable this to plainly subtract the reference channel, " \
    "which only helps if the two microphones are already matched and aligned." )

#define TAPS_TEXT N_("Filter length (ms)")
#define TAPS_LONGTEXT N_( \
    "Length of the adaptive filter. It has to cover the reverberation of the " \
    "room to cancel it; longer settings cancel more but cost proportionally " \
    "more CPU and converge more slowly." )

#define MU_TEXT N_("Adaptation rate")
#define MU_LONGTEXT N_( \
    "How fast the filter tracks the room. Higher adapts faster but leaves " \
    "more residual noise; 0 freezes what has been learned so far." )

#define FORGET_TEXT N_("Adaptation memory (s)")
#define FORGET_LONGTEXT N_( \
    "How long the filter remembers the room. Shorter recovers faster from a " \
    "bad adaptation, at the cost of some cancellation." )

#define ALIGN_TEXT N_("Microphone alignment (ms)")
#define ALIGN_LONGTEXT N_( \
    "Delay applied to the close-talk microphone so that the noise reference " \
    "runs ahead of it, which is what lets the filter model sound reaching the " \
    "close-talk microphone first. Negative values delay the reference instead." )

#define GAIN_TEXT N_("Reference gain")
#define GAIN_LONGTEXT N_( \
    "Level the reference channel is subtracted at when adaptive cancellation " \
    "is disabled." )

#define MAKEUP_TEXT N_("Output gain (dB)")
#define MAKEUP_LONGTEXT N_( \
    "Gain applied to the cancelled signal, to make up for the level lost to " \
    "cancellation." )

#define PROTECT_TEXT N_("Protect speech")
#define PROTECT_LONGTEXT N_( \
    "Slow adaptation down while you are talking. The reference microphone " \
    "hears your voice too, so without this the filter gradually learns to " \
    "cancel it along with the room." )

#define PROTECT_DB_TEXT N_("Speech detection threshold (dB)")
#define PROTECT_DB_LONGTEXT N_( \
    "How far the close-talk microphone has to rise above the noise reference " \
    "before you count as talking. Lower protects speech more and cancels " \
    "less." )

static const int primary_values[] = { 0, 1 };
static const char *const primary_texts[] = { N_("Left"), N_("Right") };

vlc_module_begin()
    set_shortname(N_("Dual microphone"))
    set_description(N_("Dual-microphone noise canceller"))
    set_help(HELP_TEXT)
    set_subcategory(SUBCAT_AUDIO_AFILTER)
    set_capability("audio filter", 0)
    set_callback(Open)

    add_integer(CFG_PREFIX "primary", 0, PRIMARY_TEXT, PRIMARY_LONGTEXT)
        change_integer_list(primary_values, primary_texts)
    add_bool(CFG_PREFIX "adaptive", true, ADAPTIVE_TEXT, ADAPTIVE_LONGTEXT)
    add_integer_with_range(CFG_PREFIX "taps", 16, 1, 200,
                           TAPS_TEXT, TAPS_LONGTEXT)
    add_float_with_range(CFG_PREFIX "mu", 0.3, 0.0, 2.0, MU_TEXT, MU_LONGTEXT)
    add_float_with_range(CFG_PREFIX "forget", 30.0, 0.1, 3600.0,
                         FORGET_TEXT, FORGET_LONGTEXT)
    add_float_with_range(CFG_PREFIX "align", 2.0, -50.0, 50.0,
                         ALIGN_TEXT, ALIGN_LONGTEXT)
    add_float_with_range(CFG_PREFIX "gain", 1.0, 0.0, 4.0,
                         GAIN_TEXT, GAIN_LONGTEXT)
    add_float_with_range(CFG_PREFIX "makeup", 0.0, -20.0, 20.0,
                         MAKEUP_TEXT, MAKEUP_LONGTEXT)
    add_bool(CFG_PREFIX "protect", true, PROTECT_TEXT, PROTECT_LONGTEXT)
    add_float_with_range(CFG_PREFIX "protect-db", 6.0, 0.0, 30.0,
                         PROTECT_DB_TEXT, PROTECT_DB_LONGTEXT)
vlc_module_end()

static const char *const ppsz_options[] = {
    "primary", "adaptive", "taps", "mu", "forget", "align", "gain", "makeup",
    "protect", "protect-db", NULL
};

/* Bounds on the settings that size a buffer. They match the ranges declared
 * above, and are enforced again here because a value coming from a filter
 * chain ("dualmic{taps=...}") is not held to the declared range. */
#define TAPS_MS_MAX   200
#define ALIGN_MS_MAX  50.f

/** Converts a non-negative delay in milliseconds to a sample count. */
static unsigned DelaySamples(float ms, unsigned rate)
{
    if (!(ms > 0.f))
        return 0;
    if (ms > ALIGN_MS_MAX)
        ms = ALIGN_MS_MAX;
    return (unsigned)((double)ms * rate / 1000.);
}

static int Open(vlc_object_t *obj)
{
    filter_t *filter = (filter_t *)obj;
    audio_format_t *infmt = &filter->fmt_in.audio;

    if (infmt->channel_type != AUDIO_CHANNEL_TYPE_BITMAP
     || infmt->i_channels != 2)
    {
        msg_Err(filter, "dual-microphone cancellation needs the two "
                        "microphones as one stereo stream");
        return VLC_EGENERIC;
    }

    filter_sys_t *sys = vlc_obj_malloc(obj, sizeof (*sys));
    if (unlikely(sys == NULL))
        return VLC_ENOMEM;

    config_ChainParse(filter, CFG_PREFIX, ppsz_options, filter->p_cfg);

    const unsigned rate = infmt->i_rate;
    struct dualmic_cfg cfg;

    cfg.rate = rate;
    cfg.adaptive = var_InheritBool(obj, CFG_PREFIX "adaptive");
    cfg.mu = var_InheritFloat(obj, CFG_PREFIX "mu");
    cfg.sub_gain = var_InheritFloat(obj, CFG_PREFIX "gain");
    cfg.protect = var_InheritBool(obj, CFG_PREFIX "protect");
    cfg.protect_ratio = powf(10.f,
                             var_InheritFloat(obj, CFG_PREFIX "protect-db") / 10.f);
    cfg.makeup = powf(10.f, var_InheritFloat(obj, CFG_PREFIX "makeup") / 20.f);

    /* A tap per sample of filter length. The default of 16 ms is a compromise:
     * it covers early reflections, which is where most of the cancellable
     * energy is, without the cost of a full reverberation tail. */
    int64_t taps_ms = var_InheritInteger(obj, CFG_PREFIX "taps");

    if (taps_ms < 1)
        taps_ms = 1;
    if (taps_ms > TAPS_MS_MAX)
        taps_ms = TAPS_MS_MAX;

    cfg.taps = (unsigned)((uint64_t)taps_ms * rate / 1000);
    if (cfg.taps < 1)
        cfg.taps = 1;

    /* Positive alignment delays the close-talk path, which is the useful
     * direction: it gives the filter room to model noise that reaches the
     * close-talk microphone before the reference. */
    float align = var_InheritFloat(obj, CFG_PREFIX "align");

    cfg.primary_delay = DelaySamples(align, rate);
    cfg.ref_delay = DelaySamples(-align, rate);

    float forget = var_InheritFloat(obj, CFG_PREFIX "forget");

    cfg.leak = (forget > 0.f) ? 1.f / (forget * rate) : 0.f;
    if (cfg.leak > 1.f)
        cfg.leak = 1.f;

    if (dualmic_init(&sys->anc, &cfg) != 0)
    {
        msg_Err(filter, "cannot allocate a %u-tap adaptive filter", cfg.taps);
        return VLC_ENOMEM;
    }

    sys->primary_ch = var_InheritInteger(obj, CFG_PREFIX "primary") != 0;
    sys->log_period = rate * 5;
    sys->since_log = 0;
    sys->resets = 0;

    msg_Dbg(filter, "%s cancellation, %u taps (%"PRId64" ms) at %u Hz, "
                    "primary channel %s",
            cfg.adaptive ? "adaptive" : "plain subtractive", cfg.taps, taps_ms,
            rate, sys->primary_ch ? "right" : "left");

    if (cfg.adaptive && cfg.taps > 4096)
        msg_Warn(filter, "%u taps is a lot; expect high CPU usage and drop "
                         CFG_PREFIX "taps if the capture cannot keep up",
                 cfg.taps);

    /* Two microphones in, one cancelled microphone out. */
    infmt->i_format = VLC_CODEC_FL32;
    aout_FormatPrepare(infmt);

    filter->fmt_out.audio = *infmt;
    filter->fmt_out.audio.i_physical_channels = AOUT_CHAN_CENTER;
    aout_FormatPrepare(&filter->fmt_out.audio);

    static const struct vlc_filter_operations ops = {
        .filter_audio = Process, .flush = Flush, .close = Close,
    };
    filter->ops = &ops;
    filter->p_sys = sys;
    return VLC_SUCCESS;
}

static block_t *Process(filter_t *filter, block_t *block)
{
    filter_sys_t *sys = filter->p_sys;
    float *samples = (float *)block->p_buffer;

    /* Mono output is half the size of the stereo input, so the cancelled
     * signal is written back over the block it came from. */
    dualmic_process(&sys->anc, samples, samples, block->i_nb_samples,
                    sys->primary_ch);
    block->i_buffer = block->i_nb_samples
                    * filter->fmt_out.audio.i_bytes_per_frame;

    if (sys->anc.resets != sys->resets)
    {
        sys->resets = sys->anc.resets;
        msg_Warn(filter, "adaptive filter diverged and was reset (%u times); "
                         "lower " CFG_PREFIX "mu", sys->resets);
    }

    sys->since_log += block->i_nb_samples;
    if (sys->since_log >= sys->log_period)
    {
        sys->since_log = 0;
        msg_Dbg(filter, "cancelling %.1f dB%s", dualmic_reduction_db(&sys->anc),
                sys->anc.near_end ? ", speech detected" : "");
    }

    return block;
}

static void Flush(filter_t *filter)
{
    filter_sys_t *sys = filter->p_sys;

    dualmic_reset(&sys->anc);
}

static void Close(filter_t *filter)
{
    filter_sys_t *sys = filter->p_sys;

    dualmic_destroy(&sys->anc);
}
