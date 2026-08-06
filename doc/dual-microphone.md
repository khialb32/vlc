# A noise-cancelling virtual microphone from two microphones

The `dualmic` audio filter takes two microphones and produces one: a
close-talk microphone you speak into (a Bluetooth headset, say) minus the room
picked up by a second, distant microphone (a phone lying on the desk, or a
built-in laptop microphone). What comes out is your voice with much less of
the room in it.

This document covers what the filter does, why the obvious version of the idea
does not work, and how to wire the result up so that other applications see it
as an ordinary microphone.

## Why "one microphone minus the other" does not work

The intuition is that if microphone A hears *voice + room* and microphone B
hears *room*, then `A - B` is the voice. It is a good intuition and it is
wrong in practice, because the room does not arrive at the two microphones as
the same signal. It arrives:

- at different times, because the microphones are in different places;
- at different levels, because they are at different distances and have
  different sensitivities;
- with a different frequency response, because each has its own path, its own
  reflections and its own capsule.

Subtracting two signals that differ in phase does not cancel them. Where they
happen to be out of phase it *adds* them, up to 6 dB louder. Averaged over a
spectrum, plain subtraction of two unmatched microphones typically makes the
signal-to-noise ratio slightly worse than doing nothing at all, which is what
the test in `test/modules/audio_filter/dualmic.c` measures.

What does work is to let a filter *learn* the path between the two
microphones. An adaptive FIR filter continuously estimates how the room, as
heard by the distant microphone, turns into the room as heard by the
close-talk microphone, and that estimate is what gets subtracted. This is the
classic Widrow adaptive noise canceller, and it handles the delay, the level
and the colouration on its own, without either microphone needing to be
matched or characterised in advance.

The filter still supports plain subtraction, via `--dualmic-adaptive=no`. It
is there to A/B against, and for the rare case of two genuinely matched and
aligned microphones.

## Protecting your voice

There is a catch that dominates the design. The distant microphone hears your
voice as well, just more quietly. The adaptive filter is driven by its own
output as an error signal, so if it is left adapting while you talk, it
discovers that it can make its output smaller by cancelling your voice — and
does. Over tens of seconds it will hollow out the very thing you wanted to
keep.

So the filter watches the ratio between the two microphones. The ratio sits at
a floor whenever only the room is audible, and jumps when you speak. The floor
is tracked with minimum statistics, so no calibration or matching is needed;
the threshold is relative to whatever the floor turns out to be. While you are
talking, adaptation stops outright rather than merely slowing down — any
residual step size is eventually enough to learn to cancel a coherent copy of
your voice. Cancellation itself keeps running the whole time; it is only the
*learning* that pauses.

Two further details matter more than they look:

- Update errors far larger than the filter's own residual are treated as
  outliers and clipped. Whatever the detector misses — the first instants of a
  speech onset, above all — arrives as a huge error, and a converged filter
  does not survive many of those. Critically, the scale used for clipping is
  tracked from the *clipped* error: feeding the raw error back into its own
  rejection threshold raises the threshold and stops it rejecting anything.
- The alignment delay doubles as look-ahead. The detector reads the undelayed
  inputs, so it sees a speech onset coming before the sample carrying it
  reaches the adaptation.

`--dualmic-protect=no` turns the protection off. It is worth listening to once
to hear what it is for.

## Placement

The physics sets a ceiling that no amount of processing gets past: whatever
fraction of your voice reaches the reference microphone comes back as
distortion, because the noise-cancelling filter is applied to it too.

So put the reference microphone **away from your mouth** and, ideally, closer
to whatever is making the noise. On a desk beside the keyboard, or facing the
window, is good. Clipped to your collar next to the headset boom is not: at
that point it hears your voice almost as well as the close-talk microphone
does, and there is nothing left to separate.

Measured on the synthetic scene in the test, going from -26 dB of voice
leakage into the reference to -16 dB costs roughly 10 dB of the final result.

## Options

All of these can be given as `--dualmic-<name>=<value>` on the command line, or
inline as `--audio-filter=dualmic{<name>=<value>,...}`.

| Option | Default | Meaning |
| --- | --- | --- |
| `primary` | `0` | Which channel is the close-talk microphone: 0 left, 1 right. |
| `adaptive` | `yes` | Learn the path. `no` plainly subtracts instead. |
| `taps` | `16` | Adaptive filter length, in ms. |
| `mu` | `0.3` | Adaptation rate. `0` freezes what has been learned. |
| `forget` | `30` | How long the filter remembers the room, in seconds. |
| `align` | `2` | Delay on the close-talk path, in ms. Negative delays the reference. |
| `gain` | `1.0` | Reference level, plain subtraction only. |
| `makeup` | `0` | Output gain, in dB. |
| `protect` | `yes` | Stop adapting while you talk. |
| `protect-db` | `6` | How far above the noise floor counts as talking. |

Longer `taps` cancels more of the room's reverberation but costs proportionally
more CPU and converges more slowly. The cost is roughly two multiply-accumulates
per tap per sample: at the default 16 ms and 48 kHz that is about 74 million per
second, which is minor on a desktop and noticeable on a small ARM board. The
filter warns in the log above 4096 taps.

`vlc -vv` reports the achieved reduction every five seconds, which is the
quickest way to tell whether the setup is working at all:

```
[...] dualmic filter debug: cancelling 14.3 dB
[...] dualmic filter debug: cancelling 2.1 dB, speech detected
```

## Getting both microphones into one stream

The filter is an ordinary audio filter, so it sees whatever VLC is playing: it
needs the two microphones as the **two channels of one stereo stream**. VLC
cannot open two capture devices at once, so this part is done by the sound
server.

On Linux with PipeWire or PulseAudio, `doc/dualmic-virtual-mic.sh` does the
whole thing:

```sh
# List what is available
./doc/dualmic-virtual-mic.sh --list

# Combine two sources, filter them, and publish the result
./doc/dualmic-virtual-mic.sh \
    --close bluez_input.AA_BB_CC_DD_EE_FF \
    --far    alsa_input.pci-0000_00_1f.3-platform.analog-stereo
```

It builds this chain:

```
close-talk mic ->  left  \
                          >-- dualmic_in (null sink) -- VLC + dualmic filter --> dualmic_out
distant mic    ->  right /                                                            |
                                                                                      v
                                                                    "Noise-cancelled microphone"
                                                                     (dualmic_out.monitor)
```

Applications then pick **Noise-cancelled microphone** as their input, the same
way they would pick any other. `--stop` tears the whole thing down again.

The script prints the VLC command it runs, so if you would rather drive it
yourself the middle of the chain is just:

```sh
vlc -I dummy --no-video \
    pulse://dualmic_in.monitor \
    --audio-filter=dualmic \
    --aout=pulse --audio-device=dualmic_out \
    --live-caching=30
```

On Windows and macOS there is no equivalent one-liner; you need a virtual audio
device (VB-CABLE, BlackHole, Loopback and similar) to play the part of the two
null sinks. The filter itself is unchanged.

## Latency and clocks

Two things are worth knowing before using this on a live call.

**Latency.** VLC is a media player, not a low-latency audio server, and the
chain above adds its buffering to the filter's own few milliseconds.
`--live-caching=30` keeps it usable for conversation; lower values risk
dropouts. If you need genuinely low latency, the same algorithm belongs in a
PipeWire filter node rather than in VLC.

**Clock drift.** Two capture devices, especially a Bluetooth one, run on
independent clocks and drift apart. The adaptive filter tracks slow drift, but
it is tracking something that is not going to stop moving. Routing both
microphones through one sound server, as the script does, is what keeps them
resampled onto a common clock; capturing them independently will not work.

Finally: the filter needs the room to actually be *audible to both*
microphones. It cancels what the two have in common. It will do nothing about
noise that only the close-talk microphone can hear, and for broadband hiss with
no usable reference, the existing single-microphone `rnnoise` filter is the
better tool. The two can be chained:
`--audio-filter=dualmic:rnnoise`.
