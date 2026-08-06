#!/bin/sh
# Build a noise-cancelling virtual microphone out of two real microphones.
#
# Combines a close-talk microphone (the one you speak into) and a distant one
# (the noise reference) into a single stereo stream, runs VLC's dualmic filter
# over it, and publishes the result as a capture device other applications can
# select.  See doc/dual-microphone.md.
#
# Copyright © 2026 VLC authors and VideoLAN
#
# This program is free software; you can redistribute it and/or modify it
# under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation; either version 2.1 of the License, or
# (at your option) any later version.

set -eu

IN_SINK=dualmic_in
OUT_SINK=dualmic_out
STATE="${XDG_RUNTIME_DIR:-/tmp}/dualmic-virtual-mic.modules"

CLOSE=
FAR=
RATE=48000
LATENCY=20
CACHING=30
REMIX="remix=no"
SETUP_ONLY=no
VLC=${VLC:-vlc}
VLC_ARGS=

usage()
{
    cat <<EOF
Usage: $0 --close SOURCE --far SOURCE [options]
       $0 --list
       $0 --stop

  --close SOURCE   the microphone you speak into (a Bluetooth headset)
  --far SOURCE     the microphone used as a noise reference (a phone, a
                   built-in microphone); keep it away from your mouth
  --list           list the capture devices available, then exit
  --stop           tear down a previous run and exit
  --setup-only     build the routing but do not start VLC
  --rate HZ        sample rate to run the chain at (default $RATE)
  --latency MS     loopback latency for each microphone (default $LATENCY)
  --caching MS     VLC live caching; lower is snappier but riskier
                   (default $CACHING)
  --remix          let the sound server remix channels, if a microphone
                   refuses to connect to its side of the stereo pair
  --vlc-args ARGS  extra arguments passed through to VLC, for instance
                   "--dualmic-taps=32 --dualmic-protect-db=4"

Applications then select the "$OUT_SINK" monitor as their input.
EOF
}

die()
{
    echo "$0: $*" >&2
    exit 1
}

# pactl exists for both PulseAudio and PipeWire's PulseAudio server.
need_pactl()
{
    command -v pactl >/dev/null 2>&1 \
        || die "pactl not found; this script needs PulseAudio or PipeWire"
}

load()
{
    # Remember every module we load, so that --stop removes exactly those and
    # leaves anything else on the system alone.
    index=$(pactl load-module "$@") \
        || die "failed to load $1 -- run with 'sh -x' to see the arguments"
    echo "$index" >> "$STATE"
    echo "  loaded $1 (module $index)"
}

stop()
{
    [ -f "$STATE" ] || { echo "nothing to tear down"; return 0; }

    # Unload in reverse order: the loopbacks depend on the sinks.
    sort -rn "$STATE" | while read -r index; do
        pactl unload-module "$index" 2>/dev/null \
            && echo "  unloaded module $index"
    done
    rm -f "$STATE"
    echo "virtual microphone removed"
}

while [ $# -gt 0 ]; do
    case $1 in
    --close) CLOSE=${2:?--close needs a source name}; shift 2;;
    --far) FAR=${2:?--far needs a source name}; shift 2;;
    --rate) RATE=${2:?--rate needs a value}; shift 2;;
    --latency) LATENCY=${2:?--latency needs a value}; shift 2;;
    --caching) CACHING=${2:?--caching needs a value}; shift 2;;
    --vlc-args) VLC_ARGS=${2:?--vlc-args needs a value}; shift 2;;
    --remix) REMIX=; shift;;
    --setup-only) SETUP_ONLY=yes; shift;;
    --list)
        need_pactl
        echo "Capture devices:"
        pactl list short sources | while read -r _ name rest; do
            case $name in
            *.monitor) ;;               # playback monitors, not microphones
            *) echo "  $name";;
            esac
        done
        exit 0;;
    --stop) need_pactl; stop; exit 0;;
    -h|--help) usage; exit 0;;
    *) usage >&2; exit 1;;
    esac
done

need_pactl
[ -n "$CLOSE" ] || { usage >&2; die "no --close microphone given"; }
[ -n "$FAR" ] || { usage >&2; die "no --far microphone given"; }
[ "$CLOSE" != "$FAR" ] \
    || die "the two microphones must be different devices"

for src in "$CLOSE" "$FAR"; do
    pactl list short sources | cut -f2 | grep -qx "$src" \
        || die "no such capture device: $src (try --list)"
done

# A previous run would fight this one over the same sink names.
[ -f "$STATE" ] && { echo "removing the previous virtual microphone"; stop; }
: > "$STATE"

cleanup_on_failure()
{
    echo "setup failed, backing out" >&2
    stop
}
trap cleanup_on_failure EXIT

echo "Building the routing:"

# The two microphones meet as the left and right channels of one stereo sink,
# whose monitor is what VLC captures.
load module-null-sink \
    sink_name="$IN_SINK" \
    rate="$RATE" \
    channels=2 \
    channel_map=front-left,front-right \
    sink_properties=device.description=dualmic-combined-input

# The cancelled result is published here; its monitor is the virtual mic.
load module-null-sink \
    sink_name="$OUT_SINK" \
    rate="$RATE" \
    channels=1 \
    channel_map=mono \
    sink_properties=device.description=Noise-cancelled-microphone

# Close-talk microphone into the left channel, reference into the right.
# shellcheck disable=SC2086
load module-loopback \
    source="$CLOSE" \
    sink="$IN_SINK" \
    channels=1 \
    channel_map=front-left \
    rate="$RATE" \
    latency_msec="$LATENCY" \
    source_dont_move=true \
    sink_dont_move=true \
    $REMIX

# shellcheck disable=SC2086
load module-loopback \
    source="$FAR" \
    sink="$IN_SINK" \
    channels=1 \
    channel_map=front-right \
    rate="$RATE" \
    latency_msec="$LATENCY" \
    source_dont_move=true \
    sink_dont_move=true \
    $REMIX

trap - EXIT

cat <<EOF

The virtual microphone is "$OUT_SINK" -- select "Monitor of
Noise-cancelled-microphone" (device $OUT_SINK.monitor) as the input in
whatever application should use it.

EOF

if [ "$SETUP_ONLY" = yes ]; then
    echo "Routing is up; start the filter yourself with:"
    echo
    echo "  PULSE_SINK=$OUT_SINK $VLC -I dummy --no-video \\"
    echo "      pulse://$IN_SINK.monitor --audio-filter=dualmic \\"
    echo "      --aout=pulse --live-caching=$CACHING"
    echo
    echo "Run '$0 --stop' when finished."
    exit 0
fi

# PULSE_SINK is what sends this VLC's output to our sink and nothing else's:
# libpulse uses it whenever a stream is connected without an explicit sink,
# which is what the PulseAudio output does unless a device has been selected.
echo "Starting the filter (Ctrl-C to stop and tear everything down):"
echo
echo "  PULSE_SINK=$OUT_SINK $VLC -I dummy --no-video pulse://$IN_SINK.monitor \\"
echo "      --audio-filter=dualmic --aout=pulse --live-caching=$CACHING $VLC_ARGS"
echo

trap stop EXIT INT TERM

# shellcheck disable=SC2086
PULSE_SINK="$OUT_SINK" "$VLC" -I dummy --no-video \
    "pulse://$IN_SINK.monitor" \
    --audio-filter=dualmic \
    --aout=pulse \
    --live-caching="$CACHING" \
    $VLC_ARGS || true
