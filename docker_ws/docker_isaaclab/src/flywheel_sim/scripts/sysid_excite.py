"""Write a wheel position-target excitation for system identification (CSV: t [s], q_cmd [rad]).

For the motor driver's position loop (kp 30 N m/rad, kd 0.2 N m s/rad, 10 kHz) with the base held fixed: a few small steps
(broadband), then a logarithmic chirp whose amplitude shrinks as 1/f so the wheel speed stays below ``--speed``
(the no-load speed is 30 rad/s). Around the loop's resonance (42 Hz, damping ratio 0.88 for the bare wheel; lower
and less damped with armature) the response shows the wheel inertia plus armature; slow parts show friction.
Zero-order held at ``--rate``.

    python sysid_excite.py [--out excite.csv] [--rate 100] [--f0 0.5] [--f1 30] [--duration 20]
"""

import argparse

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--out", default="excite.csv")
    parser.add_argument("--rate", type=float, default=100.0, help="Command rate [Hz].")
    parser.add_argument("--amplitude", type=float, default=0.5, help="Largest position amplitude [rad].")
    parser.add_argument("--speed", type=float, default=15.0, help="Largest wheel speed of the chirp [rad/s].")
    parser.add_argument("--step", type=float, default=0.1, help="Step height [rad].")
    parser.add_argument("--f0", type=float, default=0.5, help="Chirp start frequency [Hz].")
    parser.add_argument("--f1", type=float, default=30.0, help="Chirp end frequency [Hz].")
    parser.add_argument("--duration", type=float, default=20.0, help="Chirp length [s].")
    args = parser.parse_args()

    def hold(value, seconds):
        return np.full(round(seconds * args.rate), value)

    # steps: 0 -> +h -> -h -> 0, 0.5 s each
    steps = np.concatenate([hold(0.0, 0.5), hold(args.step, 0.5), hold(-args.step, 0.5), hold(0.0, 0.5)])

    t = np.arange(round(args.duration * args.rate)) / args.rate
    k = np.log(args.f1 / args.f0) / args.duration
    freq = args.f0 * np.exp(k * t)
    phase = 2 * np.pi * args.f0 * (np.exp(k * t) - 1) / k
    amplitude = np.minimum(args.amplitude, args.speed / (2 * np.pi * freq))
    chirp = amplitude * np.sin(phase)

    q_cmd = np.concatenate([steps, chirp, hold(0.0, 1.0)])
    t = np.arange(len(q_cmd)) / args.rate
    np.savetxt(args.out, np.column_stack([t, q_cmd]), delimiter=",", header="t,q_cmd", comments="", fmt="%.6g")
    print(f"wrote {args.out}: {len(t)} samples over {t[-1]:.1f} s")


if __name__ == "__main__":
    main()
