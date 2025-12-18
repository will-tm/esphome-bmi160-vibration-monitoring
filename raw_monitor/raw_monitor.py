#!/usr/bin/env python3
"""Simple MQTT vibration monitor with live charts."""

import argparse
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import paho.mqtt.client as mqtt

# Data storage
raw_samples = []
fft_magnitudes = []
fft_history = []
MAX_HISTORY = 60

# Will be set from args
args = None


def on_connect(client, userdata, flags, rc):
    topic_raw = f"{args.topic_prefix}/raw_samples"
    topic_fft = f"{args.topic_prefix}/fft_magnitudes"
    print(f"Connected to MQTT broker (rc={rc})")
    client.subscribe(topic_raw)
    client.subscribe(topic_fft)
    print(f"Subscribed to:\n  - {topic_raw}\n  - {topic_fft}")


def on_message(client, userdata, msg):
    global raw_samples, fft_magnitudes, fft_history
    topic_raw = f"{args.topic_prefix}/raw_samples"
    topic_fft = f"{args.topic_prefix}/fft_magnitudes"
    try:
        data = json.loads(msg.payload.decode())
        if msg.topic == topic_raw:
            raw_samples = data
        elif msg.topic == topic_fft:
            fft_magnitudes = data
            fft_history.append(data)
            if len(fft_history) > MAX_HISTORY:
                fft_history.pop(0)
    except json.JSONDecodeError as e:
        print(f"Error parsing JSON: {e}")


def update_plots(frame):
    ax1.clear()
    ax2.clear()
    ax3.clear()

    update_interval = args.fft_size / args.sample_rate

    if raw_samples:
        n_samples = len(raw_samples)
        time_axis = np.arange(n_samples) / args.sample_rate * 1000  # ms
        ax1.plot(time_axis, raw_samples, 'b-', linewidth=0.5)
        ax1.set_title('Raw Vibration Samples')
        ax1.set_xlabel('Time (ms)')
        ax1.set_ylabel('Acceleration (g)')
        ax1.set_ylim(-args.accel_range, args.accel_range)
        ax1.grid(True, alpha=0.3)

    if fft_magnitudes:
        n_bins = len(fft_magnitudes)
        freq_axis = np.arange(n_bins) * args.sample_rate / args.fft_size
        ax2.plot(freq_axis, fft_magnitudes, 'r-', linewidth=0.5)
        ax2.set_title('FFT Magnitudes')
        ax2.set_xlabel('Frequency (Hz)')
        ax2.set_ylabel('Magnitude')
        ax2.set_xlim(0, args.sample_rate / 2)
        ax2.grid(True, alpha=0.3)

    if len(fft_history) > 1:
        heatmap_data = np.array(fft_history).T
        time_axis = np.arange(len(fft_history)) * update_interval

        ax3.imshow(
            heatmap_data,
            aspect='auto',
            origin='lower',
            cmap='hot',
            extent=[0, time_axis[-1], 0, args.sample_rate / 2]
        )
        ax3.set_title('FFT Spectrogram')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Frequency (Hz)')

    plt.tight_layout()


def main():
    global args, ax1, ax2, ax3

    parser = argparse.ArgumentParser(description='MQTT Vibration Monitor')
    parser.add_argument('--host', required=True,
                        help='MQTT broker host')
    parser.add_argument('--port', type=int, default=1883,
                        help='MQTT broker port (default: 1883)')
    parser.add_argument('--topic-prefix', default='esphome/washing-machine-vibration',
                        help='MQTT topic prefix (default: esphome/washing-machine-vibration)')
    parser.add_argument('--fft-size', type=int, default=1024,
                        help='FFT size (default: 1024)')
    parser.add_argument('--sample-rate', type=int, default=200,
                        help='Sample rate in Hz (default: 200)')
    parser.add_argument('--accel-range', type=float, default=2.0,
                        help='Accelerometer range in g (default: 2.0)')
    args = parser.parse_args()

    update_interval = args.fft_size / args.sample_rate
    print(f"Settings: FFT={args.fft_size}, Sample Rate={args.sample_rate} Hz, "
          f"Resolution={args.sample_rate/args.fft_size:.3f} Hz, Update={update_interval:.2f}s")

    # Set up MQTT client
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    print(f"Connecting to MQTT broker at {args.host}:{args.port}...")
    client.connect(args.host, args.port, 60)
    client.loop_start()

    # Set up matplotlib
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))
    ani = FuncAnimation(fig, update_plots, interval=500)

    plt.show()

    client.loop_stop()
    client.disconnect()


if __name__ == '__main__':
    main()
