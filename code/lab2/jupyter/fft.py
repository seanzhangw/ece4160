import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import re

# Function to parse pitch, roll, and time from the file
def parse_pitch_roll_data(file_path):
    data = []
    times = []
    
    with open(file_path, 'r') as file:
        for line in file:
            # Extract numbers using regex
            match = re.findall(r"[-+]?\d*\.\d+|\d+", line)
            if match and len(match) >= 3:  # Ensure we have at least 3 values (Pitch, Roll, Time)
                pitch, roll, timestamp = float(match[0]), float(match[1]), int(match[2])
                data.append([pitch, roll])
                times.append(timestamp)
    
    return np.array(data), np.array(times)

# File path (update as needed)
file_path = "data/pitch_roll_low_pass_disturbance_115200.log"  # Replace with your actual file name

# Parse the file
data, timestamps = parse_pitch_roll_data(file_path)

# Extract pitch and roll
pitch = data[:, 0]
roll = data[:, 1]

# Compute sampling frequency (assuming timestamps are in milliseconds)
time_intervals = np.diff(timestamps) / 1000

fs = 1 / np.mean(time_intervals) if len(time_intervals) > 0 else 100  # Hz (default 100Hz if only one sample)

print(fs)

num_samples = len(pitch)

freqs = np.fft.fftfreq(num_samples, d=1/fs)[:num_samples//2]

fft_pitch = np.fft.fft(pitch)[:num_samples//2]
fft_roll = np.fft.fft(roll)[:num_samples//2]

fft_pitch_mag = np.abs(fft_pitch)
fft_roll_mag = np.abs(fft_roll)


# Plot FFT results
plt.figure(figsize=(10, 6))
plt.plot(freqs, fft_pitch_mag, label='Pitch FFT', linewidth=2)
plt.plot(freqs, fft_roll_mag, label='Roll FFT', linewidth=2)
plt.xlabel("Frequency (Hz)")
plt.ylabel("Magnitude")
plt.title("FFT of Pitch and Roll Data")
plt.legend()
plt.grid()
plt.show()
