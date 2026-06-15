# Overviwe - Tone-Based Access Control System (ESP32-C6)

The main goal of this system is to emulate a secure door mechanism (simulated via an external LED) that locks or unlocks only when the microphone detects a specific, pre-defined 4-digit sequence of acoustic tones. It acts similarly to the dual-tone multi-frequency (DTMF) signaling used in automated telephone menus.

---

##  How It Works

The system bridges real-time hardware data acquisition with digital signal processing (DSP) algorithms directly on the microcontroller:

1. **Continuous Audio Capture:** The ESP32-C6 samples audio from the analog microphone at a continuous rate of 20 kHz. We use **FreeRTOS with Queues (FIFOs)** to completely isolate the high-priority audio sampling routines from the heavy mathematical calculations, ensuring the CPU never drops data blocks.
2. **Signal Clean-up:** A real-time pre-processing block calculates the arithmetic mean of each 2048-sample block and subtracts it to completely eliminate the hardware's DC offset bias voltage, centering the waveform nicely around zero.
3. **Custom FIR Filters:** We designed three 127-point Finite Impulse Response (FIR) bandpass filters using Octave to target the specific frequencies derived from our student numbers:
   * **Symbol "0":** 2000 Hz
   * **Symbol "1":** 2720 Hz
   * **Symbol "2":** 3440 Hz
4. **Competitive Energy Logic (Winner-Takes-All):** To prevent ambient background noises or accidental musical frequencies from triggering false positives, the system runs a competitive algorithm. A frequency is only recognized if its energy crosses a strict safety baseline (Threshold of 18000) **and** is simultaneously higher than the other two channels combined.
5. **Robust FSM & Security Latch:** A 9-state hybrid Mealy-Moore Finite State Machine validates the digital inputs. It features an acoustic debounce filter (tones must be steady across two blocks) and an automatic inactivity timeout: if a sequence is left half-finished for more than 8 silent blocks, it resets for security.

---

## Filter Selectivity Results

Real bench testing inside the university laboratory showed excellent spectral insulation. The matrix below displays the average energy processed by each filter channel under different inputs:

| Input Frequency (Hz) | 2000 Hz Filter Energy | 2720 Hz Filter Energy | 3440 Hz Filter Energy | Detected Symbol |
| :--- | :---: | :---: | :---: | :---: |
| **2000 Hz** | **32400** | 1420 | 310 | `0` |
| **2720 Hz** | 890 | **31800** | 1150 | `1` |
| **3440 Hz** | 210 | 750 | **35100** | `2` |
| **Ambient Noise** | 1200 | 980 | 1450 | `None` |

---

## How to Build and Run

### Prerequisites
* **ESP-IDF Toolchain** (v5.1 or superior) installed and configured.
* An **ESP32-C6** development board.

### Quick Start Steps (VS Code Extension)
1. Clone this repository to your local machine and open the project folder in **Visual Studio Code**.
2. Make sure you have the official **ESP-IDF Extension** installed in VS Code.
3. Click on **ESP-IDF: Select Device** (the plug icon in the bottom status bar) and select the correct USB serial port connected to your ESP32-C6.
4. Click on **ESP-IDF: Build Project** (the cylinder icon) to compile the system binaries.
5. Click on **ESP-IDF: Flash Device** (the lightning bolt icon) to upload the compiled firmware to your board.
6. Click on **ESP-IDF: Monitor Program** (the computer monitor icon) to open the live telemetry serial log.

*(Tip: You can also use the **ESP-IDF: Build, Flash and Monitor** shortcut—the "all-in-one" play button icon—to run all these steps automatically in one click).*
