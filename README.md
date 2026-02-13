# PID-simulator-in-python

<img src='https://github.com/fabiomatricardi/PID-simulator-in-python/raw/main/header_image.png' width=900>

PID simulator in python refactored from [original repo Destination2Unknown/PythonPID_Simulator](https://github.com/Destination2Unknown/PythonPID_Simulator/)

You can download the executable App [from the Releases section](https://github.com/fabiomatricardi/PID-simulator-in-python/releases/tag/PID_tuning_simulator)

# Python PID Simulator

A professional-grade PID controller simulator with First Order Plus Dead Time (FOPDT) process modeling, designed for control engineers, students, and process automation professionals to visualize and tune PID controllers in a realistic simulated environment.

## ✨ Key Features

- **Realistic Process Simulation**: FOPDT model with dead time interpolation for fractional delays
- **Industrial PID Implementation**: 
  - Ti (Integral Time) in seconds (standard industrial unit) with internal conversion: `Ki = Kp / Ti`
  - Derivative-on-measurement (avoids derivative kick on setpoint changes)
  - Robust anti-windup protection with conditional integration
- **Auto-Refresh**: Real-time simulation updates as parameters change (no manual refresh button needed)
- **Fixed 1500-Second Simulation**: Consistent visualization window for performance comparison
- **Professional Visualization**: Clean matplotlib plots with SP/PV/CV response curves
- **Export Functionality**:
  - PNG export for publication-quality plots (300 DPI)
  - CSV export for detailed analysis in Excel/Python (includes all tuning parameters and time-series data)
- **Parameter Validation**: Real-time checks for critical parameters (time constant > 0, dead time ≥ 0)
- **Contextual Tooltips**: Hover explanations for all parameters and concepts
- **Compact UI**: Optimized 1000px width layout for all screen sizes

## 📦 Installation

### Prerequisites
- Python 3.8 or higher
- pip package manager

### Dependencies Installation
```bash
pip install ttkbootstrap matplotlib numpy scipy pillow
```

### Optional (for PyInstaller bundling)
```bash
pip install pyinstaller
```

## 🚀 Usage

1. **Run the application**:
   ```bash
   python fixed_PID_Tuner.py
   ```

2. **Adjust parameters** in the left (Controller) and right (Process) panels:
   - Changes automatically trigger simulation refresh
   - Invalid parameters show visual feedback in the ITAE display

3. **Interpret the plot** (middle panel):
   - **Blue line**: Setpoint (SP) - target value
   - **Red line**: Process Variable (PV) - simulated process response
   - **Green line**: Controller Output (CV) - valve/actuator position

4. **Export results**:
   - Click "Export Plot (PNG)" for high-resolution images
   - Click "Export Data (CSV)" for time-series analysis

## 🔬 Technical Implementation

### FOPDT Model Explained
The simulator uses a **First Order Plus Dead Time (FOPDT)** model to represent real-world processes:

```
τ·dPV/dt = -PV + K·CV(t-θ) + Bias
```

Where:
- **K (Gain)**: Change in PV per unit change in CV (e.g., °C/% valve opening)
- **τ (Time Constant)**: Time to reach 63.2% of final value after a step change (seconds)
- **θ (Dead Time)**: Delay between CV change and PV response (seconds)
- **Bias**: Steady-state PV value when CV = 0

FOPDT accurately models >80% of industrial processes (temperature, pressure, level, flow) and is the foundation for most PID tuning methods (Ziegler-Nichols, Cohen-Coon, etc.).

### PID Controller Implementation
The simulator implements the **parallel form** with industrial best practices:

```
CV = Kp·e + (Kp/Ti)·∫e·dt - Kd·d(PV)/dt
```

Key implementation details:
- **Ti in seconds**: User-facing parameter (standard in industrial controllers) with internal conversion to Ki = Kp/Ti
- **Derivative-on-measurement**: Uses `-Kd·d(PV)/dt` instead of `+Kd·d(SP)/dt` to prevent "derivative kick" when setpoint changes
- **Anti-windup**: Conditional integration prevents integral term windup during output saturation
- **Direction handling**: Automatically switches between Direct/Reverse action based on process gain sign

### Simulation Parameters
- Fixed duration: **1500 seconds** (consistent comparison baseline)
- Setpoint profile:
  - 0-10s: Initial steady state
  - 10-650s: First step change (+1.0 units)
  - 650-1500s: Second step change (+0.8 units)
- Process noise: Minimal (`±0.002`) for clean reference matching (adjustable in source code)

## 📊 Matching Reference Plots

To match Excel reference plots (like those from Control Station or similar tools):

1. **Use normalized parameters**:
   - Process Gain: 0.76
   - Time Constant: 39.0 sec
   - Dead Time: 18.0 sec
   - Bias: 0.0 (removed from calculations)

2. **Use industrial tuning values**:
   - Kp: 3.0
   - Ti: 40.0 sec (not Ki!)
   - Kd: 6.0 sec

3. **Ensure minimal noise**: The simulator uses low noise (`±0.002`) by default to match clean reference plots. Increase noise in `self.noise = np.random.uniform(-0.002, 0.002, ...)` if needed for realism.

## 🖼️ Screenshots

<img src='https://github.com/fabiomatricardi/PID-simulator-in-python/raw/main/screenshot001.png' width=800>

*Main application interface showing PID response simulation*

<img src='https://github.com/fabiomatricardi/PID-simulator-in-python/raw/main/screenshot002.png' width=400> <img src='https://github.com/fabiomatricardi/PID-simulator-in-python/raw/main/screenshot003.png' width=400> 

*Contextual help available for all parameters via hover tooltips*

## 📤 Export Formats

### PNG Export
- 300 DPI resolution for publication quality
- Tight layout with legends and grid lines
- Filename: User-specified with `.png` extension

### CSV Export Structure
```csv
Time (s),SP,PV,CV,P_Term,I_Term,D_Term,Kp,Ti_sec,Kd_sec,Process_Gain,Time_Constant_sec,Dead_Time_sec
0,0.0000,0.0002,0.0000,0.0000,0.0000,0.0000,3.0000,40.0000,6.0000,0.7600,39.0000,18.0000
1,0.0000,0.0003,0.0000,0.0000,0.0000,0.0000,3.0000,40.0000,6.0000,0.7600,39.0000,18.0000
...
```

## 🙏 Credits & Attribution

- Original concept and implementation: [destination0b10unknown](https://github.com/Destination2Unknown/PythonPID_Simulator/)
- Enhanced implementation: Fabio Matricardi (fabio.matricardi@gmail.com)
- FOPDT theory based on: "Process Control: Modeling, Design, and Simulation" by B. Wayne Bequette
- Industrial PID practices aligned with: ISA-75.25 and "Advanced Control Unleashed" by Robert Rice

## 📜 License

MIT License - see [LICENSE](LICENSE) file for details.

```
Copyright (c) 2024 Fabio Matricardi

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## 💡 Pro Tips

1. **For aggressive tuning**: Decrease Ti (faster integral action) and increase Kd (more damping)
2. **For sluggish processes**: Increase Kp and decrease Ti proportionally (maintain Kp/Ti ratio)
3. **Dead time dominance**: When θ/τ > 0.5, consider Smith Predictor or model predictive control
4. **Validation shortcut**: Press Tab between spinboxes for rapid parameter adjustment with auto-refresh
5. **Noise adjustment**: Edit `self.noise = np.random.uniform(-0.002, 0.002, ...)` in source code for different noise levels

---

*Developed with ♥ for control engineers worldwide*  
*Tested on Windows 10/11 with Python 3.9-3.12*

---



### 📦 Step 2: PyInstaller Build Command (Windows)

```bash
pyinstaller --windowed --onefile --collect-data ttkbootstrap --add-data "logo2.png;." --add-data "icon.ico;." --name="2026_PID_Simulator" --icon="icon.ico" fixed_PID_Tuner.py
```

#### 🔑 Key Flag Explained:
| Flag | Meaning |
|------|---------|
| `--add-data "logo.png;."` | Bundles `logo.png` into the root of the `.exe` archive<br>→ Windows uses `;` separator (Linux/macOS use `:`) |
| `--collect-data ttkbootstrap` | Bundles ttkbootstrap theme files (required!) |
| `--windowed` | Suppresses console window (mimics `.pyw` behavior) |

---

### 🧪 Verification Checklist (Before Distribution)

1. **Build succeeds** without errors
2. **Test on build machine**:
   ```bash
   dist\2026_PID_Simulator.exe
   ```
   → Logo appears with transparency ✓
3. **Test on clean Windows VM** (no Python installed):
   → Logo still appears ✓  
   → All plots render ✓  
   → "Refresh" button works ✓

---

### 💡 Pro Tips & Troubleshooting

#### ❓ "Logo missing in .exe but works in dev?"
→ You forgot the `resource_path()` patch! PyInstaller bundles files to a temp folder (`sys._MEIPASS`), not next to the `.exe`.

#### ❓ "How to bundle multiple assets?"
```bash
--add-data "logo.png;." ^
--add-data "icon.ico;." ^
--add-data "help.pdf;docs"
```
→ Structure: `source;destination` (destination is relative to bundle root)

---

### ✅ Final File Structure After Build

```
dist/
└── PID_Simulator.exe          ← Your standalone app
    (when run, extracts to temp folder containing:)
        ├── logo.png           ← Bundled asset (accessible via sys._MEIPASS)
        ├── ttkbootstrap/      ← Theme files (via --collect-data)
        └── ...                ← Other dependencies
```

With these changes, your `.exe` will:
- ✅ Display the logo with perfect transparency
- ✅ Work on any Windows machine (no Python required)
- ✅ Preserve all simulator functionality
- ✅ Show proper attribution footer

---


<img src='https://github.com/fabiomatricardi/PID-simulator-in-python/raw/main/1769884094.png' width=800>


---




This is the industry-standard approach used by professional Python desktop apps (e.g., KiCad, Thonny). 😊
