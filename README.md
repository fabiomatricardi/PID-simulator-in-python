# PID-simulator-in-python

<img src='https://github.com/fabiomatricardi/PID-simulator-in-python/raw/main/1769869391.png' width=900>

PID simulator in python refactored from [original repo Destination2Unknown/PythonPID_Simulator](https://github.com/Destination2Unknown/PythonPID_Simulator/)

You can download the executable App from the Releases section

### WHY THIS APP
The two plots in the Python PID Simulator serve **complementary purposes** that together provide a complete picture of controller performance and behavior. This dual-plot design is intentional and extremely valuable for PID tuning education and diagnostics.

---

### 📊 Plot 1: System Response (Top Plot)
**Shows:** How the *process* responds to control actions  
**Variables displayed:**
- **SP (Setpoint)** – Blue line: Target value the controller tries to achieve
- **PV (Process Variable)** – Red line: Actual measured process value (e.g., temperature, pressure)
- **CV (Control Variable)** – Dark green line: Controller's output signal sent to the actuator (e.g., valve position, heater power)

**What to look for:**
| Behavior | What it means | Tuning implication |
|----------|---------------|---------------------|
| PV closely follows SP | ✅ Good control | Tuning is effective |
| PV oscillates around SP | ⚠️ Underdamped | Reduce Kp or increase Kd |
| PV lags behind SP | ⚠️ Slow response | Increase Kp or reduce dead time |
| Steady offset between PV and SP | ⚠️ Steady-state error | Increase Ki (integral action) |
| CV hits 0% or 100% limits | ⚠️ Actuator saturation | Reduce Kp or check process model |

> 💡 **Key insight**: This plot answers *"Is the controller achieving the desired result?"*

---

### 📈 Plot 2: Controller Internals (Bottom Plot)
**Shows:** How the *PID algorithm* generates its control signal  
**Variables displayed:**
- **P Term** – Lime: Proportional response (`Kp × error`)
- **I Term** – Orange: Integral accumulation (`Ki × ∫error dt`)
- **D Term** – Purple: Derivative prediction (`Kd × d(error)/dt`)

**What to look for:**
| Term behavior | What it means | Tuning implication |
|---------------|---------------|---------------------|
| Large P spikes at setpoint changes | ✅ Expected behavior | Normal proportional action |
| I term grows steadily over time | ✅ Eliminating offset | Healthy integral action |
| I term saturates (flatlines at limits) | ⚠️ Integral windup | Implement anti-windup or reduce Ki |
| D term shows high-frequency noise | ⚠️ Measurement noise amplified | Add derivative filter or reduce Kd |
| D term opposes P term during changes | ✅ Good damping | Derivative is working correctly |

> 💡 **Key insight**: This plot answers *"Why is the controller behaving this way?"* – it reveals the *mechanism* behind the response.

---

### 🔬 Why Two Plots? Real-World Example

Imagine you see **overshoot** in the top plot (PV spikes above SP):

| Without bottom plot | With bottom plot |
|---------------------|------------------|
| ❓ *"Why is it overshooting?"* | ✅ *"P term is too aggressive (large spike) and D term is too small to dampen it"* |
| Trial-and-error tuning | Targeted tuning: Reduce Kp *and/or* increase Kd |

This diagnostic capability is why professional control engineers always examine **both** system response *and* controller internals during tuning.

---

### 📐 Educational Value

The simulator deliberately shows both perspectives because:

1. **Top plot** teaches **control objectives**:
   - Settling time
   - Overshoot
   - Steady-state error
   - Disturbance rejection

2. **Bottom plot** teaches **PID mechanics**:
   - How P provides immediate response
   - How I eliminates offset but causes windup
   - How D provides damping but amplifies noise
   - How terms interact during transients

> 🎓 This dual-view approach is used in university control courses and industrial training because it bridges theory ("how PID works") with practice ("how the process responds").

---

### 📌 Performance Metric: ITAE

The title shows **ITAE** (Integral of Time-weighted Absolute Error):
```
ITAE = ∫ t × |SP(t) - PV(t)| dt
```
- **Lower ITAE = Better performance**
- Time-weighting penalizes *persistent* errors more than brief ones
- Helps compare tuning strategies objectively (not just visually)

---

### 💡 Pro Tip for Tuning

Use the plots together in this workflow:
1. **Observe top plot**: Identify problem (e.g., "too much overshoot")
2. **Check bottom plot**: Diagnose cause (e.g., "P term too large, D term too small")
3. **Adjust parameters**: Reduce Kp *and/or* increase Kd
4. **Re-run simulation**: Verify improvement in *both* plots

This closed-loop diagnostic approach is exactly how engineers tune real industrial controllers – just without risking plant shutdowns during experimentation! 😊


---

---



### 📦 Step 2: PyInstaller Build Command (Windows)

```bash
pyinstaller ^
  --windowed ^
  --onefile ^
  --name="PID_Simulator" ^
  --collect-data ttkbootstrap ^
  --hidden-import ttkbootstrap.style ^
  --hidden-import ttkbootstrap.constants ^
  --hidden-import matplotlib.backends.backend_tkagg ^
  --add-data "logo.png;." ^
  PythonPID_Simulator.pyw
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
   dist\PID_Simulator.exe
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

#### ❓ "Want a single-folder build instead of --onefile?"
Use `--onedir` (faster startup, easier debugging):
```bash
pyinstaller ^
  --windowed ^
  --onedir ^
  --collect-data ttkbootstrap ^
  --add-data "logo.png;." ^
  PythonPID_Simulator.pyw
```
→ Output: `dist\PID_Simulator\` folder containing `.exe` + assets

#### 🔍 Debugging Tip: See where assets load from
Add this temporary debug line before logo loading:
```python
print(f"Looking for logo at: {logo_path.absolute()}")
print(f"sys._MEIPASS = {getattr(sys, '_MEIPASS', 'NOT SET')}")
```
→ Run the `.exe` from Command Prompt to see output

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

This is the industry-standard approach used by professional Python desktop apps (e.g., KiCad, Thonny). 😊
