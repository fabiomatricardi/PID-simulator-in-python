"""
Fixed Python PID Simulator
Based on original code by destination0b10unknown@gmail.com
Refactored by Fabio Matricardi - fabio.matricardi@gmail.com
Licensed under the MIT License
"""
import sys
import os
from pathlib import Path
import numpy as np
from scipy.integrate import odeint
import ttkbootstrap as ttk
from ttkbootstrap.tooltip import ToolTip
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import csv
from tkinter import filedialog, messagebox
import math


def resource_path(relative_path):
    """Get absolute path to resource, works for dev and PyInstaller bundles"""
    try:
        # PyInstaller creates a temp folder and stores path in _MEIPASS
        base_path = sys._MEIPASS
    except Exception:
        base_path = Path(__file__).parent if '__file__' in globals() else Path.cwd()
    return Path(base_path) / relative_path


class PID_Controller:
    """
    Proportional-Integral-Derivative (PID) controller with derivative-on-measurement.
    Implements: CV = Kp·e + (Kp/Ti)·∫e·dt - Kd·d(PV)/dt
    """
    
    def __init__(self):
        self.Kp = 1.0          # Proportional gain
        self.Ti = 10.0         # Integral time (seconds)
        self.Kd = 0.01         # Derivative gain (seconds)
        self.setpoint = 50.0
        self._min_output = 0.0
        self._max_output = 100.0
        self._proportional = 0.0
        self._integral = 0.0
        self._derivative = 0.0
        self._last_eD = 0.0    # Last PV value for derivative calculation
        self._lastCV = 0.0
        self._d_init = False   # Flag for derivative initialization
        self.reset()
    
    def __call__(self, PV=0.0, SP=0.0, direction="Direct"):
        """
        Calculate control output using parallel form with Ti conversion:
        Ki_internal = Kp / Ti  (when Ti > 0)
        """
        # Update setpoint
        self.setpoint = SP
        
        # P term: error calculation based on direction
        if direction == "Direct":
            e = SP - PV
        else:
            e = PV - SP
        self._proportional = self.Kp * e
        
        # I Term: Ki_internal = Kp / Ti (with Ti=0 protection)
        Ki_internal = self.Kp / self.Ti if self.Ti > 0 else 0.0
        
        # Anti-windup: Only integrate when output not saturated OR error opposes saturation
        output_before_integral = self._proportional + self._derivative
        
        if (self._min_output < output_before_integral + self._integral < self._max_output):
            # Not saturated - integrate normally
            self._integral += Ki_internal * e
        else:
            # Saturated - only integrate if error would reduce saturation
            if (output_before_integral + self._integral >= self._max_output and Ki_internal * e < 0):
                self._integral += Ki_internal * e
            elif (output_before_integral + self._integral <= self._min_output and Ki_internal * e > 0):
                self._integral += Ki_internal * e
        
        # Clamp integral term to prevent windup
        self._integral = self._clamp(self._integral, (self._min_output, self._max_output))
        
        # D term: Derivative-on-measurement (industrial best practice)
        eD = -PV
        if self._d_init:
            self._derivative = self.Kd * (eD - self._last_eD)
        else:
            self._derivative = 0.0
            self._d_init = True
        
        # Controller Output
        CV = self._proportional + self._integral + self._derivative
        CV = self._clamp(CV, (self._min_output, self._max_output))
        
        # Update stored data for next iteration
        self._last_eD = eD
        self._lastCV = CV
        return CV
    
    @property
    def components(self):
        """Get individual PID components (P, I, D terms)"""
        return self._proportional, self._integral, self._derivative
    
    @property
    def tunings(self):
        """Get current PID tunings (Kp, Ti, Kd)"""
        return self.Kp, self.Ti, self.Kd
    
    @tunings.setter
    def tunings(self, tunings):
        """Set PID tunings (Kp, Ti, Kd)"""
        self.Kp, self.Ti, self.Kd = tunings
        # Reset derivative initialization flag when Kd changes
        if self.Kd != 0:
            self._d_init = False
    
    @property
    def output_limits(self):
        """Get output limits"""
        return self._min_output, self._max_output
    
    @output_limits.setter
    def output_limits(self, limits):
        """Set output limits and clamp integral term"""
        if limits is None:
            self._min_output, self._max_output = 0, 100
            return
        self._min_output, self._max_output = limits
        self._integral = self._clamp(self._integral, limits)
    
    def reset(self):
        """Reset controller state"""
        self._proportional = 0.0
        self._integral = 0.0
        self._derivative = 0.0
        self._last_eD = 0.0
        self._d_init = False
        self._lastCV = 0.0
    
    @staticmethod
    def _clamp(value, limits):
        """Clamp value between limits"""
        lower, upper = limits
        if value is None:
            return None
        if upper is not None and value > upper:
            return upper
        if lower is not None and value < lower:
            return lower
        return value


class FOPDT_Model:
    """First Order Plus Dead Time (FOPDT) process model with interpolation for dead time."""
    
    def __init__(self):
        self.Gain = 1.0
        self.Time_Constant = 10.0
        self.Dead_Time = 2.0
        self.Bias = 0.0
        self.work_CV = []
    
    def change_params(self, data):
        """Update model parameters with validation"""
        gain, tc, dt, bias = data
        if tc <= 0:
            raise ValueError("Time Constant must be > 0")
        if dt < 0:
            raise ValueError("Dead Time cannot be negative")
        self.Gain, self.Time_Constant, self.Dead_Time, self.Bias = gain, tc, dt, bias
    
    def _calc(self, work_PV, ts):
        """Calculate derivative for ODE solver with dead time interpolation"""
        delay = ts - self.Dead_Time
        
        if delay <= 0:
            um = 0.0
        elif delay >= len(self.work_CV):
            um = self.work_CV[-1]
        else:
            # Linear interpolation for fractional dead times
            idx = int(delay)
            frac = delay - idx
            if idx + 1 < len(self.work_CV):
                um = self.work_CV[idx] * (1 - frac) + self.work_CV[idx + 1] * frac
            else:
                um = self.work_CV[idx]
        
        # Note: We're ignoring bias in the calculation as requested
        dydt = (-(work_PV) + self.Gain * um) / self.Time_Constant
        return dydt
    
    def update(self, work_PV, ts):
        """Update process variable using ODE solver"""
        y = odeint(self._calc, work_PV, ts, rtol=1e-4, atol=1e-6)
        return float(y[-1, 0])


class PID_Simulator:
    """Enhanced PID Simulator with all requested fixes."""
    
    SIM_LENGTH = 1500  # Fixed simulation length (seconds)
    
    def __init__(self):
        plt.style.use("bmh")
        self.root = ttk.Window(themename="yeti")
        self.root.title("Python PID Simulator - Fixed")
        # Set fixed window size of 1000 pixels width
        self.root.geometry("1000x700")
        
        # Configure style
        style = ttk.Style()
        style.configure(".", font=("Helvetica", 11))
        style.configure("TLabelframe.Label", font=("Helvetica", 12, "bold"))
        style.configure("Header.TLabel", font=("Helvetica", 24, "bold"))
        style.configure("Export.TButton", font=("Helvetica", 10, "bold"))
        
        # ===== TOP FRAME: Logo/Header =====
        self.top_frame = ttk.Frame(self.root, padding=10)
        self.top_frame.pack(side="top", fill="x")
        
        # Load logo
        logo_path = resource_path("logo.png")
        if logo_path.exists():
            try:
                from PIL import Image, ImageTk
                img = Image.open(logo_path)
                max_width = 600
                if img.width > max_width:
                    ratio = max_width / img.width
                    new_size = (max_width, int(img.height * ratio))
                    img = img.resize(new_size, Image.LANCZOS)
                self.logo_photo = ImageTk.PhotoImage(img)
                logo_label = ttk.Label(self.top_frame, image=self.logo_photo)
                logo_label.image = self.logo_photo
                logo_label.pack(pady=(0, 10))
            except Exception as e:
                print(f"Warning: Could not load logo: {e}")
                ttk.Label(self.top_frame, text="PID SIMULATOR", style="Header.TLabel").pack(pady=(0, 10))
        else:
            ttk.Label(self.top_frame, text="PID SIMULATOR", style="Header.TLabel").pack(pady=(0, 10))
        
        # ===== MAIN CONTENT FRAME =====
        self.master_frame = ttk.Frame(self.root)
        self.master_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        # Create three-panel layout
        self._create_controller_panel()
        self._create_plot_panel()
        self._create_process_panel()
        
        # ===== BOTTOM FRAME: Export Buttons & Attribution =====
        self.footer_frame = ttk.Frame(self.root, padding=10)
        self.footer_frame.pack(side="bottom", fill="x")
        
        # Export buttons
        export_frame = ttk.Frame(self.footer_frame)
        export_frame.pack(pady=5)
        
        ttk.Button(export_frame, text="Export Plot (PNG)", 
                  command=self.export_plot, style="Export.TButton", width=20).pack(side="left", padx=5)
        ttk.Button(export_frame, text="Export Data (CSV)", 
                  command=self.export_data, style="Export.TButton", width=20).pack(side="left", padx=5)
        
        # Attribution
        footer_text = (
            "Created by Fabio Matricardi | "
            "Refactored from GitHub: https://github.com/Destination2Unknown/PythonPID_Simulator/"
        )
        ttk.Label(
            self.footer_frame,
            text=footer_text,
            font=("Helvetica", 9),
            foreground="#555555",
            justify="center"
        ).pack(pady=(10, 5))
        
        # Initialize simulation data arrays (fixed 1500s length)
        self.SP = np.zeros(self.SIM_LENGTH)
        self.PV = np.zeros(self.SIM_LENGTH)
        self.CV = np.zeros(self.SIM_LENGTH)
        self.pterm = np.zeros(self.SIM_LENGTH)
        self.iterm = np.zeros(self.SIM_LENGTH)
        self.dterm = np.zeros(self.SIM_LENGTH)
        
        # Process noise (100x smaller for matching reference)
        np.random.seed(42)
        self.noise = np.random.uniform(-0.002, 0.002, self.SIM_LENGTH)
        
        # Instantiate controller and process model
        self.pid = PID_Controller()
        self.process_model = FOPDT_Model()
        self.itae = 0.0
        
        # Setup auto-refresh triggers
        self._setup_auto_refresh()
        
        # Generate initial plot
        self.generate_response()
    
    def _create_controller_panel(self):
        """Create left panel with PID controller settings"""
        # FIXED: Removed 'padding' parameter from LabelFrame
        self.left_frame = ttk.LabelFrame(self.master_frame, text=" Controller Settings ")
        self.left_frame.pack(side="left", fill="both", expand=True, padx=(0, 5), pady=5)
        
        # Add internal padding via frame
        pad_frame = ttk.Frame(self.left_frame, padding=10)
        pad_frame.pack(fill="both", expand=True)
        
        # GUI Variables with validation
        self.kp = ttk.DoubleVar(value=3.0)
        self.ti = ttk.DoubleVar(value=40.0)  # Changed from Ki to Ti (seconds)
        self.kd = ttk.DoubleVar(value=6.0)
        
        # Kp entry
        ttk.Label(pad_frame, text="Kp (Proportional Gain):").grid(row=0, column=0, sticky="w", pady=8)
        kp_spin = ttk.Spinbox(pad_frame, from_=-1000, to=1000, increment=0.1, 
                             textvariable=self.kp, width=12, font=("Helvetica", 11))
        kp_spin.grid(row=0, column=1, padx=10, pady=8, sticky="w")
        ToolTip(kp_spin, "Proportional gain: Larger values increase responsiveness but can cause oscillations")
        
        # Ti entry (was Ki) - now in seconds
        ttk.Label(pad_frame, text="Ti (Integral Time, sec):").grid(row=1, column=0, sticky="w", pady=8)
        ti_spin = ttk.Spinbox(pad_frame, from_=0.1, to=1000, increment=0.1, 
                             textvariable=self.ti, width=12, font=("Helvetica", 11))
        ti_spin.grid(row=1, column=1, padx=10, pady=8, sticky="w")
        ToolTip(ti_spin, "Integral time in seconds (Ti). Smaller values increase integral action. "
                        "Internal conversion: Ki = Kp / Ti")
        
        # Kd entry
        ttk.Label(pad_frame, text="Kd (Derivative Gain, sec):").grid(row=2, column=0, sticky="w", pady=8)
        kd_spin = ttk.Spinbox(pad_frame, from_=0, to=1000, increment=0.01, 
                             textvariable=self.kd, width=12, font=("Helvetica", 11))
        kd_spin.grid(row=2, column=1, padx=10, pady=8, sticky="w")
        ToolTip(kd_spin, "Derivative gain in seconds. Provides damping to reduce overshoot. "
                        "Uses derivative-on-measurement to avoid derivative kick on SP changes.")
        
        # Direction info
        dir_info = ttk.Label(pad_frame, text="Direction: Auto (based on process gain)", 
                            font=("Helvetica", 9, "italic"), foreground="#555")
        dir_info.grid(row=3, column=0, columnspan=2, pady=(15, 5), sticky="w")
        ToolTip(dir_info, "Controller direction automatically set based on process gain sign:\n"
                         "• Positive gain → Direct action (SP↑ → CV↑)\n"
                         "• Negative gain → Reverse action (SP↑ → CV↓)")
    
    def _create_plot_panel(self):
        """Create middle panel with matplotlib plot (only top plot)"""
        # FIXED: Removed 'padding' parameter from LabelFrame
        self.middle_frame = ttk.LabelFrame(self.master_frame, text=" Simulation Response ")
        self.middle_frame.pack(side="left", fill="both", expand=True, padx=5, pady=5)
        
        # Add internal padding via frame
        pad_frame = ttk.Frame(self.middle_frame, padding=10)
        pad_frame.pack(fill="both", expand=True)
        
        # Create matplotlib figure with 1 subplot (only top plot)
        self.fig = Figure(figsize=(8, 4), dpi=100)
        self.ax = self.fig.add_subplot(1, 1, 1)
        
        # Embed figure in Tkinter canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=pad_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        
        # ITAE display
        self.itae_label = ttk.Label(pad_frame, text="ITAE: Calculating...", 
                                   font=("Helvetica", 11, "bold"))
        self.itae_label.pack(pady=(5, 0))
    
    def _create_process_panel(self):
        """Create right panel with process model settings (without bias)"""
        # FIXED: Removed 'padding' parameter from LabelFrame
        self.right_frame = ttk.LabelFrame(self.master_frame, text=" Process Model (FOPDT) ")
        self.right_frame.pack(side="right", fill="both", expand=True, padx=(5, 0), pady=5)
        
        # Add internal padding via frame
        pad_frame = ttk.Frame(self.right_frame, padding=10)
        pad_frame.pack(fill="both", expand=True)
        
        # GUI Variables with validation (no bias)
        self.model_gain = ttk.DoubleVar(value=0.76)
        self.model_tc = ttk.DoubleVar(value=39.0)
        self.model_dt = ttk.DoubleVar(value=18.0)
        
        # Gain entry
        ttk.Label(pad_frame, text="Process Gain:").grid(row=0, column=0, sticky="w", pady=8)
        gain_spin = ttk.Spinbox(pad_frame, from_=-1000, to=1000, increment=0.1, 
                               textvariable=self.model_gain, width=12, font=("Helvetica", 11))
        gain_spin.grid(row=0, column=1, padx=10, pady=8, sticky="w")
        ToolTip(gain_spin, "Process gain (Kp): Change in PV per unit change in CV.\n"
                          "Positive = Direct acting process\n"
                          "Negative = Reverse acting process")
        
        # Time Constant entry
        ttk.Label(pad_frame, text="Time Constant (sec):").grid(row=1, column=0, sticky="w", pady=8)
        tc_spin = ttk.Spinbox(pad_frame, from_=0.1, to=1000, increment=0.1, 
                             textvariable=self.model_tc, width=12, font=("Helvetica", 11))
        tc_spin.grid(row=1, column=1, padx=10, pady=8, sticky="w")
        ToolTip(tc_spin, "Time constant (τ): Time to reach 63.2% of final value after step change.\n"
                        "Must be > 0. Larger values = slower process response.")
        
        # Dead Time entry
        ttk.Label(pad_frame, text="Dead Time (sec):").grid(row=2, column=0, sticky="w", pady=8)
        dt_spin = ttk.Spinbox(pad_frame, from_=0, to=1000, increment=0.1, 
                             textvariable=self.model_dt, width=12, font=("Helvetica", 11))
        dt_spin.grid(row=2, column=1, padx=10, pady=8, sticky="w")
        ToolTip(dt_spin, "Dead time (θ): Delay between CV change and PV response.\n"
                        "Cannot be negative. Fractional values supported with interpolation.")
        
        # Note: Removed bias parameter as requested
    
    def _setup_auto_refresh(self):
        """Setup trace callbacks for auto-refresh on parameter changes"""
        self.kp.trace_add("write", self._on_parameter_change)
        self.ti.trace_add("write", self._on_parameter_change)  # Ti instead of Ki
        self.kd.trace_add("write", self._on_parameter_change)
        self.model_gain.trace_add("write", self._on_parameter_change)
        self.model_tc.trace_add("write", self._on_parameter_change)
        self.model_dt.trace_add("write", self._on_parameter_change)
    
    def _on_parameter_change(self, *args):
        """Callback for auto-refresh with validation"""
        try:
            # Validate critical parameters before simulation
            if self.model_tc.get() <= 0:
                raise ValueError("Time Constant must be > 0")
            if self.model_dt.get() < 0:
                raise ValueError("Dead Time cannot be negative")
            if self.ti.get() <= 0:
                raise ValueError("Ti must be > 0 (use large value to disable integral action)")
            
            # Only refresh if values are valid numbers
            self.generate_response()
        except Exception as e:
            # Don't show error on partial input (e.g., typing "0.")
            if hasattr(e, 'args') and "could not convert" not in str(e).lower():
                self.itae_label.config(text=f"⚠️ Invalid: {str(e)[:50]}", foreground="red")
    
    def generate_response(self):
        """Generate PID response simulation with fixed 1500s duration"""
        try:
            # Validate parameters
            if self.model_tc.get() <= 0:
                raise ValueError("Time Constant must be > 0")
            if self.model_dt.get() < 0:
                raise ValueError("Dead Time cannot be negative")
            if self.ti.get() <= 0:
                raise ValueError("Ti must be > 0")
            
            # Set simulation parameters
            start_of_step = 10  # seconds
            
            # Determine controller direction based on process gain
            direction = "Direct" if self.model_gain.get() > 0 else "Reverse"
            
            # Update process model with validation
            self.process_model.change_params((
                self.model_gain.get(),
                self.model_tc.get(),
                self.model_dt.get(),
                0.0  # Set bias to 0 as requested
            ))
            
            # Update PID tunings (Ti conversion happens internally in __call__)
            self.pid.tunings = (self.kp.get(), self.ti.get(), self.kd.get())
            self.pid.reset()
            
            # Initialize PV array without bias
            self.PV[0] = self.noise[0]
            
            # Simulation loop (fixed 1500s)
            self.itae = 0.0
            for i in range(self.SIM_LENGTH - 1):
                # Setpoint profile: step changes at 10s, 650s (changed from 900s)
                if i < start_of_step:
                    self.SP[i] = 0.0  # No bias
                elif i < 650:  # Step change at 650 seconds
                    self.SP[i] = 1.0 if direction == "Direct" else -1.0
                else:
                    self.SP[i] = 0.8 if direction == "Direct" else -0.8
                
                # Calculate controller output
                self.CV[i] = self.pid(PV=self.PV[i], SP=self.SP[i], direction=direction)
                
                # Update process model
                self.process_model.work_CV = self.CV[:i+1]
                self.PV[i + 1] = self.process_model.update(self.PV[i], [i, i + 1])
                self.PV[i + 1] += self.noise[i + 1]  # Add process noise
                
                # Store individual terms
                self.pterm[i], self.iterm[i], self.dterm[i] = self.pid.components
                
                # Calculate ITAE (only after step change)
                if i >= start_of_step:
                    self.itae += (i - start_of_step) * abs(self.SP[i] - self.PV[i])
            
            # Final setpoint value
            self.SP[self.SIM_LENGTH - 1] = self.SP[self.SIM_LENGTH - 2]
            
            # Update plot
            self._update_plot()
            
            # Update ITAE display
            itae_norm = self.itae / self.SIM_LENGTH
            self.itae_label.config(text=f"ITAE: {itae_norm:.2f}", foreground="black")
            
        except Exception as e:
            messagebox.showerror("Simulation Error", f"Check parameters:\n{str(e)}")
            self.itae_label.config(text=f"⚠️ Error: {str(e)[:40]}", foreground="red")
    
    def _update_plot(self):
        """Update matplotlib plot with current simulation data (only top plot)"""
        # Clear previous plot
        self.ax.clear()
        
        time = np.arange(self.SIM_LENGTH)
        
        # Only top plot: SP, PV, CV
        self.ax.plot(time, self.SP, color="blue", linewidth=2, label="SP (Setpoint)")
        self.ax.plot(time, self.CV, color="darkgreen", linewidth=2, label="CV (Controller Output)")
        self.ax.plot(time, self.PV, color="red", linewidth=2, label="PV (Process Variable)")
        self.ax.set_ylabel("Value")
        self.ax.set_title(f"PID Response (Kp={self.kp.get():.2f}, Ti={self.ti.get():.1f}s, Kd={self.kd.get():.2f}s)", 
                          fontsize=11)
        self.ax.legend(loc="best", fontsize=9)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlim(0, self.SIM_LENGTH)
        
        # Adjust layout and draw
        self.fig.tight_layout()
        self.canvas.draw()
    
    def export_plot(self):
        """Export current plot as PNG file"""
        try:
            filename = filedialog.asksaveasfilename(
                defaultextension=".png",
                filetypes=[("PNG files", "*.png"), ("All files", "*.*")],
                title="Save Plot As"
            )
            if filename:
                self.fig.savefig(filename, dpi=300, bbox_inches="tight")
                messagebox.showinfo("Export Successful", f"Plot saved to:\n{filename}")
        except Exception as e:
            messagebox.showerror("Export Error", f"Could not save plot:\n{str(e)}")
    
    def export_data(self):
        """Export simulation data as CSV file"""
        try:
            filename = filedialog.asksaveasfilename(
                defaultextension=".csv",
                filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
                title="Save Data As"
            )
            if filename:
                with open(filename, 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    # Header
                    writer.writerow([
                        "Time (s)", "SP", "PV", "CV", 
                        "P_Term", "I_Term", "D_Term",
                        "Kp", "Ti_sec", "Kd_sec",
                        "Process_Gain", "Time_Constant_sec", "Dead_Time_sec"
                    ])
                    # Data rows
                    for i in range(self.SIM_LENGTH):
                        writer.writerow([
                            i,
                            f"{self.SP[i]:.4f}",
                            f"{self.PV[i]:.4f}",
                            f"{self.CV[i]:.4f}",
                            f"{self.pterm[i]:.4f}",
                            f"{self.iterm[i]:.4f}",
                            f"{self.dterm[i]:.4f}",
                            f"{self.kp.get():.4f}",
                            f"{self.ti.get():.4f}",
                            f"{self.kd.get():.4f}",
                            f"{self.model_gain.get():.4f}",
                            f"{self.model_tc.get():.4f}",
                            f"{self.model_dt.get():.4f}"
                        ])
                messagebox.showinfo("Export Successful", f"Data saved to:\n{filename}")
        except Exception as e:
            messagebox.showerror("Export Error", f"Could not save \n{str(e)}")


if __name__ == "__main__":
    app = PID_Simulator()
    app.root.mainloop()