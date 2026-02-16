#!/usr/bin/env python3
"""
MAPF Competition Framework - GUI Launcher (FIXED VERSION)
Fixes:
1. Threading deadlocks
2. Missing feedback during execution
3. Process hanging issues
4. GUI freezing problems
"""

import os
import sys
import subprocess
import json
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
from pathlib import Path
import threading
import queue
import time
import signal


class MAPFLauncher:
    def __init__(self, root):
        self.root = root
        self.root.title("MAPF Competition Launcher")
        self.root.geometry("1200x700")
        
        # ✅ FIX 1: Track running processes to prevent orphans
        self.running_processes = []
        
        # ✅ FIX 2: Add abort flag for long operations
        self.abort_flag = threading.Event()
        
        # Initialize log queue for thread-safe logging
        self.log_queue = queue.Queue()
        
        # Get workspace directory
        self.workspace_dir = Path(__file__).parent.absolute()
        
        # PlanViz path
        self.planviz_dir = self.workspace_dir.parent / "PlanViz"
        
        # Configure default font sizes
        self.setup_fonts()
        
        # Setup GUI
        self.setup_gui()
        
        # Check requirements on startup
        self.root.after(100, self.check_requirements)
        
        # Start log queue checker
        self.root.after(100, self.check_log_queue)
        
        # ✅ FIX 3: Setup cleanup on window close
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def on_closing(self):
        """Cleanup before closing"""
        # Kill any running processes
        for process in self.running_processes:
            try:
                process.kill()
                process.wait(timeout=2)
            except:
                pass
        
        self.root.destroy()
    
    def setup_fonts(self):
        """Setup larger fonts for better readability"""
        import tkinter.font as tkfont
        
        # Get default font and increase size
        default_font = tkfont.nametofont("TkDefaultFont")
        default_font.configure(size=14)
        
        text_font = tkfont.nametofont("TkTextFont")
        text_font.configure(size=14)
        
        fixed_font = tkfont.nametofont("TkFixedFont")
        fixed_font.configure(size=13)
    
    def setup_gui(self):
        """Setup the GUI interface"""
        # Main frame
        main_frame = ttk.Frame(self.root, padding="15")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(4, weight=1)
        
        # Title
        title_label = ttk.Label(main_frame, text="MAPF Competition Launcher", 
                                font=('Arial', 24, 'bold'))
        title_label.grid(row=0, column=0, columnspan=2, pady=15)
        
        # Log output (create early so it's available for logging)
        ttk.Label(main_frame, text="Output Log:", font=('Arial', 14, 'bold')).grid(row=5, column=0, 
                                                        sticky=(tk.W, tk.N), pady=(10, 0))
        
        self.log_text = scrolledtext.ScrolledText(main_frame, height=18, width=95, font=('Courier', 12))
        self.log_text.grid(row=6, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S),
                          pady=(0, 5))
        
        main_frame.rowconfigure(6, weight=1)
        
        # Problem selection
        ttk.Label(main_frame, text="Select Problem:", font=('Arial', 14)).grid(row=1, column=0, 
                                                            sticky=tk.W, pady=8)
        
        self.problem_var = tk.StringVar()
        self.problem_combo = ttk.Combobox(main_frame, textvariable=self.problem_var,
                                          width=65, state='readonly', font=('Arial', 12))
        self.problem_combo.grid(row=1, column=1, sticky=(tk.W, tk.E), pady=8, padx=5)
        
        # Populate problems
        self.populate_problems()
        
        # Planner selection
        ttk.Label(main_frame, text="Select Planner:", font=('Arial', 14)).grid(row=2, column=0, 
                                                            sticky=tk.W, pady=8)
        
        self.planner_var = tk.StringVar(value="implemented4")
        self.planner_combo = ttk.Combobox(main_frame, textvariable=self.planner_var,
                                          values=["implemented4", "implemented3", "implemented2", "default"], state='readonly', font=('Arial', 12))
        self.planner_combo.grid(row=2, column=1, sticky=tk.W, pady=8, padx=5)
        
        # Max time selection
        ttk.Label(main_frame, text="Max Timesteps:", font=('Arial', 14)).grid(row=3, column=0,
                                                                sticky=tk.W, pady=8)
        
        self.time_var = tk.StringVar(value="5000")
        time_entry = ttk.Entry(main_frame, textvariable=self.time_var, width=20, font=('Arial', 12))
        time_entry.grid(row=3, column=1, sticky=tk.W, pady=8, padx=5)
        
        # Buttons frame
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=4, column=0, columnspan=2, pady=15)
        
        # Create style for larger buttons
        style = ttk.Style()
        style.configure('Large.TButton', font=('Arial', 13, 'bold'), padding=10)
        
        self.run_button = ttk.Button(button_frame, text="Compile & Run", 
                                      command=self.run_simulation, style='Large.TButton')
        self.run_button.grid(row=0, column=0, padx=8)
        
        # ✅ FIX 4: Add abort button
        self.abort_button = ttk.Button(button_frame, text="Abort", 
                                       command=self.abort_operation, style='Large.TButton',
                                       state='disabled')
        self.abort_button.grid(row=0, column=1, padx=8)
        
        self.visualize_button = ttk.Button(button_frame, text="Visualize Result",
                                           command=self.visualize_result, style='Large.TButton')
        self.visualize_button.grid(row=0, column=2, padx=8)
        
        self.check_req_button = ttk.Button(button_frame, text="Check Requirements",
                                           command=self.check_requirements, style='Large.TButton')
        self.check_req_button.grid(row=0, column=3, padx=8)
        
        self.compare_button = ttk.Button(button_frame, text="Compare Planners",
                                         command=self.compare_planners, style='Large.TButton')
        self.compare_button.grid(row=0, column=4, padx=8)
        
        ttk.Button(button_frame, text="About", 
                  command=self.show_about, style='Large.TButton').grid(row=0, column=5, padx=8)
        
        ttk.Button(button_frame, text="Exit", 
                  command=self.on_closing, style='Large.TButton').grid(row=0, column=6, padx=8)
        
        # ✅ FIX 5: Add progress bar
        self.progress_var = tk.DoubleVar()
        self.progress_bar = ttk.Progressbar(main_frame, mode='indeterminate', 
                                           variable=self.progress_var, length=400)
        self.progress_bar.grid(row=8, column=0, columnspan=2, pady=10, sticky=(tk.W, tk.E))
        
        # Previously generated output files for visualization
        ttk.Label(main_frame, text="Previous Output (for visualization):", font=('Arial', 14)).grid(row=7, column=0,
                                                                                 sticky=tk.W, pady=8)
        
        self.output_var = tk.StringVar()
        self.output_combo = ttk.Combobox(main_frame, textvariable=self.output_var,
                                         width=65, state='readonly', font=('Arial', 12))
        self.output_combo.grid(row=7, column=1, sticky=(tk.W, tk.E), pady=8, padx=5)
        
        # Populate output files
        self.populate_output_files()
        
        main_frame.rowconfigure(5, weight=1)
    
    def abort_operation(self):
        """Abort current operation"""
        self.abort_flag.set()
        self.log("⚠️  Abort requested by user...")
        
        # Kill all running processes
        for process in self.running_processes:
            try:
                if process.poll() is None:  # Process still running
                    self.log(f"Terminating process {process.pid}...")
                    process.terminate()
                    time.sleep(0.5)
                    if process.poll() is None:  # Still alive
                        process.kill()
            except Exception as e:
                self.log(f"Error terminating process: {e}")
        
        self.running_processes.clear()
        self.log("✓ Operation aborted")
    
    def start_progress(self):
        """Start progress bar animation"""
        self.progress_bar.start(10)
    
    def stop_progress(self):
        """Stop progress bar animation"""
        self.progress_bar.stop()
    
    def populate_problems(self):
        """Find all problem JSON files"""
        problems = []
        example_dir = self.workspace_dir / "example_problems"
        
        if example_dir.exists():
            for domain_dir in example_dir.iterdir():
                if domain_dir.is_dir():
                    for json_file in domain_dir.glob("*.json"):
                        # Make path relative to workspace
                        rel_path = json_file.relative_to(self.workspace_dir)
                        problems.append(str(rel_path))
        
        problems.sort()
        self.problem_combo['values'] = problems
        
        if problems:
            self.problem_combo.current(0)
        
        self.log(f"Found {len(problems)} problem files")
    
    def populate_output_files(self):
        """Find all output JSON files"""
        output_files = []
        
        # Find all JSON files in the workspace root that end with _output.json or contain 'output'
        for json_file in self.workspace_dir.glob("*output*.json"):
            if json_file.is_file():
                output_files.append(json_file.name)
        
        # Also check for test.json and other result files
        for json_file in self.workspace_dir.glob("*.json"):
            if json_file.is_file() and json_file.name not in output_files:
                # Check if it looks like an output file (has 'actualPaths' or similar)
                try:
                    with open(json_file, 'r') as f:
                        data = json.load(f)
                        if 'actualPaths' in data or 'plannerPaths' in data:
                            output_files.append(json_file.name)
                except:
                    pass
        
        output_files.sort(reverse=True)  # Most recent first
        self.output_combo['values'] = output_files
        
        if output_files:
            self.output_combo.current(0)
        
        self.log(f"Found {len(output_files)} output files for visualization")
    
    def log(self, message):
        """Add message to log queue (thread-safe)"""
        self.log_queue.put(message)
    
    def check_log_queue(self):
        """Check log queue and update GUI (called from main thread)"""
        try:
            while True:
                message = self.log_queue.get_nowait()
                self.log_text.insert(tk.END, message + "\n")
                self.log_text.see(tk.END)
                # ✅ FIX 6: Force GUI update
                self.root.update_idletasks()
        except queue.Empty:
            pass
        self.root.after(100, self.check_log_queue)
    
    def show_about(self):
        """Show about dialog with creators information"""
        about_window = tk.Toplevel(self.root)
        about_window.title("About")
        about_window.geometry("500x350")
        about_window.resizable(False, False)
        
        # Center the window
        about_window.transient(self.root)
        about_window.grab_set()
        
        # Main frame
        frame = ttk.Frame(about_window, padding="20")
        frame.pack(fill=tk.BOTH, expand=True)
        
        # Title
        title_label = ttk.Label(frame, text="MAPF Competition Launcher", 
                               font=('Arial', 18, 'bold'))
        title_label.pack(pady=(0, 10))
        
        # Version
        version_label = ttk.Label(frame, text="Version 1.3.0", 
                                 font=('Arial', 12))
        version_label.pack(pady=(0, 20))
        
        # Creators section
        creators_label = ttk.Label(frame, text="Created by:", 
                                  font=('Arial', 14, 'bold'))
        creators_label.pack(pady=(0, 10))
        
        # Creators names
        creators_text = tk.Text(frame, height=5, width=40, font=('Arial', 13),
                               relief=tk.FLAT, bg=about_window.cget('bg'))
        creators_text.pack(pady=(0, 20))
        
        creators_content = """Thomas Causetti
Gabriele Ceresara
Jacopo Tedeschi"""
        
        creators_text.insert('1.0', creators_content)
        creators_text.tag_configure("center", justify='center')
        creators_text.tag_add("center", "1.0", "end")
        creators_text.config(state='disabled')
        
        # Project info
        project_label = ttk.Label(frame, text="MAPF Competition Launcher", 
                                 font=('Arial', 11, 'italic'))
        project_label.pack(pady=(0, 20))
        
        # Close button
        close_button = ttk.Button(frame, text="Close", 
                                 command=about_window.destroy,
                                 style='Large.TButton')
        close_button.pack()
        
        # Center the about window on the main window
        about_window.update_idletasks()
        x = self.root.winfo_x() + (self.root.winfo_width() // 2) - (about_window.winfo_width() // 2)
        y = self.root.winfo_y() + (self.root.winfo_height() // 2) - (about_window.winfo_height() // 2)
        about_window.geometry(f"+{x}+{y}")
    
    def check_requirements(self):
        """Check and install system requirements"""
        self.log("\n=== Checking Requirements ===")
        
        # Check for apt packages
        apt_packages = [
            'cmake',
            'build-essential',
            'libboost-all-dev',
            'python3-dev',
            'python3-pip'
        ]
        
        missing_apt = []
        for package in apt_packages:
            if not self.check_apt_package(package):
                missing_apt.append(package)
        
        if missing_apt:
            self.log(f"Missing apt packages: {', '.join(missing_apt)}")
            response = messagebox.askyesno(
                "Missing Packages",
                f"The following packages are missing:\n{', '.join(missing_apt)}\n\n"
                "Do you want to install them? (requires sudo)"
            )
            
            if response:
                self.install_apt_packages(missing_apt)
            else:
                self.log("Skipping apt package installation")
        else:
            self.log("✓ All apt packages are installed")
        
        # Check Python packages
        python_packages = ['numpy', 'pybind11']
        missing_python = []
        
        for package in python_packages:
            try:
                __import__(package)
                self.log(f"✓ Python package '{package}' is installed")
            except ImportError:
                missing_python.append(package)
                self.log(f"✗ Python package '{package}' is missing")
        
        if missing_python:
            response = messagebox.askyesno(
                "Missing Python Packages",
                f"The following Python packages are missing:\n{', '.join(missing_python)}\n\n"
                "Do you want to install them?"
            )
            
            if response:
                self.install_python_packages(missing_python)
            else:
                self.log("Skipping Python package installation")
        else:
            self.log("✓ All Python packages are installed")
        
        # Check if compiled binaries exist
        implemented_binary = self.workspace_dir / "build" / "lifelong_implemented4"
        default_binary = self.workspace_dir / "build" / "lifelong_default"
        if implemented_binary.exists() and default_binary.exists():
            self.log("✓ Compiled binaries found")
        else:
            self.log("✗ Compiled binaries not found - will need to compile")
        
        # Check for PlanViz
        self.check_planviz()
        
        self.log("=== Requirements Check Complete ===\n")
    
    def check_apt_package(self, package):
        """Check if an apt package is installed"""
        try:
            result = subprocess.run(
                ['dpkg', '-s', package],
                capture_output=True,
                text=True,
                timeout=5
            )
            return result.returncode == 0
        except:
            return False
    
    def install_apt_packages(self, packages):
        """Install apt packages"""
        self.log(f"Installing apt packages: {', '.join(packages)}")
        
        try:
            cmd = ['sudo', 'apt-get', 'install', '-y'] + packages
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1
            )
            
            # ✅ FIX 7: Stream output in real-time
            for line in iter(process.stdout.readline, ''):
                if line:
                    self.log(line.rstrip())
                # ✅ FIX 8: Update GUI during long operations
                self.root.update_idletasks()
            
            process.wait()
            
            if process.returncode == 0:
                self.log("✓ Apt packages installed successfully")
            else:
                self.log("✗ Failed to install apt packages")
        except Exception as e:
            self.log(f"✗ Error installing apt packages: {e}")
    
    def install_python_packages(self, packages):
        """Install Python packages"""
        self.log(f"Installing Python packages: {', '.join(packages)}")
        
        try:
            cmd = [sys.executable, '-m', 'pip', 'install'] + packages
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1
            )
            
            # ✅ FIX 9: Stream output in real-time
            for line in iter(process.stdout.readline, ''):
                if line:
                    self.log(line.rstrip())
                self.root.update_idletasks()
            
            process.wait()
            
            if process.returncode == 0:
                self.log("✓ Python packages installed successfully")
            else:
                self.log("✗ Failed to install Python packages")
        except Exception as e:
            self.log(f"✗ Error installing Python packages: {e}")
    
    def check_planviz(self):
        """Check if PlanViz is installed"""
        if self.planviz_dir.exists():
            run_script = self.planviz_dir / "script" / "run.py"
            if run_script.exists():
                self.log("✓ PlanViz is installed")
                return True
            else:
                self.log("✗ PlanViz directory found but run.py is missing")
                return False
        else:
            self.log("✗ PlanViz not found")
            response = messagebox.askyesno(
                "PlanViz Not Found",
                "PlanViz visualization tool is not installed.\n\n"
                "Do you want to install it?"
            )
            
            if response:
                self.install_planviz()
            return False
    
    def install_planviz(self):
        """Install PlanViz"""
        self.log("\n=== Installing PlanViz ===")
        
        try:
            # Clone PlanViz
            parent_dir = self.workspace_dir.parent
            self.log(f"Cloning PlanViz to {parent_dir}")
            
            process = subprocess.Popen(
                ['git', 'clone', 'https://github.com/MAPF-Competition/PlanViz.git'],
                cwd=parent_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1
            )
            
            # ✅ FIX 10: Stream output
            for line in iter(process.stdout.readline, ''):
                if line:
                    self.log(line.rstrip())
                self.root.update_idletasks()
            
            process.wait()
            
            if process.returncode == 0:
                self.log("✓ PlanViz cloned successfully")
                
                # Install PlanViz dependencies
                self.log("Installing PlanViz dependencies...")
                apt_packages = [
                    'python3-matplotlib',
                    'python3-numpy',
                    'python3-pandas',
                    'python3-pil',
                    'python3-scipy',
                    'python3-tk'
                ]
                
                self.install_apt_packages(apt_packages)
                self.log("✓ PlanViz installed successfully")
            else:
                self.log("✗ Failed to clone PlanViz")
        except Exception as e:
            self.log(f"✗ Error installing PlanViz: {e}")
    
    def compile_code(self):
        """Compile the C++ code"""
        self.log("\n=== Compiling Code ===")
        
        compile_script = self.workspace_dir / "compile.sh"
        
        if not compile_script.exists():
            self.log("✗ compile.sh not found")
            return False
        
        try:
            # Make sure compile.sh is executable
            os.chmod(compile_script, 0o755)
            
            process = subprocess.Popen(
                ['./compile.sh'],
                cwd=self.workspace_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1
            )
            
            self.running_processes.append(process)
            
            # ✅ FIX 11: Stream output and check for abort
            for line in iter(process.stdout.readline, ''):
                if self.abort_flag.is_set():
                    process.kill()
                    self.log("✗ Compilation aborted by user")
                    return False
                
                if line:
                    self.log(line.rstrip())
                self.root.update_idletasks()
            
            process.wait()
            self.running_processes.remove(process)
            
            if process.returncode == 0:
                self.log("✓ Compilation successful")
                return True
            else:
                self.log("✗ Compilation failed")
                return False
        except Exception as e:
            self.log(f"✗ Error during compilation: {e}")
            return False
    
    def run_simulation(self):
        """Run the simulation in a separate thread"""
        # ✅ FIX 12: Clear abort flag
        self.abort_flag.clear()
        thread = threading.Thread(target=self._run_simulation_thread, daemon=True)
        thread.start()
    
    def compare_planners(self):
        """Compare both planners by running them sequentially"""
        # ✅ FIX 13: Clear abort flag
        self.abort_flag.clear()
        thread = threading.Thread(target=self._compare_planners_thread, daemon=True)
        thread.start()
    
    def _compare_planners_thread(self):
        """Run both planners and compare their results"""
        # Disable buttons and start progress
        self.root.after(0, lambda: self.run_button.config(state='disabled'))
        self.root.after(0, lambda: self.compare_button.config(state='disabled'))
        self.root.after(0, lambda: self.abort_button.config(state='normal'))
        self.root.after(0, self.start_progress)
        
        try:
            problem_file = self.problem_var.get()
            if not problem_file:
                self.root.after(0, lambda: messagebox.showerror("Error", "Please select a problem file"))
                return
            
            try:
                max_time = int(self.time_var.get())
                if max_time <= 0:
                    raise ValueError()
            except ValueError:
                self.root.after(0, lambda: messagebox.showerror("Error", "Please enter a valid positive integer for max time"))
                return
            
            self.log(f"\n{'='*60}")
            self.log("COMPARING PLANNERS")
            self.log(f"{'='*60}")
            self.log(f"Problem: {problem_file}")
            self.log(f"Max Timesteps: {max_time}")
            self.log("")
            
            results = {}
            
            # Run both planners
            for planner in ["implemented4", "default"]:
                if self.abort_flag.is_set():
                    self.log("Comparison aborted by user")
                    break
                
                self.log(f"\n{'='*40}")
                self.log(f"Running {planner.upper()} planner...")
                self.log(f"{'='*40}")
                
                # Check if binary exists
                binary_path = self.workspace_dir / "build" / f"lifelong_{planner}"
                if not binary_path.exists():
                    self.log(f"✗ {planner} binary not found - skipping")
                    continue
                
                # Prepare output filename
                problem_name = Path(problem_file).stem
                output_file = self.workspace_dir / f"{problem_name}_{planner}_comparison.json"
                
                # Run the simulation
                cmd = [
                    str(binary_path),
                    '--inputFile', str(self.workspace_dir / problem_file),
                    '-o', str(output_file),
                    '--simulationTime', str(max_time)
                ]
                
                try:
                    start_time = time.time()
                    process = subprocess.Popen(
                        cmd,
                        cwd=self.workspace_dir,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,
                        text=True,
                        bufsize=1
                    )
                    
                    self.running_processes.append(process)
                    
                    # ✅ FIX 14: Stream output in real-time
                    for line in iter(process.stdout.readline, ''):
                        if self.abort_flag.is_set():
                            process.kill()
                            self.log(f"✗ {planner} aborted by user")
                            break
                        
                        if line:
                            self.log(line.rstrip())
                        self.root.update_idletasks()
                    
                    process.wait()
                    runtime = time.time() - start_time
                    returncode = process.returncode
                    
                    self.running_processes.remove(process)
                    
                    if returncode == 0 and output_file.exists():
                        # Parse results
                        metrics = self._parse_simulation_results(output_file, runtime)
                        if metrics:
                            results[planner] = metrics
                            self.log(f"✓ {planner} completed successfully in {runtime:.2f}s")
                        else:
                            self.log(f"✗ {planner} failed to parse results")
                    else:
                        self.log(f"✗ {planner} failed with exit code {returncode}")
                        
                except Exception as e:
                    self.log(f"✗ Error running {planner}: {e}")
            
            # Compare results
            if not self.abort_flag.is_set():
                if len(results) >= 2:
                    self._display_comparison(results, max_time)
                elif len(results) == 1:
                    planner_name = list(results.keys())[0]
                    self.log(f"\nOnly {planner_name} planner completed successfully.")
                    self._display_single_result(results[planner_name], planner_name)
                else:
                    self.log("\n✗ No planners completed successfully.")
                    
            # Refresh output files list
            self.root.after(0, self.populate_output_files)
                
        except Exception as e:
            self.log(f"✗ Error during comparison: {e}")
            import traceback
            self.log(traceback.format_exc())
        finally:
            # Re-enable buttons and stop progress
            self.root.after(0, lambda: self.run_button.config(state='normal'))
            self.root.after(0, lambda: self.compare_button.config(state='normal'))
            self.root.after(0, lambda: self.abort_button.config(state='disabled'))
            self.root.after(0, self.stop_progress)
    
    def _parse_simulation_results(self, output_file, runtime=None):
        """Parse simulation results from output file"""
        try:
            with open(output_file, 'r') as f:
                data = json.load(f)
            
            paths = data.get('actualPaths', [])
            if not paths:
                return None
            
            num_agents = len(paths)
            makespan = max(len(path) for path in paths) if paths else 0
            total_distance = sum(len(path) for path in paths) if paths else 0
            num_tasks_finished = data.get('numTaskFinished', 0)
            
            return {
                'num_agents': num_agents,
                'makespan': makespan,
                'total_distance': total_distance,
                'num_tasks_finished': num_tasks_finished,
                'runtime': runtime,
                'output_file': output_file.name
            }
        except Exception as e:
            self.log(f"Error parsing {output_file}: {e}")
            return None
    
    def _display_comparison(self, results, max_time):
        """Display comparison of planner results"""
        self.log(f"\n{'='*60}")
        self.log("PLANNER COMPARISON RESULTS")
        self.log(f"{'='*60}")
        
        # Extract data
        implemented = results.get('implemented4', {})
        default_planner = results.get('default', {})
        
        if not implemented or not default_planner:
            self.log("Missing results for comparison")
            return
        
        # Display individual results
        self.log("\nIMPLEMENTED4 PLANNER:")
        self.log(f"  Agents: {implemented['num_agents']}")
        self.log(f"  Tasks Finished: {implemented['num_tasks_finished']}")
        self.log(f"  Makespan: {implemented['makespan']}")
        self.log(f"  Total Distance: {implemented['total_distance']}")
        if implemented.get('runtime') is not None:
            self.log(f"  Runtime: {implemented['runtime']:.2f} seconds")
        
        self.log("\nDEFAULT PLANNER:")
        self.log(f"  Agents: {default_planner['num_agents']}")
        self.log(f"  Tasks Finished: {default_planner['num_tasks_finished']}")
        self.log(f"  Makespan: {default_planner['makespan']}")
        self.log(f"  Total Distance: {default_planner['total_distance']}")
        if default_planner.get('runtime') is not None:
            self.log(f"  Runtime: {default_planner['runtime']:.2f} seconds")
        
        # Calculate winner
        self.log(f"\n{'='*40}")
        self.log("COMPARISON METRICS")
        self.log(f"{'='*40}")
        
        implemented_tasks = implemented['num_tasks_finished']
        default_tasks = default_planner['num_tasks_finished']
        
        self.log(f"\nScoring (both ran for {max_time} timesteps):")
        self.log(f"  Implemented4: {implemented_tasks} tasks in {implemented.get('runtime', 0):.2f}s")
        self.log(f"  Default: {default_tasks} tasks in {default_planner.get('runtime', 0):.2f}s")
        
        if implemented_tasks > default_tasks:
            winner = "IMPLEMENTED4"
            margin = ((implemented_tasks - default_tasks) / max(default_tasks, 1)) * 100
        elif default_tasks > implemented_tasks:
            winner = "DEFAULT"
            margin = ((default_tasks - implemented_tasks) / max(implemented_tasks, 1)) * 100
        else:
            winner = "TIE"
            margin = 0
        
        self.log(f"\n🏆 WINNER: {winner}")
        if winner != "TIE":
            self.log(f"   Margin: {margin:.1f}% more tasks completed")
        
        self.log(f"\n{'='*60}")
    
    def _display_single_result(self, result, planner_name):
        """Display results for a single planner"""
        self.log(f"\n{planner_name.upper()} PLANNER RESULTS:")
        self.log(f"  Agents: {result['num_agents']}")
        self.log(f"  Tasks Finished: {result['num_tasks_finished']}")
        self.log(f"  Makespan: {result['makespan']}")
        self.log(f"  Total Distance: {result['total_distance']}")
        if result.get('runtime') is not None:
            self.log(f"  Runtime: {result['runtime']:.2f} seconds")
    
    def _run_simulation_thread(self):
        """Run the simulation (called in separate thread)"""
        # Disable buttons and start progress
        self.root.after(0, lambda: self.run_button.config(state='disabled'))
        self.root.after(0, lambda: self.abort_button.config(state='normal'))
        self.root.after(0, self.start_progress)
        
        try:
            problem_file = self.problem_var.get()
            if not problem_file:
                self.root.after(0, lambda: messagebox.showerror("Error", "Please select a problem file"))
                return
            
            try:
                max_time = int(self.time_var.get())
                if max_time <= 0:
                    raise ValueError()
            except ValueError:
                self.root.after(0, lambda: messagebox.showerror("Error", "Please enter a valid positive integer for max time"))
                return
            
            planner = self.planner_var.get()
            
            # Check if binary exists
            binary_path = self.workspace_dir / "build" / f"lifelong_{planner}"
            
            if not binary_path.exists():
                response = [False]
                
                def ask_compile():
                    response[0] = messagebox.askyesno(
                        "Compilation Required",
                        "The binary doesn't exist. Do you want to compile now?"
                    )
                
                self.root.after(0, ask_compile)
                time.sleep(0.5)  # Give time for dialog
                
                if response[0]:
                    if not self.compile_code():
                        self.root.after(0, lambda: messagebox.showerror("Error", "Compilation failed. Check the log for details."))
                        return
                else:
                    return
            
            # Prepare output filename
            problem_name = Path(problem_file).stem
            output_file = self.workspace_dir / f"{problem_name}_{planner}_output.json"
            
            self.log(f"\n=== Running Simulation ===")
            self.log(f"Problem: {problem_file}")
            self.log(f"Planner: {planner}")
            self.log(f"Max Timesteps: {max_time}")
            self.log(f"Output: {output_file.name}")
            
            # Store mapping
            mapping_file = self.workspace_dir / ".output_mapping.json"
            try:
                if mapping_file.exists():
                    with open(mapping_file, 'r') as f:
                        mappings = json.load(f)
                else:
                    mappings = {}
                
                mappings[output_file.name] = problem_file
                
                with open(mapping_file, 'w') as f:
                    json.dump(mappings, f, indent=2)
            except:
                pass
            
            # Run the simulation
            cmd = [
                str(binary_path),
                '--inputFile', str(self.workspace_dir / problem_file),
                '-o', str(output_file),
                '--simulationTime', str(max_time)
            ]
            
            self.log(f"Command: {' '.join(cmd)}\n")
            
            start_time = time.time()
            process = subprocess.Popen(
                cmd,
                cwd=self.workspace_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1
            )
            
            self.running_processes.append(process)
            
            try:
                # ✅ FIX 15: Stream output in real-time with abort check
                for line in iter(process.stdout.readline, ''):
                    if self.abort_flag.is_set():
                        process.kill()
                        self.log("✗ Simulation aborted by user")
                        return
                    
                    if line:
                        self.log(line.rstrip())
                    self.root.update_idletasks()
                
                process.wait()
                runtime = time.time() - start_time
                returncode = process.returncode
                
                self.running_processes.remove(process)
                
            except Exception as e:
                process.kill()
                runtime = time.time() - start_time
                self.log(f"Process error: {e}")
                returncode = -1
            
            if (returncode == 0 or returncode == -1) and output_file.exists():
                if returncode == 0:
                    self.log("\n✓ Simulation completed successfully")
                else:
                    self.log("\n⚠️  Simulation encountered errors but produced output")
                self.log(f"Output saved to: {output_file.name}")
                
                # Refresh output files list
                self.root.after(0, self.populate_output_files)
                
                # Select the newly created file
                def select_output():
                    if output_file.name in self.output_combo['values']:
                        self.output_combo.set(output_file.name)
                
                self.root.after(0, select_output)
                
                self.show_summary(output_file, runtime)
                
                # Ask if user wants to visualize (must be done in main thread)
                def ask_and_visualize():
                    response = messagebox.askyesno(
                        "Visualize Result?",
                        "Do you want to visualize the result with PlanViz?"
                    )
                    if response:
                        self.output_var.set(output_file.name)
                        self.visualize_result()
                
                self.root.after(0, ask_and_visualize)
            else:
                self.log("\n✗ Simulation failed")
                self.root.after(0, lambda: messagebox.showerror("Error", "Simulation failed. Check the log for details."))
        
        except Exception as e:
            self.log(f"✗ Error in simulation thread: {e}")
            import traceback
            self.log(traceback.format_exc())
        
        finally:
            # Re-enable buttons and stop progress
            self.root.after(0, lambda: self.run_button.config(state='normal'))
            self.root.after(0, lambda: self.abort_button.config(state='disabled'))
            self.root.after(0, self.stop_progress)
    
    def show_summary(self, output_file, runtime=None):
        """Show a summary of the simulation results"""
        self.log("\n=== Simulation Summary ===")
        
        try:
            with open(output_file, 'r') as f:
                data = json.load(f)
            
            paths = data.get('actualPaths', [])
            num_agents = len(paths)
            num_tasks_finished = data.get('numTaskFinished', 0)
            
            if paths:
                makespan = max(len(path) - 1 for path in paths)
                total_distance = sum(len(path) - 1 for path in paths)
            else:
                makespan = 0
                total_distance = 0
            
            self.log(f"Number of agents: {num_agents}")
            self.log(f"Tasks finished: {num_tasks_finished}")
            self.log(f"Makespan: {makespan}")
            self.log(f"Total distance: {total_distance}")
            if runtime is not None:
                self.log(f"Runtime: {runtime:.2f} seconds")
            
        except Exception as e:
            self.log(f"Could not generate summary: {e}")
    
    def visualize_result(self):
        """Visualize a result file using PlanViz"""
        output_file = self.output_var.get()
        
        if not output_file:
            messagebox.showerror("Error", "Please select an output file to visualize")
            return
        
        # Check if PlanViz is installed
        if not self.planviz_dir.exists():
            response = messagebox.askyesno(
                "PlanViz Not Found",
                "PlanViz is not installed. Do you want to install it now?"
            )
            if response:
                self.install_planviz()
                if not self.planviz_dir.exists():
                    return
            else:
                return
        
        # Read the output file to get map information
        output_path = self.workspace_dir / output_file
        
        try:
            with open(output_path, 'r') as f:
                data = json.load(f)
            
            map_file = data.get('mapFile', '')
            
            # If mapFile is not in output JSON, try to find it
            if not map_file:
                mapping_file = self.workspace_dir / ".output_mapping.json"
                input_file_path = None
                
                if mapping_file.exists():
                    try:
                        with open(mapping_file, 'r') as f:
                            mappings = json.load(f)
                            input_file_rel = mappings.get(output_file, '')
                            if input_file_rel:
                                input_file_path = self.workspace_dir / input_file_rel
                    except:
                        pass
                
                # Read map from input file
                if input_file_path and input_file_path.exists():
                    try:
                        with open(input_file_path, 'r') as f:
                            input_data = json.load(f)
                            map_file = input_data.get('mapFile', '')
                            if map_file:
                                map_file = str(input_file_path.parent / map_file)
                    except:
                        pass
                
                if not map_file:
                    self.log("✗ Map file not found in output JSON or input file")
                    messagebox.showerror(
                        "Map File Not Found",
                        f"Could not find the map file for {output_file}."
                    )
                    return
            
            # Map file path
            if not Path(map_file).is_absolute():
                map_path = self.workspace_dir / map_file
            else:
                map_path = Path(map_file)
            
            if not map_path.exists():
                messagebox.showerror("Error", f"Map file not found: {map_file}")
                return
            
            self.log(f"\n=== Launching PlanViz ===")
            self.log(f"Map: {map_file}")
            self.log(f"Plan: {output_file}")
            
            # Run PlanViz
            run_script = self.planviz_dir / "script" / "run.py"
            
            cmd = [
                sys.executable,
                str(run_script),
                '--map', str(map_path),
                '--plan', str(output_path)
            ]
            
            self.log(f"Command: {' '.join(cmd)}\n")
            
            # Run in background (don't wait)
            process = subprocess.Popen(
                cmd,
                cwd=self.planviz_dir,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            
            self.log("✓ PlanViz launched successfully")
            self.log("Close the visualization window when done.\n")
            
        except json.JSONDecodeError:
            messagebox.showerror("Error", f"Invalid JSON file: {output_file}")
        except Exception as e:
            self.log(f"✗ Error launching visualization: {e}")
            import traceback
            self.log(traceback.format_exc())
            messagebox.showerror("Error", f"Failed to launch visualization:\n{e}")


def main():
    """Main entry point"""
    root = tk.Tk()
    app = MAPFLauncher(root)
    root.mainloop()


if __name__ == "__main__":
    main()