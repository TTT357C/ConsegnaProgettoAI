#!/usr/bin/env python3
"""
MAPF Competition Framework - Modern GUI with PySide6
Enhanced UI with comparison graphs
"""

import sys
import os
import json
import subprocess
import time
import threading
import queue
from pathlib import Path
import traceback

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QComboBox, QTextEdit, QLineEdit,
    QGroupBox, QGridLayout, QProgressBar, QMessageBox, QTabWidget,
    QSplitter, QFrame
)
from PySide6.QtCore import Qt, QTimer, Signal, QObject, QMetaObject, Q_ARG, Slot
from PySide6.QtGui import QFont, QPalette, QColor, QIcon

# Import matplotlib for graphs
import matplotlib
matplotlib.use('QtAgg')
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt


class WorkerSignals(QObject):
    """Signals for worker threads"""
    log = Signal(str)
    finished = Signal()
    error = Signal(str)
    result = Signal(dict)


class MAPFLauncher(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # Track running processes
        self.running_processes = []
        self.abort_flag = threading.Event()
        
        # Get workspace directory
        self.workspace_dir = Path(__file__).parent.absolute()
        self.planviz_dir = self.workspace_dir.parent / "PlanViz"
        
        # Initialize UI
        self.init_ui()
        
        # Start requirements check
        QTimer.singleShot(100, self.check_requirements)
    
    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle("MAPF Competition Launcher")
        self.setGeometry(100, 100, 1400, 900)
        
        # Set modern dark theme
        self.set_dark_theme()
        
        # Create central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(15)
        main_layout.setContentsMargins(20, 20, 20, 20)
        
        # Title
        title_label = QLabel("MAPF Competition Launcher")
        title_font = QFont("Arial", 50, QFont.Bold)
        title_label.setFont(title_font)
        title_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title_label)
        
        # Create tabs
        self.tabs = QTabWidget()
        self.tabs.setFont(QFont("Arial", 22))
        main_layout.addWidget(self.tabs)
        
        # Tab 1: Main Controls
        self.main_tab = QWidget()
        self.setup_main_tab()
        self.tabs.addTab(self.main_tab, "Main")
        
        # Tab 2: Comparison Results
        self.comparison_tab = QWidget()
        self.setup_comparison_tab()
        self.tabs.addTab(self.comparison_tab, "Comparison Results")
        
        # Status bar
        self.statusBar().showMessage("Ready")
        self.statusBar().setFont(QFont("Arial", 20))
    
    def set_dark_theme(self):
        """Apply modern dark theme"""
        palette = QPalette()
        palette.setColor(QPalette.Window, QColor(53, 53, 53))
        palette.setColor(QPalette.WindowText, Qt.white)
        palette.setColor(QPalette.Base, QColor(35, 35, 35))
        palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
        palette.setColor(QPalette.ToolTipBase, Qt.white)
        palette.setColor(QPalette.ToolTipText, Qt.white)
        palette.setColor(QPalette.Text, Qt.white)
        palette.setColor(QPalette.Button, QColor(53, 53, 53))
        palette.setColor(QPalette.ButtonText, Qt.white)
        palette.setColor(QPalette.BrightText, Qt.red)
        palette.setColor(QPalette.Link, QColor(42, 130, 218))
        palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
        palette.setColor(QPalette.HighlightedText, Qt.black)
        
        self.setPalette(palette)
        
        # Set stylesheet for additional customization
        self.setStyleSheet("""
            QMainWindow {
                background-color: #353535;
            }
            QPushButton {
                background-color: #0078d4;
                border: none;
                color: white;
                padding: 18px 36px;
                font-size: 23px;
                font-weight: bold;
                border-radius: 5px;
                min-width: 220px;
            }
            QPushButton:hover {
                background-color: #1084d8;
            }
            QPushButton:pressed {
                background-color: #006cbe;
            }
            QPushButton:disabled {
                background-color: #555555;
                color: #888888;
            }
            QComboBox {
                background-color: #404040;
                border: 1px solid #555555;
                color: white;
                padding: 14px;
                font-size: 22px;
                border-radius: 4px;
            }
            QComboBox:hover {
                border: 1px solid #0078d4;
            }
            QComboBox::drop-down {
                border: none;
            }
            QLineEdit {
                background-color: #404040;
                border: 1px solid #555555;
                color: white;
                padding: 14px;
                font-size: 22px;
                border-radius: 4px;
            }
            QLineEdit:focus {
                border: 1px solid #0078d4;
            }
            QTextEdit {
                background-color: #2b2b2b;
                border: 1px solid #555555;
                color: #e0e0e0;
                font-family: 'Courier New';
                font-size: 20px;
                border-radius: 4px;
            }
            QGroupBox {
                border: 2px solid #555555;
                border-radius: 8px;
                margin-top: 22px;
                font-size: 23px;
                font-weight: bold;
                padding-top: 18px;
            }
            QLabel {
                font-size: 20px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
            QTabWidget::pane {
                border: 1px solid #555555;
                border-radius: 4px;
            }
            QTabBar::tab {
                background-color: #404040;
                color: white;
                padding: 18px 36px;
                margin-right: 2px;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
                font-size: 22px;
            }
            QTabBar::tab:selected {
                background-color: #0078d4;
            }
            QTabBar::tab:hover {
                background-color: #505050;
            }
            QProgressBar {
                border: 1px solid #555555;
                border-radius: 4px;
                text-align: center;
                color: white;
                background-color: #404040;
            }
            QProgressBar::chunk {
                background-color: #0078d4;
                border-radius: 3px;
            }
        """)
    
    def setup_main_tab(self):
        """Setup main control tab"""
        layout = QVBoxLayout(self.main_tab)
        layout.setSpacing(15)
        
        # Create horizontal splitter for side-by-side layout
        splitter = QSplitter(Qt.Horizontal)
        
        # Left side: Configuration and controls
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setSpacing(15)
        
        # Configuration group
        config_group = QGroupBox("Configuration")
        config_layout = QGridLayout()
        config_layout.setSpacing(10)
        
        # Problem selection
        config_layout.addWidget(QLabel("Problem:"), 0, 0)
        self.problem_combo = QComboBox()
        self.problem_combo.setMinimumWidth(400)
        config_layout.addWidget(self.problem_combo, 0, 1)
        
        # Planner selection
        config_layout.addWidget(QLabel("Planner:"), 1, 0)
        self.planner_combo = QComboBox()
        self.planner_combo.addItems(["implemented4", "implemented3", "implemented2", "default"])
        config_layout.addWidget(self.planner_combo, 1, 1)
        
        # Max timesteps
        config_layout.addWidget(QLabel("Max Timesteps:"), 2, 0)
        self.time_input = QLineEdit("5000")
        config_layout.addWidget(self.time_input, 2, 1)
        
        # Output file selection
        config_layout.addWidget(QLabel("Previous Output:"), 3, 0)
        self.output_combo = QComboBox()
        self.output_combo.setMinimumWidth(400)
        config_layout.addWidget(self.output_combo, 3, 1)
        
        config_group.setLayout(config_layout)
        left_layout.addWidget(config_group)
        
        # Buttons - 3 rows of 2 buttons
        button_layout = QGridLayout()
        button_layout.setSpacing(10)
        
        # Row 1
        self.compile_run_btn = QPushButton("Compile & Run")
        self.compile_run_btn.clicked.connect(self.run_simulation)
        button_layout.addWidget(self.compile_run_btn, 0, 0)
        
        self.abort_btn = QPushButton("Abort")
        self.abort_btn.clicked.connect(self.abort_operation)
        self.abort_btn.setEnabled(False)
        self.abort_btn.setStyleSheet("""
            QPushButton {
                background-color: #c42b1c;
            }
            QPushButton:hover {
                background-color: #d63527;
            }
        """)
        button_layout.addWidget(self.abort_btn, 0, 1)
        
        # Row 2
        self.visualize_btn = QPushButton("Visualize Result")
        self.visualize_btn.clicked.connect(self.visualize_result)
        button_layout.addWidget(self.visualize_btn, 1, 0)
        
        self.compare_btn = QPushButton("Compare Planners")
        self.compare_btn.clicked.connect(self.compare_planners)
        button_layout.addWidget(self.compare_btn, 1, 1)
        
        # Row 3
        self.check_req_btn = QPushButton("Check Requirements")
        self.check_req_btn.clicked.connect(self.check_requirements)
        button_layout.addWidget(self.check_req_btn, 2, 0)
        
        self.about_btn = QPushButton("About")
        self.about_btn.clicked.connect(self.show_about)
        button_layout.addWidget(self.about_btn, 2, 1)
        
        left_layout.addLayout(button_layout)
        
        # Progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setMaximum(0)  # Indeterminate mode
        self.progress_bar.setVisible(False)
        left_layout.addWidget(self.progress_bar)
        
        left_layout.addStretch()
        
        # Right side: Log output
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setSpacing(10)
        
        # Log output
        log_group = QGroupBox("Output Log")
        log_layout = QVBoxLayout()
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        
        log_group.setLayout(log_layout)
        right_layout.addWidget(log_group)
        
        # Add left and right widgets to splitter
        splitter.addWidget(left_widget)
        splitter.addWidget(right_widget)
        splitter.setStretchFactor(0, 1)  # Left side
        splitter.setStretchFactor(1, 1)  # Right side - 50/50 split
        
        layout.addWidget(splitter)
        
        # Populate combos
        self.populate_problems()
        self.populate_output_files()
    
    def setup_comparison_tab(self):
        """Setup comparison results tab with graphs"""
        layout = QVBoxLayout(self.comparison_tab)
        
        # Title for comparison
        self.comparison_title = QLabel("Run a comparison to see results")
        self.comparison_title.setFont(QFont("Arial", 29, QFont.Bold))
        self.comparison_title.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.comparison_title)
        
        # Create matplotlib figure with 2 subplots
        self.comparison_figure = Figure(figsize=(16, 6))
        self.comparison_figure.patch.set_facecolor('#353535')
        self.comparison_canvas = FigureCanvas(self.comparison_figure)
        layout.addWidget(self.comparison_canvas)
        
        # Create 1x2 grid of subplots
        self.ax1 = self.comparison_figure.add_subplot(1, 2, 1)
        self.ax2 = self.comparison_figure.add_subplot(1, 2, 2)
        
        # Style all axes
        for ax in [self.ax1, self.ax2]:
            ax.set_facecolor('#2b2b2b')
            ax.tick_params(colors='white', which='both')
            ax.spines['bottom'].set_color('white')
            ax.spines['top'].set_color('white')
            ax.spines['left'].set_color('white')
            ax.spines['right'].set_color('white')
            ax.xaxis.label.set_color('white')
            ax.yaxis.label.set_color('white')
            ax.title.set_color('white')
        
        self.comparison_figure.tight_layout(pad=4.0, h_pad=3.0, w_pad=3.0)
    
    def populate_problems(self):
        """Find all problem JSON files"""
        problems = []
        example_dir = self.workspace_dir / "example_problems"
        
        if example_dir.exists():
            for domain_dir in example_dir.iterdir():
                if domain_dir.is_dir():
                    for problem_file in domain_dir.glob("*.json"):
                        rel_path = problem_file.relative_to(self.workspace_dir)
                        problems.append(str(rel_path))
        
        problems.sort()
        self.problem_combo.addItems(problems)
        self.log(f"Found {len(problems)} problem files")
    
    def populate_output_files(self, select_file=None):
        """Find all output JSON files"""
        # Clear existing items
        self.output_combo.clear()
        output_files = []
        
        for json_file in self.workspace_dir.glob("*output*.json"):
            if json_file.is_file() and self._is_valid_output_file(json_file):
                output_files.append(json_file.name)
        
        for json_file in self.workspace_dir.glob("*.json"):
            if json_file.is_file() and json_file.name not in output_files:
                # Check if it looks like an output file (has 'actualPaths' or similar)
                if self._is_valid_output_file(json_file):
                    output_files.append(json_file.name)
        
        output_files.sort(reverse=True)
        self.output_combo.addItems(output_files)
        
        # Select specific file if requested
        if select_file and select_file in output_files:
            self.output_combo.setCurrentText(select_file)
        
        self.log(f"Found {len(output_files)} output files")
    
    def _is_valid_output_file(self, json_file):
        """Check if a JSON file is a valid output/map file"""
        try:
            with open(json_file, 'r') as f:
                data = json.load(f)
                # Valid output files must have actualPaths or plannerPaths
                # and other required fields
                has_paths = 'actualPaths' in data or 'plannerPaths' in data
                has_basic_info = 'teamSize' in data or 'numTaskFinished' in data
                
                # Exclude files that are clearly not output files
                # (e.g., benchmark files, problem definitions)
                is_benchmark = json_file.name.startswith('benchmark_')
                is_problem = 'agents' in data and 'tasks' in data and 'actualPaths' not in data
                
                return has_paths and has_basic_info and not is_benchmark and not is_problem
        except Exception:
            return False
    
    def log(self, message):
        """Add message to log (thread-safe)"""
        # Use invokeMethod to ensure GUI updates happen on main thread
        QMetaObject.invokeMethod(self, "_log_impl", Qt.QueuedConnection,
                                Q_ARG(str, message))
    
    @Slot(str)
    def _log_impl(self, message):
        """Internal log implementation (must run on main thread)"""
        scrollbar = self.log_text.verticalScrollBar()
        # Check if user is at the bottom before adding text
        at_bottom = scrollbar.value() >= scrollbar.maximum() - 10
        
        self.log_text.append(message)
        
        # Only auto-scroll if user was already at the bottom
        if at_bottom:
            scrollbar.setValue(scrollbar.maximum())
    
    def show_about(self):
        """Show about dialog"""
        about_text = """
<h2>MAPF Competition Launcher</h2>
<p><b>Version 2.0.0</b></p>
<br>
<h3>Created by:</h3>
<p>
• Thomas Causetti<br>
• Gabriele Ceresara<br>
• Jacopo Tedeschi
</p>
<br>
<p><i>MAPF Competition Framework with Enhanced UI</i></p>
"""
        QMessageBox.about(self, "About", about_text)
    
    def check_requirements(self):
        """Check system requirements"""
        self.log("\n=== Checking Requirements ===")
        
        # Check apt packages
        apt_packages = ['cmake', 'build-essential', 'libboost-all-dev', 'python3-dev', 'python3-pip']
        missing_apt = []
        
        for package in apt_packages:
            if not self.check_apt_package(package):
                missing_apt.append(package)
        
        if missing_apt:
            self.log(f"Missing apt packages: {', '.join(missing_apt)}")
            reply = QMessageBox.question(
                self, "Missing Packages",
                f"The following packages are missing:\n{', '.join(missing_apt)}\n\nInstall them? (requires sudo)",
                QMessageBox.Yes | QMessageBox.No
            )
            if reply == QMessageBox.Yes:
                self.install_apt_packages(missing_apt)
        else:
            self.log("✓ All apt packages installed")
        
        # Check Python packages
        python_packages = ['numpy', 'pybind11', 'matplotlib', 'PySide6']
        missing_python = []
        
        for package in python_packages:
            try:
                __import__(package)
                self.log(f"✓ {package} installed")
            except ImportError:
                missing_python.append(package)
        
        if missing_python:
            reply = QMessageBox.question(
                self, "Missing Python Packages",
                f"Missing Python packages:\n{', '.join(missing_python)}\n\nInstall them?",
                QMessageBox.Yes | QMessageBox.No
            )
            if reply == QMessageBox.Yes:
                self.install_python_packages(missing_python)
        else:
            self.log("✓ All Python packages installed")
        
        # Check binaries
        impl_binary = self.workspace_dir / "build" / "lifelong_implemented4"
        default_binary = self.workspace_dir / "build" / "lifelong_default"
        
        if impl_binary.exists() and default_binary.exists():
            self.log("✓ Compiled binaries found")
        else:
            self.log("✗ Compiled binaries not found - will need to compile")
        
        self.check_planviz()
        self.log("=== Requirements Check Complete ===\n")
    
    def check_apt_package(self, package):
        """Check if apt package is installed"""
        try:
            result = subprocess.run(
                ['dpkg', '-s', package],
                capture_output=True,
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
            
            for line in iter(process.stdout.readline, ''):
                if line:
                    self.log(line.strip())
            
            process.wait()
            
            if process.returncode == 0:
                self.log("✓ Packages installed successfully")
            else:
                self.log("✗ Error installing packages")
        except Exception as e:
            self.log(f"✗ Error: {e}")
    
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
            
            for line in iter(process.stdout.readline, ''):
                if line:
                    self.log(line.strip())
            
            process.wait()
            
            if process.returncode == 0:
                self.log("✓ Python packages installed successfully")
            else:
                self.log("✗ Error installing Python packages")
        except Exception as e:
            self.log(f"✗ Error: {e}")
    
    def check_planviz(self):
        """Check if PlanViz is installed"""
        if self.planviz_dir.exists():
            run_script = self.planviz_dir / "script" / "run.py"
            if run_script.exists():
                self.log("✓ PlanViz found")
                return True
            else:
                self.log("✗ PlanViz incomplete installation")
        else:
            self.log("✗ PlanViz not found")
            reply = QMessageBox.question(
                self, "PlanViz Not Found",
                "PlanViz visualization tool is not installed.\n\nInstall it?",
                QMessageBox.Yes | QMessageBox.No
            )
            if reply == QMessageBox.Yes:
                self.install_planviz()
        return False
    
    def install_planviz(self):
        """Install PlanViz"""
        self.log("\n=== Installing PlanViz ===")
        try:
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
            
            for line in iter(process.stdout.readline, ''):
                if line:
                    self.log(line.strip())
            
            process.wait()
            
            if process.returncode == 0:
                self.log("✓ PlanViz installed successfully")
            else:
                self.log("✗ Error installing PlanViz")
        except Exception as e:
            self.log(f"✗ Error: {e}")
    
    def abort_operation(self):
        """Abort current operation"""
        self.abort_flag.set()
        self.log("⚠️ Abort requested...")
        
        for process in self.running_processes:
            try:
                process.terminate()
                process.wait(timeout=2)
            except:
                try:
                    process.kill()
                except:
                    pass
        
        self.running_processes.clear()
        self.log("✓ Operation aborted")
    
    def compile_code(self):
        """Compile the C++ code"""
        self.log("\n=== Compiling Code ===")
        
        compile_script = self.workspace_dir / "compile.sh"
        
        if not compile_script.exists():
            self.log("✗ compile.sh not found")
            return False
        
        try:
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
            
            for line in iter(process.stdout.readline, ''):
                if self.abort_flag.is_set():
                    process.terminate()
                    return False
                if line:
                    self.log(line.strip())
            
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
        """Run simulation in thread"""
        self.abort_flag.clear()
        thread = threading.Thread(target=self._run_simulation_thread, daemon=True)
        thread.start()
    
    def compare_planners(self):
        """Compare planners in thread"""
        self.abort_flag.clear()
        thread = threading.Thread(target=self._compare_planners_thread, daemon=True)
        thread.start()
    
    def _run_simulation_thread(self):
        """Run simulation (in thread)"""
        QMetaObject.invokeMethod(self.compile_run_btn, "setEnabled", Qt.QueuedConnection, Q_ARG(bool, False))
        QMetaObject.invokeMethod(self.abort_btn, "setEnabled", Qt.QueuedConnection, Q_ARG(bool, True))
        QMetaObject.invokeMethod(self.progress_bar, "setVisible", Qt.QueuedConnection, Q_ARG(bool, True))
        
        try:
            problem_file = self.problem_combo.currentText()
            if not problem_file:
                self.log("✗ Please select a problem file")
                return
            
            try:
                max_time = int(self.time_input.text())
            except ValueError:
                self.log("✗ Invalid max timesteps value")
                return
            
            planner = self.planner_combo.currentText()
            binary_path = self.workspace_dir / "build" / f"lifelong_{planner}"
            
            if not binary_path.exists():
                self.log(f"Binary not found: {binary_path}")
                self.log("Compiling...")
                if not self.compile_code():
                    self.log("✗ Compilation failed")
                    return
            
            problem_name = Path(problem_file).stem
            output_file = self.workspace_dir / f"{problem_name}_{planner}_output.json"
            
            # Store mapping of output file to input file
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
            
            self.log(f"\n=== Running Simulation ===")
            self.log(f"Problem: {problem_file}")
            self.log(f"Planner: {planner}")
            self.log(f"Max Timesteps: {max_time}")
            self.log(f"Output: {output_file.name}")
            
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
            
            for line in iter(process.stdout.readline, ''):
                if self.abort_flag.is_set():
                    process.terminate()
                    self.log("✗ Simulation aborted")
                    return
                if line:
                    self.log(line.strip())
            
            process.wait()
            self.running_processes.remove(process)
            runtime = time.time() - start_time
            
            if process.returncode == 0:
                self.log(f"\n✓ Simulation completed in {runtime:.2f}s")
                self.show_summary(output_file, runtime)
                # Refresh output files list and select the new file
                QMetaObject.invokeMethod(self, "_refresh_and_select_output", Qt.QueuedConnection,
                                        Q_ARG(str, output_file.name))
            else:
                self.log("✗ Simulation failed")
        
        except Exception as e:
            self.log(f"✗ Error: {e}")
            self.log(traceback.format_exc())
        
        finally:
            QMetaObject.invokeMethod(self.compile_run_btn, "setEnabled", Qt.QueuedConnection, Q_ARG(bool, True))
            QMetaObject.invokeMethod(self.abort_btn, "setEnabled", Qt.QueuedConnection, Q_ARG(bool, False))
            QMetaObject.invokeMethod(self.progress_bar, "setVisible", Qt.QueuedConnection, Q_ARG(bool, False))
    
    def _compare_planners_thread(self):
        """Compare planners (in thread)"""
        QMetaObject.invokeMethod(self.compile_run_btn, "setEnabled", Qt.QueuedConnection, Q_ARG(bool, False))
        QMetaObject.invokeMethod(self.compare_btn, "setEnabled", Qt.QueuedConnection, Q_ARG(bool, False))
        QMetaObject.invokeMethod(self.abort_btn, "setEnabled", Qt.QueuedConnection, Q_ARG(bool, True))
        QMetaObject.invokeMethod(self.progress_bar, "setVisible", Qt.QueuedConnection, Q_ARG(bool, True))
        
        try:
            problem_file = self.problem_combo.currentText()
            if not problem_file:
                self.log("✗ Please select a problem file")
                return
            
            try:
                max_time = int(self.time_input.text())
            except ValueError:
                self.log("✗ Invalid max timesteps value")
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
                    break
                
                self.log(f"\n--- Running {planner.upper()} planner ---")
                
                binary_path = self.workspace_dir / "build" / f"lifelong_{planner}"
                
                if not binary_path.exists():
                    self.log(f"Binary not found, compiling...")
                    if not self.compile_code():
                        self.log(f"✗ Failed to compile {planner}")
                        continue
                
                problem_name = Path(problem_file).stem
                output_file = self.workspace_dir / f"{problem_name}_{planner}_comparison.json"
                
                # Store mapping of output file to input file
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
                
                cmd = [
                    str(binary_path),
                    '--inputFile', str(self.workspace_dir / problem_file),
                    '-o', str(output_file),
                    '--simulationTime', str(max_time)
                ]
                
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
                
                for line in iter(process.stdout.readline, ''):
                    if self.abort_flag.is_set():
                        process.terminate()
                        break
                    if line:
                        self.log(line.strip())
                
                process.wait()
                self.running_processes.remove(process)
                runtime = time.time() - start_time
                
                if process.returncode == 0 and not self.abort_flag.is_set():
                    result = self._parse_simulation_results(output_file, runtime)
                    if result:
                        results[planner] = result
                        self.log(f"✓ {planner} completed in {runtime:.2f}s")
            
            # Display comparison and graphs
            if not self.abort_flag.is_set() and len(results) == 2:
                self.comparison_results = results
                QMetaObject.invokeMethod(self, "_plot_comparison_graphs_main_thread", Qt.QueuedConnection)
                QMetaObject.invokeMethod(self, "_switch_to_comparison_tab", Qt.QueuedConnection)
            
            QMetaObject.invokeMethod(self, "populate_output_files", Qt.QueuedConnection)
        
        except Exception as e:
            self.log(f"✗ Error during comparison: {e}")
            self.log(traceback.format_exc())
        
        finally:
            QMetaObject.invokeMethod(self.compile_run_btn, "setEnabled", Qt.QueuedConnection, Q_ARG(bool, True))
            QMetaObject.invokeMethod(self.compare_btn, "setEnabled", Qt.QueuedConnection, Q_ARG(bool, True))
            QMetaObject.invokeMethod(self.abort_btn, "setEnabled", Qt.QueuedConnection, Q_ARG(bool, False))
            QMetaObject.invokeMethod(self.progress_bar, "setVisible", Qt.QueuedConnection, Q_ARG(bool, False))
    
    def _parse_simulation_results(self, output_file, runtime=None):
        """Parse simulation results"""
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
        """Display comparison results"""
        self.log(f"\n{'='*60}")
        self.log("PLANNER COMPARISON RESULTS")
        self.log(f"{'='*60}")
        
        impl = results.get('implemented4', {})
        default = results.get('default', {})
        
        if not impl or not default:
            self.log("Missing results for comparison")
            return
        
        self.log("\nIMPLEMENTED4 PLANNER:")
        self.log(f"  Agents: {impl['num_agents']}")
        self.log(f"  Tasks Finished: {impl['num_tasks_finished']}")
        self.log(f"  Makespan: {impl['makespan']}")
        self.log(f"  Total Distance: {impl['total_distance']}")
        if impl.get('runtime'):
            self.log(f"  Runtime: {impl['runtime']:.2f}s")
        
        self.log("\nDEFAULT PLANNER:")
        self.log(f"  Agents: {default['num_agents']}")
        self.log(f"  Tasks Finished: {default['num_tasks_finished']}")
        self.log(f"  Makespan: {default['makespan']}")
        self.log(f"  Total Distance: {default['total_distance']}")
        if default.get('runtime'):
            self.log(f"  Runtime: {default['runtime']:.2f}s")
        
        self.log(f"\n{'='*40}")
        self.log("COMPARISON METRICS")
        self.log(f"{'='*40}")
        
        impl_tasks = impl['num_tasks_finished']
        default_tasks = default['num_tasks_finished']
        
        self.log(f"\nScoring (both ran for {max_time} timesteps):")
        self.log(f"  Implemented4: {impl_tasks} tasks in {impl.get('runtime', 0):.2f}s")
        self.log(f"  Default: {default_tasks} tasks in {default.get('runtime', 0):.2f}s")
        
        if impl_tasks > default_tasks:
            winner = "IMPLEMENTED4"
            margin = ((impl_tasks - default_tasks) / max(default_tasks, 1)) * 100
        elif default_tasks > impl_tasks:
            winner = "DEFAULT"
            margin = ((default_tasks - impl_tasks) / max(impl_tasks, 1)) * 100
        else:
            winner = "TIE"
            margin = 0
        
        self.log(f"\n🏆 WINNER: {winner}")
        if winner != "TIE":
            self.log(f"   Margin: {margin:.1f}% more tasks completed")
        
        self.log(f"\n{'='*60}")
    
    @Slot()
    def _switch_to_comparison_tab(self):
        """Switch to comparison tab (must run in main thread)"""
        self.tabs.setCurrentWidget(self.comparison_tab)
    
    @Slot()
    def _plot_comparison_graphs_main_thread(self):
        """Plot comparison graphs in main thread"""
        if hasattr(self, 'comparison_results'):
            self._plot_comparison_graphs(self.comparison_results)
    
    def _plot_comparison_graphs(self, results):
        """Plot comparison graphs"""
        # Clear the entire figure
        self.comparison_figure.clear()
        
        # Recreate subplots
        self.ax1 = self.comparison_figure.add_subplot(1, 2, 1)
        self.ax2 = self.comparison_figure.add_subplot(1, 2, 2)
        
        # Style all axes
        for ax in [self.ax1, self.ax2]:
            ax.set_facecolor('#2b2b2b')
            ax.tick_params(colors='white', which='both')
            ax.spines['bottom'].set_color('white')
            ax.spines['top'].set_color('white')
            ax.spines['left'].set_color('white')
            ax.spines['right'].set_color('white')
            ax.xaxis.label.set_color('white')
            ax.yaxis.label.set_color('white')
            ax.title.set_color('white')
        
        impl = results.get('implemented4', {})
        default = results.get('default', {})
        
        if not impl or not default:
            return
        
        # Graph 1: Tasks Finished
        tasks = [impl['num_tasks_finished'], default['num_tasks_finished']]
        bars1 = self.ax1.bar(['Implemented4', 'Default'], tasks, color=['#0078d4', '#ff6b6b'], alpha=0.8, edgecolor='white', linewidth=1.5)
        self.ax1.set_title('Tasks Finished', fontsize=25, fontweight='bold', color='white')
        self.ax1.set_ylabel('Number of Tasks', fontsize=20, color='white')
        self.ax1.grid(True, alpha=0.3, linestyle='--', color='gray')
        
        # Add value labels on bars
        for bar in bars1:
            height = bar.get_height()
            self.ax1.text(bar.get_x() + bar.get_width()/2., height,
                         f'{int(height)}',
                         ha='center', va='bottom', fontsize=22, color='white', fontweight='bold')
        
        # Graph 2: Runtime
        runtime = [impl.get('runtime', 0), default.get('runtime', 0)]
        bars2 = self.ax2.bar(['Implemented4', 'Default'], runtime, color=['#0078d4', '#ff6b6b'], alpha=0.8, edgecolor='white', linewidth=1.5)
        self.ax2.set_title('Runtime', fontsize=25, fontweight='bold', color='white')
        self.ax2.set_ylabel('Seconds', fontsize=20, color='white')
        self.ax2.grid(True, alpha=0.3, linestyle='--', color='gray')
        
        for bar in bars2:
            height = bar.get_height()
            self.ax2.text(bar.get_x() + bar.get_width()/2., height,
                         f'{height:.2f}s',
                         ha='center', va='bottom', fontsize=22, color='white', fontweight='bold')
        
        # Update title
        winner_text = "Comparison Results"
        if impl['num_tasks_finished'] > default['num_tasks_finished']:
            winner_text = "🏆 Implemented4 Wins!"
        elif default['num_tasks_finished'] > impl['num_tasks_finished']:
            winner_text = "🏆 Default Wins!"
        else:
            winner_text = "🤝 It's a Tie!"
        
        self.comparison_title.setText(winner_text)
        
        # Style all axes
        for ax in [self.ax1, self.ax2]:
            ax.tick_params(colors='white', which='both', labelsize=18)
            for spine in ax.spines.values():
                spine.set_color('white')
        
        self.comparison_figure.tight_layout(pad=4.0, h_pad=3.0, w_pad=3.0)
        self.comparison_canvas.draw()
    
    def show_summary(self, output_file, runtime=None):
        """Show simulation summary"""
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
            
            self.log(f"Agents: {num_agents}")
            self.log(f"Tasks Finished: {num_tasks_finished}")
            self.log(f"Makespan: {makespan}")
            self.log(f"Total Distance: {total_distance}")
            if runtime:
                self.log(f"Runtime: {runtime:.2f} seconds")
            
            self.log("=== Summary Complete ===\n")
        except Exception as e:
            self.log(f"Error reading results: {e}")
    
    @Slot(str)
    def _refresh_and_select_output(self, output_file_name):
        """Refresh output files list and select the specified file"""
        self.populate_output_files(select_file=output_file_name)
        
        # Ask if user wants to visualize
        reply = QMessageBox.question(
            self, "Visualize Result?",
            "Do you want to visualize the result with PlanViz?",
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.visualize_result()
    
    def visualize_result(self):
        """Visualize result using PlanViz"""
        output_file = self.output_combo.currentText()
        
        if not output_file:
            QMessageBox.warning(self, "No Output", "Please select an output file to visualize")
            return
        
        if not self.planviz_dir.exists():
            QMessageBox.warning(
                self, "PlanViz Not Found",
                "PlanViz visualization tool is not installed.\nPlease check requirements."
            )
            return
        
        output_path = self.workspace_dir / output_file
        
        try:
            with open(output_path, 'r') as f:
                data = json.load(f)
            
            # Get map info
            map_name = data.get('mapFile', '')
            if map_name:
                if not Path(map_name).is_absolute():
                    map_path = self.workspace_dir / map_name
                else:
                    map_path = Path(map_name)
            else:
                # Try to find map from problem file via mapping
                self.log("Map file not specified in output, attempting to find from problem...")
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
                            map_name = input_data.get('mapFile', '')
                            if map_name:
                                map_path = input_file_path.parent / map_name
                            else:
                                map_path = None
                    except:
                        map_path = None
                else:
                    map_path = None
                
                if not map_path:
                    self.log("✗ Map file not found in output JSON or input file")
                    QMessageBox.critical(
                        self, "Map File Not Found",
                        f"Could not find the map file for {output_file}."
                    )
                    return
            
            if map_path and map_path.exists():
                self.log(f"\n=== Launching PlanViz ===")
                self.log(f"Output: {output_file}")
                self.log(f"Map: {map_path}")
                
                planviz_script = self.planviz_dir / "script" / "run.py"
                
                cmd = [
                    sys.executable,
                    str(planviz_script),
                    '--map', str(map_path),
                    '--plan', str(output_path)
                ]
                
                subprocess.Popen(cmd)
                self.log("✓ PlanViz launched")
            else:
                QMessageBox.warning(
                    self, "Map Not Found",
                    f"Could not find map file: {map_name}"
                )
        
        except json.JSONDecodeError:
            QMessageBox.critical(self, "Error", "Invalid JSON in output file")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error visualizing: {e}")
    
    def closeEvent(self, event):
        """Handle window close"""
        for process in self.running_processes:
            try:
                process.terminate()
                process.wait(timeout=2)
            except:
                try:
                    process.kill()
                except:
                    pass
        
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # Modern style
    
    launcher = MAPFLauncher()
    launcher.show()
    
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
