# MAPF Competition Framework - Installation Guide

This guide provides essential instructions for setting up and running the MAPF (Multi-Agent Path Finding) competition framework.

## Prerequisites

- Ubuntu/Debian Linux system
- Python 3.10 or higher
- Git

## Step 1: Clone the Repository

```bash
git clone <your-repo-url>
cd LORR24_ZioCecio
```

## Step 2: Install System Dependencies

```bash
sudo apt-get update
sudo apt-get install -y cmake build-essential libboost-all-dev python3-dev python3-pip
```

## Step 3: Install Python Dependencies

```bash
sudo apt-get install -y python3-pybind11 python3-numpy
```

## Step 4: Compile the Framework

```bash
chmod +x compile.sh
./compile.sh
```

Verify compilation:

```bash
ls -la ./build/lifelong
```

## Step 5: Test the Installation

```bash
./build/lifelong --inputFile ./example_problems/random.domain/random_32_32_20_100.json -o test.json
```

## Step 6: Install Visualization Tool (Optional)

```bash
cd ..
git clone https://github.com/MAPF-Competition/PlanViz.git
cd PlanViz
sudo apt-get install -y python3-matplotlib python3-numpy python3-pandas python3-pil python3-scipy python3-tk
```

## Step 7: Run Visualization (Optional)

```bash
python3 script/run.py --map ../LORR24_ZioCecio/example_problems/random.domain/maps/random-32-32-20.map --plan ../LORR24_ZioCecio/test.json
```

## Step 8: Calculate Performance Scores

```bash
cd ../LORR24_ZioCecio
python3 calculate_score.py test.json
```

## Basic Usage

```bash
# Run simulation
./build/lifelong --inputFile <problem_file> -o <output_file>

# Analyze results  
python3 calculate_score.py <result_file>

# Compare multiple results
python3 calculate_score.py --compare --all
```