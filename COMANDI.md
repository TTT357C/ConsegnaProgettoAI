## Prerequisites

- Ubuntu/Debian Linux system
- Python 3.10 or higher


## Install System Dependencies

```bash
sudo apt-get update
sudo apt-get install -y cmake build-essential libboost-all-dev python3-dev python3-pip
```

## Install Python Dependencies

```bash
sudo apt-get install -y python3-pybind11 python3-numpy
```

## Compile the Framework

```bash
chmod +x compile.sh
./compile.sh
```

Verify compilation:

```bash
ls -la ./build/lifelong
```

## Test the Installation

```bash
./build/lifelong --inputFile ./example_problems/random.domain/random_32_32_20_100.json -o test.json
```

## Install Visualization Tool (Optional)

```bash
cd ..
git clone https://github.com/MAPF-Competition/PlanViz.git
cd PlanViz
sudo apt-get install -y python3-matplotlib python3-numpy python3-pandas python3-pil python3-scipy python3-tk
```

## Run Visualization (Optional)

```bash
python3 script/run.py --map ../example_problems/random.domain/maps/random-32-32-20.map --plan ../test.json
```