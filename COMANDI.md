
cmake --build build && ./build/lifelong --inputFile ./example_problems/random.domain/random_32_32_20_100.json -o test_output.json

esegui:

./build/lifelong --inputFile ./example_problems/random.domain/random_32_32_20_100.json -o test_output.json

warehouse large:

cmake --build build && ./build/lifelong --inputFile ./example_problems/warehouse.domain/warehouse_large_5000.json -o viz_output.json

gui random:

cd /home/thomas/Documenti/Github/MAPF/PlanViz && python3 script/run.py --map ../LORR24_ZioCecio/example_problems/random.domain/maps/random-32-32-20.map --plan ../LORR24_ZioCecio/viz_output.json

gui warehouse:

cd /home/thomas/Documenti/Github/MAPF/PlanViz && python3 script/run.py --map ../LORR24_ZioCecio/example_problems/warehouse.domain/maps/warehouse_large.map --plan ../LORR24_ZioCecio/viz_output.json


Thomas PIBT
Cecio RHCR
Apo LNS