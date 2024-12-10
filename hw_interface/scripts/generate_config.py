#!/bin/python3
import sys
import yaml
import os

yaml.Dumper.ignore_aliases = lambda *args : True

bus_file = sys.argv[1]
out_dir = sys.argv[2]

os.makedirs(out_dir, exist_ok=True)

with open(bus_file, 'r') as f:
    bus_yaml = yaml.safe_load(f.read())

out_yaml = {'options': bus_yaml['options'], 'master': bus_yaml['master']}
for node_name, node_config in bus_yaml['nodes'].items():
    out_yaml[node_name] = bus_yaml['defaults'].copy()
    out_yaml[node_name].update(node_config)

with open(os.path.join(out_dir, 'bus.yml'), 'w') as f:
    f.write(yaml.dump(out_yaml))

motor_descriptions = []

for bus_name, bus_config in bus_yaml['buses'].items():
    for node in bus_config['nodes']:
        node_id = node['node_id']
        global_id = node['global_id']
        encoder = node['encoder']
        offset = node['offset']
        torque_constant = node['torque_constant']
        reverse = 'true' if node['reverse'] else 'false'
        motor_descriptions.append(f'{{"{bus_name}", {node_id}, {global_id}, {encoder}, {offset}, {torque_constant}, {reverse}}},')

with open(os.path.join(out_dir, 'motor_descriptions.h'), 'w') as f:
    f.write('\n'.join(motor_descriptions))
