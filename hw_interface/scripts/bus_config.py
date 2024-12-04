#!/bin/python3
import sys
import yaml

yaml.Dumper.ignore_aliases = lambda *args : True

bus_file = sys.argv[1]
out_file = sys.argv[2]

with open(bus_file, 'r') as f:
    bus_yaml = yaml.safe_load(f.read())

out_yaml = {'options': bus_yaml['options'], 'master': bus_yaml['master']}
for node_name, node_config in bus_yaml['nodes'].items():
    node_config.update(bus_yaml['defaults'])
    out_yaml[node_name] = node_config

with open(out_file, 'w') as f:
    f.write(yaml.dump(out_yaml))
