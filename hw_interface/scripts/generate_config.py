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
    node_config.update(bus_yaml['defaults'])
    out_yaml[node_name] = node_config

with open(os.path.join(out_dir, 'bus.yml'), 'w') as f:
    f.write(yaml.dump(out_yaml))

udev_rules = """# Copy this file to /etc/udev/rules.d/99-candlelight.rules and run the following commands:
# sudo udevadm control --reload-rules && sudo systemctl restart systemd-udevd && sudo udevadm trigger
"""

motor_descriptions = []

for bus_name, bus_config in bus_yaml['buses'].items():
    udev_rules += f'\nSUBSYSTEM=="net", ATTRS{{idVendor}}=="1d50", ATTRS{{idProduct}}=="606f", ATTRS{{serial}}=="{bus_config['serial_number']}", NAME="{bus_name}"'
    for node in bus_config['nodes']:
        motor_descriptions.append(f'{{"{bus_name}", {node['node_id']}, {node['global_id']}, {node['encoder']}, {node['offset']}}},')

with open(os.path.join(out_dir, '99-candlelight.rules'), 'w') as f:
    f.write(udev_rules)

with open(os.path.join(out_dir, 'motor_descriptions.h'), 'w') as f:
    f.write('\n'.join(motor_descriptions))
