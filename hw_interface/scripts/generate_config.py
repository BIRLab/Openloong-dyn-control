#!/bin/python3
import sys
import yaml
import os
import numpy as np


config_file = sys.argv[1]
out_dir = sys.argv[2]


def generate_bus_config(config_yaml):
    yaml.Dumper.ignore_aliases = lambda *args : True
    out_yaml = {'options': config_yaml['options'], 'master': config_yaml['master']}
    for node_name, node_config in config_yaml['nodes'].items():
        out_yaml[node_name] = config_yaml['defaults'].copy()
        out_yaml[node_name].update(node_config)
    with open(os.path.join(out_dir, 'bus.yml'), 'w') as f:
        f.write(yaml.dump(out_yaml))


def generate_motor_descriptions(config_yaml):
    motor_descriptions = []
    for bus_name, bus_config in config_yaml['buses'].items():
        for node in bus_config:
            node_id = node['node_id']
            global_id = node['global_id']
            encoder = node['encoder']
            offset = node['offset']
            torque_constant = node['torque_constant']
            reverse = 'true' if node['reverse'] else 'false'
            motor_descriptions.append(f'{{"{bus_name}", {node_id}, {global_id}, {encoder}, {offset}, {torque_constant}, {reverse}}},')
    with open(os.path.join(out_dir, 'motor_descriptions.h'), 'w') as f:
        f.write('\n'.join(motor_descriptions))


def generate_transforms(config_yaml):
    def to_tf(rpy, xyz):
        roll, pitch, yaw = rpy
        rx = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll), np.cos(roll)]])
        ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                       [0, 1, 0],
                       [-np.sin(pitch), 0, np.cos(pitch)]])
        rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                       [np.sin(yaw), np.cos(yaw), 0],
                       [0, 0, 1]])
        r = rz @ ry @ rx
        t = np.eye(4)
        t[:3, :3] = r
        t[:3, 3] = xyz
        return t

    def to_eigen(n, tf):
        return """
inline const Eigen::Matrix4f {0} {{
    {{{1}}},
    {{{2}}},
    {{{3}}},
    {{{4}}}
}};
""".format(n, *[', '.join([str(c) for c in r]) for r in tf])

    definitions = []
    for name, transform in config_yaml['transforms'].items():
        definitions.append(to_eigen(name, to_tf(transform['rpy'], transform['xyz'])))

    out_file = """\
#pragma once
#include <Eigen/Dense>

namespace Transforms {{
{0}
}}
""".format("\n".join(definitions))

    with open(os.path.join(out_dir, 'transforms.h'), 'w') as f:
        f.write(out_file)


def main():
    os.makedirs(out_dir, exist_ok=True)
    with open(config_file, 'r') as f:
        config_yaml = yaml.safe_load(f.read())
    generate_bus_config(config_yaml)
    generate_motor_descriptions(config_yaml)
    generate_transforms(config_yaml)


if __name__ == '__main__':
    main()
