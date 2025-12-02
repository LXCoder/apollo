#!/usr/bin/env python3

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################
"""
Convert a base map from txt to bin format
"""

import argparse
from modules.common_msgs.map_msgs.map_pb2 import Map
from modules.routing.proto.topo_graph_pb2 import Graph
from google.protobuf import text_format


def main():
    parser = argparse.ArgumentParser(
        description='Convert a base map from txt to bin format')
    parser.add_argument(
        '-i',
        '--input_file',
        help='Input base map in bin format',
        type=str,
        default='modules/map/data/gen/base_map.bin')
    parser.add_argument(
        '-o',
        '--output_file',
        help='Output base map in txt format',
        type=str,
        default='modules/map/data/gen/base_map.txt')
    parser.add_argument(
        '-t',
        '--type',
        help='map type: base, routing, sim',
        type=str,
        default='map')
    args = vars(parser.parse_args())

    input_file_name = args['input_file']
    output_file_name = args['output_file']
    map_type = args['type']

    with open(input_file_name, 'rb') as f:
        if map_type == 'map' or map_type == 'sim':
            mp = Map()
        elif map_type == 'routing':
            mp = Graph()
        mp.ParseFromString(f.read())

    # Output map
    with open(output_file_name, "w") as f:
        text_format.PrintMessage(mp,f)



if __name__ == '__main__':
    main()
