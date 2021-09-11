#!/usr/bin/env python3

# Copy the output of this script to leo_erc_nav/launch/main.launch

landmarks = {1: [7.31, 0, 0.21],
             2: [7.19, 7.55, 0.20],
             3: [18.85, -3.59, 0.36],
             4: [33.77, 6.41, 0.04],
             5: [13.22, -13.61, 0.82],
             6: [21.01, 13.21, 0.13],
             7: [20.96, 3.36, 0.17],
             8: [20.40, -19.41, 0.79],
             9: [14.77, 6.89, 0.44],
             10: [22.46, -10.36, 0.57],
             11: [31.56, -18.81, 0.58],
             12: [29.92, 11.44, 0.05],
             13: [32.79, -6.79, 0.18],
             14: [2.04, -12.02, 0.50],
             15: [7.63, 13.24, -0.01]}
waypoints = {1: [12.19, 8.73, 0.11],
             2: [25.04, 4.36, -0.09],
             3: [28.62, -6.17, 0.28],
             4: [11.63, -16.85, 0.98],
             5: [7.64, -5.55, -0.08],
             6: [27.48, -13.65, 0]}



for id, p in landmarks.items():
    print(f'<node pkg="tf2_ros" type="static_transform_publisher" name="lm_{id}_broadcaster" args="{p[0]} {p[1]} {p[2]} 0 0 0 1 world_frame lm_{id}_frame"/>')

for id, p in waypoints.items():
    print(f'<node pkg="tf2_ros" type="static_transform_publisher" name="wp_{id}_broadcaster" args="{p[0]} {p[1]} {p[2]} 0 0 0 1 world_frame wp_{id}_frame"/>')
