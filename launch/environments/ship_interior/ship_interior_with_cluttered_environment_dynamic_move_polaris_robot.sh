#!/bin/bash
rosservice call --wait /gazebo/set_model_state
rosservice call /gazebo/set_model_state '{model_state: { model_name: polaris_ranger_ev_0, pose: { position: { x: -2.00007, y: -1.99982 ,z: 0.000309 }, orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707 } }, twist: { linear: {x: 0.0 , y: 0.2 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'
exec "$@"