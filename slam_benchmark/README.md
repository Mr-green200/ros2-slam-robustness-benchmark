# SLAM Benchmark Suite
Bachelor thesis — FH Technikum Wien
Evaluating the Robustness of Graph- and Filter-based SLAM Algorithms under Odometry Noise and Dynamic Environment Changes in ROS2

## Setup

```bash
cp -r slam_benchmark ~/slam_thesis_ws/src/
cd ~/slam_thesis_ws
colcon build --symlink-install --packages-select slam_benchmark
source install/setup.bash
```

## Single run

```bash
ros2 launch slam_benchmark benchmark.launch.py \
  algorithm:=slam_toolbox noise_pct:=0 world:=turtlebot3_world \
  run_id:=0pct_001 laps:=3 duration:=300
```

## AMCL — build map first, then benchmark

```bash
# Terminal A: build the map
ros2 launch slam_benchmark create_map.launch.py world:=corridor_world laps:=2

# Terminal B: save when mapping is done
ros2 run nav2_map_server map_saver_cli -f ~/slam_thesis_ws/maps/corridor_world_map

# Run benchmark with the saved map
ros2 launch slam_benchmark benchmark.launch.py \
  algorithm:=amcl noise_pct:=0 world:=corridor_world \
  run_id:=0pct_001 laps:=3 duration:=300 \
  map_file:=~/slam_thesis_ws/maps/corridor_world_map.yaml
```

## Batch run (10 runs, averaged)

```bash
python3 src/slam_benchmark/scripts/run_batch.py \
  --algorithm slam_toolbox --noise 0 --world loop_closure_world \
  --runs 10 --laps 3 --duration 300
```

## Parameters

| Parameter | Values | Description |
|-----------|--------|-------------|
| `algorithm` | `slam_toolbox`, `amcl` | SLAM algorithm |
| `noise_pct` | `0`, `5`, `10`, `20` | Injected odometry drift (%) |
| `world` | `turtlebot3_world`, `loop_closure_world`, `corridor_world`, `dynamic_world` | Gazebo world |
| `run_id` | e.g. `0pct_001` | Run identifier used in output path |
| `laps` | integer | Laps per run |
| `duration` | seconds | Max run time (stops early if laps finish first) |
| `seed` | integer | RNG seed for noise injection |
| `map_file` | path | Pre-built map YAML (AMCL only) |
| `use_rviz` | `true`/`false` | Open RViz |

## Output

```
/tmp/slam_benchmark_results/{world}/{algorithm}/noise_{n}pct/{run_id}/
    README.txt
    ground_truth.txt   # TUM format
    estimated.txt      # TUM format
    odometry.txt       # TUM format
    ate_results.txt
    rpe_results.txt
    ate_plot.pdf
    resource_usage.txt
    resource_plot.pdf

noise_{n}pct/batch_summary/
    average_summary.txt
    average_summary.json
    resource_combined.pdf
```

