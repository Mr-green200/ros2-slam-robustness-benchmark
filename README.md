# ROS2 SLAM Robustness Benchmark

Bachelor thesis — FH Technikum Wien  
*Evaluating the Robustness of Graph- and Filter-based SLAM Algorithms under Odometry Noise and Dynamic Environment Changes in ROS2*

**Author:** Josip Juric  
**Contact:** mr23b013@technikum-wien.at

---

## Repository contents

### `slam_benchmark/`

The ROS2 package used to run the experiments. Built on ROS2 Jazzy with Gazebo Harmonic. It includes the benchmark launch files, noise injection nodes, waypoint navigator, trajectory recorder, and batch run scripts.

See `slam_benchmark/README.md` for setup and run instructions.

### `complete_run_data/`

Results from all 240 benchmark runs, organized as:

```
complete_run_data/
└── {world}/
    └── {algorithm}/
        └── {noise_level}/
            ├── {run_id}/
            │   ├── ate_plot.pdf        # ATE trajectory error plot
            │   ├── resource_plot.pdf   # CPU and RAM usage over time
            │   ├── ate_results.txt     # evo_ape output
            │   ├── rpe_results.txt     # evo_rpe output
            │   ├── resource_usage.txt  # CPU/RAM samples + summary
            │   └── README.txt          # run parameters and seed
            └── batch_summary/
                ├── average_summary.txt # mean ± std ATE/RPE over 10 runs
                ├── average_summary.json
                └── resource_combined.pdf
```

**Worlds:** `corridor_world`, `loop_closure_world`, `dynamic_world`  
**Algorithms:** `slam_toolbox`, `amcl`  
**Noise levels:** `noise_0pct`, `noise_5pct`, `noise_10pct`, `noise_20pct`  
**Runs per condition:** 10
