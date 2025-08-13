# CUT3R ROS Package for Dynamic Scene Reconstruction

This repository contains the code for the ROSBag, for the CUT3R paper, so that it can publish the ROSBags, which will be recorded for a dynamic scene reconstruction in a continuous environment.

## Recent Fixes - Frame Transformation Issues Resolved

### Problem
The aggregated point cloud was experiencing frame transformation issues where:
- Points from different frames were overlapping instead of being properly aligned
- Scenes were merging incorrectly instead of creating a sequential 360° view
- No proper coordinate system transformation was applied between frames

### Solution
The following key improvements have been implemented:

1. **Proper Frame Transformation**: Added `transform_points_to_world_frame()` method that transforms points from camera frame to world frame using camera poses
2. **Pose Tracking**: Each accumulated frame now stores its camera pose for proper alignment
3. **Sequential Accumulation**: Points are now properly transformed and aligned before accumulation
4. **Memory Management**: Periodic clearing of accumulated data to prevent memory issues
5. **Configurable Parameters**: Added parameters for circle radius, rotation speed, and accumulation limits

## Key Features

- **Continuous 360° Reconstruction**: Properly aligned point clouds for room visualization
- **Color Support**: RGB colors extracted from camera images
- **Voxel Downsampling**: Efficient point cloud processing
- **Configurable Motion**: Adjustable circular motion parameters
- **Memory Management**: Automatic cleanup of old frames

## Parameters

- `cut3r_path`: Path to CUT3R installation
- `model_path`: Path to CUT3R model file
- `publish_current_pointcloud`: Enable current frame point cloud publishing
- `publish_aggregated_pointcloud`: Enable aggregated point cloud publishing
- `publish_depth_map`: Enable depth map publishing
- `voxel_size`: Voxel size for downsampling (default: 0.05)
- `enable_colors`: Enable color extraction (default: true)
- `max_accumulated_frames`: Maximum frames before clearing (default: 72)
- `circle_radius`: Radius of circular motion (default: 1.0)
- `rotation_speed`: Degrees per frame for rotation (default: 5.0)

## Usage

### Running the Node
```bash
# Direct execution
python3 cut3r_cycle_working.py

# Using launch file
ros2 launch cut3r_launch.py

# With custom parameters
ros2 launch cut3r_launch.py circle_radius:=2.0 rotation_speed:=3.0
```

### Topics Published
- `/cut3r/current_pointcloud`: Current frame point cloud
- `/cut3r/aggregated_pointcloud`: Accumulated point cloud (360° view)
- `/cut3r/depth_map`: Depth map from current frame

### Topics Subscribed
- `/race10/cam1/color/image_raw`: Input camera images (configurable)

## Visualization in RViz2

1. Launch RViz2
2. Add PointCloud2 displays for both current and aggregated topics
3. Set the frame_id to "map"
4. The aggregated point cloud should now show a proper 360° view without overlapping scenes

## Technical Details

### Frame Transformation Process
1. Each frame's points are transformed from camera coordinates to world coordinates
2. Camera poses are tracked using circular motion simulation
3. Points are rotated for ROS coordinate system compatibility
4. Accumulated points are downsampled using voxel grid filtering

### Memory Management
- Accumulated data is cleared every 72 frames (configurable)
- Last half of frames are kept for smooth transitions
- Voxel downsampling reduces memory usage

## Troubleshooting

If you still see overlapping scenes:
1. Check that `publish_aggregated_pointcloud` is enabled
2. Verify camera topic is publishing correctly
3. Adjust `circle_radius` and `rotation_speed` parameters
4. Check logs for transformation errors
