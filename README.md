# ROS2 Data Logger

A ROS2 package for flexible data logging for any robotic platform or project. This package provides both simple and composable logging solutions using `rosbag2`, with shared configuration and consistent behavior across both modes.

## Features

### Two Launch Modes

1. **Simple Logger** (`simple_logger.launch.py`)
   - Quick and easy rosbag recording
   - Uses `ros2 bag record` CLI command
   - MCAP format with ZSTD fast compression
   - 5 GB max bag file size
   - Records all topics by default
   - Timestamp-based bag naming
   - Configurable storage location

2. **Composable Logger** (`composable_logger.launch.py`)
   - Advanced composable node architecture
   - Can be loaded into existing node containers for performance optimization
   - Full parameter customization via YAML files
   - Uses `recorder_params.yaml` for detailed configuration
   - QoS profile overrides for specific topics
   - Timestamp-based bag naming
   - Configurable storage locations

### Recording Configuration

Both loggers are configured to record:
- ✅ All topics (with exclusions)
- ✅ All services
- ✅ All actions
- ✅ Hidden and unpublished topics
- ❌ Image topics (excluded by default to save space: `sensor_msgs/msg/Image`)

**Note**: The simple logger uses command-line arguments (`-a` for all topics), while the composable logger uses `recorder_params.yaml` for the same behavior.

### Storage Features

- **Format**: MCAP (modern rosbag format)
- **Compression**: ZSTD compression for efficient storage
- **Max Bag Size**: 5 GB per file (automatically splits larger recordings)
- **QoS Override**: Custom QoS profiles for specific topics (e.g., camera images)

### Performance Optimizations

- Multi-threaded compression (2 threads by default)
- Intra-process communication when used as composable node
- File-level compression for better performance
- Configurable compression queue size

## Usage

### Simple Logger

Start basic recording with default settings:

```bash
ros2 launch ros2_data_logger simple_logger.launch.py
```

With custom name and location:

```bash
ros2 launch ros2_data_logger simple_logger.launch.py \
    name:=field_test \
    storage_uri:=/mnt/ssd/bags
```

This creates a rosbag with the name pattern `<name>_YYYYMMDD_HHMMSS` (e.g., `field_test_20251120_143022`).

**Simple logger uses:**
- MCAP storage format
- ZSTD fast compression preset
- 5 GB max bag file size (`-b 5368709120`)
- Records all topics (`-a`)
- Output directory: `<storage_uri>/<name_timestamp>/<name_timestamp>`

### Composable Logger

#### Standalone (creates new container)

```bash
ros2 launch ros2_data_logger composable_logger.launch.py
```

#### With Custom Parameters

```bash
ros2 launch ros2_data_logger composable_logger.launch.py \
    name:=my_experiment \
    storage_uri:=/path/to/bags \
    params_file_uri:=/path/to/custom_params.yaml
```

#### Load into Existing Container

```bash
ros2 launch ros2_data_logger composable_logger.launch.py \
    container:=my_existing_container
```

This loads the recorder node into an already-running container for better performance.

## Launch Arguments

### Simple Logger Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `name` | `rosbag_<timestamp>` | Prefix for the rosbag directory name |
| `storage_uri` | `~/<name_timestamp>` | Parent directory where bags will be stored |

**Example output structure:**
```
~/rosbag_20251120_143022/
└── rosbag_20251120_143022/
    ├── rosbag_20251120_143022_0.mcap
    └── metadata.yaml
```

### Composable Logger Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `name` | `rosbag_<timestamp>` | Prefix for the rosbag directory name |
| `storage_uri` | `~/<name_timestamp>` | Parent directory where bags will be stored |
| `params_file_uri` | `config/recorder_params.yaml` | Path to recorder parameter file |
| `container` | `''` (empty) | Name of existing container to load into (optional) |

**Example output structure:**
```
~/rosbag_20251120_143022/
├── rosbag_20251120_143022_0.mcap
└── metadata.yaml
```

| Argument | Default | Description |
|----------|---------|-------------|
| `log` | `false` | Enable/disable recording |

## Choosing Between Simple and Composable Logger

| Use Case | Recommended Logger |
|----------|-------------------|
| Quick testing / one-off recordings | **Simple Logger** |
| Custom QoS profiles needed | **Composable Logger** |
| Integration with other composable nodes | **Composable Logger** |
| Performance-critical applications | **Composable Logger** (with container) |
| Minimal configuration needed | **Simple Logger** |
| Need to exclude specific topic types | **Composable Logger** |
| Development / debugging | **Simple Logger** |
| Production deployments | **Composable Logger** |

## Configuration Files

### `recorder_params.yaml`

Main configuration file for the composable logger. Key parameters:

```yaml
recorder:
  ros__parameters:
    record:
      all_topics: true
      all_services: true
      all_actions: true
      exclude_topic_types: ["sensor_msgs/msg/Image"]  # Don't record images
      compression_mode: "file"
      compression_format: "zstd"
      compression_threads: 2
    
    storage:
      storage_id: "mcap"
      max_bagfile_size: 5368709120  # 5 GB
```

**Common modifications:**

- **Include images**: Remove `"sensor_msgs/msg/Image"` from `exclude_topic_types`
- **Increase compression**: Change `compression_threads` to 4 or more
- **Larger files**: Increase `max_bagfile_size`
- **Specific topics only**: Set `all_topics: false` and add `topics:` list

### `recorder_qos_profile.yaml`

QoS (Quality of Service) overrides for specific topics. Use this when topics need special reliability settings.

**Current configuration:**
```yaml
/**/**/image_raw/compressed:
  history: keep_last
  depth: 10
  reliability: best_effort
  durability: volatile
```

**Note**: The pattern `/**/**/image_raw/compressed` is used as the topic name, but **rosbag2 does not support wildcards**. You must list exact topic names:

```yaml
/cam_sync/cam0/image_raw/compressed:
  history: keep_last
  depth: 10
  reliability: best_effort

/cam_sync/cam1/image_raw/compressed:
  history: keep_last
  depth: 10
  reliability: best_effort
```

**When to use QoS overrides:**
- Camera topics that use `best_effort` reliability
- Sensor data with high publish rates
- Topics with non-standard QoS requirements

**To reduce repetition, use YAML anchors:**
```yaml
_image_qos: &image_qos
  history: keep_last
  depth: 10
  reliability: best_effort

/cam_sync/cam0/image_raw/compressed:
  <<: *image_qos

/cam_sync/cam1/image_raw/compressed:
  <<: *image_qos
```

## Directory Structure

```
ros2_data_logger/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── recorder_params.yaml       # Main recorder configuration
│   └── recorder_qos_profile.yaml  # Topic-specific QoS settings
└── launch/
    ├── composable_logger.launch.py  # Advanced composable node launcher
    └── simple_logger.launch.py      # Simple CLI-based launcher
```

## Examples

### Simple Logger Examples

#### Quick Test Recording
```bash
ros2 launch ros2_data_logger simple_logger.launch.py
```

#### Custom Name and Location
```bash
ros2 launch ros2_data_logger simple_logger.launch.py \
    name:=experiment_01 \
    storage_uri:=/mnt/ssd/data
```
Creates: `/mnt/ssd/data/experiment_01_YYYYMMDD_HHMMSS/`

### Composable Logger Examples

#### Record Everything (Including Images)

Modify `config/recorder_params.yaml`:

```yaml
exclude_topic_types: []  # Empty list = record everything
```

### Record Only Specific Topics

Modify `config/recorder_params.yaml`:

```yaml
all_topics: false
topics:
  - /odom
  - /tf
  - /tf_static
  - /joint_states
```

### Custom Storage Location and Name

**Simple logger:**
```bash
ros2 launch ros2_data_logger simple_logger.launch.py \
    name:=field_test_01 \
    storage_uri:=/mnt/ssd/experiment_data
```

**Composable logger:**
```bash
ros2 launch ros2_data_logger composable_logger.launch.py \
    name:=field_test_01 \
    storage_uri:=/mnt/ssd/experiment_data
```

Both create: `/mnt/ssd/experiment_data/field_test_01_YYYYMMDD_HHMMSS/`

### Integration with Other Containers

Load logger into the same container as other nodes for performance:

```bash
# First, launch your main application with a container
ros2 launch my_package my_app.launch.py

# Then load the logger into that container
ros2 launch ros2_data_logger composable_logger.launch.py \
    container:=my_app_container
```

## Tips and Best Practices

### Choosing the Right Logger

- **Use Simple Logger when:**
  - You need quick, no-configuration recording
  - Testing or debugging
  - Default settings are sufficient
  - You don't need custom QoS profiles

- **Use Composable Logger when:**
  - You need to exclude specific topic types (e.g., images)
  - Custom QoS profiles are required
  - Performance is critical (load into existing container)
  - You need fine-grained control over recording parameters

### Storage Management

- **Monitor disk space**: 5GB bags fill up quickly with all topics
- **Use compression**: ZSTD provides good compression with minimal CPU overhead
- **Exclude large topics**: Images, point clouds can dominate storage

### Performance

- **Use composable nodes**: Reduces IPC overhead between nodes
- **Increase compression threads**: On multi-core systems, use more threads
- **Write to fast storage**: SSDs recommended for high-throughput topics

### Workflow

1. **Development**: Use Simple Logger for quick testing
2. **Integration**: Use Composable Logger with custom params
3. **Production**: Use Composable Logger loaded into main container
4. **Long missions**: Both loggers split files at 5 GB automatically

### Common Configuration Changes

**To record images (composable logger only):**
Edit `config/recorder_params.yaml`:
```yaml
exclude_topic_types: []  # Empty list = record everything
```

**To record only specific topics (composable logger only):**
Edit `config/recorder_params.yaml`:
```yaml
all_topics: false
topics:
  - /odom
  - /tf
  - /tf_static
  - /joint_states
```

**Simple logger always records all topics** - for selective recording, use composable logger.

## Troubleshooting

### Bag files are too large

- Add more topic types to `exclude_topic_types`
- Reduce `max_bagfile_size` to split sooner
- Disable `all_services` and `all_actions` if not needed

### Missing topics in bag

- Check QoS compatibility in `recorder_qos_profile.yaml`
- Ensure `is_discovery_disabled: false`
- Set `include_hidden_topics: true` for diagnostics

### Performance issues during recording

- Increase `compression_threads`
- Use faster storage (NVMe SSD)
- Reduce `compression_queue_size` if memory-constrained

## Related Packages

- [`rosbag2`](https://github.com/ros2/rosbag2) - ROS2 bag recording infrastructure

## License

Copyright 2024 Australian Centre for Robotic Vision
