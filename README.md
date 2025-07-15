# ROS2 MIDI Controller Node

This package provides a ROS2 node for USB MIDI controllers (e.g., KORG nanoKONTROL Studio).
It converts MIDI messages (sliders, knobs, buttons, system messages) into ROS2 topics, with flexible mapping and device support via YAML config.

## Features
- Plug & play support for USB MIDI controllers
- Converts MIDI messages to ROS2 topics (sliders, knobs, buttons, system)
- Device mapping and configuration via YAML
- Modular, extensible Python codebase
- Test scripts and sample launch files included

## Prerequisites
- Ubuntu 22.04.5 LTS (Jammy Jellyfish)
- Python 3.10.12
- ROS2 Humble
- USB MIDI Controller
- ALSA MIDI support

## Installation
```bash
# Clone this repository
$ git clone https://github.com/takatronix/midi_controller_ros2.git
$ cd midi_controller_ros2

# Install dependencies
$ pip install -r requirements.txt

# (Optional) Build as ROS2 package
$ colcon build
```

## Usage
### Run as a ROS2 node
```bash
# Source your ROS2 environment
$ source /opt/ros/<ros2-distro>/setup.bash

# Launch the node
$ ros2 launch midi_controller_node midi_controller_launch.py
```

### Test script (standalone)
```bash
$ python3 test_midi_ros2.py
```

## Supported Devices
- KORG nanoKONTROL Studio (default mapping)
- Generic USB MIDI controllers (customizable via config)

## Configuration
Edit `midi_controller_node/config/devices.yaml` to change device mapping or add new devices.

## License
See [LICENSE](LICENSE).

## Contributing
Pull requests and issues are welcome!

## Contact
Takashi Otsuka

---

## Japanese Documentation
[README in Japanese](docs/README_ja.md) is also available.
