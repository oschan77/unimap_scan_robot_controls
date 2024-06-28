# ScanRobotControls

## Installation
```
git clone https://github.com/oschan77/unimap_scan_robot_controls.git
cd <unimap_scan_robot_controls
```

## Methods
### Import
```
from ScanRobotControls import ScanRobotControls
```

### Initialize
```
controls = ScanRobotControls()
```

### Start Recording
```
controls.record()
```

### Stop Recording
```
controls.stop()
```

### Convert the Latest Bag
```
controls.convert()
```

### Get the PID of the Recording Process
```
controls.get_pid()
```

## Example Usage
```
from ScanRobotControls import ScanRobotControls

controls = ScanRobotControls()
controls.record()
time.sleep(10)  # Record for 10 seconds
controls.stop()
controls.convert()
```