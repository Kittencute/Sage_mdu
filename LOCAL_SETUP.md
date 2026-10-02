# Local SAGE Setup Notes

This document contains machine-specific setup notes and fixes for running the SAGE TerraScout simulation.

Some values are specific to each development computer. Do not blindly copy machine-specific values between computers.

---

## 1. ROS Domain ID

File:

```text
src/fleet_config/fleet_config/model.py
```

Set a different ROS domain ID for each computer to keep ROS 2 communication isolated.

### dator10

```python
@property
def domain_id(self) -> int:
    return 133
```
Example:

| Computer | ROS Domain ID |
|---|---:|
| dator10 | `133` |
| dator11 | `30` |

---

## 2. Gazebo Partition

File:

```text
src/fleet_config/fleet_config/model.py
```

SAGE originally generates the Gazebo partition from the SAGE instance:

```python
@property
def gz_partition(self) -> str:
    return f"sage_{self.index}"
```

This causes a problem when two computers on the same network both run the same SAGE instance.

For example, both computers using from the git repo:

```text
sage_1
```

can cause their Gazebo simulations to discover and interact with each other.

This can result in behavior such as controlling the TerraScout on one computer and seeing the TerraScout on the other computer move as well.

Use a unique machine name as part of the Gazebo partition.

### dator10

```python
@property
def gz_partition(self) -> str:
    return f"dator10_sage_{self.index}"
```
The resulting partitions for SAGE instance 1 are:

| Computer | Gazebo Partition |
|---|---|
| dator10 | `dator10_sage_1` |
| dator11 | `dator11_sage_1` |

---

## 3. Foxglove Station Port

Use the Station Foxglove WebSocket address:

```text
ws://127.0.0.1:8767
```

## 4. TerraScout Simulation Model and Topics

When adding, removing, or modifying simulated TerraScout components or sensors, multiple files may need to be updated.

The main files are:

```text
src/terrascout_description/urdf/scout.urdf.xacro
src/terrascout_description/config/gz_bridge.yaml
src/terrascout_mission/config/simulation/foxglove_bridge.yaml
```

### `scout.urdf.xacro`

File:

```text
src/terrascout_description/urdf/scout.urdf.xacro
```

Contains the TerraScout robot model and simulated components/sensors.

For example, simulated cameras and depth sensors are defined here.

Changes here may add or remove:

- sensors
- cameras
- depth cameras
- links
- Gazebo plugins
- sensor parameters
- simulation topics

---

### `gz_bridge.yaml`

File:

```text
src/terrascout_description/config/gz_bridge.yaml
```

Contains ROS 2 ↔ Gazebo topic bridges.

When a sensor or simulated component publishes a Gazebo topic that is needed by ROS 2, its bridge should be configured here.

If a sensor is removed from the model, check whether its bridge should also be removed.

---

### `foxglove_bridge.yaml`

File:

```text
src/terrascout_mission/config/simulation/foxglove_bridge.yaml
```

Controls which ROS topics are exposed through the Foxglove bridge.

When adding a new simulated sensor, its topics may need to be added here before they appear in Foxglove.

These topics should also use the appropriate QoS configuration where required.

---

## 5. Quick Checklist for a New Development PC

Before running the simulation on another computer on the same network:

1. Choose a unique ROS domain ID.

2. Set it in:

   ```python
   @property
   def domain_id(self) -> int:
       return <unique_domain_id>
   ```

3. Choose a unique machine name for the Gazebo partition.

4. Set:

   ```python
   @property
   def gz_partition(self) -> str:
       return f"<machine_name>_sage_{self.index}"
   ```

5. Build the project if necessary:

   ```bash
   make build
   ```

6. Start the simulation:

   ```bash
   make up
   ```

7. Verify the actual Gazebo partition:

   ```bash
   docker exec sage1-world-1 sh -c 'pid=$(pgrep -f "gz sim" | head -1); tr "\0" "\n" < /proc/$pid/environ | grep GZ_PARTITION'
   ```

8. Verify that controlling the TerraScout on one development computer does not move the simulation on another computer.

---

## Important

The two settings that must be unique between development computers are:

```python
@property
def domain_id(self) -> int:
    return 133
```

and:

```python
@property
def gz_partition(self) -> str:
    return f"dator10_sage_{self.index}"
```

This keeps the ROS 2 and Gazebo simulations separated between development computers.