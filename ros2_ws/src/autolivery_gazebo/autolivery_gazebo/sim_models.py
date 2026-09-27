"""SDF for the simulated delivery vehicles and their parcels.

Each vehicle is one of PX4's own Gazebo models, merged into a wrapper that adds a
DetachableJoint to a separate parcel model. The joint holds the parcel rigidly until
payload_node publishes on the detach topic; then the parcel is an ordinary rigid body and
falls, so a hover release, a winch "release" and a fixed-wing air drop all end with the
parcel where physics puts it.

Parcels are sized and placed per airframe so they fit the landing gear:
  x500          narrow box between the legs, clear of the skids and the belly
  standard_vtol flat box under the belly plate -- the plate is this model's landing gear,
                so the parcel doubles as its landing pad until released
  rc_cessna     small, low box behind the main wheels, short enough to clear a tail
                strike on rotation
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class SimVehicle:
    base_model: str          # PX4 model under Tools/simulation/gz/models
    airframe: int            # PX4 SYS_AUTOSTART
    base_link_height: float  # base_link above the model origin (the model's own <pose>)
    parcel_size: tuple       # x, y, z metres
    parcel_mass: float       # kg
    parcel_offset: tuple     # parcel centre relative to base_link


VEHICLES = {
    'x500': SimVehicle(
        base_model='x500', airframe=4001, base_link_height=0.24,
        parcel_size=(0.16, 0.10, 0.12), parcel_mass=0.5, parcel_offset=(0.0, 0.0, -0.13)),
    'standard_vtol': SimVehicle(
        base_model='standard_vtol', airframe=4004, base_link_height=0.246,
        parcel_size=(0.30, 0.30, 0.06), parcel_mass=0.4, parcel_offset=(0.0, 0.0, -0.128)),
    'rc_cessna': SimVehicle(
        base_model='rc_cessna', airframe=4003, base_link_height=0.246,
        parcel_size=(0.11, 0.06, 0.04), parcel_mass=0.08, parcel_offset=(-0.125, 0.0, -0.08)),
}

GRIPPER_TOPICS = {
    'detach': '/autolivery/gripper/detach',
    'attach': '/autolivery/gripper/attach',
    'state': '/autolivery/gripper/state',
}


def vehicle_sdf(sim: SimVehicle, name: str, parcel_name: str, topics=GRIPPER_TOPICS) -> str:
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <include merge="true">
      <uri>model://{sim.base_model}</uri>
    </include>
    <plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">
      <parent_link>base_link</parent_link>
      <child_model>{parcel_name}</child_model>
      <child_link>link</child_link>
      <detach_topic>{topics['detach']}</detach_topic>
      <attach_topic>{topics['attach']}</attach_topic>
      <output_topic>{topics['state']}</output_topic>
      <suppress_child_warning>true</suppress_child_warning>
    </plugin>
  </model>
</sdf>
"""


def parcel_sdf(sim: SimVehicle, name: str) -> str:
    x, y, z = sim.parcel_size
    m = sim.parcel_mass
    ixx = m * (y * y + z * z) / 12.0
    iyy = m * (x * x + z * z) / 12.0
    izz = m * (x * x + y * y) / 12.0
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <link name="link">
      <inertial>
        <mass>{m}</mass>
        <inertia>
          <ixx>{ixx:.8f}</ixx><iyy>{iyy:.8f}</iyy><izz>{izz:.8f}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>{x} {y} {z}</size></box></geometry>
        <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>
      </collision>
      <visual name="box">
        <geometry><box><size>{x} {y} {z}</size></box></geometry>
        <material>
          <ambient>0.72 0.55 0.33 1</ambient>
          <diffuse>0.72 0.55 0.33 1</diffuse>
        </material>
      </visual>
      <visual name="tape">
        <geometry><box><size>{x * 1.01} {y * 0.25} {z * 1.01}</size></box></geometry>
        <material>
          <ambient>0.9 0.85 0.3 1</ambient>
          <diffuse>0.9 0.85 0.3 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def parcel_spawn_position(sim: SimVehicle, vehicle_position):
    """World position of the parcel for a vehicle spawned (unrotated) at vehicle_position."""
    vx, vy, vz = vehicle_position
    ox, oy, oz = sim.parcel_offset
    return (vx + ox, vy + oy, vz + sim.base_link_height + oz)
