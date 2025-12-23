# Hardware Specifications: The Body

Detailed analysis of the physical components required for an elite line follower.

## Sensor Arrays

| Component | Type | Resolution | Usage |
|-----------|------|------------|-------|
| **QTR-8A** | Reflectance | 8 Sensors | Primary Line Detection |
| **QTR-1A** | Reflectance | 1 Sensor | Edge/Stop detection |
| **VL53L0X**| ToF Laser | 2m Range | Obstacle Detection |

## Actuators (Motors)

- **Pololu Micro Metal Gearmotors**: High RPM (1000-3000) for competition.
- **N20 Motors**: Reliable and compact.

## Motor Drivers

- **DRV8833**: Dual H-Bridge, efficient MOSFETs.
- **TB6612FNG**: High current capacity, widely supported.

## Selection Guide
> [!TIP]
> Always choose high-torque motors if your track has steep ramps. For high-speed flats, prioritize RPM.
