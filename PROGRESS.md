# IsaacSim 5.1.0 AI Worker Setup Progress

## Environment
- OS: Ubuntu 24.04.3 LTS
- GPU: NVIDIA RTX PRO 6000 Blackwell (98GB VRAM)
- Driver: 570.211.01 / CUDA 12.8
- Conda: 25.11.1

## Steps

| # | Step | Status | Guide |
|---|------|--------|-------|
| 1 | Install IsaacSim 5.1.0 | **COMPLETED** | [guides/01-install-isaacsim.md](guides/01-install-isaacsim.md) |
| 2 | Import URDF | **COMPLETED** | [guides/02-import-urdf.md](guides/02-import-urdf.md) |
| 3 | Import Sensors | **COMPLETED** | [guides/03-import-sensors.md](guides/03-import-sensors.md) |
| 4 | Publish TF Tree (커스텀) | **IN PROGRESS** | [guides/04-publish-tf.md](guides/04-publish-tf.md) |
| 5 | Control Humanoids | Pending | [guides/05-control-humanoids.md](guides/05-control-humanoids.md) |
| 6 | Swerve Drive Control | Pending | [guides/06-swerve-drive.md](guides/06-swerve-drive.md) |
| 7 | Kinematic Override Drive | Pending | [guides/07-kinematic-override.md](guides/07-kinematic-override.md) |
| 8 | Navigation System | Pending | [guides/08-navigation-system.md](guides/08-navigation-system.md) |

## Notes
- Original docs are for IsaacSim 5.0.0, we're using **5.1.0**
- IsaacSim 5.1.0 officially supports Ubuntu 24.04
- Robot model: ROBOTIS FFW-SG2 Mobility (AI Worker)
- ZIP resource file contains USD/URDF/mesh files ready to use
