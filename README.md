# FRC2025

To run the simulation in AdvantageScope, follow these steps:

1. **Open AdvantageScope**.
2. **Create a new 3D field**.
3. **Add Sharc Robot to Assets**:
   - Download the SHARC robot model from [this link](https://drive.google.com/file/d/1VGRzSVTl8jghPXwf17nf49obtcfHuRz9/view?usp=sharing).
   - Add the downloaded file to the `Assets` folder in AdvantageScope.
4. **Select the robot configuration**:
   - Choose the `SHARC` robot in the configuration settings.
5. **Add Odometry Data to 2D Poses**:
   - Drag `AdvantageKit/RealOutputs/Odometry/Robot` to the 2D Poses section and select the `Component (Robot)`.
6. **Add Elevator Pose (First Pose) to 3D Poses**:
   - Drag `AdvantageKit/RealOutputs/Elevator/Pose/FirstPose3D` to the 3D Poses section and select the `Component (Robot)`.
7. **Add Elevator Pose (Carriage Pose) to 3D Poses**:
   - Drag `AdvantageKit/RealOutputs/Elevator/Pose/CarriagePose3D` to the 3D Poses section and select the `Component (Robot)`.
8. **Add Vision Target Tag to 3D Poses**:
   - Drag `AdvantageKit/RealOutputs/Vision/Summary/TagPoses` to the 3D Poses section and select `Vision Target`.
