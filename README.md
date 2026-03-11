# 3-DOF Arm Kinematics

## Project Description
The purpose of this codebase is to quickly test and unit test different IK implementations for a 3-DOF robotic arm.

> **Note:** This codebase is being transitioned from ROS to completely non-ROS. I normally don't use ROS, it was simply used here as a prototyping tool to test forward/inverse kinematics quickly. My 3 dof arm implementation on hardware does not use ROS at all.

---

## Build Instructions and Dependencies (Linux)

**Environment:**
* **Compiler:** GCC 11.4
* **C++ Standard:** 20 (specifically 202002)
* **Build System:** CMake / colcon
* **OS:** Tested on Ubuntu 22.04 LTS

**Dependencies:**
* **ROS 2 (Humble)** - nodes and pub/sub only
* **Eigen3** - For linear algebra
* **Pinocchio** - For rigid body dynamics and Levenberg-Marquardt IK
* **yaml-cpp** - For parsing yaml configuration files
* **tinyxml2** - For parsing URDF files
* **GTest** - For unit testing

### 1. Install Dependencies
```bash
sudo apt update
sudo apt install libyaml-cpp-dev libtinyxml2-dev ros-humble-pinocchio
```

### 2. Workspace Setup
```bash
mkdir -p ~/3dof_arm_ws/src
source /opt/ros/humble/setup.bash
```
Download the package `3dof_arm_kinematics.zip` file, unzip it, and place the contents here:
`~/3dof_arm_ws/src/3dof_arm_kinematics`

### 3. Build the Package
Navigate to the workspace root:
```bash
cd ~/3dof_arm_ws
colcon build --packages-select 3dof_arm_kinematics --symlink-install
```
Source the newly built workspace overlay:
```bash
source install/setup.bash
```

### 4. Run the Unit Tests
```bash
./build/3dof_arm_kinematics/test_AnalyticalSolver
./build/3dof_arm_kinematics/test_PinocchioSolver
./build/3dof_arm_kinematics/test_KinematicSolverSimpleGeneralized
```

### 5. Launch File
Launch the ROS nodes using the launch file with commands such as:

**a) Launch with default settings (AnalyticalSolver, default angles):**
```bash
ros2 launch 3dof_arm_kinematics evaluate_kinematics.launch.py
```

**b) Explicitly launch using the Pinocchio numerical solver:**
```bash
ros2 launch 3dof_arm_kinematics evaluate_kinematics.launch.py solver_backend:=PinocchioSolver
```

**c) Explicitly launch using the Analytical solver:**
```bash
ros2 launch 3dof_arm_kinematics evaluate_kinematics.launch.py solver_backend:=AnalyticalSolver
```

**d) Launch the Pinocchio solver and command custom joint angles:**
```bash
ros2 launch 3dof_arm_kinematics evaluate_kinematics.launch.py solver_backend:=PinocchioSolver theta1:=1.0 theta2:=-0.5 theta3:=0.2
```

### 6. Configuration & Developer Tips
* **YAML file:** `config/kinematics.yaml` is a central location for tunable parameters (i.e. link lengths). Each class that depends on YAML parameters can be configured there.
* **Bash commands:** You may want to make your own bash aliases for building, testing, and launching quickly.
* **IntelliSense:** If IntelliSense in VSCode does not work, you will need to look up how to generate compile commands with colcon, make a `compile_commands.json`, and sym-link that.

---

## Design Process, Considerations & Choices

### Core Architecture & Paradigms
* **Inheritance over Policy-Based Design:** The codebase initially used Policy-Based Design (templates) to maximize portability and avoid locking the codebase with inheritance. However, this was ultimately refactored to use inheritance for readability. This decision was made to prioritize code readability, familiarity, and to avoid strange template compilation errors. The slight performance overhead of a vtable was an acceptable trade-off for a more maintainable and widely understood OOP structure (ie. we all learn inheritance, and technically templates, but it's more familiar to people).
* **API Design:** Forward Kinematics API is kept simple (returns ee pose by value). For IK, pass-by-reference of a guess and ee_target, and returning a bool. This is not always the case, but in my experience with Teleop + IK + smoothing, when the arm goes out-of-bounds, or the IK fails and returns false, you just hold the last-known good IK solve, until the API gives you true again. I added a pass-by-ref enum if more detailed error-handling is needed.

* **Dependency Injection:** Classes utilize overloaded constructors: one for typical runtime instantiation and one specifically for injecting configuration structs during unit testing.

### Codebase Organization
* **Middleware Decoupling (`core` vs. `ros` folders):** To future-proof the codebase for a planned transition from ROS 2 to a custom DDS middleware, pure non-ROS C++ APIs are strictly separated from ROS-related code. The `core` folder contains pure non-ROS C++ code (compiled as a separate binary in CMake), while the `ros` directory needs ROS for just pub/sub nodes.
* **Entry Points:** Executable entry points (`main.cpp` files) are isolated in a dedicated `exe` folder to clearly define system binaries.
* **Header Conventions:** The `.h` extension is used for header files rather than `.hpp` to provide clearer visual distinction from `.cpp` files in your IDE directory tree.

### IK/FK Solvers
* **Analytical Solver:** The analytic solver was implemented first. My experience on 3-DOF arm hardware is analytical is the best option, fast and allows you to reason logically about elbow-up/elbow-down/arm-bar-left/arm-bar-right, and less tunable parameters.
* **Numerical Solver (Levenberg-Marquardt):** Levenberg-Marquadt was picked as another avenue to quickly try out. It has good handling of singularities, the ability to slow down and dampen before reaching them.

### Configuration Management (URDF & YAML)
* **Unified Configuration:** We support both URDF (recommended) and a centralized YAML file (`config/kinematics.yaml`) for rapid, on-the-fly prototyping of link lengths without recompiling C++ code. Currently, the `AnalyticalSolver` supports both URDF and YAML parsing, while the `PinocchioSolver` relies strictly on URDF for simplicity.
* **YAML Parsing via Composition:** Classes requiring YAML parameters instantiate a private `YamlReader` member object. This increases understandability (signals immedately, via composition, that "this class depends on the Yaml") but requires more code in the class implementation (bulky). It has both advantages and disadvantages, selected it here for maximum portability and fast understandability.

### Unit Test Strategy
* **Progression:** For unit tests, we start with single-point CAD ground truths, then advance to random joint positions within limits to do FK/IK cycles.
* **Unit Test File Organization:** Simpler/general unit tests that apply to all solvers are in one .cpp file (ie. FK/IK back-and-forth cycles). More detailed, nitty gritty tests that only work on individual classes (ie. yaml parsing, URDF parsing, specific safety stuff concerning Levenberg-Marquadt damping only) are isolated to their own respective .cpp files (one .cpp file per class).
* A mock_robot_configs folder allows unit testing of URDF and Yaml parsing. These are specific configurations of the robot to keep in the test bank if needed.

### Known Limitations & Future Work
* **Fixed Degrees of Freedom:** The codebase currently assumes a fixed 3-DOF system. Future iterations should parse the joint count dynamically from the loaded URDF.
* **Comment Density:** The codebase is heavily commented to prioritize immediate readability and onboarding over maintainability (a design choice for sure).