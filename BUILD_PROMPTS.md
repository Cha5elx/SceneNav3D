# Step-by-Step Coding Prompts for SceneNav3D

## How to Use This File

Use the prompts in order. Run one prompt at a time, review the diff, and verify the stated acceptance criteria before continuing. Keep the first implementation deliberately small.

The target architecture is:

```text
natural language
  -> named-location or Chat-Scene grounding
  -> stable semantic object ID
  -> map-frame approach pose
  -> Nav2 NavigateToPose
```

Chat-Scene must never publish `cmd_vel`. Nav2 remains responsible for motion planning and execution.

## Prompt 00: Inspect the Existing Robot Stack

```text
Inspect this workspace and the existing ROS2 Humble setup before writing code.

Goal:
- Discover the robot's existing Nav2 launch files, topics, frames, sensor topics, simulation packages, and Chat-Scene code or checkpoint locations.
- Confirm the known baseline: monocular RGB camera, lidar, IMU, likely wheel odometry, Gazebo simulation, and WSL2-hosted model inference over Wi-Fi.
- Record assumptions instead of silently guessing.

Tasks:
1. List the repository files and identify whether a ROS2 workspace already exists.
2. Find existing launch files, Nav2 params, URDF/Xacro files, maps, and sensor configuration.
3. Identify the current TF chain, especially map -> odom -> base_link and camera frames.
4. Identify RGB, CameraInfo, scan or PointCloud2, odometry, and IMU topics. Confirm whether the lidar is 2D or 3D.
5. Find the local Chat-Scene source directory and checkpoint files if they have been added.
6. Identify the Gazebo variant and existing simulation packages.
7. Record the camera intrinsics and the camera-to-base_link extrinsic transform if available.
8. Create docs/STACK_INVENTORY.md with findings, missing information, and a minimal implementation recommendation.

Constraints:
- Do not modify runtime code.
- Do not invent package names or sensor capabilities.
- Keep the document concise and factual.

Acceptance criteria:
- docs/STACK_INVENTORY.md exists.
- Every required unknown is explicitly listed.
- The recommended first coding step is justified by discovered files.
```

## Prompt 01: Scaffold the Minimal ROS2 Workspace

```text
Create the smallest ROS2 Humble workspace skeleton needed for semantic navigation.

Assume the inventory has confirmed that Nav2 already works.

Create these ament_python packages under ros2_ws/src:
- semantic_nav_interfaces
- semantic_object_map
- semantic_nav_orchestrator
- semantic_nav_bringup

Define only the minimal interfaces required for:
- storing a semantic object with object_id, label, map-frame position, confidence, and last_seen timestamp;
- resolving a text command to either a named goal pose or a semantic object ID;
- returning a clear failure reason.

Tasks:
1. Use standard ROS2 messages wherever possible.
2. Add package metadata and build files.
3. Add a README section explaining package responsibilities.
4. Add a colcon build verification command.

Constraints:
- Do not add perception or Chat-Scene integration yet.
- Do not add a database.
- Do not publish cmd_vel.
- Avoid abstractions that are not needed by the first test.

Acceptance criteria:
- colcon build succeeds.
- The interface definitions are documented.
- Each package has one clear responsibility.
```

## Prompt 02: Prove Text-to-Nav2 with Named Locations

```text
Implement the first working vertical slice: text command -> named location -> Nav2 NavigateToPose.

Goal:
- Prove the ROS2 and Nav2 integration before adding perception or ML.

Tasks:
1. Add config/named_locations.yaml with two example map-frame poses.
2. Implement a semantic_nav_orchestrator node in rclpy.
3. Accept a text command through the simplest documented ROS2 interface.
4. Resolve commands such as "go to charging area" from YAML.
5. Send the pose through nav2_msgs/action/NavigateToPose.
6. Report accepted, rejected, active, canceled, succeeded, and failed states.
7. Add unit tests for known and unknown named locations.

Constraints:
- Do not call Chat-Scene.
- Do not bypass Nav2.
- Reject unknown locations instead of choosing a fallback.

Acceptance criteria:
- In simulation, a known text command starts a Nav2 goal.
- Unknown text is rejected without robot movement.
- Unit tests pass.
```

## Prompt 03: Add a Static Semantic Object Map

```text
Add a semantic object registry backed by YAML so object navigation can be tested without perception.

Goal:
- Resolve "go near the chair" to a stable object ID and map-frame object position.

Tasks:
1. Add config/object_map.yaml with a few static objects.
2. Implement semantic_object_map as a ROS2 node with a minimal query service.
3. Store object_id, label, aliases, map-frame position, confidence, last_seen, and status.
4. Add a deterministic object resolver for exact aliases and simple label matches.
5. Return ambiguity when multiple active objects match.
6. Add tests for exact match, unknown object, inactive object, and ambiguous object.

Constraints:
- Keep persistence as YAML or JSON.
- Do not add perception yet.
- Do not silently select among ambiguous objects.

Acceptance criteria:
- Tests pass.
- The resolver returns a stable object ID or an explicit failure.
- Named-location navigation from Prompt 02 still works.
```

## Prompt 04: Generate a Safe Approach Pose

```text
Implement an approach_pose_planner for navigating near an object instead of into its center.

Goal:
- Convert a map-frame object position into a safe Nav2 goal pose.

Tasks:
1. Generate candidate poses on a configurable ring around the object.
2. Orient each candidate toward the object.
3. Reject candidates that are occupied or outside the map.
4. Prefer candidates with obstacle clearance and a valid Nav2 path.
5. Make approach distance configurable by object label.
6. Return an explicit failure if no safe pose exists.
7. Add tests using a small synthetic occupancy grid.

Constraints:
- Do not send the object's center directly to Nav2.
- Keep the candidate strategy simple and deterministic.
- Do not add optimization frameworks.

Acceptance criteria:
- A reachable object yields a collision-free approach pose.
- An enclosed object yields a clear failure.
- The selected pose faces the target object.
```

## Prompt 05: Complete the Deterministic Semantic Navigation Slice

```text
Connect the named-location resolver, semantic object map, approach pose planner, and Nav2 client.

Goal:
- Support both "go to the charging area" and "go near the chair" in simulation.

Tasks:
1. Route named places directly to their configured pose.
2. Route object requests through object resolution and approach pose planning.
3. Log the original text, selected target type, object ID if any, generated pose, and result.
4. Add cancellation support.
5. Add launch files and example configs.
6. Add an integration test with a mocked Nav2 action server.

Constraints:
- Keep language parsing deterministic.
- Reject unknown, ambiguous, inactive, and unreachable targets.
- Do not add ML code yet.

Acceptance criteria:
- Both target types work through one orchestrator.
- All rejection cases produce no Nav2 goal.
- Integration tests pass.
```

## Prompt 06: Add a Monocular RGB Perception Adapter

```text
Add online semantic object observations from the confirmed monocular RGB camera topics.

Goal:
- Maintain stable visual object tracks while keeping the grounding and navigation interfaces unchanged.
- Use manually annotated map-frame object positions for the first working version.

Tasks:
1. Subscribe to RGB and CameraInfo topics.
2. Wrap the existing detector or add the smallest suitable detector adapter.
3. Track detections across frames to produce stable track IDs.
4. Associate tracks with manually annotated semantic object IDs for the MVP.
5. Mark stale objects inactive after a configurable timeout.
6. Publish RViz markers for semantic object IDs and map-frame positions.
7. Add rosbag-based tests or fixtures for track stability and stale-object handling.

Constraints:
- Use the actual discovered camera frames and topics.
- Do not couple detector code to Nav2.
- Do not infer metric depth from a single RGB frame.
- Keep automatic metric object localization out of this prompt.

Acceptance criteria:
- Visual tracks remain stable across a short recorded sequence.
- RViz displays object IDs in the map frame.
- Stale objects are not eligible navigation targets.
```

## Prompt 07: Add Automatic Metric Object Localization

```text
Extend the monocular RGB perception adapter with conservative metric object localization.

Goal:
- Estimate map-frame positions for static visual objects without assuming an RGB-D camera.

Tasks:
1. Verify camera intrinsics, the camera-to-base_link extrinsic transform, timestamps, and robot map-frame poses.
2. Triangulate static object position candidates from multi-view bearing rays across tracked frames.
3. Reject observations with insufficient baseline, poor geometry, or high uncertainty.
4. If lidar geometry permits, add optional camera-lidar projection association as an auxiliary observation.
5. Treat 2D lidar association as conditional: the scan plane does not cover every visual object.
6. Fuse accepted observations conservatively and store position uncertainty.
7. Prevent high-uncertainty objects from becoming navigation targets.
8. Publish RViz markers that distinguish manually annotated and automatically localized objects.
9. Add fixture tests for good triangulation, insufficient baseline, high uncertainty, and unavailable lidar overlap.

Constraints:
- Do not treat monocular depth estimation as the only source of a safety-relevant navigation goal.
- Do not assume 2D lidar provides depth for every camera detection.
- Keep lidar-based Nav2 mapping and obstacle avoidance independent from semantic object localization.

Acceptance criteria:
- A static fixture with adequate camera motion yields a bounded map-frame object estimate.
- Poor geometry produces an explicit rejection.
- High-uncertainty objects cannot start navigation.
```

## Prompt 08: Build an Offline Chat-Scene Adapter

```text
Integrate the locally available Chat-Scene source and checkpoint in offline mode first.

Goal:
- Given a recorded scene snapshot and a natural-language object request, return a semantic object ID.

Tasks:
1. Inspect the exact local Chat-Scene commit, environment files, model entry points, and checkpoint format.
2. Keep upstream Chat-Scene code isolated under ml/chat_scene or reference it as an external path.
3. Implement an adapter under ml/adapters that converts the semantic object-map snapshot into the input expected by the available Chat-Scene version.
4. Preserve a mapping between Chat-Scene object identifiers and ROS2 semantic object IDs.
5. Add a CLI for offline inference.
6. Create a small evaluation fixture with language queries and expected object IDs.
7. Record latency, GPU memory use, and failure modes.

Constraints:
- Do not rewrite upstream training code.
- Do not connect the model to Nav2 yet.
- Pin dependency versions from the owned codebase instead of guessing current versions.
- If the available model expects ScanNet-style preprocessed data, document the exact conversion gap and implement only the smallest adapter needed for a fixture.

Acceptance criteria:
- The CLI loads the local checkpoint.
- At least one fixture query resolves to the expected ROS2 object ID.
- Unknown or ambiguous results are represented explicitly.
```

## Prompt 09: Expose Chat-Scene as an Asynchronous ROS2 Grounding Service

```text
Expose the verified offline Chat-Scene adapter to ROS2 without blocking robot control.

Goal:
- Replace the deterministic object resolver with a configurable Chat-Scene resolver while preserving the same orchestrator behavior.

Tasks:
1. Run model inference in a separate Python process or clearly isolated ROS2 node.
2. Add a timeout and explicit error states.
3. Pass a frozen semantic-map snapshot into each inference request.
4. Return the selected semantic object ID, confidence if available, and a short machine-readable reason.
5. Keep the deterministic resolver selectable for testing.
6. Add tests for success, timeout, model error, empty result, ambiguous result, and stale object after inference.

Constraints:
- No cmd_vel publication.
- Never move the robot after timeout, ambiguity, or stale-target failure.
- Do not reload model weights per request.

Acceptance criteria:
- The orchestrator can switch resolvers through configuration.
- Model failure cannot start a Nav2 goal.
- Existing deterministic tests continue to pass.
```

## Prompt 10: Add Bringup, Simulation, and Observability

```text
Create a reproducible semantic-navigation bringup flow for WSL2 simulation.

Tasks:
1. Add launch files for deterministic mode and Chat-Scene mode.
2. Add documented parameters for topics, frames, timeouts, approach distances, and model paths.
3. Add RViz markers for semantic objects, selected targets, and generated approach poses.
4. Add structured logs for each command lifecycle.
5. Add a rosbag recording command for RGB, CameraInfo, TF, odometry, IMU, scan or PointCloud2, semantic observations, and navigation status.
6. Document WSL2-specific DDS networking checks separately from application logic.
7. Document the Wi-Fi communication boundary between WSL2 model inference and the physical robot.

Constraints:
- Do not hard-code machine-specific paths.
- Keep simulation and real-robot configs separate.

Acceptance criteria:
- One launch command starts the simulation semantic-navigation stack.
- A failed language command is diagnosable from logs.
- A selected object and approach pose are visible in RViz.
```

## Prompt 11: Verify the End-to-End Simulation Scenarios

```text
Create and run an end-to-end simulation verification plan.

Cover:
1. Named location success.
2. Unique object success.
3. Two objects with the same label causing ambiguity.
4. Unknown target.
5. Stale object.
6. Object with no reachable approach pose.
7. Model timeout.
8. Nav2 rejection or cancellation.
9. High-uncertainty monocular object localization.

Tasks:
- Add automated tests where feasible.
- Add a concise manual checklist for RViz and Gazebo checks.
- Record commands and expected results in docs/SIMULATION_VERIFICATION.md.

Constraints:
- A rejected scenario must send no Nav2 goal.
- Do not weaken checks to make tests pass.

Acceptance criteria:
- All automated tests pass.
- Manual checks have explicit expected outcomes.
```

## Prompt 12: Prepare Low-Speed Real-Robot Deployment

```text
Prepare the verified simulation stack for controlled indoor deployment on the physical ROS2 robot.

Goal:
- Deploy conservatively without changing the semantic-navigation contract.

Tasks:
1. Compare real-robot topics, frames, sensor calibration, and Nav2 params with simulation.
2. Create separate real-robot config files.
3. Verify WSL2-to-robot DDS communication for topics, TF, services, and Nav2 actions.
4. Verify the Wi-Fi failure behavior between the WSL2 inference workstation and the physical robot.
5. Configure conservative speed limits and verify the physical emergency stop.
6. Run tests in this order: named pose, manually annotated static object, automatically localized static object, ambiguous object, unreachable object, network interruption.
7. Create docs/REAL_ROBOT_CHECKLIST.md with commands, rollback steps, and observed results.

Constraints:
- Keep the first tests low speed and supervised.
- Do not run open-environment tests before controlled tests pass.
- Network or model failure must not generate a new motion command.

Acceptance criteria:
- The checklist is complete.
- Each controlled test has a recorded result.
- Failure cases leave the robot stopped or under Nav2's normal safe behavior.
```

## Prompt 13: Review Before Extending Scope

```text
Review the implemented semantic-navigation project before adding new features.

Inspect:
- correctness of map-frame transforms;
- monocular triangulation geometry and uncertainty thresholds;
- conditional camera-lidar association behavior;
- object-ID stability;
- stale-object handling;
- ambiguity handling;
- approach-pose collision checks;
- Nav2 action lifecycle;
- model timeout isolation;
- configuration portability between simulation and the real robot;
- test coverage for failure cases.

Output:
1. Findings ordered by severity with file and line references.
2. Remaining risks.
3. The smallest justified next feature, if any.

Do not refactor unrelated code during the review.
```

## Source Anchors

- Chat-Scene paper: https://arxiv.org/abs/2312.08168
- Chat-Scene official repository: https://github.com/ZzZZCHS/Chat-Scene
- Chat-Scene preprocessing directory: https://github.com/ZzZZCHS/Chat-Scene/tree/dev/preprocess
- Nav2 Humble NavigateToPose action: https://api.nav2.org/actions/humble/navigatetopose.html
- Nav2 concepts: https://docs.nav2.org/concepts/index.html
