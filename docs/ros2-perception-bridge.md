# ROS 2 Perception Bridge

The `inspectron_perception_bridge` package converts compressed robot-camera
images into structured `SceneAssessment` messages for the Inspectron safety
supervisor.

## Topics

| Direction | Topic | Type |
|---|---|---|
| Input | `/inspectron/camera/compressed` | `sensor_msgs/msg/CompressedImage` |
| Output | `/inspectron/scene_assessment` | `inspectron_safety_supervisor/msg/SceneAssessment` |

The camera frame ID is used as the waypoint. Each assessment receives a unique
evidence ID derived from the waypoint, image timestamp, and frame sequence.

## Providers

The bridge supports:

- Ollama;
- OpenAI-compatible VLM services;
- a deterministic fixture provider used only by tests.

API keys are read from the environment and are not stored in ROS parameters.

## Image validation

The bridge accepts JPEG and PNG images. Before inference it verifies that:

- the image is not empty;
- the image is within the configured size limit;
- the declared format is supported;
- the data has a valid JPEG or PNG signature.

Temporary image files are deleted after inference.

## Fail-closed behavior

If image validation, VLM communication, JSON parsing, or assessment validation
fails, the bridge publishes:

- unknown traversability;
- no unverified hazard claims;
- an inspect-closer action;
- zero confidence;
- zero view quality.

The safety supervisor and motion controller prevent motion for this result.

Only one inference request is queued at a time. Extra frames are dropped while
inference is active, preventing stale-image backlogs.

## Configuration

The default configuration is located at:

`ros2/inspectron_perception_bridge/config/perception_bridge.yaml`

It defines the provider, service URL, model, timeout, image-size limit, Ollama
context window, default waypoint, and scene identifier.

## Validation

The implementation includes:

- assessment-to-message conversion tests;
- JPEG and PNG validation tests;
- malformed and oversized image tests;
- deterministic VLM assessment tests;
- fail-closed publication tests;
- ROS graph integration tests;
- clean-shutdown validation.

The complete ROS 2 stack passes 27 tests.
