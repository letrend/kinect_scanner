# TCP Protocol

All TCP messages are newline-delimited JSON objects.

## UI control server

Default endpoint: `127.0.0.1:5056`

Incoming commands:

```json
{"type":"start","seq":1}
{"type":"stop","seq":2}
{"type":"pause","paused":true,"seq":3}
{"type":"reset","seq":4}
{"type":"extract_mesh","seq":5}
{"type":"save_mesh","path":"/tmp/scan.ply","seq":6}
{"type":"load_stl","path":"/path/to/object.stl","seq":7}
{"type":"get_status","seq":8}
{"type":"set_params","params":{"poseSource":"actuated_tcp"},"seq":9}
```

`set_params.poseSource` accepts `icp`, `actuated_tcp`, or `simulation`.

Common `set_params` fields:

```json
{
  "angleStartDeg": 0,
  "angleEndDeg": 360,
  "angleStepDeg": 10,
  "stageStartMm": 0,
  "stageEndMm": 100,
  "stageStepMm": 25,
  "framesPerPose": 1,
  "turntableRadiusMm": 150,
  "turntableHeightMm": 40,
  "kinectOffsetXMm": 0,
  "kinectOffsetYMm": -150,
  "kinectOffsetZMm": -1000,
  "kinectRollDeg": 0,
  "kinectPitchDeg": 0,
  "kinectYawDeg": 0,
  "stageAxisX": 0,
  "stageAxisY": -1,
  "stageAxisZ": 0,
  "actuatorTcpHost": "0.0.0.0",
  "actuatorTcpPort": 5055,
  "simStlPath": "/path/to/object.stl",
  "simDepthNoiseMm": 0,
  "simDropoutPercent": 0,
  "maxTruncation": 0.03,
  "depthEdgeThreshold": 0.02,
  "normalThreshold": 0.03
}
```

Replies are either:

```json
{"type":"ack","seq":1,"ok":true,"message":"start accepted"}
{"type":"error","seq":1,"code":"unknown_command","message":"..."}
{"type":"status","seq":8,"state":"idle","frame":0,"fps":0,"meshAvailable":false}
```

The UI also broadcasts event objects such as `scan_started`, `scan_stopped`,
`mesh_ready`, and `error`.

## Actuator server

Default endpoint: `0.0.0.0:5055`

The scanner listens as a TCP server. The actuator controller connects as a
client, reads target commands, moves the hardware, and publishes feedback.

Scanner-to-actuator command:

```json
{
  "type": "cmd",
  "seq": "1",
  "turntable_angle_deg": 90.0,
  "linear_stage_mm": 25.0,
  "joint_names": ["turntable_angle_deg", "linear_stage_mm"],
  "positions": [90.0, 25.0]
}
```

Actuator-to-scanner feedback:

```json
{
  "type": "state",
  "seq": 1,
  "turntable_angle_deg": 89.9,
  "linear_stage_mm": 25.1,
  "moving": false
}
```

The named-joint form is also accepted:

```json
{
  "type": "state",
  "joint_names": ["turntable_angle_deg", "linear_stage_mm"],
  "positions": [89.9, 25.1],
  "moving": false
}
```

The scanner integrates a frame after feedback is within `angleToleranceDeg`
and `stageToleranceMm`, `moving` is false, and `targetSettleMs` has elapsed.
