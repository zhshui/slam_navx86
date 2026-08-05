# MQ Global Route Geometry Design

## Goal

Make every global route MQ message internally consistent: `source`, `target`,
`path`, `path_length`, and `frame_id` must all describe the same ROS global
planner `Path`.

## Data Source

`/move_base/GlobalPlanner/plan` is the sole geometry source. `source` is the
first pose, `target` is the last pose, and both yaw values come from those
poses. The adapter must not use `state.last_goal` or a separate live TF lookup
for route endpoints.

## Long Paths

MQ messages contain at most 200 path points. Paths above that limit are sampled
at evenly distributed indices while always retaining the first and last pose.
`path_length` is computed from every pose in the original path, not from the
sampled representation.

## Change Detection

The route fingerprint includes every path position rounded to millimeter
precision, the frame ID, and both endpoint yaw values. A replan with unchanged
endpoints but changed intermediate geometry, frame, or target yaw therefore
produces a new route message.

## Protocol

The route body includes `frame_id`, sourced from `Path.header.frame_id` with
`map` as the fallback. Existing `ref_cmd_id` behavior remains unchanged: it is
consumed only by the first route whose endpoint matches the pending MQ goal.

## Error Handling

Invalid or non-finite path coordinates prevent route publication and emit a
throttled ROS warning. The adapter must never substitute `{0, 0, 0}` for a
failed endpoint lookup.

## Verification

Pure Python tests cover endpoint preservation, full-path length, internal
geometry fingerprinting, finite-value validation, and sampling limits. Existing
route reference tests and Python compilation checks remain required.
