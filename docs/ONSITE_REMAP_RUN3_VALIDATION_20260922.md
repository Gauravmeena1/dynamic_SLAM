# onsite_remap_run3_20260915 validation

## Verdict

`/home/acm/fungi/outputs_malong/onsite_remap_run3_20260915` is suitable as a
3D semantic-map source for `robotic_agent` when the robot is localized in the
matching Kachaka map:

- map name: `自動建圖0915`
- map ID: `7ccec1a8-883d-410d-ab81-326dd7b32f98`

It must not be mixed with the currently observed `lab_0919` map
(`c2e40f1b-1e0c-48f5-9170-29f80bc6fc9b`). Loading the matching 0915 Kachaka
map, or producing a new alignment against `lab_0919`, is required before live
navigation.

## Input completeness

- RGB frames: 407/407
- depth frames: 407/407
- camera poses: 407/407
- deploy stages: 15/15
- submaps: 26
- loop closures: 20 accepted out of 22 candidates
- semantic instances after filtering: 164
- pose log and Kachaka map carry the same 0915 map ID

The semantic table includes five `bottle` instances, two `table` instances,
four `sofa` instances, and other room-scale landmarks.

## 3D-to-2D alignment

All 407 camera frames were paired with Kachaka poses.

| Metric | Result |
| --- | ---: |
| Sim(2) scale | 0.825938 |
| Rotation | -5.955876 deg |
| Translation | (0.702032, -0.760383) m |
| Lever arm norm | 0.1448 m |
| RMSE, all frames | 0.1039 m |
| Median residual | 0.0804 m |
| Inliers at 0.10 m | 258/407 |
| Inlier RMSE | 0.0635 m |
| Maximum residual | 0.2800 m |

The overlay shows the reconstructed camera path following the Kachaka path in
the mapped free-space region without a global rotation, reflection, or offset
failure. Local loop differences are consistent with the measured residuals.
The semantic projection places 158/164 instance centers inside the 2D map.

Artifacts:

- `artifacts/alignment_onsite_remap_run3_20260915/sim2_with_lever.yaml`
- `artifacts/alignment_onsite_remap_run3_20260915/overlay_alignment.png`
- `artifacts/alignment_onsite_remap_run3_20260915/onsite_remap_run3_20260915_instances_map.csv`

## robotic_agent compatibility

The standard exporter produced all three point-cloud NPZ files, the instance
table, alignment YAML, and object-query parameters. It also verified all 164
instance centers through both transform chains with a maximum difference of
`8.951e-16 m`.

Using the same native-map occupancy strategy as the validated 0911 bundle,
the production `select_goal_around_point` implementation found a free-space
navigation goal for every bottle instance (5/5):

| Instance | Goal found | Clearance | Stand-off |
| ---: | :---: | ---: | ---: |
| 54 | yes | 0.301 m | 0.200 m |
| 76 | yes | 0.285 m | 0.800 m |
| 124 | yes | 0.300 m | 0.900 m |
| 137 | yes | 0.300 m | 0.250 m |
| 157 | yes | 0.285 m | 0.300 m |

## Occupancy-map caveat

Do not use the point-cloud-derived occupancy generated during this validation.
Its floor fit warned about 7.3 cm/m tilt and 29.5 cm RMS, causing excessive
obstacle marking. Use the converted Kachaka native map for navigation-goal
selection:

- `artifacts/robotic_agent_onsite_remap_run3_20260915/occupancy_native_onsite_remap_run3_20260915.yaml`

The Kachaka navigation stack remains the authority for path planning and
collision avoidance.

## Remaining boundary

This validates map loading, semantic object localization, and navigation-goal
selection. It does not by itself validate the final camera re-detection,
grasp pose, arm motion, or successful physical pickup. A live bottle pick test
on the matching map is still required.
