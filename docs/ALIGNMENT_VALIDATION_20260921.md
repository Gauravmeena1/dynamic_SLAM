# Alignment validation — `ec129_0911_bridge`

Validation date: 2026-09-21

## Inputs

- 3D run: `/home/acm/fungi/outputs_malong/ec129_0911_bridge`
- Kachaka poses: `pose2d_20260911.csv`
- Occupancy map: `kachaka_2d_ec129_0911.{png,yaml}`
- Output: `artifacts/alignment_ec129_0911_bridge/`

The RGB, camera-pose, trajectory, and run-stat streams all contain 910 frames. Alignment input construction matched 909 frames and discarded one frame outside the usable pose-log interval.

The pose log contains only one `map_id`; no map-save boundary can be inferred. The fit therefore uses the 909 matched frames as one continuous run.

## Sim(2) result

| Metric | Result | Acceptance / reference |
|---|---:|---:|
| Scale | 0.855173 | Healthy ma-long reference: 0.839 |
| Rotation | -77.145° | Coordinate-frame dependent |
| Translation | (+0.528, -0.179) m | Coordinate-frame dependent |
| Camera lever arm | 0.198 m | Healthy reference: 0.178 m |
| RMSE, all frames | 0.108 m | Must be ≤ 0.300 m |
| Median residual | 0.083 m | — |
| Inliers at 0.1 m | 577 / 909 (63.5%) | — |
| Inlier RMSE | 0.066 m | — |
| 95th percentile | 0.194 m | — |
| Maximum residual | 0.276 m | — |

Residual RMSE by chronological third:

- First: 0.132 m
- Middle: 0.089 m
- Last: 0.100 m

The final third does not degrade relative to the first, so there is no evidence of accumulating alignment drift in this run.

## Overlay review

The final overlay contains 11,691 Kachaka pose samples, 910 transformed camera poses, and 170 semantic instances. The green Kachaka trajectory and blue transformed SLAM trajectory follow the same loops and overall footprint; visible local offsets are consistent with the measured residual distribution.

Result: **PASS for 3D-to-2D trajectory alignment.**

## Occupancy-map coverage caveat

Semantic-instance centroids by occupancy-map pixel:

| Region | Instances |
|---|---:|
| Free | 61 |
| Occupied | 3 |
| Unknown | 100 |
| Outside map bounds | 6 |

This is not an alignment-fit failure: the trajectory fit passes, while the occupancy map is known to have incomplete coverage, especially on the east side. The current transform is suitable for the mapped portion of the room. Full-room semantic navigation should remain blocked until the Kachaka 2D map is rescanned and the alignment is rerun against that map.
