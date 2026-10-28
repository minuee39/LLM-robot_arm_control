#!/usr/bin/env python3
"""Offline diagnostic candidates only; never publish a pose or change planning."""
import argparse
import json
import hashlib
import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vision.box_boundary import SquareBoundaryRejected, fit_square_boundary_candidate
from vision.box_plane import fit_top_plane_candidate
from vision.box_pose_samples import mask_depth_samples
from vision.hsv_block_detector import HsvBlockDetector


def analyze_capture(path, *, saturation, sim_render_contract_confirmed, opening_kernel_size=3,
                    plane_distance_limit_m=.0005):
    if sim_render_contract_confirmed is not True:
        raise ValueError("optical-Z/registered simulator RenderProduct contract confirmation required")
    path = Path(path)
    metadata = json.loads((path / "metadata.json").read_text())
    with np.load(path / "rgbd.npz", allow_pickle=False) as data:
        rgb, depth = data["rgb"].copy(), data["depth"].copy()
    report = analyze_arrays(rgb, depth, metadata, saturation=saturation,
                            sim_render_contract_confirmed=sim_render_contract_confirmed,
                            opening_kernel_size=opening_kernel_size,
                            plane_distance_limit_m=plane_distance_limit_m)
    report["capture_path"] = str(path.resolve())
    return report


def analyze_arrays(rgb, depth, metadata, *, saturation,
                   sim_render_contract_confirmed, opening_kernel_size=3, ransac_trials=2048,
                   plane_distance_limit_m=.0005):
    """Shared measured-array path for offline replay and live diagnostics."""
    if sim_render_contract_confirmed is not True:
        raise ValueError("optical-Z/registered simulator RenderProduct contract confirmation required")
    if (type(plane_distance_limit_m) not in (int, float)
            or not math.isfinite(plane_distance_limit_m)
            or not 0 < plane_distance_limit_m <= .0005):
        raise ValueError('plane distance limit must be finite within (0, 0.0005] metres')
    if (rgb.shape[:2] != depth.shape or depth.shape != (metadata["height"], metadata["width"])
            or any(metadata["d"]) or not np.allclose(np.asarray(metadata["r"]).reshape(3, 3), np.eye(3))
            or len({metadata["rgb_frame"], metadata["depth_frame"], metadata["camera_info_frame"]}) != 1):
        raise ValueError("incompatible RGB-D/CameraInfo dimensions, distortion, rectification or frames")
    limits = dict(distance_limit_m=plane_distance_limit_m, max_tilt_rad=.3, min_inliers=20,
                  min_inlier_fraction=.15, min_minor_span_m=.008, trials=ransac_trials)
    boundary_limits = dict(expected_side_m=.03, max_size_error_m=.003,
                           max_plane_residual_m=.0005, edge_band_m=.0015,
                           min_edge_support=6, min_edge_span_fraction=.6,
                           max_hull_residual_m=.0015)
    # Installed Isaac Camera.get_pointcloud uses raster centers (u+.5,v+.5)
    # with principal point width/2,height/2. Keep the raw CameraInfo K intact.
    pixel_offset = .5
    revision_data = dict(extrinsic_revision=metadata['calibration_revision'],
                         k=metadata['k'], d=metadata['d'], r=metadata['r'],
                         width=metadata['width'], height=metadata['height'],
                         pixel_center_offset_px=pixel_offset,
                         geometry_algorithm='top_plane_square_boundary_v2_pixel_center',
                         plane_limits=limits, boundary_limits=boundary_limits,
                         saturation=saturation, opening_kernel_size=opening_kernel_size)
    measurement_revision = hashlib.sha256(json.dumps(revision_data, sort_keys=True).encode()).hexdigest()
    results = []
    for detection in HsvBlockDetector(min_saturation=saturation,
                                      opening_kernel_size=opening_kernel_size).detect(rgb):
        result = dict(object_id=detection.name, bbox=list(detection.bbox),
                      diagnostic_candidate=False, collision_model_approved=False)
        results.append(result)
        try:
            samples = mask_depth_samples(
                depth, detection.mask, encoding=metadata["depth_encoding"], intrinsics=metadata["k"],
                optical_to_base_footprint=metadata["optical_to_base_footprint"],
                rgb_stamp=metadata["rgb_stamp"], depth_stamp=metadata["depth_stamp"],
                # Offline exposure-relative validation is not current motion freshness.
                now=max(metadata["rgb_stamp"], metadata["depth_stamp"]),
                max_age_s=.01, max_sync_delta_s=.01, stamp_clock=metadata["stamp_clock"],
                calibration_revision=metadata["calibration_revision"], mask_source="hsv_color_mask",
                registered_rectified=True, depth_semantics="optical_z",
                pixel_center_offset_px=pixel_offset)
            result.update(mask_pixels=samples.mask_pixel_count,
                          valid_depth_fraction=samples.valid_depth_fraction,
                          interior_points=int(samples.interior.sum()))
            median = np.median(samples.points_base_m, axis=0)
            if abs(median[0]) > .85 or abs(median[1]) > .35 or not -.02 < median[2] < .15:
                raise ValueError("outside Pallet tabletop diagnostic region")
            plane = fit_top_plane_candidate(samples, **limits)
            top = samples.points_base_m[
                np.abs((samples.points_base_m-plane.centroid_base_m) @ plane.normal_base) <= plane_distance_limit_m]
            boundary = fit_square_boundary_candidate(
                top, plane.normal_base, plane.centroid_base_m,
                image_edge_clipped=samples.image_edge_clipped, **boundary_limits)
            result.update(diagnostic_candidate=True,
                          top_center_base_m=boundary.top_center_base_m.tolist(),
                          box_center_candidate_m=(boundary.top_center_base_m-.025*plane.normal_base).tolist(),
                          rotation_base=boundary.rotation_base.tolist(),
                          normal_base=plane.normal_base.tolist(),
                          yaw_mod_quarter_turn_rad=boundary.yaw_mod_quarter_turn_rad,
                          observed_size_xy_m=boundary.observed_size_xy_m.tolist(),
                          edge_support_counts=list(boundary.edge_support_counts),
                          hull_residual_max_m=boundary.hull_residual_max_m,
                          plane_residual_rms_m=plane.residual_rms_m)
        except ValueError as error:
            result["rejected"] = str(error)
            if isinstance(error, SquareBoundaryRejected):
                result["rejected_diagnostics"] = error.diagnostics
    return dict(diagnostic_only=True, collision_model_approved=False,
                frame="base_footprint", unit="m",
                exposure_stamps={key: metadata[key] for key in ("rgb_stamp", "depth_stamp", "stamp_clock")},
                calibration_revision=measurement_revision,
                extrinsic_revision=metadata['calibration_revision'],
                pixel_center_offset_px=pixel_offset,
                current_motion_freshness="not_evaluated_offline",
                saturation=saturation, opening_kernel_size=opening_kernel_size,
                plane_limits=limits, boundary_limits=boundary_limits,
                box_height_m=.05, uncertainty="not_certified", detections=results)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--saturation", type=int, default=70)
    parser.add_argument("--confirmed-sim-render-contract", action="store_true")
    parser.add_argument("--opening-kernel-size", type=int, choices=(1, 3, 5, 7, 9), default=3)
    parser.add_argument('--plane-distance-limit-m', type=float, default=.0005,
                        help='Diagnostic inlier distance; may tighten but not exceed 0.0005 m.')
    args = parser.parse_args()
    print(json.dumps(analyze_capture(args.capture, saturation=args.saturation,
                                   sim_render_contract_confirmed=args.confirmed_sim_render_contract,
                                   opening_kernel_size=args.opening_kernel_size,
                                   plane_distance_limit_m=args.plane_distance_limit_m), indent=2))


if __name__ == "__main__":
    main()
