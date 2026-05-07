#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import json
from pathlib import Path

import cv2
import numpy as np


# =========================
# Config
# =========================
SCRIPT_DIR = Path(__file__).resolve().parent
DATA_DIR = SCRIPT_DIR / "data"
IMAGE_DIR = DATA_DIR / "images"
CSV_PATH = DATA_DIR / "poses.csv"
OUTPUT_DIR = DATA_DIR / "handeye_output"

BOARD_COLS = 11   # inner corners per row
BOARD_ROWS = 8    # inner corners per col
SQUARE_SIZE = 0.03  # meter

# 这项非常关键：
# True  -> poses.csv 里的 pose 表示 base_link 下的末端位姿，即 ^bT_g
# False -> poses.csv 里的 pose 表示 gripper->base，需要先取逆
CSV_POSE_IS_BASE_TO_GRIPPER = True

# 至少多少组样本才开始算
MIN_SAMPLE_COUNT = 5


# =========================
# Basic math helpers
# =========================
def make_object_points(board_cols: int, board_rows: int, square_size: float) -> np.ndarray:
    objp = np.zeros((board_rows * board_cols, 3), dtype=np.float64)
    idx = 0
    for i in range(board_rows):
        for j in range(board_cols):
            objp[idx, 0] = j * square_size
            objp[idx, 1] = i * square_size
            objp[idx, 2] = 0.0
            idx += 1
    return objp


def quat_xyzw_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    n = np.linalg.norm(q)
    if n < 1e-12:
        raise ValueError("Quaternion norm is zero.")
    q /= n
    x, y, z, w = q

    R = np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
        [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ], dtype=np.float64)
    return R


def rot_to_quat_xyzw(R: np.ndarray) -> np.ndarray:
    # Returns [x, y, z, w]
    m = R.astype(np.float64)
    tr = np.trace(m)

    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (m[2, 1] - m[1, 2]) / S
        qy = (m[0, 2] - m[2, 0]) / S
        qz = (m[1, 0] - m[0, 1]) / S
    elif (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
        S = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        qw = (m[2, 1] - m[1, 2]) / S
        qx = 0.25 * S
        qy = (m[0, 1] + m[1, 0]) / S
        qz = (m[0, 2] + m[2, 0]) / S
    elif m[1, 1] > m[2, 2]:
        S = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        qw = (m[0, 2] - m[2, 0]) / S
        qx = (m[0, 1] + m[1, 0]) / S
        qy = 0.25 * S
        qz = (m[1, 2] + m[2, 1]) / S
    else:
        S = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        qw = (m[1, 0] - m[0, 1]) / S
        qx = (m[0, 2] + m[2, 0]) / S
        qy = (m[1, 2] + m[2, 1]) / S
        qz = 0.25 * S

    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    q /= np.linalg.norm(q)
    return q


def make_transform(R: np.ndarray, t_xyz) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t_xyz, dtype=np.float64).reshape(3)
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


# =========================
# Chessboard detection + PnP
# =========================
def detect_chessboard(image: np.ndarray, board_cols: int, board_rows: int):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    pattern_size = (board_cols, board_rows)

    found = False
    corners = None

    # Prefer SB if available
    if hasattr(cv2, "findChessboardCornersSB"):
        try:
            found, corners = cv2.findChessboardCornersSB(gray, pattern_size)
        except Exception:
            found = False
            corners = None

    if not found:
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
        if found:
            criteria = (
                cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                30,
                0.001
            )
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    return found, corners


def solve_target_to_camera(image_path: Path, K: np.ndarray, D: np.ndarray,
                           board_cols: int, board_rows: int, square_size: float):
    image = cv2.imread(str(image_path))
    if image is None:
        return False, None, None, None, f"Cannot read image: {image_path}"

    found, corners = detect_chessboard(image, board_cols, board_rows)
    if not found or corners is None:
        return False, None, None, None, f"Chessboard not found: {image_path.name}"

    objp = make_object_points(board_cols, board_rows, square_size)

    ok, rvec, tvec = cv2.solvePnP(
        objp,
        corners,
        K,
        D,
        flags=cv2.SOLVEPNP_ITERATIVE
    )
    if not ok:
        return False, None, None, None, f"solvePnP failed: {image_path.name}"

    R, _ = cv2.Rodrigues(rvec)
    return True, R.astype(np.float64), tvec.reshape(3, 1).astype(np.float64), corners, ""


# =========================
# Load CSV
# =========================
def load_rows(csv_path: Path):
    rows = []
    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def row_to_camera_params(row):
    fx = float(row["fx"])
    fy = float(row["fy"])
    cx = float(row["cx"])
    cy = float(row["cy"])

    K = np.array([
        [fx, 0.0, cx],
        [0.0, fy, cy],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)

    d = []
    for i in range(8):
        key = f"d{i}"
        d.append(float(row[key]) if key in row and row[key] != "" else 0.0)
    D = np.array(d, dtype=np.float64).reshape(-1, 1)
    return K, D


def row_to_T_bg(row):
    px = float(row["px"])
    py = float(row["py"])
    pz = float(row["pz"])
    qx = float(row["qx"])
    qy = float(row["qy"])
    qz = float(row["qz"])
    qw = float(row["qw"])

    R = quat_xyzw_to_rot(qx, qy, qz, qw)
    T = make_transform(R, [px, py, pz])

    if CSV_POSE_IS_BASE_TO_GRIPPER:
        return T   # ^bT_g
    else:
        return invert_transform(T)  # convert ^gT_b to ^bT_g


# =========================
# Evaluation
# =========================
def evaluate_method(samples, R_cam2gripper, t_cam2gripper):
    """
    Compute how consistent the fixed chessboard target appears in base frame:
    ^bT_t = ^bT_g * ^gT_c * ^cT_t
    """
    T_gc = make_transform(R_cam2gripper, t_cam2gripper.reshape(3))

    base_target_positions = []

    for s in samples:
        T_bg = s["T_bg"]            # ^bT_g
        T_ct = make_transform(s["R_target2cam"], s["t_target2cam"].reshape(3))  # ^cT_t
        T_bt = T_bg @ T_gc @ T_ct   # ^bT_t
        base_target_positions.append(T_bt[:3, 3].copy())

    pts = np.array(base_target_positions, dtype=np.float64)
    mean_xyz = pts.mean(axis=0)
    std_xyz = pts.std(axis=0)
    std_norm = float(np.linalg.norm(std_xyz))

    return {
        "mean_base_target_xyz": mean_xyz.tolist(),
        "std_base_target_xyz": std_xyz.tolist(),
        "std_norm": std_norm,
    }


# =========================
# Main
# =========================
def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    if not CSV_PATH.exists():
        raise FileNotFoundError(f"poses.csv not found: {CSV_PATH}")

    if not IMAGE_DIR.exists():
        raise FileNotFoundError(f"images dir not found: {IMAGE_DIR}")

    rows = load_rows(CSV_PATH)
    if len(rows) == 0:
        raise RuntimeError("poses.csv is empty.")

    print(f"[INFO] Loaded {len(rows)} rows from: {CSV_PATH}")
    print(f"[INFO] Using images dir: {IMAGE_DIR}")
    print(f"[INFO] preview/ is NOT used for calculation.")
    print(f"[INFO] __pycache__/ is NOT used.")

    usable_samples = []
    skipped = []

    for idx, row in enumerate(rows, start=1):
        image_name = row["image_name"]
        image_path = IMAGE_DIR / image_name

        if not image_path.exists():
            skipped.append(f"{idx}: image missing -> {image_name}")
            continue

        K, D = row_to_camera_params(row)
        T_bg = row_to_T_bg(row)

        ok, R_t2c, t_t2c, corners, msg = solve_target_to_camera(
            image_path, K, D, BOARD_COLS, BOARD_ROWS, SQUARE_SIZE
        )
        if not ok:
            skipped.append(f"{idx}: {msg}")
            continue

        usable_samples.append({
            "row_index": idx,
            "image_name": image_name,
            "K": K,
            "D": D,
            "T_bg": T_bg,  # ^bT_g
            "R_target2cam": R_t2c,  # ^cR_t
            "t_target2cam": t_t2c,  # ^ct_t
        })

    print(f"[INFO] Usable samples: {len(usable_samples)}")
    print(f"[INFO] Skipped samples: {len(skipped)}")
    if skipped:
        print("[INFO] Skipped detail:")
        for s in skipped:
            print("   ", s)

    if len(usable_samples) < MIN_SAMPLE_COUNT:
        raise RuntimeError(
            f"Too few usable samples: {len(usable_samples)}. "
            f"Need at least {MIN_SAMPLE_COUNT}."
        )

    # Prepare calibrateHandEye inputs
    R_gripper2base = []
    t_gripper2base = []
    R_target2cam = []
    t_target2cam = []

    for s in usable_samples:
        T_bg = s["T_bg"]
        R_gripper2base.append(T_bg[:3, :3].astype(np.float64))
        t_gripper2base.append(T_bg[:3, 3].reshape(3, 1).astype(np.float64))
        R_target2cam.append(s["R_target2cam"])
        t_target2cam.append(s["t_target2cam"])

    methods = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }

    # Some OpenCV builds also have ANDREFF
    if hasattr(cv2, "CALIB_HAND_EYE_ANDREFF"):
        methods["ANDREFF"] = cv2.CALIB_HAND_EYE_ANDREFF

    all_results = {}
    best_method = None
    best_std = None

    for method_name, method_flag in methods.items():
        try:
            R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
                R_gripper2base,
                t_gripper2base,
                R_target2cam,
                t_target2cam,
                method=method_flag
            )

            R_cam2gripper = np.asarray(R_cam2gripper, dtype=np.float64).reshape(3, 3)
            t_cam2gripper = np.asarray(t_cam2gripper, dtype=np.float64).reshape(3, 1)

            # Inverse: gripper -> camera
            R_gripper2cam = R_cam2gripper.T
            t_gripper2cam = -R_cam2gripper.T @ t_cam2gripper

            q_cam2gripper = rot_to_quat_xyzw(R_cam2gripper)
            q_gripper2cam = rot_to_quat_xyzw(R_gripper2cam)

            consistency = evaluate_method(usable_samples, R_cam2gripper, t_cam2gripper)

            result = {
                "method": method_name,
                "sample_count": len(usable_samples),
                "board_cols": BOARD_COLS,
                "board_rows": BOARD_ROWS,
                "square_size": SQUARE_SIZE,
                "csv_pose_is_base_to_gripper": CSV_POSE_IS_BASE_TO_GRIPPER,

                "R_cam2gripper": R_cam2gripper.tolist(),
                "t_cam2gripper": t_cam2gripper.reshape(3).tolist(),
                "q_cam2gripper_xyzw": q_cam2gripper.tolist(),

                "R_gripper2cam": R_gripper2cam.tolist(),
                "t_gripper2cam": t_gripper2cam.reshape(3).tolist(),
                "q_gripper2cam_xyzw": q_gripper2cam.tolist(),

                "consistency": consistency,
                "usable_images": [s["image_name"] for s in usable_samples],
            }

            all_results[method_name] = result

            out_path = OUTPUT_DIR / f"handeye_{method_name.lower()}.json"
            with open(out_path, "w", encoding="utf-8") as f:
                json.dump(result, f, indent=2, ensure_ascii=False)

            print(f"[OK] {method_name} -> {out_path}")
            print(f"     t_cam2gripper = {result['t_cam2gripper']}")
            print(f"     q_cam2gripper = {result['q_cam2gripper_xyzw']}")
            print(f"     consistency std norm = {consistency['std_norm']:.6f}")

            if best_std is None or consistency["std_norm"] < best_std:
                best_std = consistency["std_norm"]
                best_method = method_name

        except Exception as e:
            print(f"[WARN] {method_name} failed: {e}")

    if len(all_results) == 0:
        raise RuntimeError("All hand-eye methods failed.")

    summary = {
        "best_method": best_method,
        "best_std_norm": best_std,
        "methods": list(all_results.keys()),
        "notes": [
            "preview/ images are not used in calculation.",
            "__pycache__/ is not used.",
            "If result looks wrong, try setting CSV_POSE_IS_BASE_TO_GRIPPER = False and rerun.",
            "More samples (15~20) usually produce more stable results."
        ]
    }

    summary_path = OUTPUT_DIR / "summary.json"
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)

    print("")
    print("[DONE] Hand-eye calculation finished.")
    print(f"[DONE] Summary saved to: {summary_path}")
    print(f"[DONE] Best method: {best_method}, std_norm = {best_std:.6f}")


if __name__ == "__main__":
    main()