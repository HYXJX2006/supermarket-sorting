#!/usr/bin/env python3
"""多类别商品 YOLO 数据生成器的冒烟版。

仅用于离线生成训练数据：标签来自仿真中的世界坐标和深度门控，
不参与比赛运行时决策，也不把固定商品位置写入运行程序。
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np

# official gen_dataset.py supplies the simulator, camera projection and DR helpers.
PERCEPTION_DIR = Path("/workspace/supermarket_sorting_task/examples/supermarket_sorting/perception")
sys.path.insert(0, str(PERCEPTION_DIR))
import gen_dataset as gd  # noqa: E402


CLASS_NAMES = [
    "chengzi",
    "heweidao",
    "kele",
    "kouxiangtang",
    "maidong",
    "pingguo",
    "sanmingzhi",
    "shupian",
    "zhijin",
]
CLASS_TO_ID = {name: i for i, name in enumerate(CLASS_NAMES)}

# Conservative approximate half extents for projected training boxes.
# These are only dataset-label geometry, not grasp dimensions.
BOX_HALF = {
    "chengzi": np.array([0.045, 0.045, 0.050]),
    "pingguo": np.array([0.045, 0.045, 0.050]),
    "kele": np.array([0.035, 0.035, 0.080]),
    "maidong": np.array([0.035, 0.035, 0.080]),
    "heweidao": np.array([0.045, 0.045, 0.075]),
    "sanmingzhi": np.array([0.045, 0.035, 0.070]),
    "shupian": np.array([0.050, 0.035, 0.070]),
    "zhijin": np.array([0.045, 0.035, 0.070]),
    "kouxiangtang": np.array([0.045, 0.035, 0.070]),
}


SHELF_SURFACE = {"L1": 0.499, "L2": 0.851, "L3": 1.189}


def randomize_layout(raw_layout, rng):
    """随机交换 45 个商品与 45 个货架槽位，并返回 XML 覆盖位置。

    商品 body 名称和对应 3DGS 资产保持不变，只交换它们占据的槽位；
    这样生成的标签与 Server 的随机换位语义一致。
    """
    slots = [
        (slot["world_position"][0], slot["world_position"][1], slot["level"])
        for slot in raw_layout
    ]
    half_heights = [
        slot["world_position"][2] - SHELF_SURFACE[slot["level"]]
        for slot in raw_layout
    ]
    body_indices = list(range(len(raw_layout)))
    slot_indices = list(range(len(raw_layout)))
    rng.shuffle(slot_indices)
    fixed = [i for i, (body_i, slot_i) in enumerate(zip(body_indices, slot_indices))
             if body_i == slot_i]
    if len(fixed) == 1 and len(slot_indices) > 1:
        i = fixed[0]
        j = 0 if i != 0 else 1
        slot_indices[i], slot_indices[j] = slot_indices[j], slot_indices[i]
    elif len(fixed) > 1:
        fixed_slots = [slot_indices[i] for i in fixed]
        fixed_slots = fixed_slots[1:] + fixed_slots[:1]
        for i, slot_i in zip(fixed, fixed_slots):
            slot_indices[i] = slot_i

    new_layout = []
    position_overrides = {}
    for body_i, slot_i in enumerate(slot_indices):
        x, y, level = slots[slot_i]
        z = SHELF_SURFACE[level] + half_heights[body_i]
        body = raw_layout[body_i]["body"]
        position_overrides[body] = (x, y, z)
        item = raw_layout[body_i].copy()
        for key in ("shelf", "level", "column", "aruco_id"):
            item[key] = raw_layout[slot_i][key]
        item["world_position"] = [x, y, z]
        new_layout.append(item)
    return new_layout, position_overrides


def _write_runtime_xml(position_overrides, variant_index):
    text = gd.SOURCE_XML.read_text().replace("__REPO_ROOT__", str(gd.TASK_DIR))
    for body_name, (x, y, z) in position_overrides.items():
        pattern = re.compile(
            r'(<body name="' + re.escape(body_name) + r'"[^>]*?pos=")[^"]*(")'
        )
        text, count = pattern.subn(rf"\g<1>{x:.5f} {y:.5f} {z:.5f}\g<2>", text)
        if count != 1:
            raise RuntimeError(
                f"random dataset: expected one XML body for {body_name}, got {count}"
            )
    path = Path(f"/tmp/retail_competition_dataset_runtime_{variant_index}.xml")
    path.write_text(text, encoding="utf-8")
    return path


def build_sim(layout, position_overrides, variant_index):
    """Build one headless 3DGS simulator for one randomized product layout."""
    cfg = gd.MMK2Cfg()
    cfg.mjcf_file_path = str(_write_runtime_xml(position_overrides, variant_index))
    cfg.use_gaussian_renderer = True
    cfg.enable_render = True
    cfg.headless = True
    cfg.obj_list = [slot["body"] for slot in layout]
    cfg.gs_model_dict = gd._local_robot_gs_model_dict()
    cfg.gs_model_dict["background"] = gd._resolve_background_ply()
    for slot in layout:
        cfg.gs_model_dict[slot["body"]] = slot["gs_ply"]
    cfg.obs_rgb_cam_id = [gd.HEAD_CAM_ID]
    cfg.obs_depth_cam_id = [gd.HEAD_CAM_ID]
    cfg.lidar_s2_sim = False
    cfg.render_set = {"fps": 24, "width": gd.IMG_W, "height": gd.IMG_H}
    sim = gd.MMK2Base(cfg)
    sim.reset()
    return sim


def projected_bbox(slot, K, T_cw):
    half = BOX_HALF[slot["object_kind"]]
    return gd._projected_bbox(slot["world_position"], K, T_cw, half=half)


def label_slots(depth_m, K, T_cw, slots):
    boxes = []
    for slot in slots:
        proj = gd.project_world_to_px(slot["world_position"], K, T_cw)
        if proj is None:
            continue
        u, v, z = proj
        ui, vi = int(round(u)), int(round(v))
        if not (0 <= ui < gd.IMG_W and 0 <= vi < gd.IMG_H):
            continue
        depth = gd._patch_depth(depth_m, ui, vi)
        if depth <= 0.0 or abs(depth - z) > gd.DEPTH_TOL:
            continue
        box = projected_bbox(slot, K, T_cw)
        if box is not None:
            boxes.append((CLASS_TO_ID[slot["object_kind"]], *box))
    return boxes


def boxes_to_yolo(boxes):
    lines = []
    for class_id, x0, y0, x1, y1 in boxes:
        cx = (x0 + x1) / 2.0 / gd.IMG_W
        cy = (y0 + y1) / 2.0 / gd.IMG_H
        bw = (x1 - x0) / gd.IMG_W
        bh = (y1 - y0) / gd.IMG_H
        lines.append(f"{class_id} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")
    return "\n".join(lines)


def save_sample(out_dir, split, name, image, boxes):
    cv2.imwrite(str(out_dir / "images" / split / f"{name}.jpg"), image)
    (out_dir / "labels" / split / f"{name}.txt").write_text(
        boxes_to_yolo(boxes), encoding="utf-8"
    )


def main():
    parser = argparse.ArgumentParser(description="生成九类商品 YOLO 数据集冒烟样本")
    parser.add_argument("--frames", type=int, default=8)
    parser.add_argument("--variants", type=int, default=0)
    parser.add_argument("--pose-mode", choices=("baseline", "wide"), default="wide")
    parser.add_argument("--out", default="/workspace/baseline/debug_data/multiclass_dataset_smoke")
    parser.add_argument("--seed", type=int, default=11)
    parser.add_argument("--random-layout", action="store_true",
                        help="按 Server 语义随机交换商品与货架槽位")
    parser.add_argument("--layout-variants", type=int, default=1,
                        help="生成多少个独立随机商品布局；启用后每个布局均重建一次仿真器")
    args = parser.parse_args()

    rng = np.random.default_rng(args.seed)
    out_dir = Path(args.out)
    if out_dir.exists():
        shutil.rmtree(out_dir)
    for split in ("train", "val"):
        (out_dir / "images" / split).mkdir(parents=True, exist_ok=True)
        (out_dir / "labels" / split).mkdir(parents=True, exist_ok=True)
    (out_dir / "debug").mkdir(parents=True, exist_ok=True)

    K = gd.head_cam_K()
    raw_layout = json.loads(gd.LAYOUT_JSON.read_text())
    random_layout = bool(args.random_layout or args.layout_variants > 1)
    layout_variants = max(1, int(args.layout_variants)) if random_layout else 1
    frames_per_layout = int(np.ceil(args.frames / layout_variants))
    print(f"[multi-gen] classes={CLASS_NAMES}", flush=True)
    print(f"[multi-gen] slots={len(raw_layout)} frames={args.frames} "
          f"random_layout={random_layout} layout_variants={layout_variants}", flush=True)

    frames = 0
    images = 0
    boxes_total = 0
    class_counts = {name: 0 for name in CLASS_NAMES}
    attempts = 0
    for layout_index in range(layout_variants):
        if random_layout:
            layout, position_overrides = randomize_layout(raw_layout, rng)
        else:
            layout, position_overrides = raw_layout, {}
        sim = build_sim(layout, position_overrides, layout_index)
        layout_frames = 0
        while (frames < args.frames and layout_frames < frames_per_layout
               and attempts < args.frames * 8):
            attempts += 1
            base_xy, yaw, slide, pitch = gd.sample_pose(rng, args.pose_mode)
            gd.set_robot_pose(sim, base_xy, yaw, slide, pitch)
            sim.render()
            rgb = sim.img_rgb_obs_s[gd.HEAD_CAM_ID]
            depth_m = sim.img_depth_obs_s[gd.HEAD_CAM_ID]
            T_cw = gd.T_cam_world(sim)
            boxes = label_slots(depth_m, K, T_cw, layout)
            if not boxes:
                continue

            rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            split = "val" if rng.random() < 0.15 else "train"
            base = f"f{frames:04d}_l{layout_index:02d}"
            variants = [rgb_bgr] + gd.domain_randomise(rng, rgb_bgr, args.variants)
            for vi, image in enumerate(variants):
                save_sample(out_dir, split, f"{base}_v{vi}", image, boxes)
                images += 1
            debug = rgb_bgr.copy()
            for class_id, x0, y0, x1, y1 in boxes:
                cv2.rectangle(debug, (x0, y0), (x1, y1), (0, 255, 0), 1)
                cv2.putText(debug, CLASS_NAMES[class_id], (x0, max(12, y0 - 3)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)
                class_counts[CLASS_NAMES[class_id]] += 1
            cv2.imwrite(str(out_dir / "debug" / f"{base}.jpg"), debug)
            frames += 1
            layout_frames += 1
            boxes_total += len(boxes)
            print(f"[multi-gen] layout={layout_index + 1}/{layout_variants} "
                  f"frame={frames} boxes={len(boxes)}", flush=True)
        del sim

    (out_dir / "data.yaml").write_text(
        "path: " + str(out_dir) + "\n"
        "train: images/train\n"
        "val: images/val\n"
        f"nc: {len(CLASS_NAMES)}\n"
        "names: [" + ", ".join(CLASS_NAMES) + "]\n",
        encoding="utf-8",
    )
    summary = {
        "classes": CLASS_NAMES,
        "frames": frames,
        "images": images,
        "boxes": boxes_total,
        "class_counts": class_counts,
        "attempts": attempts,
        "random_layout": random_layout,
        "layout_variants": layout_variants,
        "pose_mode": args.pose_mode,
        "seed": args.seed,
    }
    (out_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print("[multi-gen] done " + json.dumps(summary, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
