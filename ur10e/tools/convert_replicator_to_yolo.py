import argparse
import json
import shutil
from pathlib import Path


SOURCE_BLOCK_NAMES = {"red_block", "blue_block", "green_block", "block"}
YOLO_CLASS_NAME = "block"
IMAGE_EXTENSIONS = {".png", ".jpg", ".jpeg"}
BBOX_KEYS = ("bounding_box_2d_tight", "boundingBox2DTight", "bbox_2d_tight")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input-dir",
        default=str(Path(__file__).resolve().parents[1] / "datasets" / "block_synthetic" / "raw_replicator"),
        help="Replicator BasicWriter output directory",
    )
    parser.add_argument(
        "--output-dir",
        default=str(Path(__file__).resolve().parents[1] / "datasets" / "block_synthetic" / "yolo"),
        help="YOLO dataset output directory",
    )
    parser.add_argument("--train-ratio", type=float, default=0.8)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    return parser.parse_args()


def iter_images(input_dir):
    for path in sorted(input_dir.rglob("*")):
        if path.suffix.lower() in IMAGE_EXTENSIONS and "rgb" in path.stem.lower():
            yield path


def frame_id(path):
    digits = "".join(ch for ch in path.stem if ch.isdigit())
    return digits or path.stem


def find_bbox_file(input_dir, image_path):
    image_frame_id = frame_id(image_path)
    candidates = []
    for path in input_dir.rglob("*"):
        if path.suffix.lower() not in {".json", ".npy"}:
            continue
        name = path.name.lower()
        if "bounding" not in name and "bbox" not in name:
            continue
        if "label" in name:
            continue
        if image_frame_id in frame_id(path) or frame_id(path) in image_frame_id:
            candidates.append(path)
    if candidates:
        return sorted(candidates, key=lambda item: (item.suffix.lower() != ".npy", item.name))[0]
    return None


def find_label_map_file(input_dir, bbox_path):
    bbox_frame_id = frame_id(bbox_path)
    candidates = []
    for path in input_dir.rglob("*.json"):
        name = path.name.lower()
        if "bounding" not in name and "bbox" not in name:
            continue
        if "label" not in name and "info" not in name:
            continue
        if bbox_frame_id in frame_id(path) or frame_id(path) in bbox_frame_id:
            candidates.append(path)
    if candidates:
        return sorted(candidates)[0]
    return None


def load_label_map(path):
    if path is None:
        return {}
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    return extract_label_map(data)


def extract_label_map(data):
    if isinstance(data, dict):
        label_map = {}
        for key, value in data.items():
            key_label = key if key in SOURCE_BLOCK_NAMES else None
            value_label = None
            if isinstance(value, str):
                value_label = value
            elif isinstance(value, dict):
                value_label = label_from_record(value)

            label = key_label or value_label
            if label in SOURCE_BLOCK_NAMES:
                try:
                    label_map[int(key)] = label
                except ValueError:
                    semantic_id = value.get("semanticId") if isinstance(value, dict) else None
                    if semantic_id is not None:
                        label_map[int(semantic_id)] = label

            nested = extract_label_map(value)
            label_map.update(nested)
        return label_map

    if isinstance(data, list):
        label_map = {}
        for item in data:
            if isinstance(item, dict):
                label = label_from_record(item)
                semantic_id = item.get("semanticId", item.get("semantic_id", item.get("id")))
                if label in SOURCE_BLOCK_NAMES and semantic_id is not None:
                    label_map[int(semantic_id)] = label
                label_map.update(extract_label_map(item))
        return label_map

    return {}


def load_bbox_records(path):
    if path.suffix.lower() == ".json":
        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)
        return extract_records(data)

    if path.suffix.lower() == ".npy":
        import numpy as np

        data = np.load(path, allow_pickle=True)
        if data.dtype.names:
            return [
                {field_name: row[field_name].item() for field_name in data.dtype.names}
                for row in data
            ]
        return extract_records(data.tolist())

    return []


def extract_records(data):
    if isinstance(data, dict):
        for key in BBOX_KEYS:
            if key in data:
                return extract_records(data[key])
        if {"x_min", "y_min", "x_max", "y_max"} <= set(data):
            return [data]
        if {"x_min", "y_min", "width", "height"} <= set(data):
            return [data]
        records = []
        for value in data.values():
            records.extend(extract_records(value))
        return records

    if isinstance(data, list):
        records = []
        for item in data:
            records.extend(extract_records(item))
        return records

    return []


def label_from_record(record):
    for key in ("semanticLabel", "semantic_label", "label", "class", "name"):
        value = record.get(key)
        if isinstance(value, str):
            return value

    for key in ("primPath", "prim_path", "path"):
        value = record.get(key)
        if isinstance(value, str):
            for class_name in SOURCE_BLOCK_NAMES:
                if class_name in value:
                    return class_name

    return None


def record_semantic_id(record):
    for key in ("semanticId", "semantic_id", "id"):
        if key in record:
            return int(record[key])
    return None


def bbox_from_record(record):
    if {"x_min", "y_min", "x_max", "y_max"} <= set(record):
        return float(record["x_min"]), float(record["y_min"]), float(record["x_max"]), float(record["y_max"])

    if {"xmin", "ymin", "xmax", "ymax"} <= set(record):
        return float(record["xmin"]), float(record["ymin"]), float(record["xmax"]), float(record["ymax"])

    if {"x_min", "y_min", "width", "height"} <= set(record):
        x_min = float(record["x_min"])
        y_min = float(record["y_min"])
        return x_min, y_min, x_min + float(record["width"]), y_min + float(record["height"])

    if "bbox" in record and len(record["bbox"]) == 4:
        x_min, y_min, x_max, y_max = record["bbox"]
        return float(x_min), float(y_min), float(x_max), float(y_max)

    return None


def yolo_line(record, label_map, image_width, image_height):
    semantic_id = record_semantic_id(record)
    label = label_from_record(record)
    if label is None and semantic_id is not None:
        label = label_map.get(semantic_id)
    if label not in SOURCE_BLOCK_NAMES:
        return None

    bbox = bbox_from_record(record)
    if bbox is None:
        return None

    x_min, y_min, x_max, y_max = bbox
    x_min = max(0.0, min(float(image_width), x_min))
    x_max = max(0.0, min(float(image_width), x_max))
    y_min = max(0.0, min(float(image_height), y_min))
    y_max = max(0.0, min(float(image_height), y_max))

    width = x_max - x_min
    height = y_max - y_min
    if width <= 1.0 or height <= 1.0:
        return None

    x_center = (x_min + x_max) / 2.0 / image_width
    y_center = (y_min + y_max) / 2.0 / image_height
    norm_width = width / image_width
    norm_height = height / image_height
    return f"0 {x_center:.6f} {y_center:.6f} {norm_width:.6f} {norm_height:.6f}"


def write_dataset_yaml(output_dir):
    yaml_path = output_dir / "block.yaml"
    yaml_path.write_text(
        "\n".join(
            [
                f"path: {output_dir}",
                "train: images/train",
                "val: images/val",
                "names:",
                f"  0: {YOLO_CLASS_NAME}",
                "",
            ]
        ),
        encoding="utf-8",
    )


def convert(input_dir, output_dir, train_ratio, image_width, image_height):
    images = list(iter_images(input_dir))
    if not images:
        raise RuntimeError(f"No RGB images found under {input_dir}")

    for split in ("train", "val"):
        (output_dir / "images" / split).mkdir(parents=True, exist_ok=True)
        (output_dir / "labels" / split).mkdir(parents=True, exist_ok=True)

    converted = 0
    skipped = []
    train_count = int(len(images) * train_ratio)

    for index, image_path in enumerate(images):
        split = "train" if index < train_count else "val"
        bbox_path = find_bbox_file(input_dir, image_path)
        if bbox_path is None:
            skipped.append((image_path, "bbox file not found"))
            continue

        label_map = load_label_map(find_label_map_file(input_dir, bbox_path))
        records = load_bbox_records(bbox_path)
        lines = [
            line
            for line in (yolo_line(record, label_map, image_width, image_height) for record in records)
            if line is not None
        ]
        if not lines:
            skipped.append((image_path, f"no usable labels in {bbox_path.name}"))
            continue

        target_image = output_dir / "images" / split / image_path.name
        target_label = output_dir / "labels" / split / f"{image_path.stem}.txt"
        shutil.copy2(image_path, target_image)
        target_label.write_text("\n".join(lines) + "\n", encoding="utf-8")
        converted += 1

    write_dataset_yaml(output_dir)
    return converted, skipped


def main():
    args = parse_args()
    input_dir = Path(args.input_dir).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    converted, skipped = convert(input_dir, output_dir, args.train_ratio, args.width, args.height)
    print(f"[DONE] Converted {converted} images to YOLO format")
    print(f"[INFO] YOLO dataset: {output_dir}")
    print(f"[INFO] Dataset yaml: {output_dir / 'block.yaml'}")
    if skipped:
        print(f"[WARN] Skipped {len(skipped)} images")
        for image_path, reason in skipped[:10]:
            print(f"  {image_path.name}: {reason}")


if __name__ == "__main__":
    main()
