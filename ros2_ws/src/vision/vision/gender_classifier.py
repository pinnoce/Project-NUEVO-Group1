from __future__ import annotations

from pathlib import Path

import cv2
import ncnn
import numpy as np

from .model_utils import _load_ultralytics_metadata


GENDER_LABEL_MAP = {
    "class_a": "male",
    "class_b": "female",
}


class GenderNcnnClassifier:
    """NCNN runtime for the Ultralytics-exported YOLO-cls gender model.

    The .pt ships with placeholder class names class_a/class_b; this wrapper
    remaps them to male/female before returning a label.
    """

    input_name = "in0"
    output_name = "out0"

    def __init__(
        self,
        model_path: str | Path,
        input_size: int = 224,
        ncnn_threads: int = 0,
        female_threshold: float = 0.20, # lower this to make "female" stickier against frame-to-frame jitter; raise to require more confidence
    ) -> None:
        self.model_path = Path(model_path).expanduser()
        self.input_size = int(input_size)
        self.ncnn_threads = int(ncnn_threads)
        # Decision threshold on P(female). >= threshold → female, else male.
        # Default 0.5 reproduces pure argmax. Lower to make "female" stickier
        # against frame-to-frame jitter; raise to require more confidence.
        self.female_threshold = float(female_threshold)

        metadata = _load_ultralytics_metadata(self.model_path / "metadata.yaml")
        self.raw_names = metadata["names"]
        # Resolve which output index corresponds to "female" so the threshold
        # is applied to the right probability regardless of model export order.
        self._female_index = next(
            (idx for idx, name in self.raw_names.items()
             if GENDER_LABEL_MAP.get(str(name), str(name)) == "female"),
            None,
        )

        param_path = self.model_path / "model.ncnn.param"
        bin_path = self.model_path / "model.ncnn.bin"
        if not param_path.is_file() or not bin_path.is_file():
            raise FileNotFoundError(
                "NCNN model folder must contain model.ncnn.param and model.ncnn.bin: "
                f"{self.model_path}"
            )

        self._net = ncnn.Net()
        self._net.opt.use_vulkan_compute = False
        if self.ncnn_threads > 0:
            ncnn.set_omp_dynamic(0)
            ncnn.set_omp_num_threads(self.ncnn_threads)
            self._net.opt.num_threads = self.ncnn_threads
        self._net.load_param(str(param_path))
        self._net.load_model(str(bin_path))

    def predict(self, crop_bgr: np.ndarray) -> tuple[str, float]:
        if crop_bgr is None or crop_bgr.size == 0:
            return ("unknown", 0.0)

        prepped = _resize_shortest_edge_then_center_crop(crop_bgr, self.input_size)

        input_mat = ncnn.Mat.from_pixels(
            np.ascontiguousarray(prepped),
            ncnn.Mat.PixelType.PIXEL_BGR2RGB,
            self.input_size,
            self.input_size,
        )
        # Ultralytics YOLO-cls default normalization is just /255 (mean=0, std=1).
        input_mat.substract_mean_normalize(
            [0.0, 0.0, 0.0],
            [1.0 / 255.0, 1.0 / 255.0, 1.0 / 255.0],
        )

        with self._net.create_extractor() as extractor:
            extractor.input(self.input_name, input_mat)
            _, output = extractor.extract(self.output_name)

        # NCNN export already applies softmax in the graph — out0 is probabilities.
        probs = np.array(output).reshape(-1).astype(np.float32)
        if probs.size == 0:
            return ("unknown", 0.0)

        if self._female_index is not None and self._female_index < probs.size:
            prob_female = float(probs[self._female_index])
            if prob_female >= self.female_threshold:
                chosen_index = self._female_index
            else:
                chosen_index = next(
                    (idx for idx in range(probs.size) if idx != self._female_index),
                    int(np.argmax(probs)),
                )
        else:
            chosen_index = int(np.argmax(probs))
        raw_label = str(self.raw_names.get(chosen_index, f"class_{chosen_index}"))
        label = GENDER_LABEL_MAP.get(raw_label, raw_label)
        return (label, float(probs[chosen_index]))


def _resize_shortest_edge_then_center_crop(image_bgr: np.ndarray, size: int) -> np.ndarray:
    """Match torchvision T.Resize(size, BILINEAR) + T.CenterCrop(size) used by Ultralytics YOLO-cls."""
    original_height, original_width = image_bgr.shape[:2]
    if original_height == 0 or original_width == 0:
        return np.zeros((size, size, 3), dtype=image_bgr.dtype)

    scale = float(size) / float(min(original_height, original_width))
    resized_height = max(size, int(round(original_height * scale)))
    resized_width = max(size, int(round(original_width * scale)))
    resized = cv2.resize(
        image_bgr,
        (resized_width, resized_height),
        interpolation=cv2.INTER_LINEAR,
    )

    crop_y = (resized_height - size) // 2
    crop_x = (resized_width - size) // 2
    return resized[crop_y : crop_y + size, crop_x : crop_x + size]
