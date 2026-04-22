# Vision Model Data

The default ROS vision node model is an exported Ultralytics YOLO26n NCNN
model:

```text
ros2_ws/src/vision/data/yolo26n_ncnn_imgsz_640/
```

It was exported with image size 640 and is tracked in Git because the NCNN
runtime files are small enough for this project.

The ROS runtime loads `model.ncnn.param`, `model.ncnn.bin`, and
`metadata.yaml` directly with the `ncnn` Python package. It does not install or
import the Ultralytics Python package.

Default detection classes:

- `traffic light`
- `stop sign`
- `person`

Default runtime settings use `model_imgsz:=640`, `confidence_threshold:=0.25`,
`process_rate_hz:=5.0`, and `ncnn_threads:=4`.

To try another Ultralytics model, export it to NCNN outside the ROS runtime,
place the exported model folder under this directory, and launch the node with:

```bash
ros2 run vision vision_node --ros-args \
  -p model_path:=/ros2_ws/src/vision/data/<model_folder> \
  -p model_imgsz:=640
```

For all COCO classes, pass an empty class filter:

```bash
ros2 run vision vision_node --ros-args -p class_filter:=""
```

To check live output:

```bash
ros2 topic echo /vision/detections
```

Ignored local model files remain useful for experiments:

- `*.weights`
- `*.pt`
- `*.onnx`
- `*.engine`
