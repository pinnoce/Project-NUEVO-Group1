# Vision Node Plan

The ROS vision node uses an exported Ultralytics YOLO26n NCNN model. The old
Darknet/YOLOv4 path is removed, and the ROS runtime loads NCNN directly
without importing Ultralytics or Torch.

## Runtime Model

Default model:

```text
ros2_ws/src/vision/data/yolo26n_ncnn_imgsz_640/
```

The model is exported at image size 640 and is tracked in Git because the NCNN
runtime files are small enough for the project repository.

Default runtime parameters:

- `model_path`: exported NCNN model directory
- `model_imgsz`: `640`
- `class_filter`: `traffic light,stop sign,person`
- `confidence_threshold`: `0.25`
- `process_rate_hz`: `5.0`
- `ncnn_threads`: `4`
- `camera_device`: `/dev/video10`
- `camera_width`: `640`
- `camera_height`: `480`

Set `class_filter` to an empty string to publish all model classes:

```bash
ros2 run vision vision_node --ros-args -p class_filter:=""
```

Echo the detection topic from another shell inside the ROS container:

```bash
ros2 topic echo /vision/detections
```

## Message Contract

Topic:

```text
/vision/detections
```

Message type:

```text
bridge_interfaces/msg/VisionDetectionArray
```

Behavior:

- one message is published per processed frame
- empty frames publish an empty `detections` list
- `class_name` uses the Ultralytics/COCO class name, for example
  `traffic light`, `stop sign`, or `person`
- bounding boxes use pixel coordinates in the captured image
- traffic-light detections include a `color` attribute when color
  classification is enabled

## Student Extension Points

The main processing loop keeps object-specific logic visible:

- `traffic light`: classifies red/green/other and attaches a `color` attribute
- `face`: placeholder for student face/customer analysis with a custom model
- `my_object`: placeholder for student custom object-specific checks

Students can add their own object logic by filtering on `detection.class_name`
and attaching attributes with:

```python
detection.add_attribute("name", "value", score)
```

## Trying Another Ultralytics Model

Export the model to NCNN outside the ROS node, then put the exported folder
under:

```text
ros2_ws/src/vision/data/
```

Launch with:

```bash
ros2 run vision vision_node --ros-args \
  -p model_path:=/ros2_ws/src/vision/data/<model_folder> \
  -p model_imgsz:=640 \
  -p class_filter:="traffic light,stop sign,person"
```

The benchmark and export scripts under `vision_model_benchmark/` are kept as a
native Ubuntu reference workflow and are intentionally separate from the ROS
package.
