import math

from nuevo_bridge.mag_calibration import MagCalibrationController, MagCalibrationResult, fit_soft_iron_calibration


def _mat_vec_mul(matrix, vector):
    return (
        matrix[0] * vector[0] + matrix[1] * vector[1] + matrix[2] * vector[2],
        matrix[3] * vector[0] + matrix[4] * vector[1] + matrix[5] * vector[2],
        matrix[6] * vector[0] + matrix[7] * vector[1] + matrix[8] * vector[2],
    )


def _generate_samples():
    radius = 48.0
    offset = (8.0, -5.0, 3.0)
    distortion = (
        1.18, 0.12, 0.04,
        0.00, 0.86, -0.07,
        0.03, 0.05, 1.09,
    )
    samples = []
    for i in range(480):
        theta = (2.0 * math.pi * i) / 480.0
        phi = math.acos(1.0 - 2.0 * ((i + 0.5) / 480.0))
        sphere = (
            radius * math.sin(phi) * math.cos(theta),
            radius * math.sin(phi) * math.sin(theta),
            radius * math.cos(phi),
        )
        distorted = _mat_vec_mul(distortion, sphere)
        samples.append((
            distorted[0] + offset[0],
            distorted[1] + offset[1],
            distorted[2] + offset[2],
        ))
    return samples


def main() -> None:
    samples = _generate_samples()
    result = fit_soft_iron_calibration(samples)
    assert result is not None
    assert abs(result.offset[0] - 8.0) < 3.0
    assert abs(result.offset[1] + 5.0) < 2.0
    assert abs(result.offset[2] - 3.0) < 2.0
    assert (result.std_norm / result.mean_norm) < 0.2

    sent = []
    controller = MagCalibrationController(sender=lambda cmd, data: sent.append((cmd, data)) or True)
    controller.observe("sensor_mag_cal_status", {"state": 1})
    controller._start_time -= controller.MIN_DURATION_S + 1.0
    for sample in samples:
        controller.observe("sensor_imu", {"magX": sample[0], "magY": sample[1], "magZ": sample[2]})
    assert sent, "controller did not emit calibration apply command"
    cmd, data = sent[-1]
    assert cmd == "sensor_mag_cal_cmd"
    assert data["command"] == 4
    assert len(data["softIronMatrix"]) == 9

    sent = []
    controller = MagCalibrationController(sender=lambda cmd, data: sent.append((cmd, data)) or True)
    controller.observe("sensor_mag_cal_status", {"state": 1})
    controller._sampling = True
    controller._start_time -= controller.MIN_DURATION_S + 1.0
    controller._samples = list(samples[:-1])
    xs = [sample[0] for sample in controller._samples]
    ys = [sample[1] for sample in controller._samples]
    zs = [sample[2] for sample in controller._samples]
    controller._min = [min(xs), min(ys), min(zs)]
    controller._max = [max(xs), max(ys), max(zs)]

    tiny_growth_sample = (
        controller._max[0] + controller.SPAN_GROWTH_EPS_UT * 0.5,
        samples[-1][1],
        samples[-1][2],
    )
    controller.observe("sensor_imu", {"magX": tiny_growth_sample[0], "magY": tiny_growth_sample[1], "magZ": tiny_growth_sample[2]})
    assert sent, "controller did not emit calibration apply command after sub-epsilon span growth"
    cmd, data = sent[-1]
    assert cmd == "sensor_mag_cal_cmd"
    assert data["command"] == 4

    sent = []
    controller = MagCalibrationController(sender=lambda cmd, data: sent.append((cmd, data)) or True)
    controller.observe("sensor_mag_cal_status", {"state": 1})
    controller._sampling = True
    controller._start_time -= controller.MAX_DURATION_S + 1.0
    controller._samples = list(samples)
    xs = [sample[0] for sample in controller._samples]
    ys = [sample[1] for sample in controller._samples]
    zs = [sample[2] for sample in controller._samples]
    controller._min = [min(xs), min(ys), min(zs)]
    controller._max = [max(xs), max(ys), max(zs)]
    controller._best_result = fit_soft_iron_calibration(samples)
    assert controller._best_result is not None
    controller._best_std_ratio = controller._best_result.std_norm / controller._best_result.mean_norm
    controller.observe("sensor_imu", {"magX": samples[-1][0], "magY": samples[-1][1], "magZ": samples[-1][2]})
    assert sent, "controller did not apply best fit on timeout"
    cmd, data = sent[-1]
    assert cmd == "sensor_mag_cal_cmd"
    assert data["command"] == 4

    # Timeout fallback to hard-iron-only when full fit is not available
    sent = []
    controller = MagCalibrationController(sender=lambda cmd, data: sent.append((cmd, data)) or True)
    controller.observe("sensor_mag_cal_status", {"state": 1})
    controller._sampling = True
    controller._start_time -= controller.MAX_DURATION_S + 1.0
    planar_samples = [(20.0 + 0.2 * i, -15.0 + 0.1 * i, 2.0 + 0.05 * i) for i in range(160)]
    controller._samples = list(planar_samples)
    xs = [sample[0] for sample in controller._samples]
    ys = [sample[1] for sample in controller._samples]
    zs = [sample[2] for sample in controller._samples]
    controller._min = [min(xs), min(ys), min(zs)]
    controller._max = [max(xs), max(ys), max(zs)]
    controller.observe("sensor_imu", {"magX": planar_samples[-1][0], "magY": planar_samples[-1][1], "magZ": planar_samples[-1][2]})
    assert sent, "controller did not apply hard-iron fallback on timeout"
    cmd, data = sent[-1]
    assert cmd == "sensor_mag_cal_cmd"
    assert data["command"] == 4
    assert data["softIronMatrix"] == list((1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0))

    # Timeout should prefer hard-iron fallback over a poor soft-iron fit
    sent = []
    controller = MagCalibrationController(sender=lambda cmd, data: sent.append((cmd, data)) or True)
    controller.observe("sensor_mag_cal_status", {"state": 1})
    controller._sampling = True
    controller._start_time -= controller.MAX_DURATION_S + 1.0
    planar_samples = [(20.0 + 0.2 * i, -15.0 + 0.1 * i, 2.0 + 0.05 * i) for i in range(160)]
    controller._samples = list(planar_samples)
    xs = [sample[0] for sample in controller._samples]
    ys = [sample[1] for sample in controller._samples]
    zs = [sample[2] for sample in controller._samples]
    controller._min = [min(xs), min(ys), min(zs)]
    controller._max = [max(xs), max(ys), max(zs)]
    controller._best_result = MagCalibrationResult(
        offset=(1.0, 2.0, 3.0),
        matrix=(2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 2.0),
        span=(1.0, 1.0, 1.0),
        mean_norm=10.0,
        std_norm=10.0,
    )
    controller._best_std_ratio = controller.TIMEOUT_STD_RATIO + 0.25
    controller.observe("sensor_imu", {"magX": planar_samples[-1][0], "magY": planar_samples[-1][1], "magZ": planar_samples[-1][2]})
    assert sent, "controller did not fall back to hard-iron calibration when timeout fit quality was poor"
    cmd, data = sent[-1]
    assert cmd == "sensor_mag_cal_cmd"
    assert data["command"] == 4
    assert data["softIronMatrix"] == list((1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0))

    print("PASS: mag calibration fit")


if __name__ == "__main__":
    main()
