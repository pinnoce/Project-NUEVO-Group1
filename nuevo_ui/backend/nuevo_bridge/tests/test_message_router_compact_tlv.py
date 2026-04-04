import ctypes

from nuevo_bridge.TLV_TypeDefs import DC_ENABLE, DC_HOME, DC_RESET_POSITION, SENSOR_MAG_CAL_CMD, SERVO_STATE_ALL, SYS_DIAG_RSP, SYS_INFO_RSP, SYS_ODOM_PARAM_RSP, SYS_POWER, SYS_STATE
from nuevo_bridge.message_router import MessageRouter
from nuevo_bridge.payloads import PayloadDCEnable, PayloadDCHome, PayloadDCResetPosition, PayloadMagCalCmd, PayloadServoStateAll, PayloadSysDiagRsp, PayloadSysInfoRsp, PayloadSysOdomParamRsp, PayloadSysPower, PayloadSysState
from tlvcodec import DecodeErrorCode, Decoder, Encoder


class _DummyWsManager:
    connections = []


def main() -> None:
    router = MessageRouter(_DummyWsManager())
    messages = []

    def callback(error_code, frame_header, tlv_list):
        assert error_code == DecodeErrorCode.NoError
        assert frame_header.numTlvs == 2
        for tlv_type, _tlv_len, tlv_data in tlv_list:
            decoded = router.decode_incoming(tlv_type, tlv_data)
            assert decoded is not None
            if isinstance(decoded, list):
                messages.extend(decoded)
            else:
                messages.append(decoded)

    status = PayloadSysState()
    status.state = 2
    status.warningFlags = 0
    status.errorFlags = 0
    status.runtimeFlags = 0x01
    status.uptimeMs = 1234
    status.lastRxMs = 20
    status.lastCmdMs = 25

    voltage = PayloadSysPower()
    voltage.batteryMv = 12100
    voltage.rail5vMv = 5000
    voltage.servoRailMv = 6000
    voltage.batteryType = 2
    voltage.reserved = 0
    voltage.timestamp = 1234

    encoder = Encoder(deviceId=1, crc=True)
    encoder.addPacket(SYS_STATE, ctypes.sizeof(status), status)
    encoder.addPacket(SYS_POWER, ctypes.sizeof(voltage), voltage)
    length, buffer = encoder.wrapupBuffer()

    decoder = Decoder(callback=callback, crc=True)
    decoder.decode(buffer[:length])

    assert len(messages) == 2
    assert messages[0]["topic"] == "sys_state"
    assert messages[0]["data"]["state"] == 2
    assert messages[1]["topic"] == "sys_power"
    assert messages[1]["data"]["servoRailMv"] == 6000
    assert messages[1]["data"]["batteryType"] == 2

    outgoing = router.handle_outgoing("dc_enable", {"motorNumber": 1, "mode": 2})
    assert outgoing is not None
    tlv_type, payload = outgoing
    assert tlv_type == DC_ENABLE
    assert isinstance(payload, PayloadDCEnable)
    assert payload.motorId == 0
    assert payload.mode == 2

    outgoing = router.handle_outgoing("dc_reset_position", {"motorNumber": 2})
    assert outgoing is not None
    tlv_type, payload = outgoing
    assert tlv_type == DC_RESET_POSITION
    assert isinstance(payload, PayloadDCResetPosition)
    assert payload.motorId == 1

    outgoing = router.handle_outgoing("dc_home", {"motorNumber": 3, "direction": 1, "homeVelocity": 320})
    assert outgoing is not None
    tlv_type, payload = outgoing
    assert tlv_type == DC_HOME
    assert isinstance(payload, PayloadDCHome)
    assert payload.motorId == 2
    assert payload.direction == 1
    assert payload.homeVelocity == 320

    outgoing = router.handle_outgoing("sensor_mag_cal_cmd", {"command": 4, "offsetX": 1.0, "offsetY": -2.0, "offsetZ": 3.0})
    assert outgoing is not None
    tlv_type, payload = outgoing
    assert tlv_type == SENSOR_MAG_CAL_CMD
    assert isinstance(payload, PayloadMagCalCmd)
    assert payload.offsetX == 1.0
    assert payload.offsetY == -2.0
    assert payload.offsetZ == 3.0
    assert [payload.softIronMatrix[i] for i in range(9)] == [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

    info = PayloadSysInfoRsp()
    info.sensorCapabilityMask = 0x01
    info.dcHomeLimitGpio[0] = 40
    decoded = router.decode_incoming(SYS_INFO_RSP, bytes(info))
    assert decoded is not None

    diag = PayloadSysDiagRsp()
    diag.uartRxErrors = 7
    decoded = router.decode_incoming(SYS_DIAG_RSP, bytes(diag))
    assert decoded is not None

    odom = PayloadSysOdomParamRsp()
    odom.wheelDiameterMm = 80.0
    odom.wheelBaseMm = 300.0
    odom.initialThetaDeg = 45.0
    odom.leftMotorId = 1
    odom.leftMotorDirInverted = 1
    odom.rightMotorId = 3
    odom.rightMotorDirInverted = 0
    decoded = router.decode_incoming(SYS_ODOM_PARAM_RSP, bytes(odom))
    assert decoded is not None

    servo = PayloadServoStateAll()
    servo.pca9685Connected = 1
    servo.pca9685Error = 0
    servo.enabledMask = 0b0000000000000101
    servo.pulseUs[0] = 1500
    servo.pulseUs[2] = 1700
    decoded = router.decode_incoming(SERVO_STATE_ALL, bytes(servo))
    assert decoded is not None
    if isinstance(decoded, list):
        servo_message = decoded[0]
    else:
        servo_message = decoded
    servo_data = servo_message["data"]
    assert servo_data["enabledMask"] == 0b0000000000000101
    assert servo_data["channels"][0]["enabled"] is True
    assert servo_data["channels"][1]["enabled"] is False
    assert servo_data["channels"][2]["enabled"] is True

    cached = router.get_cached_ws_messages()
    topics = [message["topic"] for message in cached]
    assert "sys_info_rsp" in topics
    assert "sys_diag_rsp" in topics
    assert "sys_odom_param_rsp" in topics
    assert "servo_state_all" in topics
    assert router._latest_ws_messages["sys_diag_rsp"]["data"]["uartRxErrors"] == 7
    assert router._latest_ws_messages["sys_odom_param_rsp"]["data"]["leftMotorNumber"] == 2

    print("PASS: message router compact tlv")


if __name__ == "__main__":
    main()
