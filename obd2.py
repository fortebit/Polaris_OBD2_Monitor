# OBD2 Module - Asynchronously collect realtime data from vehicle diagnostics services

from fortebit.polaris import polaris
import can

OBD_FUNCTIONAL_REQ_STD_ID = 0x7DF
OBD_PHYSICAL_REQ_STD_ID = 0x7E0  # up to 7E7
OBD_PHYSICAL_RES_STD_ID = 0x7E8  # up to 7EF (8 + Req Id)
OBD_PHYSICAL_STD_MASK = can.FRAME_STD_MASK ^ 0x007

OBD_FUNCTIONAL_REQ_EXT_ID = can.FRAME_EXT_FLAG | 0x18DB33F1
OBD_PHYSICAL_REQ_EXT_ID = can.FRAME_EXT_FLAG | 0x18DA00F1  # up to 0x18DAFFF1
OBD_PHYSICAL_RES_EXT_ID = can.FRAME_EXT_FLAG | 0x18DAF100  # up to 0x18DAF1FF
OBD_PHYSICAL_EXT_MASK = can.FRAME_EXT_MASK ^ 0x00FF

OBD_SERVICE_REALTIME = 0x01

OBD_PID_SUPPORTED_01_20 = 0x00
OBD_PID_ENGINE_COOLANT_TEMP = 0x05
OBD_PID_ENGINE_RPM = 0x0C
OBD_PID_VEHICLE_SPEED = 0x0D

_canbus = None
_txd = bytearray(8)
_rxd = bytearray(8)
_ext = False
_active = False
_sync = False


def _init():
    # enable transceiver
    pinMode(polaris.internal.PIN_CAN_STANDBY, OUTPUT)
    digitalWrite(polaris.internal.PIN_CAN_STANDBY, LOW)
    sleep(10)


def _deinit():
    # enable transceiver
    pinMode(polaris.internal.PIN_CAN_STANDBY, OUTPUT)
    digitalWrite(polaris.internal.PIN_CAN_STANDBY, HIGH)
    sleep(10)


def _request(mode, pid):
    global _txd
    _txd[0] = 2
    _txd[1] = mode
    _txd[2] = pid
    _txd[3] = 0  # x55
    _txd[4] = 0  # x55
    _txd[5] = 0  # x55
    _txd[6] = 0  # x55
    _txd[7] = 0  # x55
    return _txd

temp = 0
rpm = 0
kmh = 0


def _decode(data):
    global rpm, kmh, temp
    if len(data) < 3 or data[0] < 3 or data[1] != (0x40 | OBD_SERVICE_REALTIME):
        return False
    if data[2] == OBD_PID_ENGINE_COOLANT_TEMP:
        temp = data[3] - 40
    if data[2] == OBD_PID_ENGINE_RPM:
        rpm = (data[3] * 256 + data[4]) / 4
    if data[2] == OBD_PID_VEHICLE_SPEED:
        kmh = data[3]
    return True


def _send(service, pid):
    global _sync
    if _canbus is None:
        return False
    try:
        _canbus.transmit(
            OBD_FUNCTIONAL_REQ_EXT_ID if _ext else OBD_FUNCTIONAL_REQ_STD_ID,
            8, _request(service, pid), 100
        )
        msg = _canbus.receive(_rxd, 100)
        _sync = _decode(_rxd)
    except TimeoutError:
        _sync = False
    return _sync


def _detect(baudrate):
    global _canbus, _ext
    # open CAN bus and set rx filter
    if _canbus is not None:
        _canbus.done()
        sleep(100)
    _canbus = can.Can(CAN0, baudrate)
    f = _canbus.add_filter(OBD_PHYSICAL_RES_STD_ID, OBD_PHYSICAL_STD_MASK | can.FRAME_EXT_FLAG | can.FRAME_RTR_FLAG)
    _ext = False
    if _send(OBD_SERVICE_REALTIME, OBD_PID_SUPPORTED_01_20):
        print("Found OBD on standard CAN at %d bps" % baudrate)
        sleep(100)
        return True
    _canbus.del_filter(f)
    sleep(100)
    _canbus.add_filter(OBD_PHYSICAL_RES_EXT_ID, OBD_PHYSICAL_EXT_MASK | can.FRAME_EXT_FLAG | can.FRAME_RTR_FLAG)
    _ext = True
    if _send(OBD_SERVICE_REALTIME, OBD_PID_SUPPORTED_01_20):
        print("Found OBD on extended CAN at %d bps" % baudrate)
        sleep(100)
        return True
    _canbus.done()
    _canbus = None
    sleep(100)
    return False


def _run():
    global _active, _canbus
    _init()
    while _active:
        try:
            if not _sync:
                # attempt discovery
                if not _detect(250000):
                    if not _detect(500000):
                        print("OBD service not found on CAN bus")
                        sleep(1000)
                        continue
            sleep(10)
            _send(OBD_SERVICE_REALTIME, OBD_PID_ENGINE_RPM)
            sleep(10)
            _send(OBD_SERVICE_REALTIME, OBD_PID_VEHICLE_SPEED)
            sleep(10)
            _send(OBD_SERVICE_REALTIME, OBD_PID_ENGINE_COOLANT_TEMP)
            sleep(100)
        except:
            _active = False
            break
    _canbus.done()
    _deinit()
    _canbus = None


def start():
    global _active
    if _active:
        raise ValueError
    _active = True
    while _canbus is not None:
        sleep(500)
    thread(_run)


def stop():
    global _active
    if not _active:
        raise ValueError
    _active = False
    while _canbus is not None:
        sleep(500)


def is_running():
    return _active


def is_talking():
    return _sync
