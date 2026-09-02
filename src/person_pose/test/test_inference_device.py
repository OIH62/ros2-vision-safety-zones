from person_pose.inference_device import resolve_device


class FakeCuda:
    def __init__(self, available: bool, count: int = 1):
        self.available = available
        self.count = count

    def is_available(self):
        return self.available

    def device_count(self):
        return self.count


class FakeTorch:
    def __init__(self, available: bool, count: int = 1):
        self.cuda = FakeCuda(available, count)


def test_auto_uses_first_cuda_device_when_available():
    assert resolve_device("auto", FakeTorch(True)) == ("cuda:0", None)


def test_auto_falls_back_to_cpu_without_cuda():
    device, reason = resolve_device("auto", FakeTorch(False))
    assert device == "cpu"
    assert "unavailable" in reason


def test_forced_cuda_can_fall_back_to_cpu():
    device, reason = resolve_device("cuda:1", FakeTorch(False), True)
    assert device == "cpu"
    assert "cuda:1" in reason


def test_cpu_is_always_honored():
    assert resolve_device("cpu", FakeTorch(True)) == ("cpu", None)
