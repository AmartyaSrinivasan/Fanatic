from fanatic.pid import PID


def test_p_only_output():
    pid = PID(kp=2.0, ki=0.0, kd=0.0, out_min=-100.0, out_max=100.0)
    assert pid.update(1.5, 0.01) == 3.0


def test_integ_grows():
    pid = PID(kp=0.0, ki=2.0, kd=0.0, out_min=-100.0, out_max=100.0)
    l = []
    for i in range(5):
        l.append(pid.update(1.0, 0.1))
    for i in range(1, 5):
        assert l[i] > l[i - 1]


def test_no_first_call_spike():
    pid = PID(kp=0.0, ki=0.0, kd=2.0, out_min=-100.0, out_max=100.0)
    assert pid.update(1.0, 1 / 240) == 0.0


def test_no_spike_after_reset():
    pid = PID(kp=0.0, ki=0.0, kd=2.0, out_min=-100.0, out_max=100.0)
    l = []
    for i in range(5):
        l.append(pid.update(1.0, 1 / 240))
    pid.reset()
    assert pid.update(1.0, 1 / 240) == 0.0


def test_clamp_test():
    pid = PID(kp=100.0, ki=0.0, kd=0.0, out_min=-1.0, out_max=1.0)
    x = pid.update(9000.0, 0.1)
    pid.reset()
    y = pid.update(-10000.0, 0.1)
    z = x == 1.0 and y == -1.0
    assert z
