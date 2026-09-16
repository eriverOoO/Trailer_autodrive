from serial_communication_pkg.protocol import encode_command
import pytest


def test_original_wire_format():
    assert encode_command(-7, -90, -90) == b's-7l-90r-90\n'
    assert encode_command(0, 0, 0) == b's0l0r0\n'


@pytest.mark.parametrize('values', [(8, 0, 0), (0, 256, 0), (0, 0, -256)])
def test_rejects_out_of_range(values):
    with pytest.raises(ValueError):
        encode_command(*values)
