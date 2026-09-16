from trailer_parking_pkg.launch_factory import hardware_parking_launch


def generate_launch_description():
    return hardware_parking_launch(False)
