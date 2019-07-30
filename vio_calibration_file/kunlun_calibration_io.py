def CheckKunlunFormat(data_loaded):
    required_keys = ['timedifference_cam_imu', 'accelerometer_crossterms', 'accelerometer_noise_density']
    for key in required_keys:
        if not data_loaded.has_key(key):
            return False
    return True

def LoadKunlunCalibrationFile(calibration_filename):
    import yaml
    from vio_model import VIOModel
    from vio_model import CameraIMUExtrinsicModel
    import numpy as np

    data_loaded = None
    with open(calibration_filename, 'r') as stream:
        data_loaded = yaml.safe_load(stream)
    if not CheckKunlunFormat(data_loaded):
        return None

    imu_an_aw_gn_gw = (data_loaded['accelerometer_noise_density'], data_loaded['accelerometer_random_walk'], data_loaded['gyro_noise_density'], data_loaded['gyro_random_walk'])
    vio_model = VIOModel(
        imu_an_aw_gn_gw=imu_an_aw_gn_gw, 
        cam_intrin=data_loaded['intrinsics'], 
        cam_distor=data_loaded['distortion_coeffs'], 
        resolution=data_loaded['resolution'], 
        t_rolling_shutter=data_loaded['shutter_time'])
    vio_model.cam_imu_ex = CameraIMUExtrinsicModel.InitByImuToCam(np.array(data_loaded['T_cam_imu']), data_loaded['timedifference_cam_imu'])
    return vio_model

def ExportKunlunCalibrationFile(vio_model, output_filename):
    from vio_model import VIOModel
    from jinja2 import Environment, FileSystemLoader
    import os
    env = Environment(loader = FileSystemLoader(os.path.dirname(os.path.realpath(__file__)) + '/templates'), trim_blocks=True, lstrip_blocks=True)
    template = env.get_template('kunlun_calibration_template.yaml')

    config_data = vio_model.as_dict()
    with open(output_filename, 'w') as output_file:
        output_file.write(template.render(config_data))

if __name__ == "__main__":
    from sys import argv
    from vio_model import VIOModel

    if len(argv) < 2:
        print("eg.: python " + __file__ + " calibration.yaml")
        quit()

    input_filename = argv[1]
    model = LoadKunlunCalibrationFile(input_filename)
    ExportKunlunCalibrationFile(model, "kunlun_test.yaml")