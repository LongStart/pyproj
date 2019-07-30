import yaml
import numpy as np

def ParserKalibrOutputFilename(kalibr_output_folder, calibration_bag_filename):
    output_basename_suffix = calibration_bag_filename.replace('/','').replace('.bag', '.yaml')
    imu_output = kalibr_output_folder + '/imu-' + output_basename_suffix
    imu_cam_output = kalibr_output_folder + '/camchain-imucam-' + output_basename_suffix
    return (imu_output, imu_cam_output)

def ParseIMUCalibrationFile(calibration_filename):
    from vio_model import IMUModel
    stream = open(calibration_filename, 'r')
    data_loaded = yaml.safe_load(stream)['imu0']
    stream.close()
    an_aw_gn_gw = (data_loaded["accelerometer_noise_density"], data_loaded["accelerometer_random_walk"], data_loaded["gyroscope_noise_density"], data_loaded["gyroscope_random_walk"])
    topic_name = data_loaded["rostopic"]
    return (IMUModel(an_aw_gn_gw), topic_name)

def ParseIMUCamCalibrationFile(calibration_filename):
    from vio_model import PinHoleRollingShutterCameraModel
    from vio_model import CameraIMUExtrinsicModel
    stream = open(calibration_filename, 'r')
    data_loaded = yaml.safe_load(stream)['cam0']
    topic_name = data_loaded['rostopic']
    extrinsic = CameraIMUExtrinsicModel(np.array(data_loaded['T_cam_imu']), data_loaded['timeshift_cam_imu'])
    cam_model = PinHoleRollingShutterCameraModel(data_loaded['intrinsics'], data_loaded['resolution'], data_loaded['distortion_coeffs'] + [0.], 0.)
    return (cam_model, extrinsic, topic_name)

def LoadVIOModelFromKalibrOutput(kalibr_output_folder, calibration_bag_filename):
    from vio_model import VIOModel
    imu_filename, imu_cam_filename = ParserKalibrOutputFilename(kalibr_output_folder, calibration_bag_filename)
    vio_model = VIOModel()
    vio_model.imu, vio_model.imu_topic = ParseIMUCalibrationFile(imu_filename)
    vio_model.cam, vio_model.cam_imu_ex, vio_model.cam_topic = ParseIMUCamCalibrationFile(imu_cam_filename)

if __name__ == "__main__":
    from sys import argv
    if len(argv) < 3:
        print('eg.: python ' + __file__ + ' kalibr_output_folder[/home/xiache02] calibration_bag_filename[/home/dataset/test.bag]')
        quit()
    
    kalibr_output_folder, calibration_bag_filename = argv[1:3]
    LoadVIOModelFromKalibrOutput(kalibr_output_folder, calibration_bag_filename)

