def ExportVINSConfigFormat(vio_model, output_filename):
    from vio_model import VIOModel
    from jinja2 import Environment, FileSystemLoader
    import os
    env = Environment(loader = FileSystemLoader(os.path.dirname(os.path.realpath(__file__)) + '/templates'), trim_blocks=True, lstrip_blocks=True)
    template = env.get_template('vins_config_template.yaml')

    config_data = vio_model.as_dict()
    config_data["cam_imu_ex"]["rotation_cam_to_imu"] = vio_model.cam_imu_ex.rotation_cam_to_imu.ravel()
    config_data["cam_imu_ex"]["translation_cam_to_imu"] = vio_model.cam_imu_ex.translation_cam_to_imu.ravel()
    with open(output_filename, 'w') as output_file:
        output_file.write(template.render(config_data))

if __name__ == "__main__":
    from sys import argv
    from vio_model import VIOModel
    

    if len(argv) < 2:
        print("eg.: python " + __file__ + " output_config.yaml")
        quit()

    output_filename = argv[1]
    ExportVINSConfigFormat(VIOModel(), output_filename)