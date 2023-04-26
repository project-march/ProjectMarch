import ros2.src.shared.march_description.urdf.xml_generator as xml_generator

if __name__ == '__main__':
    gen = xml_generator.XMLGenerator()
    gen('march8_v0_ultralight_pre.xml', 'march8_v0_ultralight.yaml')
