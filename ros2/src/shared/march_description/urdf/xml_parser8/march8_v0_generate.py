from march_gym.tools import xml_generator

if __name__ == '__main__':
    gen = xml_generator.XMLGenerator()
    gen('march8_v0_pre.xml', 'march8_v0.yaml')
