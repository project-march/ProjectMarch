"""Written by MVIII."""
from march_gym.tools import xml_generator

if __name__ == '__main__':
    gen = xml_generator.XMLGenerator()
    gen('march7_v1_pre.xml', 'march7_v1.yaml')