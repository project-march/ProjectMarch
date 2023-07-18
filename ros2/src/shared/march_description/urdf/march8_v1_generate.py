"""Created by MVIII."""
import argparse

import xml_generator

if __name__ == '__main__':

    parser = argparse.ArgumentParser(
        prog='MARCH 8 XML Generator',
        description='Generates a Mujoco XML from a "pre-xml" containing the '
                    'structure of the exoskeleton, and a YAML that can '
                    'parametrize values like mass, intertia and constraints, '
                    'in said XML.')

    parser.add_argument('mode',
                        help='Mode for which to generate an XML.',
                        nargs='?',
                        default='free',
                        type=str,
                        choices=xml_generator.get_modes())

    args = parser.parse_args()

    gen = xml_generator.XMLGenerator(mode=args.mode)
    gen('march8/march8_v1_pre.xml', 'march8/march8_v0.yaml')

    print('Done parsing march8_v1 XML')
