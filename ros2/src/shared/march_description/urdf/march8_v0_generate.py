"""Created by MVIII."""
import xml_generator

if __name__ == "__main__":
    gen = xml_generator.XMLGenerator()
    gen("march8/march8_v0_pre.xml", "march8/march8_v0.yaml")

    print("Done parsing march8_v0 XML")
