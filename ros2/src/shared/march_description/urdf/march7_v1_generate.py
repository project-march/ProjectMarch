"""Created by MVIII."""
import xml_generator

if __name__ == "__main__":
    gen = xml_generator.XMLGenerator()
    gen("march7/march7_v1_pre.xml", "march7/march7_v1.yaml")

    print("Done parsing march7_v1 XML")
