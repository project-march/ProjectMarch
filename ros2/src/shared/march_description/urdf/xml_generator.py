"""Copyright (C) 2023. Stichting Moving Bird.

Joy Brand, joy.brand@projectmarch.nl
Thijn Hoekstra, thijn.hoekstra@projectmarch.nl

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""

import re
import copy
import yaml
import datetime

from typing import TextIO

from mathparser import parser as pp
from mathparser import compute

DISCLAIMER_TEXT = """\
<!-- \n\
Copyright (C) 2023. Stichting Moving Bird \n\
Joy Brand, joy.brand@projectmarch.nl \n\
Thijn Hoekstra, thijn.hoekstra@projectmarch.nl \n\
\n\
This program is free software: you can redistribute it and/or modify \n\
it under the terms of the GNU General Public License as published by \n\
the Free Software Foundation, either version 3 of the License, or \n\
(at your option) any later version. \n\
\n\
This program is distributed in the hope that it will be useful, \n\
but WITHOUT ANY WARRANTY; without even the implied warranty of \n\
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the \n\
GNU General Public License for more details. \n\
\n\
See <https://www.gnu.org/licenses/>.
-->
"""
SYMBOLS = [
    " + ",
    " - ",
    " * ",
    " / ",
]


class XMLGenerator:
    """Generates an XML usable in MuJoCo from a description.

    Uses a (modified) `.yaml` exoskeleton description and a "pre-XML" to generate a MuJoCo-ready XML format (MJCF).
    Pre-XML files are identical to MJCF XML files with the exception of that attributes also allow for flags relating
    to data stored in a YAML descrition. Such a flag starts with `${` and ends with }`, and contains a reference to
    which value in the tree structure of the YAML should replace the flag in the compiled XML.

    This path can be simple, like `<geom name="cube" type="box" size="${size} 1 1" pos="0 0 0" class="collision"/>`,
    which links to a key called `size` inside the YAML with the corresponding value. A more complex example is:
    `<geom name="cube" type="box" size="${box/size/x} ${box/size/y} ${box/size/z}" pos="0 0 0" class="collision"/>`,
    which references a triplet of values down two layers in the YAML. Note that this notation can be simplified by
    using a single flag for all three values:
    `<geom name="cube" type="box" size="${box/size/x box/size/y box/size/z" pos="0 0 0" class="collision"/>`. In the
    cases in which all the parent paths in the YAML are identical for all the values contained in the flag, as is
    also the case in the example above, additional simplification is possible by replacing the parent path in all but
    one of the instances by a dot (.), e.g.:
    `<geom name="cube" type="box" size="${box/size/x ./y ./z" pos="0 0 0" class="collision"/>`

    Examples:
        In the following example, pre-XML `march7_v1_pre.xml` is converted into a fully-fledged MJCF XML using
        `march7_v1_pre.xml`. In this case, the name of the output file defaults to `march7_v1.xml`.

        >>> gen = XMLGenerator()
        >>> gen('march7_v1_pre.xml', 'march7_v1.yaml')
    """

    def __init__(self, mark_compile_time: bool = True):
        """Creates an XMLGenerator.

        Attributes:
            mark_compile_time (bool): If set to True, a header is added to the output file, containing a time stamp
                of when the XML was created. This conveniently also notes which pre-XML and YAML file where used.
                Defaults to True.
        """
        self.mark = mark_compile_time
        self.prexml_fname = None
        self.yaml_fname = None

        self.current_line: int = None

    def _main(self, xml_file: TextIO, data: dict, out: TextIO):
        """Main operation of the XMLGenerator.

        Args:
            xml_file (TextIO): Pre-XML file.
            data (dict): Dictionary representation of the data stored in the YAML.
            out (TextIO): File to which to output the generated XML.
        """
        if self.mark:
            self._add_timestamp_and_disclaimer(out)

        for ii, line in enumerate(xml_file):

            self.current_line = ii

            completed_line = copy.copy(line)
            attributes = self._get_attributes_in_line(line)

            if attributes:
                for attribute in attributes:
                    completed_line = self._replace_attribute(attribute, completed_line, data, ii)

                completed_line = self._remove_flags(completed_line)
                completed_line = self._cancel_double_negs_naive(completed_line)

                out.write(completed_line)
            else:
                out.write(line)

    def _replace_attribute(self, attribute, completed_line, data, ii):
        prefix = self._get_prefix(attribute)

        verbatim_placeholder = self._get_placeholders(attribute)

        placeholders, new_attribute = self._add_prefix_to_relative_placeholders(verbatim_placeholder, attribute, prefix)
        placeholders = list(set(placeholders))

        keys = self._get_keys_from_string(placeholders)

        for key, placeholder in zip(keys, placeholders):

            replacement = self._get_value_from_yaml_dict(data, key)

            if replacement is None:
                raise KeyError(
                    f"Error: Could not find requested data {key}, as specified "
                    f'in line {ii} of {self.prexml_fname} at "{placeholder}". '
                    f"Please check that the placeholder in {self.prexml_fname} matches "
                    f"with {self.yaml_fname}."
                )

            new_attribute = new_attribute.replace(placeholder, str(replacement))

        if any(symbol in attribute for symbol in SYMBOLS):
            new_attribute = self._parse_math_in_attribute(new_attribute)

        return completed_line.replace(attribute, new_attribute)

    def _add_prefix_to_relative_placeholders(self, placeholders, completed_line, prefix):
        abs_placeholders = copy.copy(placeholders)

        def replacer(text):
            return text.replace("./", prefix)

        for i, placeholder in enumerate(abs_placeholders):
            if "./" in placeholder:
                abs_placeholders[i] = replacer(placeholder)
                completed_line = replacer(completed_line)

        return abs_placeholders, completed_line

    def _parse_math_in_attribute(self, attribute):

        new_attribute = copy.copy(attribute)

        for symbol in SYMBOLS:
            new_attribute = new_attribute.replace(symbol, symbol.strip())

        attr_contents = self._get_contents_inside_flag(new_attribute)

        new_attr_contents = self._cancel_double_neg_in_eqs(attr_contents)
        new_attr_contents = self._cancel_plusminus_in_eqs(new_attr_contents)

        parsed_attr_contents = []
        for value_to_parse in new_attr_contents.split(" "):
            try:
                ast = pp.parse(value_to_parse)
                result = compute.compute(ast)
                result = round(result, 6)

            except ValueError:
                result = value_to_parse

            parsed_attr_contents.append(str(result))
        parsed_attr_contents = " ".join(parsed_attr_contents)
        return new_attribute.replace(attr_contents, str(parsed_attr_contents))

    def _cancel_double_neg_in_eqs(self, text):
        """Replaces double negatives in equations.

        Replaces double negatives in equations with a plus or with a blank space
        """
        text = re.sub("--", r"+", text)
        return re.sub(r"^\+(.)", r"\1", text)

    def _cancel_plusminus_in_eqs(self, text):
        """Replaces +- or -+ with - in equations."""
        return re.sub(r"\+-|-\+", "-", text)

    def _get_attributes_in_line(self, line):
        return re.findall(r"\${.*?}", line)

    def _add_timestamp_and_disclaimer(self, out):
        text = DISCLAIMER_TEXT.format(
            datetime.datetime.now().strftime("%d/%m/%Y %H:%M:%S"), self.prexml_fname, self.yaml_fname
        )
        out.write(text)

    def __call__(self, prexml_fname: str, yaml_fname: str, output_fname: str = None) -> None:
        """Call to make that file is opened.

        :param prexml_fname:
        :param yaml_fname:
        :param output_fname:
        :return:
        """
        self.prexml_fname = prexml_fname
        self.yaml_fname = yaml_fname

        if not output_fname:
            output_fname = prexml_fname.replace("_pre.xml", ".xml").replace("pre.xml", ".xml")

        with open(yaml_fname, mode="r") as file:
            data = self._read_yaml(file)

        with open(prexml_fname, mode="r") as file, open(output_fname, mode="w") as out:
            self._main(file, data, out)

    def _read_yaml(self, yaml_file):
        return yaml.safe_load(yaml_file)

    def _cancel_double_negs_naive(self, text):
        return text.replace("--", "")

    def _remove_flags(self, text):
        return re.sub(r"\${([^;]*?)}", r"\1", text)

    def _get_value_from_yaml_dict(self, data, key_list):
        result = copy.copy(data)
        for key in key_list:
            if key not in result:
                return None

            result = result[key]
        return result

    def _get_placeholders(self, attribute):
        return re.findall(r"([\w.]+(?:\/\w+)+)\}*", attribute)

    def _get_contents_inside_flag(self, text):
        return re.match(r"\${([^;]*?)}", text).group(1)

    def _add_prefix_to_key(self, key, prefix_keys):
        return prefix_keys + [key]

    def _get_suffix_for_relative_keys(self, attribute):
        return re.findall(r"/(\w+)[ })]", attribute)

    def _get_keys_from_string(self, placeholders):
        def convert(placeholder):
            return placeholder.split("/")

        if isinstance(placeholders, (list, tuple)):
            out = []
            for placeholder in placeholders:
                out.append(convert(placeholder))
            return out

        elif isinstance(placeholders, str):
            return convert(placeholders)

        else:
            raise ValueError("Could not get keys to YAML from.")

    def _get_prefix(self, attribute):
        return re.search(r"(\w+/)+", attribute).group(0)


if __name__ == "__main__":
    pass
