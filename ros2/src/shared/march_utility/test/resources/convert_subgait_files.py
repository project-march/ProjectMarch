from pathlib import Path
import re

if __name__ == '__main__':
    for path in Path('walk').rglob('*.subgait'):
        print(path.name)

        file = open(path, "r")
        content = file.read()
        replacements = []
        for match in re.finditer('{nsecs: (.*), secs: (.*)}', content):
            groups = match.groups()
            replacements.append((match.start(), groups[0]))

