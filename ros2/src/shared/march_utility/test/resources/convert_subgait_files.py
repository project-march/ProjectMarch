from pathlib import Path
import re

if __name__ == '__main__':
    for path in Path('ik_test').rglob('*.subgait'):
        file = open(path, "r")
        content = file.read()
        replacements = []
        for match in re.finditer('{nsecs: (.*), secs: (.*)}', content):
            groups = match.groups()
            nsec, sec = int(groups[0]), int(groups[1])
            replacements.append((match.start(), match.end(), str(int(sec * 1e9) + nsec)))

        for start, end, replacement in reversed(replacements):
            content = content[:start] + replacement + content[end:]

        file.close()
        file = open(path, "w")
        file.write(content)

