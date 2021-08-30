import re


def select_same_subgait_versions(gait: str, subgaits: dict, prefix: str, postfix: str):
    selected_versions = {}

    if prefix == "" and postfix == "":
        return selected_versions

    if prefix == "":
        prefix = ".*"
    if postfix == "":
        postfix = ".*"

    for subgait, versions in subgaits.items():
        subgait_no_underscores = subgait.replace("_", "")
        regex_string = (
            f"{prefix}(_?{gait})?_({subgait}|{subgait_no_underscores})_{postfix}"
        )
        pattern = re.compile(regex_string)

        for version_name in versions:
            match = pattern.match(version_name)
            if match is not None:
                selected_versions[subgait] = version_name

    return selected_versions
