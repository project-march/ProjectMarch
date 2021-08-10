from pathlib import Path
import yaml
import copy

old_stand_position_start = {
    "left_ankle": {"position": 0.0, "time_from_start": 0, "velocity": 0.0},
    "left_hip_aa": {"position": 0.0349, "time_from_start": 0, "velocity": 0.0},
    "left_hip_fe": {"position": -0.1745, "time_from_start": 0, "velocity": 0.0},
    "left_knee": {"position": 0.0, "time_from_start": 0, "velocity": 0.0},
    "right_ankle": {"position": 0.0, "time_from_start": 0, "velocity": 0.0},
    "right_hip_aa": {"position": 0.0349, "time_from_start": 0, "velocity": 0.0},
    "right_hip_fe": {"position": -0.1745, "time_from_start": 0, "velocity": 0.0},
    "right_knee": {"position": 0.0, "time_from_start": 0, "velocity": 0.0},
}
old_stand_position_end = copy.deepcopy(old_stand_position_start)

new_stand_position_start = {
    "left_ankle": {"position": 0.0524, "time_from_start": 0, "velocity": 0.0},
    "left_hip_aa": {"position": 0.0349, "time_from_start": 0, "velocity": 0.0},
    "left_hip_fe": {"position": -0.1745, "time_from_start": 0, "velocity": 0.0},
    "left_knee": {"position": 0.0, "time_from_start": 0, "velocity": 0.0},
    "right_ankle": {"position": 0.0524, "time_from_start": 0, "velocity": 0.0},
    "right_hip_aa": {"position": 0.0349, "time_from_start": 0, "velocity": 0.0},
    "right_hip_fe": {"position": -0.1745, "time_from_start": 0, "velocity": 0.0},
    "right_knee": {"position": 0.0, "time_from_start": 0, "velocity": 0.0},
}
new_stand_position_end = copy.deepcopy(new_stand_position_start)

# actually for dicts in dicts
def almost_equal_nested_dict(dict1, dict2):
    result = True
    for key_outer, value in dict1.items():
        for key_inner, true_value in value.items():
            result = result and (
                round(true_value, 4) == round(dict2[key_outer][key_inner], 4)
            )
    return result


start_positions_set = 0
end_positions_set = 0
paths_that_failed = []
for path in Path(
    "/home/pmarch/march/ros2/src/gaits/march_gait_files/airgait_vi/"
).rglob("*.subgait"):
    try:
        file = open(path, "r")
        print(path)
        content = yaml.full_load(file)
        final_time = content["duration"]
        current_start_position = {}
        current_end_position = {}

        for joint, setpoint_list in content["joints"].items():
            current_start_position[joint] = setpoint_list[0]
            current_end_position[joint] = setpoint_list[-1]

            old_stand_position_end[joint]["time_from_start"] = final_time
            new_stand_position_end[joint]["time_from_start"] = final_time

        if almost_equal_nested_dict(current_start_position, old_stand_position_start):
            for joint, setpoint_list in content["joints"].items():
                content["joints"][joint][0] = new_stand_position_start[joint]
            start_positions_set += 1
        if almost_equal_nested_dict(current_end_position, old_stand_position_end):
            for joint, setpoint_list in content["joints"].items():
                content["joints"][joint][-1] = new_stand_position_end[joint]
            end_positions_set += 1
        file.close()

        file = open(path, "w")
        yaml.dump(content, file)

    except Exception as e:
        paths_that_failed.append(path)
        print(e)

print(f"the paths {paths_that_failed} failed.")
print(f"there were {start_positions_set} start positions changed")
print(f"there were {end_positions_set} end positions changed")
