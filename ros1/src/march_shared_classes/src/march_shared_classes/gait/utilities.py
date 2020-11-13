def weighted_average_dictionaries(base_dictionary, other_dictionary, parameter):
    """Computes the weighted average of the entries of two dictionaries with normalised weight parameter."""
    if len(base_dictionary) != len(other_dictionary):
        raise KeyError('Dictionaries do not have the same number of entries.')

    resulting_dictionary = {}
    for key in base_dictionary.keys():
        try:
            resulting_dictionary[key] = weighted_average(base_dictionary[key],
                                                         other_dictionary[key], parameter)
        except KeyError as e:
            raise KeyError('Dictionaries must have the same keys for a weighted average. other_dictionary misses '
                           '{key}'.format(key=e.args[0]))

    return resulting_dictionary


def weighted_average(base_value, other_value, parameter):
    """Compute the weighted average of two values with normalised weight parameter."""
    return base_value * (1 - parameter) + other_value * parameter


def merge_dictionaries(dic_one, dic_two):
    """Combines the key value pairs of two dicitonaries into a new dicionary."""
    merged_dic = {}
    for key_one in dic_one:
        if key_one not in dic_two or dic_one[key_one] == dic_two[key_one]:
            merged_dic[key_one] = dic_one[key_one]
        else:
            raise KeyError('Dictionaries to be merged both contain key {key} with differing values'.
                           format(key=key_one))
    for key_two in dic_two:
        if key_two not in dic_one or dic_one[key_two] == dic_two[key_two]:
            merged_dic[key_two] = dic_two[key_two]
        else:
            raise KeyError('Dictionaries to be merged both contain key {key} with differing values'.
                           format(key=key_two))
    return merged_dic
