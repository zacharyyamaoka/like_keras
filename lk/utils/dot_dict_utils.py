def dict_to_dot_dict(d: dict, parent_key: str = "", sep: str = "."):
    """Flatten a nested dictionary into dot-separated keys."""
    items = {}
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.update(dict_to_dot_dict(v, new_key, sep=sep))
        else:
            items[new_key] = v
    return items


def dot_dict_to_dict(d: dict, sep: str = "."):
    """Convert a dot-separated flat dictionary back to nested form."""
    result = {}
    for k, v in d.items():
        parts = k.split(sep)
        d_ref = result
        for part in parts[:-1]:
            if part not in d_ref:
                d_ref[part] = {}
            d_ref = d_ref[part]
        d_ref[parts[-1]] = v
    return result


if __name__ == "__main__":
    d = {"a": {"b": {"c": 1}}}
    print(dict_to_dot_dict(d))
    print(dot_dict_to_dict(dict_to_dot_dict(d)))
