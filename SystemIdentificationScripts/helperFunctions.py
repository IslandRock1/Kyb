
def logg_data(path: str, items: list[list]) -> None:
    """

    :param path: Name of a file, with or without an extension
    :param items: list of angle/angle velocity, voltage, time
    :return:
    """
    if not path.endswith(".csv"):
        path += ".csv"

    with open(path, "a") as logg:
        for (a, v, t) in zip(*items):
            logg.write(f"{a},{v},{t}\n")