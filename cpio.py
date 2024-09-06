testcases = [
    [1, 2, 0, 1, 1, 1],
]


def calculate_grabs(park_elements: list[bool]) -> tuple[list[int], list[int]]:
    """Calculate where the grabs are.

    Args:
        park_elements (tuple[list[int], list[int]]): The park element slots. True means it is a lake element.

    Raises:
        ValueError: Something weird happened.
    """
    for i in range(6):
        subset = park_elements[i : i + 3]
        if sum(subset) == 4:
            if i == 4:  # wrapped around
                return ([5, -1], [(i - 2) % 6])
            elif i == 5:
                return ([6, 0], [(i - 2) % 6])
            elif i in (1, 2):
                return ([i + 1], [1, 4])
            else:
                return ([i + 1], [(i - 2) % 6])

    error_message = "No triplets were found."
    raise ValueError(error_message)


for tc in testcases:
    tc = tc * 2
    print(calculate_grabs(tc))
