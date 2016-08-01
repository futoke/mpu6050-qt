CHUNK_SIZE = 100


def fill_coords_queue(coords: list) -> list:

    """

    :param coords:
    :return:
    """
    coords_queue = [[c[0]] for c in coords]

    for (begin_coord, end_coord), queue_item in zip(coords, coords_queue):
        if end_coord - begin_coord > 0:
            while queue_item[-1] < end_coord:
                if end_coord - queue_item[-1] < CHUNK_SIZE:
                    queue_item += [end_coord]
                else:
                    queue_item += [queue_item[-1] + CHUNK_SIZE]
        else:
            while queue_item[-1] > end_coord:
                if queue_item[-1] - end_coord < CHUNK_SIZE:
                    queue_item += [end_coord]
                else:
                    queue_item += [queue_item[-1] - CHUNK_SIZE]

    max_len = max(map(len, coords_queue))

    for queue_item in coords_queue:
        queue_item_len = len(queue_item)
        if queue_item_len < max_len:
            queue_item += (max_len - queue_item_len) * [queue_item[-1]]

    return zip(*coords_queue)


coords_v = fill_coords_queue((-1000, -1240), (100, 1234), (10, 456), (400,
                                                                      450))

for i in coords_v:
    print(i)

