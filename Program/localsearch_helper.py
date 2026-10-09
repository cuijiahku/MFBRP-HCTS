import random

def load_routes(individual):
    truck_route = [[_ for _ in t] for t in individual.scheme['trucks']]
    return truck_route

def rebuild_route(route, internal_nodes):
    if not route:
        return route
    return [route[0]] + internal_nodes + [route[-1]]

def single_node_swap(route):
    eligible = [idx for idx, truck_route in enumerate(route) if len(truck_route) > 3]
    if not eligible:
        return route
    route_idx = random.choice(eligible)
    idx_1, idx_2 = random.sample(range(1, len(route[route_idx]) - 1), 2)
    route[route_idx][idx_1], route[route_idx][idx_2] = (
        route[route_idx][idx_2],
        route[route_idx][idx_1],
    )
    return route

def sub_tour_swap(route):
    eligible = [idx for idx, truck_route in enumerate(route) if len(truck_route) > 3]
    if not eligible:
        return route
    route_idx = random.choice(eligible)
    route_no_depot = route[route_idx][1:-1]
    node_count = len(route_no_depot)
    first_start = random.randrange(node_count - 1)
    first_end = random.randrange(first_start + 1, node_count)
    second_start = random.randrange(first_end, node_count)
    second_end = random.randrange(second_start + 1, node_count + 1)
    new_nodes = (
        route_no_depot[:first_start]
        + route_no_depot[second_start:second_end]
        + route_no_depot[first_end:second_start]
        + route_no_depot[first_start:first_end]
        + route_no_depot[second_end:]
    )
    route[route_idx] = rebuild_route(route[route_idx], new_nodes)
    return route

def single_node_relocation(route):
    eligible = [idx for idx, truck_route in enumerate(route) if len(truck_route) > 3]
    if not eligible:
        return route
    route_idx = random.choice(eligible)
    route_no_depot = route[route_idx][1:-1]
    selected_node = random.randrange(len(route_no_depot))
    positions = [
        position for position in range(len(route_no_depot))
        if position not in (selected_node, selected_node + 1)
    ]
    if not positions:
        return route
    selected_position = random.choice(positions)
    route_no_depot.insert(selected_position, route_no_depot.pop(selected_node))
    route[route_idx] = rebuild_route(route[route_idx], route_no_depot)
    return route

def multiple_node_relocation(route):
    eligible = [idx for idx, truck_route in enumerate(route) if len(truck_route) > 4]
    if not eligible:
        return route
    route_idx = random.choice(eligible)
    route_no_depot = route[route_idx][1:-1]
    sub_idx = sorted(random.sample(range(len(route_no_depot) + 1), 2))
    while sub_idx[0] == sub_idx[1] or sub_idx[1] - sub_idx[0] == len(route_no_depot):
        sub_idx = sorted(random.sample(range(len(route_no_depot) + 1), 2))
    sub_list = route_no_depot[sub_idx[0]:sub_idx[1]]
    residual_list = route_no_depot[:sub_idx[0]] + route_no_depot[sub_idx[1]:]
    insertion_positions = [
        position for position in range(len(residual_list) + 1)
        if position != sub_idx[0]
    ]
    selected_index = random.choice(insertion_positions)
    new_route = residual_list[:selected_index] + sub_list + residual_list[selected_index:]
    route[route_idx] = rebuild_route(route[route_idx], new_route)
    return route

def two_opt(route):
    eligible = [idx for idx, truck_route in enumerate(route) if len(truck_route) >= 4]
    if not eligible:
        return route
    route_idx = random.choice(eligible)
    first_arc = random.randrange(len(route[route_idx]) - 3)
    second_arc = random.randrange(first_arc + 2, len(route[route_idx]) - 1)
    route[route_idx] = (
        route[route_idx][:first_arc + 1]
        + route[route_idx][first_arc + 1:second_arc + 1][::-1]
        + route[route_idx][second_arc + 1:]
    )
    return route

def single_node_insertion(route, unvisited):
    if not route or not unvisited:
        return route

    route_idx = random.randrange(len(route))
    selected_node = random.choice(unvisited)
    selected_position = random.randrange(1, len(route[route_idx]))
    route[route_idx].insert(selected_position, selected_node)
    return route

def multiple_node_insertions(route, unvisited):
    if not route or len(unvisited) < 2:
        return route

    route_idx = random.randrange(len(route))
    sample_size = random.randint(2, len(unvisited))
    selected_nodes = random.sample(unvisited, sample_size)
    selected_position = random.randrange(1, len(route[route_idx]))
    route[route_idx][selected_position:selected_position] = selected_nodes
    return route


def single_node_deletion(route):
    eligible = [idx for idx, truck_route in enumerate(route) if len(truck_route) > 3]
    if not eligible:
        return route
    route_idx = random.choice(eligible)
    internal_nodes = route[route_idx][1:-1]
    del internal_nodes[random.randrange(len(internal_nodes))]
    route[route_idx] = rebuild_route(route[route_idx], internal_nodes)
    return route


def multiple_node_deletions(route):
    eligible = [idx for idx, truck_route in enumerate(route) if len(truck_route) > 4]
    if not eligible:
        return route
    route_idx = random.choice(eligible)
    internal_nodes = route[route_idx][1:-1]
    node_count = len(internal_nodes)
    delete_count = random.randint(2, node_count - 1)
    delete_start = random.randint(0, node_count - delete_count)
    new_nodes = (
        internal_nodes[:delete_start]
        + internal_nodes[delete_start + delete_count:]
    )
    route[route_idx] = rebuild_route(route[route_idx], new_nodes)
    return route

def inter_route_single_node_relocation(route, distance_matrix):
    source_routes = [
        route_idx
        for route_idx, truck_route in enumerate(route)
        if len(truck_route) > 3
    ]
    if not source_routes or len(route) < 2:
        return route

    source_idx = random.choice(source_routes)
    destination_idx = random.choice(
        [route_idx for route_idx in range(len(route)) if route_idx != source_idx]
    )
    source_route = route[source_idx]
    destination_route = route[destination_idx]
    node_idx = random.randrange(1, len(source_route) - 1)
    node = source_route.pop(node_idx)

    best_position = min(
        range(1, len(destination_route)),
        key=lambda position: (
            distance_matrix[destination_route[position - 1], node]
            + distance_matrix[node, destination_route[position]]
            - distance_matrix[
                destination_route[position - 1],
                destination_route[position],
            ]
        ),
    )
    destination_route.insert(best_position, node)
    return route


def inter_route_single_node_exchange(route, distance_matrix):
    eligible_routes = [
        route_idx
        for route_idx, truck_route in enumerate(route)
        if len(truck_route) > 2
    ]
    if len(eligible_routes) < 2:
        return route

    first_idx, second_idx = random.sample(eligible_routes, 2)
    first_route = route[first_idx]
    second_route = route[second_idx]
    first_node = first_route.pop(random.randrange(1, len(first_route) - 1))
    second_node = second_route.pop(random.randrange(1, len(second_route) - 1))

    first_position = min(
        range(1, len(first_route)),
        key=lambda position: (
            distance_matrix[first_route[position - 1], second_node]
            + distance_matrix[second_node, first_route[position]]
            - distance_matrix[first_route[position - 1], first_route[position]]
        ),
    )
    second_position = min(
        range(1, len(second_route)),
        key=lambda position: (
            distance_matrix[second_route[position - 1], first_node]
            + distance_matrix[first_node, second_route[position]]
            - distance_matrix[second_route[position - 1], second_route[position]]
        ),
    )
    first_route.insert(first_position, second_node)
    second_route.insert(second_position, first_node)
    return route


def inter_route_multiple_node_relocation(route, distance_matrix):
    source_routes = [
        route_idx
        for route_idx, truck_route in enumerate(route)
        if len(truck_route) > 4
    ]
    if not source_routes or len(route) < 2:
        return route

    source_idx = random.choice(source_routes)
    destination_idx = random.choice(
        [route_idx for route_idx in range(len(route)) if route_idx != source_idx]
    )
    source_route = route[source_idx]
    destination_route = route[destination_idx]
    internal_count = len(source_route) - 2
    segment_length = random.randint(2, min(3, internal_count - 1))
    segment_start = random.randint(1, len(source_route) - segment_length - 1)
    segment = source_route[segment_start:segment_start + segment_length]
    del source_route[segment_start:segment_start + segment_length]

    internal_cost = sum(
        distance_matrix[segment[idx], segment[idx + 1]]
        for idx in range(len(segment) - 1)
    )
    destination_position = min(
        range(1, len(destination_route)),
        key=lambda position: (
            distance_matrix[destination_route[position - 1], segment[0]]
            + internal_cost
            + distance_matrix[segment[-1], destination_route[position]]
            - distance_matrix[
                destination_route[position - 1],
                destination_route[position],
            ]
        ),
    )
    destination_route[destination_position:destination_position] = segment
    return route


def inter_route_multiple_node_exchange(route, distance_matrix):
    eligible_routes = [
        route_idx
        for route_idx, truck_route in enumerate(route)
        if len(truck_route) > 3
    ]
    if len(eligible_routes) < 2:
        return route

    first_idx, second_idx = random.sample(eligible_routes, 2)
    first_route = route[first_idx]
    second_route = route[second_idx]
    first_length = random.randint(2, min(3, len(first_route) - 2))
    second_length = random.randint(2, min(3, len(second_route) - 2))
    first_start = random.randint(1, len(first_route) - first_length - 1)
    second_start = random.randint(1, len(second_route) - second_length - 1)
    first_segment = first_route[first_start:first_start + first_length]
    second_segment = second_route[second_start:second_start + second_length]
    del first_route[first_start:first_start + first_length]
    del second_route[second_start:second_start + second_length]

    first_internal_cost = sum(
        distance_matrix[second_segment[idx], second_segment[idx + 1]]
        for idx in range(len(second_segment) - 1)
    )
    first_position = min(
        range(1, len(first_route)),
        key=lambda position: (
            distance_matrix[first_route[position - 1], second_segment[0]]
            + first_internal_cost
            + distance_matrix[second_segment[-1], first_route[position]]
            - distance_matrix[first_route[position - 1], first_route[position]]
        ),
    )
    second_internal_cost = sum(
        distance_matrix[first_segment[idx], first_segment[idx + 1]]
        for idx in range(len(first_segment) - 1)
    )
    second_position = min(
        range(1, len(second_route)),
        key=lambda position: (
            distance_matrix[second_route[position - 1], first_segment[0]]
            + second_internal_cost
            + distance_matrix[first_segment[-1], second_route[position]]
            - distance_matrix[second_route[position - 1], second_route[position]]
        ),
    )
    first_route[first_position:first_position] = second_segment
    second_route[second_position:second_position] = first_segment
    return route
