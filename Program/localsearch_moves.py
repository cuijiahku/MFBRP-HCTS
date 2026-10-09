try:
    from .localsearch_helper import (
        load_routes,
        inter_route_multiple_node_exchange,
        inter_route_multiple_node_relocation,
        inter_route_single_node_exchange,
        inter_route_single_node_relocation,
        multiple_node_deletions,
        multiple_node_insertions,
        multiple_node_relocation,
        single_node_insertion,
        single_node_deletion,
        single_node_relocation,
        single_node_swap,
        sub_tour_swap,
        two_opt,
    )
    from .individual import Individual
except ImportError:
    from localsearch_helper import (
        load_routes,
        inter_route_multiple_node_exchange,
        inter_route_multiple_node_relocation,
        inter_route_single_node_exchange,
        inter_route_single_node_relocation,
        multiple_node_deletions,
        multiple_node_insertions,
        multiple_node_relocation,
        single_node_insertion,
        single_node_deletion,
        single_node_relocation,
        single_node_swap,
        sub_tour_swap,
        two_opt,
    )
    from individual import Individual

def _make_individual(
    source_individual,
    data,
    truck_routes,
    evaluator=None,
    evaluation_cache=None,
):
    if source_individual._route_signature(truck_routes) == source_individual._route_signature(
        source_individual.truck_routes
    ):
        return None
    return Individual(
        False,
        data,
        truck_routes,
        evaluator=evaluator,
        evaluation_cache=evaluation_cache,
        previous_details=source_individual.solution_details,
    )


def move_1(individual: Individual, data, evaluator=None, evaluation_cache=None):
    truck_route = load_routes(individual)
    truck_routes = single_node_swap(truck_route)
    return _make_individual(individual, data, truck_routes, evaluator, evaluation_cache)


def move_2(individual: Individual, data, evaluator=None, evaluation_cache=None):
    truck_route = load_routes(individual)
    truck_routes = sub_tour_swap(truck_route)
    return _make_individual(individual, data, truck_routes, evaluator, evaluation_cache)


def move_3(individual: Individual, data, evaluator=None, evaluation_cache=None):
    truck_route = load_routes(individual)
    truck_routes = single_node_relocation(truck_route)
    return _make_individual(individual, data, truck_routes, evaluator, evaluation_cache)


def move_4(individual: Individual, data, evaluator=None, evaluation_cache=None):
    truck_route = load_routes(individual)
    truck_routes = multiple_node_relocation(truck_route)
    return _make_individual(individual, data, truck_routes, evaluator, evaluation_cache)


def move_5(individual: Individual, data, evaluator=None, evaluation_cache=None):
    truck_route = load_routes(individual)
    truck_routes = two_opt(truck_route)
    return _make_individual(individual, data, truck_routes, evaluator, evaluation_cache)


def move_6(individual: Individual, data, evaluator=None, evaluation_cache=None):
    truck_route = load_routes(individual)
    unvisited_truck = set([0]+list(data.V_d))-set([i for r in truck_route for i in r]) 
    truck_routes = single_node_insertion(truck_route, list(unvisited_truck))
    return _make_individual(individual, data, truck_routes, evaluator, evaluation_cache)


def move_7(individual: Individual, data, evaluator=None, evaluation_cache=None):
    truck_route = load_routes(individual)
    unvisited_truck = set([0]+list(data.V_d))-set([i for r in truck_route for i in r]) 
    truck_routes = multiple_node_insertions(truck_route, list(unvisited_truck))
    return _make_individual(individual, data, truck_routes, evaluator, evaluation_cache)


def move_8(individual: Individual, data, evaluator=None, evaluation_cache=None):
    truck_route = load_routes(individual)
    truck_routes = single_node_deletion(truck_route)
    return _make_individual(individual, data, truck_routes, evaluator, evaluation_cache)


def move_9(individual: Individual, data, evaluator=None, evaluation_cache=None):
    truck_route = load_routes(individual)
    truck_routes = multiple_node_deletions(truck_route)
    return _make_individual(individual, data, truck_routes, evaluator, evaluation_cache)


def move_10(individual: Individual, data, evaluator=None, evaluation_cache=None):
    truck_route = load_routes(individual)
    truck_routes = inter_route_single_node_relocation(
        truck_route,
        data.distance_matrix,
    )
    return _make_individual(individual, data, truck_routes, evaluator, evaluation_cache)


def move_11(individual: Individual, data, evaluator=None, evaluation_cache=None):
    truck_route = load_routes(individual)
    truck_routes = inter_route_single_node_exchange(
        truck_route,
        data.distance_matrix,
    )
    return _make_individual(individual, data, truck_routes, evaluator, evaluation_cache)


def move_12(individual: Individual, data, evaluator=None, evaluation_cache=None):
    truck_route = load_routes(individual)
    truck_routes = inter_route_multiple_node_relocation(
        truck_route,
        data.distance_matrix,
    )
    return _make_individual(individual, data, truck_routes, evaluator, evaluation_cache)


def move_13(individual: Individual, data, evaluator=None, evaluation_cache=None):
    truck_route = load_routes(individual)
    truck_routes = inter_route_multiple_node_exchange(
        truck_route,
        data.distance_matrix,
    )
    return _make_individual(individual, data, truck_routes, evaluator, evaluation_cache)


def get_local_search_moves():
    return [
        move_1,
        move_2,
        move_3,
        move_4,
        move_5,
        move_6,
        move_7,
        move_8,
        move_9,
        move_10,
        move_11,
        move_12,
        move_13,
    ]
