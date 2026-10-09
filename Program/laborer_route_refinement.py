

class LaborerRouteRefinement:
    def refine_final_worker_routes(self, details):
        for route_detail in details["routes"]:
            for region, routes in route_detail["internal_routes"].items():
                route_detail["internal_routes"][region] = self._refine_worker_routes(
                    region,
                    routes,
                    route_detail["pick"][region, 0],
                    route_detail["pick"][region, 1],
                    route_detail["battery_swap"][region],
                )

            route_detail["internal_distance"] = sum(
                self.calculate_distance(worker_route)
                for routes in route_detail["internal_routes"].values()
                for worker_route in routes
            )
            route_detail["laborer_time"] = sum(
                self.calculate_travel_time(worker_route, self.laborer_time_matrix)
                for routes in route_detail["internal_routes"].values()
                for worker_route in routes
            )
            route_detail["total_time"] = (
                route_detail["truck_time"]
                + route_detail["laborer_time"]
                + route_detail["service_time"]
            )

        details["route_times"] = [route["total_time"] for route in details["routes"]]
        details["totals"]["internal_distance"] = sum(
            route["internal_distance"] for route in details["routes"]
        )
        details["costs"] = self._calculate_costs(
            details["totals"]["loading"],
            details["totals"]["battery_swaps"],
            details["totals"]["internal_distance"],
            details["totals"]["truck_distance"],
            details["totals"]["deviation"],
        )
        details["total_cost"] = details["costs"]["total"]
        return details

    def _refine_worker_routes(
        self,
        region,
        routes,
        man_pickups,
        ebike_pickups,
        swaps,
    ):
        two_opt_cache = {}

        def optimize_route(route):
            route_key = tuple(route)
            optimized = two_opt_cache.get(route_key)
            if optimized is None:
                optimized = tuple(self._worker_two_opt(route))
                two_opt_cache[route_key] = optimized
            return list(optimized)

        best_routes = [list(route) for route in routes]
        best_cost = sum(self.calculate_distance(route) for route in best_routes)

        while True:
            improving_routes = None
            improving_cost = best_cost
            visited = {
                node for route in best_routes for node in route[1:-1]
            }

            def evaluate(candidate_routes):
                nonlocal improving_routes, improving_cost
                candidate_routes = [
                    route for route in candidate_routes if len(route) > 2
                ]
                if not self._worker_routes_feasible(
                    candidate_routes, man_pickups, ebike_pickups, swaps
                ):
                    return
                candidate_routes = [
                    optimize_route(route) for route in candidate_routes
                ]
                cost = sum(
                    self.calculate_distance(route) for route in candidate_routes
                )
                if cost < improving_cost - 1e-12:
                    improving_cost = cost
                    improving_routes = candidate_routes

            for route_index, route in enumerate(best_routes):
                for position in range(1, len(route) - 1):
                    old_node = route[position]
                    for new_node in self.region[region]:
                        if (
                            new_node in visited
                            or self.node_type[new_node] != self.node_type[old_node]
                        ):
                            continue
                        candidate = [list(worker_route) for worker_route in best_routes]
                        candidate[route_index][position] = new_node
                        evaluate(candidate)
                        evaluate(self._improve_worker_layout(
                            candidate,
                            man_pickups,
                            ebike_pickups,
                            swaps,
                            optimize_route,
                        ))

            for new_node in self.region[region]:
                if new_node in visited:
                    continue
                for route_index, route in enumerate(best_routes):
                    for position in range(1, len(route)):
                        candidate = [list(worker_route) for worker_route in best_routes]
                        candidate[route_index].insert(position, new_node)
                        evaluate(candidate)
                        inserted_positions = [
                            (source_index, source_position)
                            for source_index, source_route in enumerate(candidate)
                            for source_position in range(1, len(source_route) - 1)
                            if source_route[source_position] != new_node
                        ]
                        for source_index, source_position in inserted_positions:
                            for target_index, target_route in enumerate(candidate):
                                if source_index == target_index:
                                    continue
                                for target_position in range(1, len(target_route)):
                                    redistributed = [
                                        list(worker_route) for worker_route in candidate
                                    ]
                                    node = redistributed[source_index].pop(source_position)
                                    redistributed[target_index].insert(target_position, node)
                                    evaluate(redistributed)

            positions = [
                (route_index, position)
                for route_index, route in enumerate(best_routes)
                for position in range(1, len(route) - 1)
            ]
            for source_index, source_position in positions:
                candidate = [list(route) for route in best_routes]
                candidate[source_index].pop(source_position)
                evaluate(candidate)

                for target_index, target_route in enumerate(best_routes):
                    for target_position in range(1, len(target_route)):
                        candidate = [list(route) for route in best_routes]
                        node = candidate[source_index].pop(source_position)
                        insertion_position = target_position
                        if (
                            source_index == target_index
                            and target_position > source_position
                        ):
                            insertion_position -= 1
                        candidate[target_index].insert(insertion_position, node)
                        evaluate(candidate)

            for first_index, (first_route, first_position) in enumerate(positions):
                for second_route, second_position in positions[first_index + 1:]:
                    candidate = [list(route) for route in best_routes]
                    candidate[first_route][first_position], candidate[second_route][second_position] = (
                        candidate[second_route][second_position],
                        candidate[first_route][first_position],
                    )
                    evaluate(candidate)

            if improving_routes is None:
                return best_routes
            best_routes = improving_routes
            best_cost = improving_cost

    def _improve_worker_layout(
        self,
        routes,
        man_pickups,
        ebike_pickups,
        swaps,
        optimize_route,
    ):
        current = [
            optimize_route(route) for route in routes if len(route) > 2
        ]
        current_cost = sum(self.calculate_distance(route) for route in current)

        while True:
            improving_routes = None
            improving_cost = current_cost
            positions = [
                (route_index, position)
                for route_index, route in enumerate(current)
                for position in range(1, len(route) - 1)
            ]

            def consider(candidate):
                nonlocal improving_routes, improving_cost
                candidate = [
                    route for route in candidate if len(route) > 2
                ]
                if not self._worker_routes_feasible(
                    candidate, man_pickups, ebike_pickups, swaps
                ):
                    return
                candidate = [optimize_route(route) for route in candidate]
                cost = sum(self.calculate_distance(route) for route in candidate)
                if cost < improving_cost - 1e-12:
                    improving_cost = cost
                    improving_routes = candidate

            for source_index, source_position in positions:
                for target_index, target_route in enumerate(current):
                    for target_position in range(1, len(target_route)):
                        candidate = [list(route) for route in current]
                        node = candidate[source_index].pop(source_position)
                        insertion_position = target_position
                        if (
                            source_index == target_index
                            and target_position > source_position
                        ):
                            insertion_position -= 1
                        candidate[target_index].insert(insertion_position, node)
                        consider(candidate)

            for first_index, (first_route, first_position) in enumerate(positions):
                for second_route, second_position in positions[first_index + 1:]:
                    candidate = [list(route) for route in current]
                    candidate[first_route][first_position], candidate[second_route][second_position] = (
                        candidate[second_route][second_position],
                        candidate[first_route][first_position],
                    )
                    consider(candidate)

            if improving_routes is None:
                return current
            current = improving_routes
            current_cost = improving_cost
