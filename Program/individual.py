try:
    from .MCP import Model_clst
    from .node_instruction import NodeInstruction
except ImportError:
    from MCP import Model_clst
    from node_instruction import NodeInstruction

class Individual:
    def __init__(
        self,
        init=False,
        data=None,
        truck_routes=None,
        evaluator=None,
        evaluation_cache=None,
        previous_details=None,
    ):
        self.data = data
        self.truck_routes = truck_routes
        self.evaluator = evaluator
        self.evaluation_cache = evaluation_cache
        self.previous_details = previous_details
        self.scheme = None
        self.solution_details = None
        self.route_times = []
        self.costs = {}
        self.totals = {}
        self.set_individual(init=init, truck_routes=truck_routes)
        self.fit_v = self.solution_evaluation()
        
    def generate_the_route_for_trucks(self):
        return Model_clst(self.data).Solve()

    def assign_scheme(self, truck_route):
        return {"trucks": truck_route}

    def get_solution_details(self):
        return self.solution_details

    def get_route_details(self):
        return self.solution_details["routes"] if self.solution_details else []

    def solution_evaluation(self):
        truck_routes = self.scheme['trucks']
        details = self._evaluate_routes(truck_routes)

        if details["route_times"] and max(details["route_times"]) > self.data.T_max + 1e-9:
            truck_routes = self.repair(details["route_times"], truck_routes, self.data)
            self.set_individual(init=False, truck_routes=truck_routes)
            details = self._evaluate_routes(self.scheme["trucks"])

        truck_routes, details = self._remove_inactive_regions(self.scheme["trucks"], details)

        self._store_solution_details(details)
        return details["total_cost"]

    def _evaluate_routes(self, truck_routes):
        signature = self._route_signature(truck_routes)
        if self.evaluation_cache is not None:
            cached = self.evaluation_cache.get(signature)
            if cached is not None:
                return cached

        evaluator = self.evaluator or NodeInstruction(self.data)
        details = evaluator.Evaluate_solution(
            truck_routes,
            previous_details=self.previous_details,
        )
        if self.evaluation_cache is not None:
            self.evaluation_cache[signature] = details
        return details

    @staticmethod
    def _route_signature(truck_routes):
        return tuple(tuple(int(node) for node in route) for route in truck_routes)

    def _store_solution_details(self, details):
        self.solution_details = details
        self.route_times = details["route_times"]
        self.costs = details["costs"]
        self.totals = details["totals"]

    def repair(self, total_times, truck_routes, data):
        feasible_routes = []
        for route_idx, route in enumerate(truck_routes):
            if not route:
                feasible_routes.append([])
                continue

            new_route = list(route)
            time_exceedance = total_times[route_idx] - data.T_max
            while time_exceedance > 0 and len(new_route) > 2:
                best_removal_idx = -1
                max_reduction = 0
                for idx in range(1, len(new_route) - 1):
                    reduction = self._travel_reduction(new_route, idx, data)
                    if reduction > max_reduction:
                        max_reduction = reduction
                        best_removal_idx = idx

                if best_removal_idx <= 0 or max_reduction <= 0:
                    break

                del new_route[best_removal_idx]
                time_exceedance = self._evaluate_route_time(new_route) - data.T_max

            feasible_routes.append(new_route)
        return feasible_routes

    def _evaluate_route_time(self, route):
        evaluator = self.evaluator or NodeInstruction(self.data)
        return evaluator.evaluate_route_time(route)

    def _remove_inactive_regions(self, truck_routes, details):
        current_routes = [list(route) for route in truck_routes]
        candidate_routes = []
        for route_idx, route in enumerate(current_routes):
            if len(route) <= 2:
                candidate_routes.append(route)
                continue

            kept_route = [route[0]]
            inactive_regions = []
            for region in route[1:-1]:
                if self._is_inactive_region(details, route_idx, region):
                    inactive_regions.append(region)
                else:
                    kept_route.append(region)
            if len(kept_route) == 1 and inactive_regions:
                kept_route.append(min(
                    inactive_regions,
                    key=lambda region: (
                        self.data.distance_matrix[route[0], region]
                        + self.data.distance_matrix[region, route[-1]]
                    ),
                ))
            kept_route.append(route[-1])
            candidate_routes.append(kept_route)

        if self._route_signature(candidate_routes) == self._route_signature(truck_routes):
            return current_routes, details

        candidate_details = self._evaluate_routes(candidate_routes)
        if candidate_details["total_cost"] <= details["total_cost"]:
            self.set_individual(init=False, truck_routes=candidate_routes)
            return candidate_routes, candidate_details

        return current_routes, details

    def _is_inactive_region(self, details, route_idx, region):
        route_details = details["routes"][route_idx]
        has_pick = any(
            route_details["pick"].get((region, bike_type), 0) != 0
            for bike_type in self.data.B
        )
        has_drop = any(
            route_details["drop"].get((region, bike_type), 0) != 0
            for bike_type in self.data.B
        )
        has_swap = route_details["battery_swap"].get(region, 0) != 0
        has_internal_route = bool(route_details["internal_routes"].get(region))
        return not (has_pick or has_drop or has_swap or has_internal_route)

    @staticmethod
    def _travel_reduction(route, node_idx, data):
        previous_node = route[node_idx - 1]
        node = route[node_idx]
        next_node = route[node_idx + 1]
        return (
            data.distance_matrix[previous_node, node]
            + data.distance_matrix[node, next_node]
            - data.distance_matrix[previous_node, next_node]
        )

    def set_individual(self, init, truck_routes=None):
        if init:
            assert truck_routes is None
            routes = Model_clst(self.data).Solve()
        else:
            assert truck_routes is not None
            routes = truck_routes
        self.truck_routes = [list(route) for route in routes]
        self._validate_truck_routes()
        self.scheme = self.assign_scheme(self.truck_routes)

    def _validate_truck_routes(self):
        if len(self.truck_routes) != len(self.data.K):
            raise ValueError(
                f"Expected {len(self.data.K)} truck routes, "
                f"received {len(self.truck_routes)}"
            )

        service_regions = []
        valid_regions = set(self.data.V_d)
        start_depot = self.data.N_0[0]
        end_depot = self.data.N_0[-1]
        for route_idx, route in enumerate(self.truck_routes):
            if len(route) < 2 or route[0] != start_depot or route[-1] != end_depot:
                raise ValueError(
                    f"Route {route_idx} must start at {start_depot} "
                    f"and end at {end_depot}: {route}"
                )
            invalid_regions = set(route[1:-1]) - valid_regions
            if invalid_regions:
                raise ValueError(
                    f"Route {route_idx} contains invalid service regions: "
                    f"{sorted(invalid_regions)}"
                )
            service_regions.extend(route[1:-1])

        if len(service_regions) != len(set(service_regions)):
            raise ValueError("A service region appears more than once across truck routes")

    def __eq__(self, other):
        return self.scheme == other.scheme
