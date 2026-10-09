try:
    from .cache import LRUCache
except ImportError:
    from cache import LRUCache


class NodeInstructionHelper:
    def _worker_routes_feasible(self, routes, man_pickups, ebike_pickups, swaps):
        type_counts = {0: 0, 1: 0, 2: 0}
        available_low_pickup_slots = 0
        for route in routes:
            route_counts = {0: 0, 1: 0, 2: 0}
            for node in route[1:-1]:
                route_counts[self.node_type[node]] += 1
            if route_counts[2] > self.C_d_bty:
                return False
            base_bike_load = route_counts[0] + route_counts[1]
            if base_bike_load > self.C_d:
                return False
            available_low_pickup_slots += min(
                route_counts[2], self.C_d - base_bike_load
            )
            for node_type in type_counts:
                type_counts[node_type] += route_counts[node_type]

        low_pickups = ebike_pickups - type_counts[1]
        return (
            type_counts[0] == man_pickups
            and type_counts[2] == swaps
            and 0 <= low_pickups <= available_low_pickup_slots
        )

    def _form_worker_route(self, region, arc_set):
        arc_map = dict(arc_set)
        current_node = region
        path = [current_node]
        for _ in range(len(arc_map)):
            current_node = arc_map[current_node]
            path.append(current_node)
        path.append(path[0])
        return path

    def _find_min_insertion(self, arc_set, unvisited, op_num=None):
        best_key = None
        selected_node = None
        selected_arc = None
        for start, end in arc_set:
            for node in unvisited:
                cost = (
                    self.distance_matrix[start, node]
                    + self.distance_matrix[node, end]
                    - self.distance_matrix[start, end]
                )
                operation_value = 1
                if op_num is not None:
                    operation_value = self._first_node_operation_value(node, op_num)
                candidate_key = (-operation_value, cost)
                if best_key is None or candidate_key < best_key:
                    best_key = candidate_key
                    selected_node = node
                    selected_arc = (start, end)
        return selected_node, best_key[1], selected_arc

    def _worker_two_opt(self, route):
        current = list(route)
        visited_routes = {tuple(current)}
        while len(current) >= 4:
            best_delta = 0.0
            best_start = None
            best_end = None
            for start in range(1, len(current) - 2):
                previous_node = current[start - 1]
                first_node = current[start]
                for end in range(start + 1, len(current) - 1):
                    last_node = current[end]
                    next_node = current[end + 1]
                    delta = (
                        self.distance_matrix[previous_node, last_node]
                        + self.distance_matrix[first_node, next_node]
                        - self.distance_matrix[previous_node, first_node]
                        - self.distance_matrix[last_node, next_node]
                    )
                    if delta < best_delta - 1e-12:
                        best_delta = delta
                        best_start = start
                        best_end = end
            if best_start is None:
                break
            candidate = list(current)
            candidate[best_start:best_end + 1] = reversed(
                candidate[best_start:best_end + 1]
            )
            signature = tuple(candidate)
            if signature in visited_routes:
                break
            current = candidate
            visited_routes.add(signature)
        return current

    def calculate_distance(self, route):
        return sum(
            self.distance_matrix[start, end]
            for start, end in zip(route, route[1:])
        )

    @staticmethod
    def calculate_travel_time(route, time_matrix):
        return sum(time_matrix[start, end] for start, end in zip(route, route[1:]))

    def _reset_depot_balance(self):
        start_depot = self.N_0[0]
        end_depot = self.N_0[-1]
        for b in self.B:
            self.surplus[start_depot, b] = 0
            self.deficit[start_depot, b] = 0
            self.surplus[end_depot, b] = 0
            self.deficit[end_depot, b] = 0

    def _assign_pickups(self, route, pick, available_capacity, remaining_space):
        customer_route = route[1:-1]
        for offset, region in enumerate(reversed(customer_route)):
            next_region = route[-offset - 1]
            remaining_space[region] = min(
                remaining_space[next_region] + sum(
                    -pick[next_region, b] + self.deficit[next_region, b]
                    for b in self.B
                ),
                self.C_t,
            )
            remaining_type_space = remaining_space[region]
            for b in self.B:
                available_capacity[region, b] = min(
                    available_capacity[next_region, b]
                    + self.deficit[next_region, b]
                    - pick[next_region, b],
                    remaining_type_space,
                )
                pick[region, b] = min(
                    available_capacity[region, b],
                    self.surplus[region, b],
                )
                remaining_type_space -= pick[region, b]

    def _assign_dropoffs(self, route, pick, drop, truck_load):
        for idx, region in enumerate(route[1:-1]):
            prev_region = route[idx]
            for b in self.B:
                drop[region, b] = min(
                    truck_load[prev_region, b],
                    self.deficit[region, b],
                )
                truck_load[region, b] = (
                    truck_load[prev_region, b]
                    - drop[region, b]
                    + pick[region, b]
                )

    def _assign_battery_swaps(self, route, pick, drop, battery_swap):
        for region in route[1:-1]:
            total_e_bikes = self.initial[region, 1]
            functional_e_bikes = self.initial[region, 2]
            low_battery_e_bikes = total_e_bikes - functional_e_bikes

            
            
            
            if total_e_bikes >= self.target[region, 1]:
                battery_swap[region] = max(
                    pick[region, 1]
                    + self.target[region, 1]
                    - functional_e_bikes,
                    0,
                )
            else:
                
                
                battery_swap[region] = low_battery_e_bikes

    def _build_unvisited_nodes(self, region, pick_n, battery_swap):
        unvisited = []
        for b in self.B:
            if pick_n[region, b] > 0:
                unvisited.extend(
                    i for i in self.region[region]
                    if i in self.bike_type_sets[b]
                )

        if battery_swap[region] > 0:
            existing = set(unvisited)
            unvisited.extend(
                i for i in self.region[region]
                if i in self.bike_type_sets[self.LOW_BATTERY_TYPE] and i not in existing
            )
        else:
            unvisited = [
                i for i in unvisited
                if i not in self.bike_type_sets[self.LOW_BATTERY_TYPE]
            ]
        return unvisited

    def _serve_node(self, node, op_num, bike_load, battery_load):
        type_of_node = self.node_type.get(node)
        if type_of_node == self.LOW_BATTERY_TYPE:
            battery_load += 1
            op_num[type_of_node] -= 1
            
            
            
            if op_num[1] > 0:
                bike_load += 1
                op_num[1] -= 1
        else:
            bike_load += 1
            op_num[type_of_node] -= 1
        return bike_load, battery_load

    def _remove_finished_types(self, unvisited, op_num):
        return [
            i for i in unvisited
            if op_num[self.node_type[i]] > 0
        ]

    def _feasible_unvisited_nodes(
        self,
        unvisited,
        bike_load,
        battery_load,
        op_num,
    ):
        return [
            node for node in unvisited
            if (
                (
                    node in self.bike_type_sets[self.LOW_BATTERY_TYPE]
                    and battery_load < self.C_d_bty
                    and (
                        op_num[1] == 0
                        or bike_load < self.C_d
                    )
                ) or (
                    node not in self.bike_type_sets[self.LOW_BATTERY_TYPE]
                    and bike_load < self.C_d
                )
            )
        ]

    def _evaluate_route(self, route_idx, route):
        cache_key = tuple(int(node) for node in route)
        cached = self._truck_route_evaluation_cache.get(cache_key)
        if cached is not None:
            if cached["route_index"] == route_idx:
                return cached
            cached_for_position = dict(cached)
            cached_for_position["route_index"] = route_idx
            return cached_for_position

        pick_n, drop_n, battery_swap = self.allocate_num(route)
        internal_routes = self.Inserting_algorithm(route, pick_n, drop_n, battery_swap)

        route_loading = sum(pick_n.values()) + sum(drop_n.values())
        route_bs = sum(battery_swap.values())
        route_internal_distance = sum(
            self.calculate_distance(internal_route)
            for regional_routes in internal_routes.values()
            for internal_route in regional_routes
        )
        route_truck_distance = self.calculate_distance(route)
        laborer_time = sum(
            self.calculate_travel_time(internal_route, self.laborer_time_matrix)
            for regional_routes in internal_routes.values()
            for internal_route in regional_routes
        )
        truck_time = self.calculate_travel_time(route, self.truck_time_matrix)
        service_time = self.LT * route_loading + self.ST * route_bs

        detail = {
            "route_index": route_idx,
            "truck_route": list(route),
            "internal_routes": internal_routes,
            "pick": pick_n,
            "drop": drop_n,
            "battery_swap": battery_swap,
            "loading": route_loading,
            "battery_swaps": route_bs,
            "internal_distance": route_internal_distance,
            "truck_distance": route_truck_distance,
            "truck_time": truck_time,
            "laborer_time": laborer_time,
            "service_time": service_time,
            "total_time": truck_time + laborer_time + service_time,
        }
        self._truck_route_evaluation_cache[cache_key] = detail
        return detail

    def _initialize_final_inventory(self, final):
        for region in self.V_d:
            final[region, 0] = self.initial[region, 0]
            final[region, 1] = self.initial[region, 2]

    def _update_final_inventory(self, final, region, detail):
        pick_n = detail["pick"]
        drop_n = detail["drop"]
        battery_swap = detail["battery_swap"]
        final[region, 0] = (
            self.initial[region, 0]
            - pick_n[region, 0]
            + drop_n[region, 0]
        )
        final[region, 1] = (
            self.initial[region, 2]
            - pick_n[region, 1]
            + drop_n[region, 1]
            + battery_swap[region]
        )

    def _calculate_deviation(self, final, bike_dev):
        for region in self.V_d:
            for b in self.B:
                bike_dev[region, b] = max(0, self.target[region, b] - final[region, b])
        return sum(bike_dev.values())

    def _calculate_costs(
        self,
        total_loading,
        total_bs,
        total_distance,
        truck_distance,
        total_dev,
    ):
        loading_cost = self.LC * total_loading + self.LC * total_bs
        travel_cost_D = self.TC_D * total_distance
        travel_cost_T = self.TC_T * truck_distance
        penalty_cost = self.w * total_dev
        total_cost = penalty_cost + travel_cost_D + travel_cost_T + loading_cost
        return {
            "penalty": penalty_cost,
            "travel_laborer": travel_cost_D,
            "travel_truck": travel_cost_T,
            "loading": loading_cost,
            "total": total_cost,
        }
