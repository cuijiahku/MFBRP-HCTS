try:
    from .laborer_route_refinement import LaborerRouteRefinement
    from .cache import LRUCache
    from .node_instruction_helper import NodeInstructionHelper
except ImportError:
    from laborer_route_refinement import LaborerRouteRefinement
    from cache import LRUCache
    from node_instruction_helper import NodeInstructionHelper

class NodeInstruction(LaborerRouteRefinement, NodeInstructionHelper):
    LOW_BATTERY_TYPE = 2

    def __init__(self, data):
        self.x_coord = data.x_cor
        self.y_coord = data.y_cor
        self.n_num = data.n_num
        self.LT, self.ST = data.LT, data.ST
        self.C_t = data.C_t
        self.C_d = data.C_d
        self.C_d_bty = data.C_d_bty
        self.T_max = data.T_max
        self.TC_D = data.TC_D
        self.TC_T = data.TC_T
        self.LC = data.LC
        self.w = data.w
        self.distance_matrix = data.distance_matrix
        self.truck_time_matrix = data.truck_time_matrix
        self.laborer_time_matrix = data.laborer_time_matrix
        self.region = data.region
        self.B = data.B
        self.K = data.K
        self.N_0 = data.N_0
        self.V_d = data.V_d
        self.N_c = [self.N_0[0]] + self.V_d + [self.N_0[-1]]
        self.V_0 = data.V_r
        self.V_1 = data.V_eu
        self.V_2 = [x for x in data.V_e if x not in data.V_eu]
        self.bike_type = {b: getattr(self, f"V_{b}") for b in range(3)}
        self.bike_type_sets = {
            0: set(self.V_0),
            1: set(self.V_1),
            2: set(self.V_2),
        }
        self.node_type = {
            node: bike_type
            for bike_type, nodes in self.bike_type_sets.items()
            for node in nodes
        }
        self.region_node_sets = {r: set(nodes) for r, nodes in self.region.items()}
        self.initial = data.initial
        self.target = data.target
        self.surplus = data.surplus
        self.deficit = data.deficit
        self._truck_route_evaluation_cache = LRUCache(max_entries=30_000)

    def allocate_num(self, route):
        self._reset_depot_balance()
        pick = {(r, b): 0 for r in route[1:] for b in self.B}
        drop = {(r, b): 0 for r in route[1:-1] for b in self.B}
        available_capacity = {(r, b): 0 for r in route for b in self.B}
        truck_load = {(r, b): 0 for r in route for b in self.B}
        remaining_space = {r: 0 for r in route}
        battery_swap = {r: 0 for r in route}

        self._assign_pickups(route, pick, available_capacity, remaining_space)
        self._assign_dropoffs(route, pick, drop, truck_load)
        self._assign_battery_swaps(route, pick, drop, battery_swap)
        return pick, drop, battery_swap

    def allocate_regional_route(self, region, pick_n, drop_n, battery_swap):
        unvisited = self._build_unvisited_nodes(region, pick_n, battery_swap)
        paths = []

        op_num = {
            0: pick_n[region, 0],
            1: pick_n[region, 1],
            2: battery_swap[region],
        }

        while unvisited:
            visited_arcs = []
            accumulated_cap = 0
            accumulated_bty_cap = 0
            feasible_nodes = self._feasible_unvisited_nodes(
                unvisited,
                accumulated_cap,
                accumulated_bty_cap,
                op_num,
            )
            if not feasible_nodes:
                raise ValueError(
                    f"No feasible laborer operation in region {region}; "
                    "check bike and battery capacities"
                )
            first_node = min(
                feasible_nodes,
                key=lambda node: (
                    -self._first_node_operation_value(node, op_num),
                    self.distance_matrix[region, node],
                ),
            )
            visited_arcs.append((region, first_node))
            unvisited.remove(first_node)
            accumulated_cap, accumulated_bty_cap = self._serve_node(
                first_node,
                op_num,
                accumulated_cap,
                accumulated_bty_cap,
            )
            unvisited = self._remove_finished_types(unvisited, op_num)

            while unvisited:
                feasible_nodes = self._feasible_unvisited_nodes(
                    unvisited,
                    accumulated_cap,
                    accumulated_bty_cap,
                    op_num,
                )
                if not feasible_nodes:
                    break
                selected_node, _, insertion_arc = self._find_min_insertion(
                    visited_arcs,
                    feasible_nodes,
                    op_num,
                )
                unvisited.remove(selected_node)
                accumulated_cap, accumulated_bty_cap = self._serve_node(
                    selected_node,
                    op_num,
                    accumulated_cap,
                    accumulated_bty_cap,
                )
                visited_arcs.extend([
                    (insertion_arc[0], selected_node),
                    (selected_node, insertion_arc[1]),
                ])
                visited_arcs.remove(insertion_arc)
                unvisited = self._remove_finished_types(unvisited, op_num)

            paths.append(self._worker_two_opt(
                self._form_worker_route(region, visited_arcs)
            ))
        return paths

    def _first_node_operation_value(self, node, op_num):
        if (
            self.node_type[node] == self.LOW_BATTERY_TYPE
            and op_num[1] > 0
            and op_num[2] > 0
        ):
            return 2
        return 1

    def local_search_worker_routes(self, routes):
        return [self._worker_two_opt(route) for route in routes]


    def Inserting_algorithm(self, route, pick_n, drop_n, battery_swap):
        internal_routes = {} 
        for r in route[1:-1]:
            internal_routes[r] = self.allocate_regional_route(r, pick_n, drop_n, battery_swap)
        return internal_routes
    
    def Evaluate_fitness(self, input_route):
        details = self.Evaluate_solution(input_route)
        return details["total_cost"], details["route_times"]

    def evaluate_route_time(self, route):
        return self._evaluate_route(0, route)["total_time"]

    def Evaluate_solution(self, input_route, previous_details=None):
        final = {}
        bike_dev = {}
        total_loading = 0
        total_bs = 0
        total_distance = 0
        truck_distance = 0
        route_times = [] 
        route_details = []
        self._initialize_final_inventory(final)

        previous_routes = (
            previous_details.get("routes", [])
            if previous_details is not None
            else []
        )
        for route_idx, route in enumerate(input_route):
            if (
                route_idx < len(previous_routes)
                and previous_routes[route_idx]["truck_route"] == list(route)
            ):
                detail = previous_routes[route_idx]
            else:
                detail = self._evaluate_route(route_idx, route)

            total_loading += detail["loading"]
            total_bs += detail["battery_swaps"]
            total_distance += detail["internal_distance"]
            truck_distance += detail["truck_distance"]
            route_times.append(detail["total_time"])
            route_details.append(detail)
            
            for region in route[1:-1]:
                self._update_final_inventory(final, region, detail)

        total_dev = self._calculate_deviation(final, bike_dev)
        costs = self._calculate_costs(
            total_loading,
            total_bs,
            total_distance,
            truck_distance,
            total_dev,
        )
        return {
            "total_cost": costs["total"],
            "route_times": route_times,
            "routes": route_details,
            "final": final,
            "bike_dev": bike_dev,
            "totals": {
                "loading": total_loading,
                "battery_swaps": total_bs,
                "internal_distance": total_distance,
                "truck_distance": truck_distance,
                "deviation": total_dev,
            },
            "costs": costs,
        }
