
import math


class HCTSInitializationHelper:
    @staticmethod
    def _default_mcp_limits(region_count):
        if region_count <= 15:
            return 60, 0.001
        if region_count <= 30:
            return 150, 0.01
        if region_count <= 60:
            return 300, 0.02
        if region_count <= 120:
            return 600, 0.03
        if region_count <= 240:
            return 900, 0.05
        return 1200, 0.05

    @staticmethod
    def _default_gamma(node_count):
        return 50

    @staticmethod
    def _default_hcts_time_limit(node_count):
        return 7200

    def _generate_heuristic_initial_routes(self):
        regions = self._sort_regions_by_angle()
        route_count = len(self.data.K)
        if route_count <= 0:
            raise ValueError("At least one truck is required")

        routes = [[self.data.N_0[0], self.data.N_0[-1]] for _ in self.data.K]
        route_loads = [0.0 for _ in self.data.K]

        for region in regions:
            route_idx = min(range(route_count), key=lambda idx: route_loads[idx])
            routes[route_idx].insert(-1, region)
            route_loads[route_idx] += self._estimated_region_workload(region)

        for route in routes:
            if len(route) > 3:
                route[1:-1] = self._nearest_neighbor_order(route[1:-1])
        return routes

    def _sort_regions_by_angle(self):
        depot = self.data.N_0[0]
        depot_x = self.data.x_cor[depot]
        depot_y = self.data.y_cor[depot]
        return sorted(
            self.data.V_d,
            key=lambda region: (
                math.atan2(
                    self.data.y_cor[region] - depot_y,
                    self.data.x_cor[region] - depot_x,
                ),
                self.data.distance_matrix[depot, region],
            ),
        )

    def _estimated_region_workload(self, region):
        handling = sum(
            abs(self.data.initial[region, b] - self.data.target[region, b])
            for b in self.data.B
        )
        return handling + self.data.distance_matrix[self.data.N_0[0], region]

    def _nearest_neighbor_order(self, regions):
        unvisited = set(regions)
        ordered = []
        current = self.data.N_0[0]
        while unvisited:
            selected = min(
                unvisited,
                key=lambda region: self.data.distance_matrix[current, region],
            )
            ordered.append(selected)
            unvisited.remove(selected)
            current = selected
        return ordered
