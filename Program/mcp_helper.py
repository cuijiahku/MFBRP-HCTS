
from copy import deepcopy


def DeltaPlus(i,V):
    delta_plus = deepcopy(V)
    delta_plus.remove(0)
    if i != 0:
        delta_plus.remove(i)
    return delta_plus

def DeltaMinus(i,V):
    delta_minus = deepcopy(V)
    if i != delta_minus[-1]:
        delta_minus.remove(i)
        del delta_minus[-1]
    else:
        del delta_minus[-1]
    return delta_minus


class MCPHelper:
    def _initialize_estimated_service_quantities(self):
        self.p = {}
        self.d = {}
        self.s = {}
        for r in self.V_d:
            self.s[r] = self.initial[r,1]-self.initial[r,2]
            for b in self.B:
                self.p[r,b] = max(0, self.initial[r,b]-self.target[r,b])
                self.d[r,b] = max(0, self.target[r,b]-self.initial[r,b])

    def _estimated_handling_quantity(self, region):
        return sum(
            self.p[region, b] + self.d[region, b]
            for b in self.B
        )

    def _extract_routes(self, route, delimiter):
        final_route = [[] for _ in self.K]
        start_idx = 0
        for end_idx in delimiter:
            route_for_k = route[start_idx:end_idx]
            start_idx = end_idx
            if len(route_for_k) == 0:
                continue
            truck_idx = int(route_for_k[0, -1])
            final_route[truck_idx] = self._trace_route(route_for_k)
        return final_route

    def _trace_route(self, route_for_k):
        if route_for_k[0, 0] == 0 and route_for_k[0, 1] == self.N_0[-1]:
            return [self.N_0[0], self.N_0[-1]]

        next_node = {int(start): int(end) for start, end, _ in route_for_k}
        start = self.N_0[0]
        end = self.N_0[-1]
        route = [start]
        visited = {start}
        for _ in range(len(self.V_d) + 2):
            current = route[-1]
            if current == end:
                return route
            if current not in next_node:
                raise ValueError(f"MCP route stops before the end depot at node {current}")
            following = next_node[current]
            if following in visited:
                raise ValueError(f"MCP route contains a cycle at node {following}")
            route.append(following)
            visited.add(following)
        raise ValueError("MCP route exceeds the maximum possible route length")

    def _validate_initial_routes(self, initial_routes):
        if len(initial_routes) != len(self.K):
            raise ValueError("MCP warm start must contain one route per truck")

        valid_regions = set(self.V_d)
        assigned_regions = []

        for k in self.K:
            route = [int(node) for node in initial_routes[k]]
            if route[0] != self.N_0[0] or route[-1] != self.N_0[-1]:
                raise ValueError(f"MCP warm-start route {k} has invalid depots")
            if not route[1:-1]:
                raise ValueError(f"MCP warm-start route {k} visits no region")
            if any(region not in valid_regions for region in route[1:-1]):
                raise ValueError(f"MCP warm-start route {k} contains an invalid region")
            assigned_regions.extend(route[1:-1])

        if len(assigned_regions) != len(set(assigned_regions)) or set(assigned_regions) != valid_regions:
            raise ValueError("MCP warm start must assign every region exactly once")
