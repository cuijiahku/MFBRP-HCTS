
import gurobipy as gp
import numpy as np
import math
import time
try:
    from .mcp_helper import MCPHelper, DeltaPlus, DeltaMinus
except ImportError:
    from mcp_helper import MCPHelper, DeltaPlus, DeltaMinus

class Model_clst(MCPHelper):
    def __init__(self, data):
        self.x_coord = data.x_cor
        self.y_coord = data.y_cor
        self.distance_matrix = data.distance_matrix
        self.truck_time_matrix = data.truck_time_matrix
        self.LT, self.ST = data.LT, data.ST
        self.LC = data.LC
        self.C_d_bty = data.C_d_bty
        self.T_max = data.T_max
        self.w = data.w
        self.region = data.region
        self.B = data.B
        self.K = data.K
        self.n_k = len(self.K)
        self.N_0 = data.N_0
        self.V_d = data.V_d
        self.N_c = [self.N_0[0]] + self.V_d + [self.N_0[-1]]
        self.E = [(s,t) for s in self.N_c for t in self.N_c if s!=t]
        self.route_order_bound = len(self.N_c)
        self.initial = data.initial
        self.target = data.target

        self._initialize_estimated_service_quantities()

        self.m = gp.Model("MCP")
        self._define_variables()
        self._add_constraints()
        self._set_objective()

    def _define_variables(self):
        self.z = self.m.addVars(((r,k) for r in self.N_c for k in self.K),
                        vtype = gp.GRB.BINARY,
                        name = 'z')
        self.y = self.m.addVars(((s,t,k) for (s,t) in self.E for k in self.K),
                        vtype = gp.GRB.BINARY,
                        name = 'y')
        self.f = self.m.addVars(((s,k) for s in self.N_c for k in self.K),
                        vtype = gp.GRB.CONTINUOUS, lb=0.0, name = 'f')
        self.b_sur = self.m.addVars(((b,k) for b in self.B for k in self.K),
                        vtype = gp.GRB.CONTINUOUS, lb=0.0, name = 'b_sur')
        self.b_def = self.m.addVars(((b,k) for b in self.B for k in self.K),
                        vtype = gp.GRB.CONTINUOUS, lb=0.0, name = 'b_def')
        self.T_total = self.m.addVar(vtype = gp.GRB.CONTINUOUS, lb=0.0, name ='T_total')

    def _add_constraints(self):
        for r in self.V_d:
            self.m.addConstr(gp.quicksum(self.z[r, k] for k in self.K) == 1,
                             name=f'assign_{r}')

        for k in self.K:
            for t in self.V_d:
                self.m.addConstr(
                    gp.quicksum(self.y[s, t, k] for s in self.N_c if s != t) == self.z[t, k],
                    name=f'flow_in_{t}_{k}')
                self.m.addConstr(
                    gp.quicksum(self.y[t, s, k] for s in self.N_c if s != t) == self.z[t, k],
                    name=f'flow_out_{t}_{k}')

        for k in self.K:
            self.m.addConstr(
                gp.quicksum(self.y[0, t, k] for t in self.V_d) == 1,
                name=f'start_{k}')
            self.m.addConstr(
                gp.quicksum(self.y[t, self.N_0[-1], k] for t in self.V_d) == 1,
                name=f'end_{k}')

        for k in self.K:
            for (s, t) in self.E:
                self.m.addConstr(
                    self.f[t, k] - self.f[s, k] >= 1 - self.route_order_bound * (1 - self.y[s, t, k]),
                    name=f'flow_conservation_{s}_{t}_{k}')
            for t in self.N_c:
                self.m.addConstr(
                    self.f[t, k] <= self.route_order_bound,
                    name=f'capacity_{t}_{k}')

        for k in self.K:
            for b in self.B:
                lhs = gp.quicksum(self.z[r, k] * (self.initial[r, b] - self.target[r, b])
                                  for r in self.V_d)
                self.m.addConstr(lhs == self.b_def[b, k] - self.b_sur[b, k],
                                 name=f'balance_{b}_{k}')

        for k in self.K:
            travel_time = gp.quicksum(
                self.y[s, t, k] * self.truck_time_matrix[s, t]
                for (s, t) in self.E
            )
            service_time = gp.quicksum(
                self.z[r, k] * (
                    self.LT * self._estimated_handling_quantity(r) +
                    self.ST * self.s[r]
                )
                for r in self.V_d
            )
            self.m.addConstr(travel_time + service_time <= self.T_total,
                             name=f'time_{k}')

    def _set_objective(self):
        penalty = self.w * gp.quicksum(self.b_sur[b, k] + self.b_def[b, k] for b in self.B for k in self.K)
        self.m.setObjective(self.T_total + penalty, sense=gp.GRB.MINIMIZE)

    def _set_route_mip_start(self, initial_routes):
        if initial_routes is None:
            return
        self._validate_initial_routes(initial_routes)

        for variable in self.z.values():
            variable.Start = 0.0
        for variable in self.y.values():
            variable.Start = 0.0
        for variable in self.f.values():
            variable.Start = 0.0

        for k in self.K:
            route = [int(node) for node in initial_routes[k]]
            for position, node in enumerate(route):
                self.f[node, k].Start = float(position)
            for region in route[1:-1]:
                self.z[region, k].Start = 1.0
            for start, end in zip(route, route[1:]):
                self.y[start, end, k].Start = 1.0

    def Solve(self, time_limit=3600, mip_gap=0.02, initial_routes=None):
        self._set_route_mip_start(initial_routes)
        self.m.Params.MIPGap = mip_gap
        self.m.Params.timeLimit = time_limit
        self.m.Params.LogToConsole=False
        self.m.Params.Threads = 8
        self.m.Params.MIPFocus = 1
        self.m.Params.Heuristics = 0.2
        self.m.Params.Presolve = 2
        start = time.time()
        self.m.optimize()
        print(f"MCP solve time: {time.time() - start:.2f} sec")

        if self.m.SolCount == 0:
            return None

        solution  = {}
        delimiter = []
        count = 0
        for k in self.K:
            for (s,t) in self.E:
                if math.isclose(self.y[s,t,k].X,1):
                    solution[s,t,k] = 1
                    count = count+1
            delimiter.append(count)

        route = np.array(list(solution.keys()))
        if len(route) == 0:
            return None

        return self._extract_routes(route, delimiter)


