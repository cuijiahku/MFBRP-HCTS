
import time
from tqdm import tqdm

try:
    from .main_algorithm_helper import HCTSInitializationHelper
    from .individual import Individual
    from .MCP import Model_clst
    from .node_instruction import NodeInstruction
    from .cache import LRUCache
    from .localsearch_moves import get_local_search_moves
    from .hcts_search import HCTS_Search
except ImportError:
    from main_algorithm_helper import HCTSInitializationHelper
    from individual import Individual
    from MCP import Model_clst
    from node_instruction import NodeInstruction
    from cache import LRUCache
    from localsearch_moves import get_local_search_moves
    from hcts_search import HCTS_Search

class HCTS_algorithm(HCTSInitializationHelper, HCTS_Search):
    def __init__(self, data, config=None):
        self.data = data
        self.max_iter_no_improve = 5000
        self.max_cpu_time = self._default_hcts_time_limit(data.n_num)
        self.tabu_tenure = 50
        self.gamma = self._default_gamma(data.n_num)
        self.intensification_interval = 250
        self.mcp_time_limit, self.mcp_mip_gap = self._default_mcp_limits(
            len(data.V_d)
        )
        self.initial_solution_method = "mcp"
        self.moves = get_local_search_moves()

        if config:
            for key, value in config.items():
                if hasattr(self, key):
                    setattr(self, key, value)

        self.tabu_list = {}
        self.evaluator = NodeInstruction(data)
        self.evaluation_cache = LRUCache(max_entries=10_000)
        self.best_individual = None
        self.best_fitness = float('inf')
        self.iteration = 0
        self.no_improve_count = 0
        self.start_time = None

    def run(self, data):
        self.start_time = time.time()
        print("start generating initial solution...")

        initial_individual = self._generate_initial_solution()
        self.best_individual = initial_individual
        self.best_fitness = initial_individual.fit_v
        current_individual = initial_individual

        pbar = tqdm(desc="Tabu Search", unit="iter")
        while not self._termination_condition():
            current_individual = self._intensify_from_incumbent(current_individual)
            candidate_set = self._generate_candidates(current_individual)
            if not candidate_set and self.elapsed_time() >= self.max_cpu_time:
                break
            best_candidate, best_candidate_fitness = self._select_best_candidate(
                candidate_set, current_individual
            )

            if best_candidate is None:
                self.no_improve_count += 1
                self.iteration += 1
                pbar.update(1)
                pbar.set_postfix(best=self.best_fitness, no_improve=self.no_improve_count)
                continue

            current_individual = best_candidate

            if best_candidate_fitness < self.best_fitness:
                self.best_fitness = best_candidate_fitness
                self.best_individual = best_candidate
                self.no_improve_count = 0
                tqdm.write(f"* New best solution: {self.best_fitness:.2f} (iter {self.iteration})")
            else:
                self.no_improve_count += 1

            self._add_solution_to_tabu(current_individual)

            self.iteration += 1
            pbar.update(1)
            pbar.set_postfix(best=self.best_fitness, no_improve=self.no_improve_count)

        pbar.close()
        refined_details = self.evaluator.refine_final_worker_routes(
            self.best_individual.solution_details
        )
        self.best_individual._store_solution_details(refined_details)
        self.best_individual.fit_v = refined_details["total_cost"]
        self.best_fitness = self.best_individual.fit_v
        end_time = time.time()
        print(f"\nTabu Search completed. Iterations: {self.iteration}, Best Fitness: {self.best_fitness:.2f}")
        print(f"Total CPU time: {end_time - self.start_time:.2f} seconds")

        return self.best_fitness, self.best_individual

    def elapsed_time(self):
        return time.time() - self.start_time

    def _generate_initial_solution(self):
        truck_routes = None
        if self.initial_solution_method in ("mcp", "auto"):
            warm_start_routes = self._generate_heuristic_initial_routes()
            remaining_time = max(self.max_cpu_time - self.elapsed_time(), 0.01)
            effective_mcp_time_limit = min(self.mcp_time_limit, remaining_time)
            print(
                f"MCP settings: time limit={effective_mcp_time_limit:.2f}s, "
                f"MIP gap={100 * self.mcp_mip_gap:g}%"
            )
            mcp = Model_clst(self.data)
            truck_routes = mcp.Solve(
                time_limit=effective_mcp_time_limit,
                mip_gap=self.mcp_mip_gap,
                initial_routes=warm_start_routes,
            )
            if truck_routes is None:
                print("MCP did not return an incumbent; using heuristic initial solution.")

        if truck_routes is None:
            truck_routes = (
                warm_start_routes
                if self.initial_solution_method in ("mcp", "auto")
                else self._generate_heuristic_initial_routes()
            )

        individual = Individual(
            init=False,
            data=self.data,
            truck_routes=truck_routes,
            evaluator=self.evaluator,
            evaluation_cache=self.evaluation_cache,
        )
        return individual


