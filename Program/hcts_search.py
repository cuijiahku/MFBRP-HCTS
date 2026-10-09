
import math
import random

class HCTS_Search:
    def _termination_condition(self):
        if self.no_improve_count >= self.max_iter_no_improve:
            print(f"Reached maximum iterations without improvement {self.max_iter_no_improve}")
            return True
        if (self.elapsed_time()) >= self.max_cpu_time:
            print(f"Reached maximum CPU time {self.max_cpu_time} seconds")
            return True
        return False

    def _generate_candidates(self, individual):
        total_nodes = len(self.data.N_0)
        candidate_size = self.gamma + (total_nodes ** 0.3) * math.log(total_nodes)
        candidate_size = int(candidate_size)

        candidates = []
        seen = {self._route_signature(individual.truck_routes)}
        attempts = 0
        max_attempts = candidate_size + max(10, candidate_size // 4)
        while len(candidates) < candidate_size and attempts < max_attempts:
            if self.elapsed_time() >= self.max_cpu_time:
                break
            attempts += 1
            move_func = random.choice(self.moves)
            new_individual = move_func(
                individual,
                self.data,
                self.evaluator,
                self.evaluation_cache,
            )
            if new_individual is None:
                continue
            signature = self._route_signature(new_individual.truck_routes)
            if signature in seen:
                continue
            seen.add(signature)
            candidates.append(new_individual)
        return candidates

    def _intensify_from_incumbent(self, current_individual):
        interval = self.intensification_interval
        if (
            interval is None
            or interval <= 0
            or self.no_improve_count == 0
            or self.no_improve_count % interval != 0
        ):
            return current_individual

        self.tabu_list.clear()
        return self.best_individual

    def _select_best_candidate(self, candidates, current_individual):
        self._clean_tabu_list()
        best = None
        best_fit = float("inf")

        for cand in candidates:
            solution_key = self._route_signature(cand.truck_routes)
            is_tabu = solution_key in self.tabu_list

            if cand.fit_v < self.best_fitness:
                is_tabu = False

            if not is_tabu and cand.fit_v < best_fit:
                best = cand
                best_fit = cand.fit_v

        return best, best_fit

    def _add_solution_to_tabu(self, new_individual):
        solution_key = self._route_signature(new_individual.truck_routes)
        self.tabu_list[solution_key] = self.iteration + self.tabu_tenure
        self._clean_tabu_list()

    def _clean_tabu_list(self):
        expired = [k for k, v in self.tabu_list.items() if v <= self.iteration]
        for k in expired:
            del self.tabu_list[k]

    @staticmethod
    def _route_signature(truck_routes):
        return tuple(tuple(int(node) for node in route) for route in truck_routes)
