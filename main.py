import heapq
import random
import math
from collections import deque
import time
import tkinter as tk
from tkinter import ttk
import threading

#Environment & Grid Cell

class GridCell:
    def __init__(self, terrain_cost=1, is_obstacle=False):
        self.terrain_cost = terrain_cost
        self.is_obstacle = is_obstacle

class GridEnvironment:
    def __init__(self):
        self.width = 0
        self.height = 0
        self.grid = []
        self.packages = []
        self.agent_position = (0, 0)
        self.fuel_limit = 1000
        self.time_limit = 1000
        self.moving_obstacles = []  

    def is_blocked(self, pos, time_step):
        r, c = pos
        if not (0 <= r < self.height and 0 <= c < self.width):
            return True
        if self.grid[r][c].is_obstacle:
            return True
        for sched in self.moving_obstacles:
            if isinstance(sched, dict):
                if time_step in sched and sched[time_step] == pos:
                    return True
            elif isinstance(sched, list):
                if time_step < len(sched) and sched[time_step] == pos:
                    return True
        return False

    def get_cost(self, pos, time_step):
        """Return cost if not blocked, else inf."""
        if self.is_blocked(pos, time_step):
            return float('inf')
        r, c = pos
        return self.grid[r][c].terrain_cost

    def compute_path_cost(self, path, start_time=0):
        """Sum terrain costs of path excluding the start cell, using arrival times."""
        if not path:
            return None
        total = 0
        for i, pos in enumerate(path):
            if i == 0:
                continue
            t = start_time + i  
            cost = self.get_cost(pos, t)
            if cost == float('inf'):
                return float('inf')
            total += cost
        return total

    def load_from_file(self, filename):
        """Populate environment for test maps. For 'dynamic.map' we include a moving obstacle schedule."""
        if filename == "small.map":
            self.width, self.height = 5, 5
            self.grid = [[GridCell() for _ in range(self.width)] for _ in range(self.height)]
            self.grid[2][2].is_obstacle = True
            self.grid[3][1].is_obstacle = True
            self.packages = [((1, 1), (4, 4))]
            self.agent_position = (0, 0)
            self.moving_obstacles = []
        elif filename == "medium.map":
            self.width, self.height = 10, 10
            self.grid = [[GridCell() for _ in range(self.width)] for _ in range(self.height)]
            for i in range(3, 7):
                for j in range(3, 7):
                    self.grid[i][j].terrain_cost = 3
            self.grid[5][5].is_obstacle = True
            self.grid[6][2].is_obstacle = True
            self.grid[7][7].is_obstacle = True
            self.packages = [((0, 1), (9, 9)), ((2, 2), (8, 8))]
            self.agent_position = (0, 0)
            self.moving_obstacles = []
        elif filename == "large.map":
            self.width, self.height = 10, 10
            self.grid = [[GridCell() for _ in range(self.width)] for _ in range(self.height)]
            for i in range(self.height):
                for j in range(self.width):
                    if (i + j) % 5 == 0 and (i, j) not in [(0,0),(1,1),(9,9)]:
                        self.grid[i][j].is_obstacle = True
            self.packages = [((1, 1), (9, 9)), ((2, 2), (8, 8))]
            self.agent_position = (0, 0)
            self.moving_obstacles = []
        elif filename == "dynamic.map":
            self.width, self.height = 8, 8
            self.grid = [[GridCell() for _ in range(self.width)] for _ in range(self.height)]
            self.grid[2][2].is_obstacle = True
            self.grid[2][3].is_obstacle = True
            self.grid[3][3].is_obstacle = True
            self.packages = [((1, 1), (7, 7))]
            self.agent_position = (0, 0)
            schedule = {}
            path = [(3, c) for c in range(0, self.width)]
            for t in range(0, 40):
                schedule[t] = path[t % len(path)]
            self.moving_obstacles = [schedule]

#Planners

class UninformedPlanner:
    def __init__(self, environment):
        self.env = environment

    def get_neighbors(self, position, arrival_time):
        """Return neighbors as (pos, cost) where arrival_time is the time index when neighbor is reached."""
        x, y = position
        neighbors = []
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:  # right, down, left, up
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.env.height and 0 <= ny < self.env.width:
                cost = self.env.get_cost((nx, ny), arrival_time)
                if cost < float('inf'):
                    neighbors.append(((nx, ny), cost))
        return neighbors

class BFSPlanner(UninformedPlanner):
    def plan(self, start, goal):
        queue = deque([(start, [start])])
        visited = set([start])
        nodes_expanded = 0
        while queue:
            position, path = queue.popleft()
            nodes_expanded += 1
            if position == goal:
                return path[1:], nodes_expanded
            for neighbor, _ in self.get_neighbors(position, len(path)):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))
        return None, nodes_expanded

class UniformCostPlanner(UninformedPlanner):
    def plan(self, start, goal):
        priority_queue = [(0, start, [start])]
        visited = set()
        nodes_expanded = 0
        while priority_queue:
            cost, position, path = heapq.heappop(priority_queue)
            nodes_expanded += 1
            if position in visited:
                continue
            if position == goal:
                return path[1:], nodes_expanded
            visited.add(position)
            for neighbor, move_cost in self.get_neighbors(position, len(path)):
                if neighbor not in visited:
                    new_cost = cost + move_cost
                    heapq.heappush(priority_queue, (new_cost, neighbor, path + [neighbor]))
        return None, nodes_expanded

class AStarPlanner(UninformedPlanner):
    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def plan(self, start, goal):
        open_set = [(self.heuristic(start, goal), 0, start, [start])]
        closed = set()
        nodes_expanded = 0
        while open_set:
            f, g, current, path = heapq.heappop(open_set)
            nodes_expanded += 1
            if current == goal:
                return path[1:], nodes_expanded
            if (current, len(path)) in closed:
                continue
            closed.add((current, len(path)))
            arrival_time = len(path) 
            for neighbor, move_cost in self.get_neighbors(current, arrival_time):
                if (neighbor, len(path)+1) in closed:
                    continue
                tentative_g = g + move_cost
                heapq.heappush(open_set, (tentative_g + self.heuristic(neighbor, goal), tentative_g, neighbor, path + [neighbor]))
        return None, nodes_expanded

class HillClimbingPlanner(UninformedPlanner):
    def __init__(self, environment, restarts=5):
        super().__init__(environment)
        self.restarts = restarts

    def plan(self, start, goal):
        best_path = None
        best_cost = float('inf')
        nodes_expanded = 0
        for _ in range(self.restarts):
            current = start
            path = [start]
            visited = set([start])
            steps = 0
            while current != goal and steps < (self.env.width * self.env.height * 2):
                neighbors = [n for n, _ in self.get_neighbors(current, len(path))]
                nodes_expanded += len(neighbors)
                neighbors = [n for n in neighbors if n not in visited]
                if not neighbors:
                    break
                neighbors.sort(key=lambda n: abs(n[0]-goal[0]) + abs(n[1]-goal[1]))
                if random.random() < 0.3 and len(neighbors) > 1:
                    next_pos = random.choice(neighbors[:min(3,len(neighbors))])
                else:
                    next_pos = neighbors[0]
                path.append(next_pos)
                visited.add(next_pos)
                current = next_pos
                steps += 1
            if current == goal:
                cost = self.env.compute_path_cost(path, start_time=0)
                if cost is not None and cost < best_cost:
                    best_cost = cost
                    best_path = path[1:]
        return best_path, nodes_expanded

class SimulatedAnnealingPlanner(UninformedPlanner):
    def __init__(self, environment, initial_temp=50, cooling_rate=0.95):
        super().__init__(environment)
        self.initial_temp = initial_temp
        self.cooling_rate = cooling_rate

    def plan(self, start, goal):
        current_path = self.generate_random_path(start, goal, max_length=self.env.width*self.env.height//2)
        if not current_path:
            return None, 0
        current_cost = self.env.compute_path_cost(current_path, start_time=0)
        temperature = self.initial_temp
        nodes_expanded = 0
        iters = 0
        while temperature > 0.5 and iters < 1000:
            new_path = self.mutate_path(current_path, goal)
            nodes_expanded += 1
            new_cost = self.env.compute_path_cost(new_path, start_time=0)
            if new_cost is not None and (new_cost < current_cost or random.random() < math.exp((current_cost - new_cost) / max(1e-9, temperature))):
                current_path = new_path
                current_cost = new_cost
            temperature *= self.cooling_rate
            iters += 1
        if current_path and current_path[-1] == goal:
            return current_path[1:], nodes_expanded
        return None, nodes_expanded

    def generate_random_path(self, start, goal, max_length=50):
        path = [start]
        current = start
        for _ in range(max_length):
            if current == goal:
                break
            neighbors = [n for n, _ in self.get_neighbors(current, len(path))]
            if not neighbors:
                break
            current = random.choice(neighbors)
            path.append(current)
        return path

    def mutate_path(self, path, goal):
        new_path = path.copy()
        if len(new_path) > 2 and random.random() < 0.5:
            idx = random.randint(1, len(new_path)-1)
            neighbors = [n for n, _ in self.get_neighbors(new_path[idx-1], idx)]
            if neighbors:
                new_path[idx] = random.choice(neighbors)
        else:
            if new_path[-1] != goal:
                neighbors = [n for n, _ in self.get_neighbors(new_path[-1], len(new_path))]
                if neighbors:
                    new_path.append(random.choice(neighbors))
        return new_path

#Delivery agent & GUI

class DeliveryAgent:
    def __init__(self, environment, planner_type='astar', gui_callback=None):
        self.env = environment
        self.position = environment.agent_position
        self.packages = environment.packages.copy()
        self.delivered = []
        self.fuel_remaining = environment.fuel_limit
        self.time_elapsed = 0
        self.planner_type = planner_type
        self.planner = self.create_planner(planner_type)
        self.log = []
        self.gui_callback = gui_callback

    def create_planner(self, planner_type):
        if planner_type == 'bfs':
            return BFSPlanner(self.env)
        elif planner_type == 'ucs':
            return UniformCostPlanner(self.env)
        elif planner_type == 'astar':
            return AStarPlanner(self.env)
        elif planner_type == 'hillclimb':
            return HillClimbingPlanner(self.env)
        elif planner_type == 'simanneal':
            return SimulatedAnnealingPlanner(self.env)
        else:
            return AStarPlanner(self.env)

    def plan_path(self, start, goal):
        """Return path (excluding start) and nodes_expanded."""
        return self.planner.plan(start, goal)

    def execute_delivery(self):
        self.log.append(f"Starting delivery using {self.planner_type} planner")
        self.log.append(f"Starting at: {self.position}, Fuel left: {self.fuel_remaining}")
        while self.packages and self.fuel_remaining > 0 and self.time_elapsed < self.env.time_limit:
            package_pos, destination = self.packages[0]
            self.log.append(f"Finding path to package at {package_pos} (time={self.time_elapsed})")
            path_to_package, nodes_expanded = self.plan_path(self.position, package_pos)
            path_cost = None
            if path_to_package:
                full_path = [self.position] + path_to_package
                path_cost = self.env.compute_path_cost(full_path, start_time=self.time_elapsed)
            self.log.append(f"Nodes expanded: {nodes_expanded}, Path cost: {path_cost}")
            if not path_to_package:
                self.log.append("Could not find a way to the package!")
                return False
            if not self.follow_path(path_to_package, package_pos):
                return False
            self.log.append(f"Got the package at {package_pos}")
            self.packages.pop(0)
            self.log.append(f"Finding path to delivery point at {destination} (time={self.time_elapsed})")
            path_to_dest, nodes_expanded = self.plan_path(self.position, destination)
            path_cost = None
            if path_to_dest:
                full_path = [self.position] + path_to_dest
                path_cost = self.env.compute_path_cost(full_path, start_time=self.time_elapsed)
            self.log.append(f"Nodes expanded: {nodes_expanded}, Path cost: {path_cost}")
            if not path_to_dest:
                self.log.append("Could not find a way to the destination!")
                return False
            if not self.follow_path(path_to_dest, destination):
                return False
            self.log.append(f"Package delivered to {destination}")
            self.delivered.append((package_pos, destination))
        success = len(self.packages) == 0
        self.log.append(f"Delivery finished: {success}")
        self.log.append(f"Packages delivered: {len(self.delivered)}")
        self.log.append(f"Fuel left: {self.fuel_remaining}")
        self.log.append(f"Time taken: {self.time_elapsed}")
        return success

    def follow_path(self, path, target):
        if not path:
            return False
        for next_pos in path:
            if self.fuel_remaining <= 0:
                self.log.append("Ran out of fuel!")
                return False
            arrival_time = self.time_elapsed + 1
            cell_cost = self.env.get_cost(next_pos, arrival_time)
            if cell_cost == float('inf'):
                self.log.append(f"Hit a blocked cell at {next_pos} at time {arrival_time}, replanning...")
                new_path, nodes_expanded = self.plan_path(self.position, target)
                self.log.append(f"Replan nodes expanded: {nodes_expanded}")
                if not new_path:
                    self.log.append("Could not find a new path!")
                    return False
                return self.follow_path(new_path, target)
            self.fuel_remaining -= cell_cost
            self.time_elapsed += 1
            self.position = next_pos
            if self.gui_callback:
                self.gui_callback(self)
                time.sleep(0.15)
            if self.time_elapsed % 5 == 0:
                self.log.append(f"Time {self.time_elapsed}: Moved to {next_pos}, fuel: {self.fuel_remaining}")
        return True

#Tkinter GUI App

class DeliveryApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Autonomous Delivery Agent")
        self.root.geometry("1000x700")
        self.env = None
        self.agent = None
        self.running = False
        self.create_widgets()

    def create_widgets(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        control_frame = ttk.LabelFrame(main_frame, text="Controls", padding="5")
        control_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        ttk.Label(control_frame, text="Map:").grid(row=0, column=0, sticky=tk.W)
        self.map_var = tk.StringVar(value="small.map")
        map_combo = ttk.Combobox(control_frame, textvariable=self.map_var,
                                 values=["small.map", "medium.map", "large.map", "dynamic.map"],
                                 width=15)
        map_combo.grid(row=0, column=1, padx=5)
        ttk.Label(control_frame, text="Algorithm:").grid(row=0, column=2, sticky=tk.W, padx=(20,0))
        self.algo_var = tk.StringVar(value="astar")
        algo_combo = ttk.Combobox(control_frame, textvariable=self.algo_var,
                                  values=["bfs", "ucs", "astar", "hillclimb", "simanneal"], width=15)
        algo_combo.grid(row=0, column=3, padx=5)
        ttk.Button(control_frame, text="Run", command=self.run_simulation).grid(row=0, column=4, padx=10)
        ttk.Button(control_frame, text="Compare All", command=self.compare_algorithms).grid(row=0, column=5, padx=10)
        ttk.Button(control_frame, text="Reset", command=self.reset).grid(row=0, column=6, padx=10)
        viz_frame = ttk.LabelFrame(main_frame, text="Visualization", padding="5")
        viz_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        self.canvas = tk.Canvas(viz_frame, width=600, height=600, bg="white")
        self.canvas.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        info_frame = ttk.LabelFrame(main_frame, text="Information", padding="5")
        info_frame.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)
        self.info_text = tk.Text(info_frame, width=40, height=35)
        self.info_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        scrollbar = ttk.Scrollbar(info_frame, orient=tk.VERTICAL, command=self.info_text.yview)
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.info_text.configure(yscrollcommand=scrollbar.set)
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=0)
        main_frame.rowconfigure(1, weight=1)
        viz_frame.columnconfigure(0, weight=1)
        viz_frame.rowconfigure(0, weight=1)
        info_frame.columnconfigure(0, weight=1)
        info_frame.rowconfigure(0, weight=1)

    def update_gui(self, agent):
        self.info_text.delete(1.0, tk.END)
        for log_entry in agent.log[-50:]:
            self.info_text.insert(tk.END, log_entry + "\n")
        self.info_text.see(tk.END)
        self.draw_grid(agent)
        self.root.update()

    def draw_grid(self, agent):
        self.canvas.delete("all")
        if not self.env:
            return
        cell_size = min(600 // self.env.width, 600 // self.env.height)
        for i in range(self.env.height):
            for j in range(self.env.width):
                x1, y1 = j * cell_size, i * cell_size
                x2, y2 = x1 + cell_size, y1 + cell_size
                if self.env.grid[i][j].is_obstacle:
                    color = "black"
                elif self.env.grid[i][j].terrain_cost > 1:
                    color = "lightgreen"
                else:
                    color = "white"
                self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline="gray")
                if self.env.grid[i][j].terrain_cost > 1:
                    self.canvas.create_text(x1 + cell_size//2, y1 + cell_size//2,
                                            text=str(self.env.grid[i][j].terrain_cost))
        t = agent.time_elapsed
        for sched in self.env.moving_obstacles:
            if isinstance(sched, dict) and t in sched:
                r,c = sched[t]
                cx, cy = c*cell_size + cell_size//2, r*cell_size + cell_size//2
                self.canvas.create_rectangle(cx-8, cy-8, cx+8, cy+8, fill="gray", outline="black")
        for package_pos, destination in self.env.packages + agent.delivered:
            px, py = package_pos[1]*cell_size + cell_size//2, package_pos[0]*cell_size + cell_size//2
            dx, dy = destination[1]*cell_size + cell_size//2, destination[0]*cell_size + cell_size//2
            self.canvas.create_oval(px-5, py-5, px+5, py+5, fill="blue", outline="blue")
            self.canvas.create_rectangle(dx-5, dy-5, dx+5, dy+5, fill="red", outline="red")
        ax, ay = agent.position[1]*cell_size + cell_size//2, agent.position[0]*cell_size + cell_size//2
        self.canvas.create_oval(ax-8, ay-8, ax+8, ay+8, fill="yellow", outline="black")
        stats_text = f"Fuel: {agent.fuel_remaining} | Time: {agent.time_elapsed} | Delivered: {len(agent.delivered)}/{len(self.env.packages + agent.delivered)}"
        self.canvas.create_text(300, 10, text=stats_text, anchor=tk.N, fill="black")

    def run_simulation(self):
        if self.running:
            return
        self.running = True
        self.info_text.delete(1.0, tk.END)
        self.env = GridEnvironment()
        self.env.load_from_file(self.map_var.get())
        self.agent = DeliveryAgent(self.env, self.algo_var.get(), self.update_gui)
        def run_agent():
            start_time = time.perf_counter()
            success = self.agent.execute_delivery()
            end_time = time.perf_counter()
            result_text = f"\nPlanner used: {self.algo_var.get()}\nSuccess: {success}\nPackages delivered: {len(self.agent.delivered)}\nFuel remaining: {self.agent.fuel_remaining}\nTime elapsed (sim): {self.agent.time_elapsed}\nWall-clock compute time: {end_time - start_time:.6f} sec\n"
            self.info_text.insert(tk.END, result_text)
            self.info_text.see(tk.END)
            self.running = False
        thread = threading.Thread(target=run_agent, daemon=True)
        thread.start()

    def compare_algorithms(self):
        if self.running:
            return
        self.running = True
        self.info_text.delete(1.0, tk.END)
        self.info_text.insert(tk.END, "Running comparison of all algorithms...\n")
        map_files = ["small.map", "medium.map", "large.map", "dynamic.map"]
        planners = ['bfs', 'ucs', 'astar', 'hillclimb', 'simanneal']
        results = []
        def run_comparison():
            for map_file in map_files:
                for planner in planners:
                    self.info_text.insert(tk.END, f"\nTrying {planner} on {map_file}...\n")
                    self.info_text.see(tk.END)
                    env = GridEnvironment()
                    env.load_from_file(map_file)
                    start_time = time.perf_counter()
                    agent = DeliveryAgent(env, planner)
                    success = agent.execute_delivery()
                    end_time = time.perf_counter()
                    total_path_cost = 0
                    results.append({
                        'map': map_file,
                        'planner': planner,
                        'success': success,
                        'delivered': len(agent.delivered),
                        'fuel_remaining': agent.fuel_remaining,
                        'time_elapsed': agent.time_elapsed,
                        'computation_time': end_time - start_time
                    })
            self.info_text.insert(tk.END, "\n" + "="*80 + "\n")
            self.info_text.insert(tk.END, "SUMMARY OF RESULTS:\n")
            self.info_text.insert(tk.END, "="*80 + "\n")
            self.info_text.insert(tk.END, "Map\tPlanner\tSuccess\tDelivered\tFuel\tTime\tComp.Time\n")
            for res in results:
                line = f"{res['map']}\t{res['planner']}\t{res['success']}\t{res['delivered']}\t{res['fuel_remaining']}\t{res['time_elapsed']}\t{res['computation_time']:.4f}\n"
                self.info_text.insert(tk.END, line)
            self.info_text.see(tk.END)
            self.running = False
        thread = threading.Thread(target=run_comparison, daemon=True)
        thread.start()

    def reset(self):
        self.running = False
        self.env = None
        self.agent = None
        self.canvas.delete("all")
        self.info_text.delete(1.0, tk.END)

def main():
    root = tk.Tk()
    app = DeliveryApp(root)
    root.mainloop()

if __name__ == '__main__':
    main()
