# Autonomous Delivery Agent in a 2D Grid Environment

**GitHub Repository:** [https://github.com/darshshukla7/Autonomous-Delivery-Agent](https://github.com/darshshukla7/Autonomous-Delivery-Agent)

---

## Project Overview
This project implements an **autonomous delivery agent** that navigates a 2D grid-based city environment to pick up and deliver packages efficiently. The agent handles **static and dynamic obstacles**, considers **terrain costs**, and optimizes for **fuel and time constraints**.

### Planning Algorithms
- **Uninformed search:** BFS, Uniform-Cost Search  
- **Informed search:** A* with Manhattan heuristic  
- **Local search / replanning:** Hill Climbing with random restarts, Simulated Annealing  

The agent can **replan dynamically** when obstacles appear and provides logs for experimental analysis. A **Tkinter GUI** visualizes the environment, agent movements, and delivery progress.

---

## Key Features
- Models grid cells with terrain costs and obstacles  
- Dynamic replanning for moving obstacles  
- Fuel and time constraints for realistic delivery simulation  
- Tkinter GUI shows the agent, packages, deliveries, and moving obstacles  
- Logs nodes expanded, path costs, and agent actions for experimental analysis  
- Compare multiple algorithms across several maps  

---

## Repository Structure
1) main.py
2) Maps
   - Small.map
   - Medium.map
   - Large.map
   - Dynamic.map
3) README.md
4) Project Report.pdf
5) Demo.mp4

---

## Map Format
The maps in the `maps/` folder use the following format:  
- `0` = normal terrain (movement cost = 1)  
- `1` = static obstacle (cannot traverse)  
- `2, 3, ...` = higher terrain cost (more costly to move)  
- Dynamic obstacles and packages are handled in code; their positions are hardcoded.  
---

## Requirements
- Python 3.8+  
- Standard libraries: `tkinter`, `heapq`, `random`, `math`, `time`, `threading`, `collections`  
No additional installation required.

---

## Usage

### GUI
1. Run the project:
```bash
python main.py
```
2. Select the map: small, medium, large, dynamic
3. Select the algorithm: bfs, ucs, astar, hillclimb, simanneal
4. Click Run to execute delivery or Compare All to benchmark all planners
5. Observe agent movements: packages (blue), delivery points (red), obstacles (black/gray)
6. Logs show fuel, time, nodes expanded, and delivery progress

### Dynamic Replanning
- The agent automatically replans if a cell becomes blocked (moving obstacles)
- Logs provide evidence for dynamic replanning proof-of-concept

---
## Project Report
Project Report.pdf contains :
1. Environment model & agent design
2. Heuristics and planner details
3. Experimental results
4. Analysis & conclusions

---
## Future Improvements
- Support external map files for reproducibility.
- CLI execution for automated testing
- Save logs to files for experiment reproducibility
- Support diagonal movement and more complex dynamic obstacles

---
