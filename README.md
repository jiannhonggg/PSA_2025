![My Cover Image](images/Cover%20Image.png)

# PSA CodeSprint 2025: Smart Port Operations – AI for Horizontal Transport Optimisation (Node), Algorithm Challenge. Time Reduction of 45.25%

## The foucs of this problem statment was to design a scheduling and routing strategy to minimize congestion, balance yard utilization and improve overall efficiency.


The original planner was inefficient. It created massive bottlenecks for a few key reasons:
* **Truck "Bunching":** All the trucks would rush to the same area at the same time.
* **Yard Bottlenecks:** It would send all containers to one or two "popular" yards, creating huge traffic jams while other yards sat empty.
* **"Dumb" GPS:** The truck routes were rigid and inefficient. Taking the long way for every instance, even if there was a faster non conflicting way. 


### Our Solution: A 3-Part Optimization Strategy

#### 1. Smart Truck Assignments (`select_HT`)
**Before:** The original logic was likely very simple (e.g., "pick the first available HT" or "pick the closest HT to the first stop").

**After:** We implemented a sophisticated heuristic cost function. Instead of just looking at the first leg of the journey, our logic calculates a total cost for the entire job cycle (e.g., Buffer -> QC -> Buffer -> Yard -> Buffer).

* It heavily weights the immediate leg (immediate_w) but also considers the downstream legs (downstream_w).

* It includes a recent_penalty to avoid assigning jobs to the same HT repeatedly, which helps prevent congestion and bunching.

* It adds a spread_penalty to encourage selecting HTs that are further from other already selected HTs in the same planning tick.

#### 2. Dynamic Yard Load-Balancing (`select_yard`)
**Before:** The default was probably to just use the yard_name provided by the job, leading to massive pile-ups at a single yard.

**After:** We implemented a load-balancing algorithm. Our logic calculates a score for all available yards based on two competing factors

* Proximity: The physical travel distance from the HT's buffer spot (dist_score).

* Fairness: The current workload of the yard (fairness_score), which penalizes yards that are already over-utilized (yard_usage). This balanced approach (tuned with fairness_weight) prevents bottlenecks by spreading the work across all yards.

#### 3. Intelligent & Adaptive Pathfinding (The "GPS")
This was the biggest win. We completely rebuilt the port's navigation logic.

**Before:** The original paths were rigid, one-way loops (e.g., always go to x=1 to travel up, always go to x=42 to travel down), which was extremely inefficient.

**After:** We implemented intelligent, conditional pathfinding. We analyzed the port layout (lanes at y=4, y=5, y=7, y=12) and created custom logic that takes massive shortcuts.

* Example: In get_path_from_buffer_to_QC, if the QC was to the right of the HT (QC_in_coord.x >= curr_x), our code takes a direct shortcut by moving down to the y=5 lane and traveling directly there. The old logic would have forced the HT to travel all the way left to x=1 first, adding dozens of unnecessary steps.

* We applied a similar "shortcut" principle to all four pathing functions, drastically cutting down on travel time for each job.



Overall our final solution achieved a 45.26% performance improvement over the baseline, reducing the total simulation time from 1,167,610 ticks to 639,180 ticks.
This result was not achieved with a complex deep learning model, but through rigorous bottleneck analysis and classical algorithm design—a practical, high-performance approach that highlights several key engineering takeaways.

**Key Technical Learnings:**
This project was a powerful lesson in engineering pragmatism. We determined that the problem was not one of prediction (which AI is good at) but one of optimization and scheduling in a highly constrained environment. A well-designed heuristic algorithm proved to be more performant, relaible than an AI solution would have been. Our gains came from systematically identifying and solving the correct problems. The core bottlenecks were not just travel distance, but resource contention and emergent gridlock. We also learned to treat the port as an interconnected system. A "fix" in one area (like faster pathfinding) could create a new bottleneck elsewhere (like the yard). Our solution succeeded by balancing all components, leading to a globally-optimized system rather than a locally-optimized one.

**Note:** 

Our solution was not officially judged due to a late submission. This was a direct result of a documented global AWS S3 outage on the submission day which prevented our upload.
While this outcome was unfortunate, it was a powerful real-world lesson in dependency risk, infrastructure resilience, and the critical need for contingency planning. It proved that even the most optimized algorithm is coupled to the systems it runs on, and a robust engineering mindset must account for external failures. A key learning point we will be carring forward.





## Environment Setup

Before working on the problem, please create a Python virtual environment and install all required dependencies.

If you are familiar with Python environments, you can directly install from `requirements.txt`. Otherwise, follow these step-by-step instructions in your Command Prompt (Windows):

```shell
# Create a Python virtual environment. Here I used Python 3.11.
python -m pip install virtualenv
python -m virtualenv py_env_codesprint
py_env_codesprint\Scripts\activate

# Install pip-tools to manage dependencies
python -m pip install pip-tools

# Compile and synchronize dependencies from requirements.in to requirements.txt
pip-compile -v --rebuild -o requirements.txt --trusted-host pypi.org --trusted-host pypi.python.org --trusted-host files.pythonhosted.org
pip-sync requirements.txt

# (Optional) Set up Jupyter kernel for experimenting with notebooks
python -m ipykernel install --user --name=py_env_codesprint_kernel
```

To activate your environment:

```shell
py_env_codesprint\Scripts\activate
```

## Development

To get started, run the simulation with a GUI by executing:

```shell
python gui.py
```

You will see a simulation of the port terminal operations.

### About the Code Base

The simulation consists of two main components:

- **Planning Engine**: Where scheduling and navigating algorithms are implemented.
- **Operation Engine**: Simulates execution of planned jobs.

Your task is to improve the default planning algorithm in the Planning Engine for better performance.

### Project Structure

```
INA_DS_CS/
│
├── data/
│ └── input.csv # Input job details for the planning algorithm
├── logs/       # Log files for each simulation run
├── src/        # Source code
│ └── operate/
│ ├── plan/
| |  └── job_planner.py # Your main implementation goes here!
│ ├── ui/
│ └── utils/
├── cli.py      # Command-line interface entry point
└── gui.py      # GUI interface entry point
```

- `input.csv` contains the job details fed into the planning algorithm.
- `job_planner.py` is the core of the Planning Engine — your modifications should be made here (Search for **YOUR TASK HERE**). Refer to the provided documentation for details on the default algorithm logic.
- `gui.py` runs the simulation with a graphical interface so you can observe and fine-tune your algorithm. You can adjust `number_of_refresh_per_physical_second` in the code to control simulation speed.
- `cli.py` runs the simulation without GUI for faster batch execution. It outputs `output.csv` in the data folder and logs in `logs/`.

### Running the CLI simulation

Run:

```
python cli.py
```

The simulation may take a few minutes. At the end, it will report the total simulation time (e.g., 1,167,610 seconds) — **this is the key metric you need to optimize and reduce**. Note that this is the simulation time when system checks for all job completion condition. A precise time of the last job completion can be found in `end_time` column of `output.csv` file.

## Submission Guidelines

When you are ready to submit your solution, please prepare the following:

1. YouTube Presentation video explaining your solution.
2. PowerPoint Slides supporting your presentation.
3. Source Code:
   - Modified `job_planner.py` plus any supporting scripts that depend only on `job_planner.py`.
   - Other modifications outside of this are not allowed.
   - Zip these files into Codes_TeamName.zip.
   - Include a `README.txt` describing how to run your code and integrate it into the existing codebase.
4. Simulation Results:
   - `data/output.csv`
   - the latest log file from `logs/` demonstrating your improved results.
