# PTask_Scheduler
ROS 2 periodic task scheduling example

# PTask_Scheduler

Using ROS 2, this project implements a system for scheduling periodic tasks. This includes three tasks, `A`, `B` and `C`, with the following parameters:

| Task | Period (pi) | Deadline (di) | Execution Time (ei) |
|------|-------------|---------------|---------------------|
| A    | 16ms        | 10ms          | 4ms                 |
| B    | 12ms        | 8ms           | 2ms                 |
| C    | 32ms        | 26ms          | 12ms                |

## Features

- Scheduling of periodic tasks with ROS 2 timers
- Logs job release, execution, and completion times
- Simulates real-time execution and response time
- Checks if there are any that misses the deadlines

## Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/kari-sri/PTask_Scheduler.git
   cd PTask_Scheduler


## Build ROS2 package: 

2. colcon build 

## Source environment:

3. . install/setup.bash

## Run the Task Scheduler:

4. ros2 run ptask_scheduler scheduler


##

##Result

[INFO] [1728150601.841665200] [task_scheduler]: Task B (Job Release: ka_scheduler) started
[INFO] [1728150601.845015026] [task_scheduler]: Task B completed in 3 ms
[INFO] [1728150601.845118591] [task_scheduler]: Task C (Job Release: C) started
[INFO] [1728150601.858095222] [task_scheduler]: Task C completed in 12 ms
[INFO] [1728150601.859135226] [task_scheduler]: Task A (Job Release: A) started
[INFO] [1728150601.864311294] [task_scheduler]: Task A completed in 5 ms
[INFO] [1728150601.864453190] [task_scheduler]: Task B (Job Release: ka_scheduler) started
[INFO] [1728150601.866972764] [task_scheduler]: Task B completed in 2 ms
[INFO] [1728150601.867070103] [task_scheduler]: Task A (Job Release: A) started
[INFO] [1728150601.871814018] [task_scheduler]: Task A completed in 4 ms
[INFO] [1728150601.871860013] [task_scheduler]: Task C (Job Release: C) started
[INFO] [1728150601.885870153] [task_scheduler]: Task C completed in 14 ms
[INFO] [1728150601.885978140] [task_scheduler]: Task A (Job Release: A) started
[INFO] [1728150601.890305077] [task_scheduler]: Task A completed in 4 ms
[INFO] [1728150601.890379799] [task_scheduler]: Task B (Job Release: ka_scheduler) started
[INFO] [1728150601.897185627] [task_scheduler]: Task B completed in 6 ms
[INFO] [1728150601.897361855] [task_scheduler]: Task A (Job Release: A) started
[INFO] [1728150601.902172569] [task_scheduler]: Task A completed in 4 ms
[INFO] [1728150601.902255410] [task_scheduler]: Task B (Job Release: ka_scheduler) started
[INFO] [1728150601.905180055] [task_scheduler]: Task B completed in 2 ms
[INFO] [1728150601.905293041] [task_scheduler]: Task C (Job Release: C) started
[INFO] [1728150601.937478139] [task_scheduler]: Task C completed in 31 ms
[INFO] [1728150601.938119109] [task_scheduler]: Task A (Job Release: A) started
[INFO] [1728150601.943720345] [task_scheduler]: Task A completed in 5 ms
[INFO] [1728150601.944256171] [task_scheduler]: Task B (Job Release: ka_scheduler) started
[INFO] [1728150601.947102706] [task_scheduler]: Task B completed in 2 ms
[INFO] [1728150601.947239700] [task_scheduler]: Task C (Job Release: C) started

