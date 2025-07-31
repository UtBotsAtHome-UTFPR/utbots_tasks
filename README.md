# utbots_tasks
- ROS repository designed to orchestrate tasks for RoboCup@Home.
- Currently implemented with ROS Yasmin library for statemachines

## Installation

```bash
cd <ros2_ws>
git clone https://github.com/UtBotsAtHome-UTFPR/utbots_tasks.git
cd utbots_tasks
```

### Dependencies
To install all required packages for the tasks, run:
```bash
./install_repos.sh utbots_repos.txt --setup-cfg
```
This should install all needed repositories with all packages and all their dependencies from setup.sh scripts (ideally, if some error ocurrs you may need to acess the repositories and install dependencies as documented in the README.md files). If the packages need python requirements, the script should update their setup.cfg files and create the needed virtualenvs as specified in the `utbots_repos.txt` file.

**OBS**: in `utbots_repos.txt`, a `*` means all packages in that repository will have their requirements installed in the same python env

### Build
```bash
cd <ros2_ws>
colcon build
source install/setup.bash
# TODO: Add rosdep install
```

### Update packages
To update all packages from git, you can run:
```bash
./git_pull_all.sh
```

## Running

### Tasks
All tasks are implemented in .py files in `./utbots_tasks/tasks/` and each one has a correspondent launch file in `./launch`.

You should run the launchfile to bringup all needed nodes with the right parameters in a terminal:

```bash
ros2 launch utbots_tasks <task_name>.launch.py
```

And then in another terminal (to better visualize the logs): 

```bash
ros2 run utbots_tasks <task_name>
```

### States
In `./states` you can find a library of states and sub statemachines available for reusing in more than one task. Some of the scripts have statemachines to test the implemented states. It is a common practice to import this states to the task scripts so we don't need to copy or rewrite it.

## Adding new tasks
Create the relevant states if they are generic enough to be reused in other tasks in scripts located in `./states`, write your task script in `./tasks` and *don't forget to **update the setup.py** with your new scrips so you can run them with ROS*.