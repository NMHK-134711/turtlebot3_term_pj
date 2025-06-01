# TurtleBot3 Term Project

## How to run this project

### Prep

- ROS2 must be installed
- Git must be installed
- cartographer must be installed
- nav2 must be installed

### Run code (Open terminal at Home)

1. 
   ```bash
   mkdir turtlebot3_term_pj
   ```

2. 
   ```bash
   cd turtlebot3_term_pj
   ```

3. 
   ```bash
   git clone https://github.com/NMHK-134711/turtlebot3_term_pj.git
   ```

4. 
   ```bash
   colcon build
   ```

5. 
   ```bash
   source ~/.bashrc
   ```

6. 
   ```bash
   ros2 launch turtlebot3_term_pj master_launch.py
   ```
