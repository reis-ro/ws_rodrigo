# your_python_module.py
import numpy as np

def generate_path(start, goal):
    try:
        start_x, start_y = start[0], start[1]
        goal_x, goal_y = goal[0], goal[1]

        path = np.linspace([start_x, start_y], [goal_x, goal_y], num=10)

        path_dict = [{"x": float(point[0]), "y": float(point[1])} for point in path]
        return path_dict

    except Exception as e:
        print(f"Error generating path: {e}")
        return []  # Return an empty list on error
