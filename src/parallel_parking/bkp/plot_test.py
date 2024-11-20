import numpy as np
import matplotlib.pyplot as plt

def calculate_tangent_direction(center, point):
    """Calculate the tangent vector at a given point on a circle."""
    # Calculate the radius vector
    radius_vector = np.array([point[0] - center[0], point[1] - center[1]])
    
    # Compute the tangent vector (perpendicular to radius vector)
    # Rotate radius vector by 90 degrees to get the tangent (clockwise or counterclockwise)
    tangent_vector = np.array([radius_vector[1], -radius_vector[0]])
    
    # Normalize the tangent vector
    tangent_vector_normalized = tangent_vector / np.linalg.norm(tangent_vector)
    
    return tangent_vector_normalized

def plot_circle_with_tangents(center, radius, overhang_distance, num_points=100):
    """Plot the circle and the tangent points at each point along the circle."""
    # Generate points on the circle
    angles = np.linspace(0, 2*np.pi, num_points)
    circle_points = np.array([
        [center[0] + radius * np.cos(theta), center[1] + radius * np.sin(theta)] 
        for theta in angles
    ])
    
    # Plot the circle
    fig, ax = plt.subplots()
    circle = plt.Circle(center, radius, color='b', fill=False, linestyle='--', label='Circle')
    ax.add_artist(circle)
    
    # Plot the tangents at each point on the circle
    for point in circle_points:
        # Calculate the tangent direction
        tangent_direction = calculate_tangent_direction(center, point)
        
        # Calculate the point along the tangent line at the overhang distance
        tangent_point = point + overhang_distance * tangent_direction
        
        # Plot the original point and the tangent point
        ax.plot([point[0], tangent_point[0]], [point[1], tangent_point[1]], 'r-')  # Tangent line
        ax.plot(tangent_point[0], tangent_point[1], 'ro')  # Tangent point
    
    # Set equal aspect ratio and show plot
    ax.set_aspect('equal')
    plt.legend()
    plt.show()

# Example usage:
center = [0, 0]
radius = 3
overhang_distance = 1  # Distance from the circle point along the tangent line
plot_circle_with_tangents(center, radius, overhang_distance)
