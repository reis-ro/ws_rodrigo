import carla
import math

def get_vehicle_attributes(vehicle):
    # Get vehicle's bounding box (dimensions)
    bounding_box = vehicle.bounding_box
    vehicle_width = bounding_box.extent.x * 2  # Multiply by 2 for full width

    # Get physics control to retrieve dynamic attributes
    physics_control = vehicle.get_physics_control()

    # Extract the wheelbase (distance between front and rear axles)
    wheels = physics_control.wheels
    front_left_wheel = wheels[0]
    rear_left_wheel = wheels[1]
    wheel_base = abs(front_left_wheel.position.x - rear_left_wheel.position.x)

    # Get maximum steering angle (in radians)
    max_steering_angle = wheels[0].max_steer_angle*math.pi/180

    # Calculate the minimum turning radius using the wheelbase and max steering angle
    min_turning_radius = wheel_base / math.tan(max_steering_angle)

    return vehicle_width, min_turning_radius

client = carla.Client('localhost', 2000)
world = client.get_world()

# Assuming you already have a vehicle spawned and know its ID or reference
car_id = 135
vehicle = world.get_actor(car_id)  # Replace <vehicle_id> with your vehicle's actor ID

# Get the vehicle attributes
vehicle_width, min_turning_radius = get_vehicle_attributes(vehicle)

print(f"Vehicle Width: {vehicle_width:.2f} meters")
print(f"Minimum Turning Radius: {min_turning_radius:.2f} meters")

