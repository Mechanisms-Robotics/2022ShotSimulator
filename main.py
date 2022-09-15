# Imports
import math
import os
import numpy as np

from matplotlib import pyplot as plt

plt.style.use('dark_background')

# Constants
MIN_DISTANCE = 1.5  # meters
MAX_DISTANCE = 7.5  # meters
MIN_HOOD_ANGLE = 21.0  # degrees
MAX_HOOD_ANGLE = 50.0  # degrees
MIN_RPM = 500.0  # rpm
MAX_RPM = 4000.0  # rpm

DISTANCE_DELTA = 0.1
HOOD_ANGLE_DELTA = 1.0
RPM_DELTA = 5.0

FLYWHEEL_DIAMETER = 0.1  # meters
SHOT_HEIGHT = 1.0  # meters
BALL_MASS = 0.27  # kilos
GOAL_HEIGHT = 2.64  # meters
BALL_DIAMETER = 0.24  # meters

VALID_SHOTS_FILE_PATH = 'valid_shots.npy'
BEST_SHOTS_FILE_PATH = 'best_shots.npy'


# Main function
def main():
    # Check if a valid shots file already exists
    if not os.path.exists(VALID_SHOTS_FILE_PATH):
        # Set distance, angle, and rpm to the min value
        distance = MIN_DISTANCE
        angle = MIN_HOOD_ANGLE
        rpm = MIN_RPM

        # Initialize valid shots array
        valid_shots = []

        # Loop over distance
        while distance <= MAX_DISTANCE:
            # Loop over angle
            while angle <= MAX_HOOD_ANGLE:
                # Loop over rpm
                while rpm <= MAX_RPM:
                    # Check if the shot makes it into the goal
                    if test_shot(distance, angle, rpm):
                        # If it does append it to valid_shots
                        valid_shots.append((distance, angle, rpm))

                    # Increment rpm by RPM_DELTA
                    rpm += RPM_DELTA

                # Reset rpm
                rpm = MIN_RPM

                # Increment angle by HOOD_ANGLE_DELTA
                angle += HOOD_ANGLE_DELTA

            # Reset rpm
            rpm = MIN_RPM

            # Reset angle
            angle = MIN_HOOD_ANGLE

            # Increment distance by DISTANCE_DELTA
            distance += DISTANCE_DELTA

        # Convert valid shots to a numpy array
        valid_shots = np.array(valid_shots)

        # Save valid shots to VALID_SHOTS_FILE_PATH
        np.save(VALID_SHOTS_FILE_PATH, valid_shots)
    else:
        if not os.path.exists(BEST_SHOTS_FILE_PATH):
            # If a valid shots file already exists load it up
            valid_shots = np.load(VALID_SHOTS_FILE_PATH)

            # Initialize the best shots array
            best_shots = []
            best_distances = []

            # Loop through every valid shot
            for shot in valid_shots:
                # Test the shot
                shot_positions = test_shot(shot[0], shot[1], shot[2], return_positions=True)

                # Check if the shot goes too high
                too_high = False
                for position in shot_positions:
                    if position[2] > 5.0:
                        too_high = True

                # If it does cut it
                if too_high:
                    continue

                # Check if the shot is too flat
                down_vel = (shot_positions[-2][2] - shot_positions[-1][2]) / 0.01

                # If it is cut it
                if down_vel < 1.5:
                    continue

                # Create fake error shots
                error_factor = 0.25
                error_shots = [
                    (
                        shot[0] + DISTANCE_DELTA * error_factor,
                        shot[1] + HOOD_ANGLE_DELTA * error_factor,
                        shot[2] + RPM_DELTA * error_factor
                    ),
                    (
                        shot[0] + DISTANCE_DELTA * error_factor,
                        shot[1] + HOOD_ANGLE_DELTA * error_factor,
                        shot[2] - RPM_DELTA * error_factor
                    ),
                    (
                        shot[0] + DISTANCE_DELTA * error_factor,
                        shot[1] - HOOD_ANGLE_DELTA * error_factor,
                        shot[2] + RPM_DELTA * error_factor
                    ),
                    (
                        shot[0] + DISTANCE_DELTA * error_factor,
                        shot[1] - HOOD_ANGLE_DELTA * error_factor,
                        shot[2] - RPM_DELTA * error_factor
                    ),
                    (
                        shot[0] - DISTANCE_DELTA * error_factor,
                        shot[1] + HOOD_ANGLE_DELTA * error_factor,
                        shot[2] + RPM_DELTA * error_factor
                    ),
                    (
                        shot[0] - DISTANCE_DELTA * error_factor,
                        shot[1] + HOOD_ANGLE_DELTA * error_factor,
                        shot[2] - RPM_DELTA * error_factor
                    ),
                    (
                        shot[0] - DISTANCE_DELTA * error_factor,
                        shot[1] - HOOD_ANGLE_DELTA * error_factor,
                        shot[2] - RPM_DELTA * error_factor
                    ),
                ]

                # Check the shot's sensitivity to error
                too_sensitive = False
                for error_shot in error_shots:
                    if not test_shot(error_shot[0], error_shot[1], error_shot[2]):
                        too_sensitive = True

                # If it is too sensitive cut it
                if too_sensitive:
                    continue

                # If we already have a best shot at this distance cut it
                if shot[0] in best_distances:
                    continue

                # If it passed all those tests append it to the best shots array and the distance to best distances
                best_shots.append(shot)
                best_distances.append(shot[0])

            # Convert best shots to a numpy array
            best_shots = np.array(best_shots)

            # Save best shots to BEST_SHOTS_FILE_PATH
            np.save(BEST_SHOTS_FILE_PATH, best_shots)
        else:
            # If a best shots file exists load it up
            best_shots = np.load(BEST_SHOTS_FILE_PATH)

            # Print all the distances and hood angles
            print('Hood Angle')
            for shot in best_shots:
                print(f'({round(shot[0], 2)}, {shot[1]})')

            # Print a newline
            print()

            # Print all the distances and RPMs
            print('RPM')
            for shot in best_shots:
                print(f'({round(shot[0], 2)}, {shot[2]})')

            # Initialize means
            mean_distance = 0.0
            mean_hood_angle = 0.0
            mean_rpm = 0.0

            # Sum values
            for shot in best_shots:
                mean_distance += round(shot[0], 2)
                mean_hood_angle += shot[1]
                mean_rpm += shot[2]

            # Calculate means
            mean_distance /= best_shots.shape[0]
            mean_hood_angle /= best_shots.shape[0]
            mean_rpm /= best_shots.shape[0]

            # Calculate the numerator and denominator for the hood angle line of best fit
            numerator = 0.0
            denomintator = 0.0
            for shot in best_shots:
                numerator += (round(shot[0], 2) - mean_distance) * (shot[1] - mean_hood_angle)
                denomintator += (round(shot[0], 2) - mean_distance)**2

            # Calculate the slope and intercept of the hood angle line of best fit
            hood_angle_slope = numerator / denomintator
            hood_angle_intercept = mean_hood_angle - hood_angle_slope * mean_distance

            # Calculate the numerator for the rpm line of best fit
            numerator = 0.0
            for shot in best_shots:
                numerator += (round(shot[0], 2) - mean_distance) * (shot[2] - mean_rpm)

            # Calculate the slope and intercept of the rpm line of best fit
            rpm_slope = numerator / denomintator
            rpm_intercept = mean_rpm - rpm_slope * mean_distance

            # Print the LOBF equations
            print()
            print(f'Hood Angle LOBF: y={round(hood_angle_slope, 2)}x+{round(hood_angle_intercept, 2)}')
            print(f'RPM LOBF: y={round(rpm_slope, 2)}x+{round(rpm_intercept, 2)}')


# Tests if a shot makes it into the goal
def test_shot(distance, angle, rpm, return_positions=False):
    # Calculate the exit velocity of the ball in meters per second
    exit_vel = rpm * ((FLYWHEEL_DIAMETER * math.pi) / 60.0)

    # Calculate the x and y component of the velocity vector
    vx = math.cos(math.radians(90.0 - angle)) * exit_vel
    vy = math.sin(math.radians(90.0 - angle)) * exit_vel

    # Initialize positions array, as well as cur velocity and cur position
    positions = [(0.0, 0.0, SHOT_HEIGHT)]
    cur_velocity = (vx, vy)
    cur_position = positions[-1]

    # Initialize time and set the time delta
    time = 0.01
    time_delta = 0.01

    # Loop until break
    while True:
        # Check if this is the first timestep
        if time != time_delta:
            # If it isn't check if the ball is on it's way down through the goal height
            if positions[-1][2] < positions[-2][2] and positions[-1][2] <= GOAL_HEIGHT:
                # Check if the ball passed through the goal position
                if abs(GOAL_HEIGHT - positions[-1][2]) < 0.05:
                    if abs(distance - positions[-1][1]) < 0.05:
                        if return_positions:
                            return np.array(positions)
                        else:
                            return True

                # Break out of this loop
                break

        # Calculate gravity force
        gravity_force = BALL_MASS * -9.81

        # Calculate air drag force
        air_drag_force = (
                1 / 2 * 0.08 *
                (cur_velocity[0] ** 2 + cur_velocity[1] ** 2) * 0.5 *
                (math.pi * (BALL_DIAMETER / 2.0)) ** 2
        )

        # Calculate air drag force vector
        air_drag_force_vector = (air_drag_force * -cur_velocity[0], air_drag_force * -cur_velocity[1])

        # Update velocity with the acceleration caused by the forces acting on the ball
        cur_velocity = (
            cur_velocity[0] + air_drag_force_vector[0] * time_delta,
            cur_velocity[1] + ((gravity_force / BALL_MASS) * time_delta) + (air_drag_force_vector[1]
                                                                            * time_delta)
        )

        # Update position based on the current velocity
        cur_position = (
            time,
            cur_position[1] + cur_velocity[0] * time_delta,
            cur_position[2] + cur_velocity[1] * time_delta
        )

        # Append the current position to the positions array
        positions.append(cur_position)

        # Increment timestep
        time += time_delta

    return False


# Run the main function
main()
