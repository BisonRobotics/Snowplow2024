import math

pi = math.pi
turn_radius = 2

def direction(start_point, end_point):
    change = (end_point[0] - start_point[0], end_point[1] - start_point[1])
   
    if change[0] == 0:
        return pi / 2

    return math.atan(change[1] / change[0])

def circle_tuple(point_a, point_b, direction):
    #defining the circle

    # find first tuple
    rad_a = math.radians(direction + 90)
    tuple_a = (point_b[0] + turn_radius * math.sin(rad_a), point_b[1] + turn_radius * math.cos(rad_a))
    
    # find second tuple
    rad_b = math.radians(direction - 90)
    tuple_b = (point_b[0] + turn_radius * math.sin(rad_b), point_b[1] + turn_radius * math.cos(rad_b))
    
    # determine which tuple should be used and which direction
    if math.dist(point_a, tuple_a) <= math.dist(point_a, tuple_b):
        return tuple_a, 1 #turn right one of these could possibly be inverted need to test
    else:
        return tuple_b, -1 # turn left

def final_direction(tuple, point, direction):
    tuple_a = (tuple[0] + turn_radius * math.cos(direction), tuple[1] + turn_radius * math.sin(direction))
    tuple_b = (tuple[0] - turn_radius * math.cos(direction), tuple[1] - turn_radius * math.sin(direction))

    if math.dist(point, tuple_a) <= math.dist(point, tuple_b):
        return 1 #forward
    else:
        return -1 #backward

def turn_path(start_point, start_direction, end_point, end_direction):
    #defining the end circle
    end_tuple, end_turn = circle_tuple(start_point, end_point, end_direction)
    start_tuple, start_turn = circle_tuple(end_tuple, start_point, start_direction) 

    root_two = 4 # dont worry about the name...
    
    # Determine if the same direction of turns are needed
    if start_turn == end_turn:
        start_angle = direction(start_tuple, end_tuple) + start_turn * (pi / 2)
        tan_point_start = (start_tuple[0] + root_two * turn_radius * math.cos(start_angle), start_tuple[1] + root_two * turn_radius * math.sin(start_angle))
        
        end_angle = direction(end_tuple, start_tuple) + end_turn * (pi / 2)
        tan_point_end = (end_tuple[0] + root_two * turn_radius * math.cos(end_angle), end_tuple[1] + root_two * turn_radius * math.sin(end_angle))

    # If not then different math is needed :)
    else:
        hypotenuse = math.dist(start_tuple, end_tuple)
        short = 2 * turn_radius
        theta = math.atan2(end_tuple[1] - start_tuple[1], end_tuple[0] - start_tuple[0]) + math.asin(short / hypotenuse) - pi / 2
        tan_point_start= (start_tuple[0] + root_two * turn_radius * math.cos(theta), start_tuple[1] + root_two * turn_radius * math.sin(theta))
        tan_point_end = (end_tuple[0] + root_two * turn_radius * math.cos(theta + pi), end_tuple[1] + root_two * turn_radius * math.sin(theta + pi))
    
    # Find the initial direction the robot needs to move in
    direction_movement_start = final_direction(start_tuple, tan_point_start, start_direction)
    direction_movement_end = -1 * final_direction(end_tuple, tan_point_end, end_direction) # needs to be reversed
    
    return (start_turn, direction_movement_start, tan_point_start, tan_point_end, direction_movement_end, end_turn, end_point)

# outputs starting turn direction, point which robot starts going straight, point robot stops going straight, ending turn direction, destination
#right = 1 left = -1

if __name__ == "__main__":
    start_turn, direction_movement_start, tan_point_start, tan_point_end, direction_movement_end, end_turn, end_point = turn_path((3,4),90,(8,10),0)
    print(f"Start turn: {start_turn}")
    print(f"End turn: {end_turn}")
    print(f"Tan point start: {tan_point_start}")
    print(f"Tan point end: {tan_point_end}")
    print(f"Direction movement start: {direction_movement_start}")
    print(f"Direction movement end: {direction_movement_end}")
    print(f"End point: {end_point}")
