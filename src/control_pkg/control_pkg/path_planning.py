import math

pi = math.pi
turn_radius = 2

def direction(start_point, end_point):
    change = (end_point[0]-start_point[0],end_point[1] - start_point[1])
    if (change[0] == 0):
        return pi/2
    return math.atan(change[1]/change[0])

def circle_tuple(point_a, point_b, direction):
    #defining the circle

    # find first tuple
    rad_a = math.radians(direction + 90)
    tuple_a = (math.sin(rad_a), math.cos(rad_a))
    tuple_a = (point_a[0] + turn_radius * tuple_a[0], point_a[1] + turn_radius * tuple_a[1])

    # find second tuple
    rad_b = math.radians(direction - 90)
    tuple_b = (math.sin(rad_b), math.cos(rad_b))
    tuple_b= (point_a[0] + turn_radius * tuple_b[0],point_a[1] + turn_radius * tuple_b[1])

    if (math.dist(point_b, tuple) <= math.dist(point_b, tuple)):
        return tuple_a, 1 #turn right one of these could possibly be inverted need to test
    else:
        return tuple_b, -1 #turn left

def turn_path(start_point, start_direction, end_point, end_direction):
    #defining the end circle
    end_tuplea = (math.sin(math.radians(end_direction + 90))), math.cos(math.radians((end_direction + 90)))
    end_tuplea = (end_point[0]+ turn_radius * end_tuplea[0],end_point[1] + turn_radius * end_tuplea[1])
    
    
    end_tupleb = (math.sin(math.radians(end_direction-90))),math.cos(math.radians((end_direction-90)))
    end_tupleb = (end_point[0]+ turn_radius * end_tupleb[0],end_point[1] + turn_radius * end_tupleb[1])
    

    if (math.dist(start_point, end_tuplea) <= math.dist(start_point, end_tupleb)):
        end_tuple = end_tuplea
        end_turn = 1 #turn right one of these could possibly be inverted need to test
    else:
        end_tuple = end_tupleb
        end_turn = -1 #turn left

    #define the start circle
    start_tuplea = (math.sin(math.radians(start_direction+90))),math.cos(math.radians((start_direction+90)))
    start_tuplea = (start_point[0]+ turn_radius * start_tuplea[0],start_point[1] + turn_radius * start_tuplea[1])

    start_tupleb = (math.sin(math.radians(start_direction-90))),math.cos(math.radians((start_direction-90)))
    start_tupleb = (start_point[0]+ turn_radius * start_tupleb[0],start_point[1] + turn_radius * start_tupleb[1])
    

    if (math.dist(start_tuplea, end_tuple) <= math.dist(start_tupleb, end_tuple)):
        start_tuple = start_tuplea
        start_turn = 1 #turn right
    else:
        start_tuple = start_tupleb
        start_turn = -1 #turn left

    # --- UNTESTED CODE TO REDUCE THE STUFF BEFORE ---
    #end_tuple, end_turn = circle_tuple(end_point, start_point, start_direction)
    #start_tuple, start_turn = circle_tuple(start_point, end_tuple, end_direction) # not sure if it is end_tuple or end_point

    root_two = 4 # dont worry about the name...
    
    # if we are using both a or both b
    if start_turn == end_turn:
    # if ((start_tuple == start_tuplea and end_tuple == end_tuplea) or (start_tuple == start_tupleb and end_tuple == end_tupleb)):
        if start_turn == 1:
            tan_point_start = (start_tuple[0] + root_two*turn_radius*(math.cos(direction(start_tuple,end_tuple) + pi / 2)), start_tuple[1] + root_two*turn_radius*(math.sin(direction(start_tuple,end_tuple) + pi / 2)))
        else:
            tan_point_start = (start_tuple[0] + root_two*turn_radius*(math.cos(direction(start_tuple,end_tuple) - pi / 2)) , start_tuple[1] + root_two*turn_radius*(math.sin(direction(start_tuple,end_tuple) - pi / 2)))
            
        if end_turn == 1:
            tan_point_end = (end_tuple[0] + root_two*turn_radius*(math.cos(direction(end_tuple, start_tuple) + pi / 2)), end_tuple[1] + root_two*turn_radius*(math.sin(direction(end_tuple, start_tuple) + pi / 2)))
        else:
            tan_point_end = (end_tuple[0] + root_two*turn_radius*(math.cos(direction(end_tuple, start_tuple) - pi / 2)), end_tuple[1] + root_two*turn_radius*(math.sin(direction(end_tuple, start_tuple) - pi / 2)))

    else:
        hypotenuse = math.dist(start_tuple,end_tuple)
        short = 2*turn_radius
        theta = math.atan2(end_tuple[1]-start_tuple[1],end_tuple[0]-start_tuple[0])+math.asin(short/hypotenuse)- pi/2
        tan_point_start= (start_tuple[0]+root_two*turn_radius*math.cos(theta),start_tuple[1]+root_two*turn_radius*math.sin(theta))
        tan_point_end = (end_tuple[0]+root_two*turn_radius*math.cos(theta+pi),end_tuple[1]+root_two*turn_radius*math.sin(theta+pi))
    
    placeholder_tuple1= (start_tuple[0]+turn_radius*math.cos(start_direction),start_tuple[1]+turn_radius*math.sin(start_direction))
    placeholder_tuple2 = (start_tuple[0]-turn_radius*math.cos(start_direction),start_tuple[1]-turn_radius*math.sin(start_direction))

    if (math.dist(tan_point_start, placeholder_tuple1)<= math.dist(tan_point_start, placeholder_tuple2)):
        direction_movement_start = 1 #forward
    
    else:
        direction_movement_start = -1 #backward

    if (math.dist(tan_point_end, (end_tuple[0]-turn_radius*math.cos(end_direction),end_tuple[1]-turn_radius*math.sin(end_direction)))<= math.dist(tan_point_end, (end_tuple[0]+turn_radius*math.cos(end_direction),end_tuple[1]+turn_radius*math.sin(end_direction)))):
        direction_movement_end = 1 #forward
    
    else:
        direction_movement_end = -1 #backward
    
    return (start_turn,direction_movement_start,tan_point_start,tan_point_end,direction_movement_end,end_turn,end_point)

# outputs starting turn direction, point which robot starts going straight, point robot stops going straight, ending turn direction, destination
#right = 1 left = -1



if __name__ == "__main__":
    start_turn,direction_movement_start,tan_point_start,tan_point_end,direction_movement_end,end_turn,end_point = turn_path((3,4),90,(8,10),0)
    print(start_turn)
    print(direction_movement_start)
    print(tan_point_start)
    print(tan_point_end)
    print(direction_movement_end)
    print(end_turn)
    print(end_point)