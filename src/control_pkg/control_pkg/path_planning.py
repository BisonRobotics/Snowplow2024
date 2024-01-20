import math

pi = math.pi
turn_radius = 2.25
radians = (pi/180)

def direction(start_point, end_point):
    change = (end_point[0] - start_point[0], end_point[1] - start_point[1])
    
    if change[0] == 0:
        return math.pi / 2 if change[1] > 0 else 3 * math.pi / 2
    if change [1] == 0:
        return math.pi if change[0] < 0 else 0
    return math.pi + math.atan(change[1] / change[0]) if change[0] < 0 else math.atan(change[1] / change[0])

# finds center point of a circle for the robot to drive around and the direction it should take
def pivot_tuple(point_a, point_b, direction):
    #defining the circle

    # find first tuple
    rad_a = math.radians(direction + 90)
    tuple_a = (point_b[0] + turn_radius * math.cos(rad_a), point_b[1] + turn_radius * math.sin(rad_a))
    
    # find second tuple
    rad_b = math.radians(direction - 90)
    tuple_b = (point_b[0] + turn_radius * math.cos(rad_b), point_b[1] + turn_radius * math.sin(rad_b))
    
    # determine which tuple should be used and which direction
    if math.dist(point_a, tuple_a) <= math.dist(point_a, tuple_b):
        return tuple_a, -1 #turn left one of these could possibly be inverted need to test
    else:
        return tuple_b, 1 # turn right

# determines the direction the robot should move in
def final_direction(tuple, point, direction):
    rad = math.radians(direction)
    tuple_a = (tuple[0] + turn_radius * math.cos(rad), tuple[1] + turn_radius * math.sin(rad))
    tuple_b = (tuple[0] - turn_radius * math.cos(rad), tuple[1] - turn_radius * math.sin(rad))

    if math.dist(point, tuple_a) <= math.dist(point, tuple_b):
        return 1 #forward
    else:
        return -1 #backward

def turn_path(start_point, start_direction, end_point, end_direction):
    #defining the end circle
    end_tuplea = (math.cos(math.radians(end_direction + 90)), math.sin(math.radians(end_direction + 90)))
    end_tuplea = (end_point[0] + turn_radius * end_tuplea[0],end_point[1] + turn_radius * end_tuplea[1])
    
    
    end_tupleb = (math.cos(math.radians(end_direction - 90)), math.sin(math.radians(end_direction - 90)))
    end_tupleb = (end_point[0] + turn_radius * end_tupleb[0], end_point[1] + turn_radius * end_tupleb[1])
    

    if (math.dist(start_point, end_tuplea) <= math.dist(start_point, end_tupleb)):
        end_tuple = end_tuplea
        end_turn = -1 #turn left one of these could possibly be inverted need to test (flipped >)
    else:
        end_tuple = end_tupleb
        end_turn = 1 #turn right

    #define the start circle
    start_tuplea = (math.cos(math.radians(start_direction + 90)), math.sin(math.radians(start_direction + 90)))
    start_tuplea = (start_point[0] + turn_radius * start_tuplea[0], start_point[1] + turn_radius * start_tuplea[1])

    start_tupleb = (math.cos(math.radians(start_direction - 90)), math.sin(math.radians(start_direction-90)))
    start_tupleb = (start_point[0] + turn_radius * start_tupleb[0], start_point[1] + turn_radius * start_tupleb[1])
    

    if (math.dist(start_tuplea, end_tuple) <= math.dist(start_tupleb, end_tuple)):
        start_tuple = start_tuplea
        start_turn = -1 #turn left
    else:
        start_tuple = start_tupleb
        start_turn = 1 #turn right

    # --- UNTESTED CODE TO REDUCE THE STUFF BEFORE ---
    # end_tuple, end_turn = pivot_tuple(start_point, end_point, end_direction)
    # start_tuple, start_turn = pivot_tuple(end_tuple, start_point, start_direction) # not sure if it is end_tuple or end_point
    
    # if we are using both a or both b
    #m = (end_tuple[1]-start_tuple[1]) / (end_tuple[0]-start_tuple[0]) #slope 
    #b = start_tuple[1] - m * start_tuple[0]

    if start_turn == end_turn:
    # if ((start_tuple == start_tuplea and end_tuple == end_tuplea) or (start_tuple == start_tupleb and end_tuple == end_tupleb)):
            
        tan_point_starta = (start_tuple[0] + turn_radius * (math.cos(direction(start_tuple, end_tuple) - pi / 2)), start_tuple[1] + turn_radius * (math.sin(direction(start_tuple,end_tuple) - pi / 2))) #counterclockwise
        
        tan_point_startb = (start_tuple[0] + turn_radius * (math.cos(direction(start_tuple, end_tuple) + pi / 2)) , start_tuple[1] + turn_radius * (math.sin(direction(start_tuple,end_tuple) + pi / 2))) #clockwise
            
            
        tan_point_endb = (end_tuple[0] + turn_radius * (math.cos(direction(end_tuple, start_tuple) - pi / 2)), end_tuple[1] + turn_radius * (math.sin(direction(end_tuple, start_tuple) - pi / 2))) #counterclockwise
            
        tan_point_enda = (end_tuple[0] + turn_radius * (math.cos(direction(end_tuple, start_tuple) + pi / 2)), end_tuple[1] + turn_radius * (math.sin(direction(end_tuple, start_tuple) + pi / 2))) #clockwise

        #c = abs(direction(start_tuple,tan_point_starta)-direction(start_tuple,start_point)+ math.radians(start_direction) - direction(tan_point_starta,tan_point_enda))%(2*math.pi)
        if abs(direction(start_tuple, tan_point_starta) - direction(start_tuple,start_point) + math.radians(start_direction) - direction(tan_point_starta, tan_point_enda) )% (2 * math.pi) <= 0.0001:
            tan_point_start = tan_point_starta
            tan_point_end = tan_point_enda
        else:
            tan_point_start = tan_point_startb
            tan_point_end = tan_point_endb
    else:
        hypotenuse = math.dist(start_tuple, end_tuple)
        short = 2 * turn_radius
        theta = math.atan2(end_tuple[1] - start_tuple[1], end_tuple[0] - start_tuple[0]) + math.asin(short / hypotenuse) - pi / 2
        tan_point_starta = (start_tuple[0] + turn_radius * math.cos(theta), start_tuple[1] + turn_radius * math.sin(theta))
        tan_point_enda = (end_tuple[0] + turn_radius * math.cos(theta+pi), end_tuple[1] + turn_radius * math.sin(theta + pi))
        if abs((direction(start_tuple, tan_point_starta) - direction(start_tuple, start_point) + math.radians(start_direction) - direction(tan_point_starta, tan_point_enda))) % (2 * math.pi) <= 0.0001:
            tan_point_start = tan_point_starta
            tan_point_end = tan_point_enda
        else: 
                
            if end_tuple[0]-start_tuple[0] == 0:
                inv_slope = 0
                b2 = tan_point_starta[1] - inv_slope * tan_point_starta[0]
                b3 = tan_point_enda[1] - inv_slope * tan_point_enda[0] 
                tan_point_startb = 2 * start_tuple[0] - tan_point_starta[0], b2
                tan_point_endb = 2 * start_tuple[0] - tan_point_enda[0], b3
            else: 
                slope = (end_tuple[1] - start_tuple[1]) / (end_tuple[0] - start_tuple[0])
                b1 = start_tuple[1] - slope * start_tuple[0]
                
                if slope != 0:
                    inv_slope = -1 / slope
                    b2 = tan_point_starta[1] - inv_slope * tan_point_starta[0]
                    b3 = tan_point_enda[1] - inv_slope * tan_point_enda[0] 
                    intersectionstart = ((b2 - b1) / (slope - inv_slope),0)
                    intersectionend = ((b3 - b1) / (slope - inv_slope),0)
                    intersectionstart= (intersectionstart[0],intersectionstart[0] * slope + b1)
                    intersectionend = (intersectionend[0], intersectionend[0] * slope + b1) 
                    tan_point_startb = (2 * intersectionstart[0]- tan_point_starta[0], 2 * intersectionstart[1] - tan_point_starta[1])
                    tan_point_endb = (2 * intersectionend[0] - tan_point_enda[0],2 * intersectionend[1]- tan_point_enda[1])
                else:
                    tan_point_startb = (tan_point_starta[0],start_tuple[1]*2-tan_point_starta[1])
                    tan_point_endb = (tan_point_enda[0],end_tuple[1]*2-tan_point_enda[1])

            tan_point_start = tan_point_startb
            tan_point_end = tan_point_endb
  
    
    placeholder_tuple1= (start_tuple[0]+turn_radius*math.cos(math.radians(start_direction)),start_tuple[1]+turn_radius*math.sin(math.radians(start_direction)))
    placeholder_tuple2 = (start_tuple[0]-turn_radius*math.cos(math.radians(start_direction)),start_tuple[1]-turn_radius*math.sin(math.radians(start_direction)))

    if (math.dist(tan_point_start, placeholder_tuple1)<= math.dist(tan_point_start, placeholder_tuple2)):
        direction_movement_start = 1 #forward
    
    else:
        direction_movement_start = -1 #backward

    if math.dist(tan_point_end, (end_tuple[0] - turn_radius * math.cos(math.radians(end_direction)), end_tuple[1] - turn_radius * math.sin(math.radians(end_direction)))) <= math.dist(tan_point_end, (end_tuple[0] + turn_radius * math.cos(math.radians(end_direction)), end_tuple[1] + turn_radius * math.sin(math.radians(end_direction)))):
        direction_movement_end = 1 #forward
    
    else:
        direction_movement_end = -1 #backward
    
    # --- UNTESTED CODE TO REDUCE THE STUFF BEFORE ---
    # Find the initial direction the robot needs to move in
    # direction_movement_start = final_direction(start_tuple, tan_point_start, start_direction)
    # direction_movement_end = final_direction(end_tuple, tan_point_end, end_direction)
    distance1, distance3 = 0, 0
    
    if start_turn == -1:
        distance1 = round(turn_radius * (round((direction(start_tuple, tan_point_start) - direction(start_tuple, start_point)), 3) % (2*pi)), 3)
        if (distance1 > turn_radius * pi):
            distance1 = 2 * turn_radius * pi - distance1
            direction_movement_start = direction_movement_start * -1
    else:
        distance1 = round(turn_radius * (round((direction(start_tuple, start_point) - direction(start_tuple, tan_point_start)), 3) % (2*pi)), 3)
        if (distance1 > turn_radius * pi):
            distance1 = 2 * turn_radius * pi - distance1
            direction_movement_start = direction_movement_start * -1
    
    distance2 = round(math.dist(tan_point_start, tan_point_end),3)
    
    if end_turn == -1:
        distance3 = round(turn_radius * (round((direction(end_tuple, end_point) - direction(end_tuple, tan_point_end)), 3) % (2*pi)), 3)
        if (distance3 > turn_radius * pi):
            distance3 = round(2 * turn_radius * pi - distance3 , 3)
            direction_movement_end = direction_movement_end * -1
    else:
        distance3 = round(turn_radius * (round((direction(end_tuple, tan_point_end) - direction(end_tuple, end_point)), 3) % (2*pi)), 3)
        if (distance3 > turn_radius * pi):
            distance3 = round(2 * turn_radius * pi - distance3 , 3)
            direction_movement_end = direction_movement_end * -1
            
    #return (start_turn, direction_movement_start, tan_point_start, tan_point_end, direction_movement_end, end_turn, end_point)
    return (start_turn, direction_movement_start, distance1, 0, 1, distance2, end_turn, direction_movement_end, distance3)
# outputs starting turn direction, point which robot starts going straight, point robot stops going straight, ending turn direction, destination
#right = 1 left = -1



if __name__ == "__main__":
   # start_turn,direction_movement_start,tan_point_start,tan_point_end,direction_movement_end,end_turn,end_point = turn_path((0,6),0,(8,6),180)
    start_turn,direction_movement_start,distance1,mid_turn,direction_movement_mid,distance2,end_turn,direction_movement_end,distance3 = turn_path((0.14,-2.5),90,(-2.5,2),180)

    # Verbose
    print(f"Start turn: {start_turn}")
    print(f"Direction movement start: {direction_movement_start}")
    print(f"Distance of segment 1: {distance1}")
    print(f"Middle turn: {mid_turn}")
    print(f"Direction movement mid: {direction_movement_mid}")
    print(f"Distance of segment 2: {distance2}")
    print(f"End turn: {end_turn}")
    print(f"Direction movement end: {direction_movement_end}")
    print(f"Distance of segment 3: {distance3}")
