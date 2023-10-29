class Tools():    

    def clamp(num:float, min_value:float, max_value:float)->float:
        """ Method which takes three parameters where n is the number we 
            would like to clip and the range to be used for clipping the number."""
        
        if num < min_value:
            return min_value
        elif num > max_value:
            return max_value
        else:
            return num
        
    def deadband(value:float, size:float)->float:
        return value if abs(value) >= size else 0

    def potentiometer_to_degrees(pot_value: float)->float:
        return -0.084482109 * pot_value + 82.33745763

    def degrees_to_potentiometer(degree_value: float)->float:
        return round((degree_value - 82.33745763) / -0.084482109, 0)
