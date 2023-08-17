

class Tools():    

    def clamp(num, min_value, max_value):
        """ Method which takes three parameters where n is the number we 
            would like to clip and the range to be used for clipping the number."""
        
        if num < min_value:
            return min_value
        elif num > max_value:
            return max_value
        else:
            return num