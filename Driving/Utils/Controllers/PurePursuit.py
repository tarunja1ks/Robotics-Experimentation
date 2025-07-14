import math
class PurePursuit:
    def __init__(self,jackal_robot,path,lookahead=0.5):
        stuff=True
        self.path=path
        self.lookahead=lookahead
        self.jackal_robot=jackal_robot
        self.current_index=0
    def find_lookahead_point(self): # finding the next lookahead point
        robot_position = self.jackal_robot.localize() # localizes properly
        for i in range(self.current_index, len(self.path) - 1):
            start, end = self.path[i], self.path[i + 1] # looking in the line between waypoints
            # Compute quadratic coefficients for lookahead
            dx, dy = end[0] - start[0], end[1] - start[1]
            fx, fy = start[0] - robot_position[0], start[1] - robot_position[1]
            a, b, c = dx**2 + dy**2, 2 * (fx * dx + fy * dy), fx**2 + fy**2 - self.lookahead**2
            discriminant = b**2 - 4 * a * c

            if discriminant >= 0: # finding and returning intersection/updating the points crossed to make sure it doesn't go backwards
                discriminant_sqrt = math.sqrt(discriminant)
                t1 = (-b + discriminant_sqrt) / (2 * a)
                t2 = (-b - discriminant_sqrt) / (2 * a)

                if 0 <= t1 <= 1:
                    self.current_index = i  # Update current index
                    return (start[0] + t1 * dx, start[1] + t1 * dy)
                if 0 <= t2 <= 1:
                    self.current_index = i  # Update current index
                    return (start[0] + t2 * dx, start[1] + t2 * dy)

        return self.path[-1] # returns last point if nothing found(shouldnt ever occur but fault tolerance)

        

    
    
    
    