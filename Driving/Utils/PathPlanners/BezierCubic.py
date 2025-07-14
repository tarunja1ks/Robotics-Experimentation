import numpy as np
class Bezier:
    def __init__(self,p, sliders):
        self.control_points=[
            np.array([0, 0, 0]),
            np.array([2, 1, 0]),
            np.array([4, -1, 0]),
            np.array([6, 0, 0])
        ]
        self.p=p
        self.t=0
        self.sliders=sliders
      
    def draw_bezier_path(self):
        """Draws the Bezier curve based on the current control points."""
        self.p.removeAllUserDebugItems()  # Clear previous debug lines
        for i in range(len(self.path) - 1):
            self.p.addUserDebugLine(
                self.path[i].tolist(),
                self.path[i + 1].tolist(),
                lineColorRGB=[1, 0, 0],
                lineWidth=70
            )
    def get_bezier_curve(self, t): 
        """Computes a point on the Bezier curve at parameter t."""
        cp = self.control_points
        return (1 - t)**3 * cp[0] + 3 * (1 - t)**2 * t * cp[1] + 3 * (1 - t) * t**2 * cp[2] + t**3 * cp[3]
    
    

    def update_control_points(self):
        """Updates the control points based on the slider values."""
        for i, slider in enumerate(self.sliders):
            control_point_idx = i // 3  # Each control point has 3 sliders (X, Y, Z)
            axis_idx = i % 3  # 0 for X, 1 for Y, 2 for Z
            value = self.p.readUserDebugParameter(slider)
            self.control_points[control_point_idx][axis_idx] = value

    def update_sliders(self,sliders):
        self.sliders=sliders
        
    def get_Path(self,length=200): # getting multiple points in the path to use to draw it in pybullet
        iteration=1/length
        self.path=[]
        time=0
        while(time<=1):
            self.path.append(self.get_bezier_curve(time))
            time+=iteration
        return self.path