import numpy as np

class SlidingModeControl:
    def __init__(self, lambda_, delta_, kD_):
        self.lambda_ = lambda_
        self.delta_ = delta_
        self.kD_ = kD_
    
    def sliding_surface(self, error):
        return error + self.lambda_ * error
    
    def continous_control(self, sliding_surface):
        return -self.lambda_ * sliding_surface
    
    def discontinous_control(self, sliding_surface):
        if abs(sliding_surface) > self.delta_:
            return -self.kD_ * sliding_surface / abs(sliding_surface)
        else:
            return -self.kD_ * sliding_surface / self.delta_
        
    def control_law(self, error):
        sliding_surface = self.sliding_surface(error)
        u_cont = self.continous_control(sliding_surface)
        u_disc = self.discontinous_control(sliding_surface)
        return (u_cont + u_disc) * -1
    
    def takeoff(self, error, derivative_error):
        # sliding_surface = error + self.lambda_ * derivative_error
        # if sliding_surface > self.delta_:
        #     control = -self.kD_
        # elif sliding_surface < -self.delta_:
        #     control = self.kD_
        # else:
        #     control = -self.kD_ * sliding_surface / self.delta_
        # return control * 10
        sliding_surface = self.sliding_surface(error)
        u_cont = self.continous_control(sliding_surface)
        u_disc = self.discontinous_control(sliding_surface)
        return (u_cont + u_disc) * -1