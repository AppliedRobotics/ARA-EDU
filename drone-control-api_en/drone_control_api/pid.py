class PID:
    def __init__(self, Kp, Ti, Td):
        self.Kp = Kp
        self.Td = Td
        self.Ti = Ti
        self.curr_error = 0
        self.prev_error = 0
        self.sum_error = 0
        self.prev_error_deriv = 0
        self.curr_error_deriv = 0
        self.control = 0

    def update_control(self, current_error, reset_prev=False):
        self.prev_error = self.curr_error
        self.curr_error = current_error
        
        # Calculating the integral error
        self.sum_error = self.sum_error + self.curr_error

        # Calculating the derivative error
        self.curr_error_deriv = self.curr_error - self.prev_error

        # Calculating the PID Control
        self.control = self.Kp * self.curr_error + self.Ti * self.sum_error + self.Td * self.curr_error_deriv

    def get_control(self):
        return self.control
