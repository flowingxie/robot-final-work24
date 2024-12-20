class PID:
    def __init__(self, kp, ki, kd):
        self.__version__ = '0.0.1'
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.err = 0
        self.sum_err = 0
        self.last_err = 0
    def set_err(self, err):
        self.err = err
        self.sum_err += err
    @property
    def output(self):
        out = self.kp * self.err
        out += self.ki * self.sum_err
        out += self.kd * (self.err - self.last_err)
        #print(out, self.kp * self.err, self.ki * self.sum_err, self.kd * (self.err - self.last_err))
        self.last_err = self.err
        return out