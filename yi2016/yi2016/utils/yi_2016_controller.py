import GPy

class Yi2016Controller:
    def __init__(self):
        pass

    # Algorithm 1 Initialize
    def set_initial_point(self, initial_points): 
        self._T = initial_points  # T is data set
    
    def dataset_size(self):
        return len(self.T)

    # Algorithm 1 Loop 1)
    def _train_gp_surface_model(self):
        pass

    # Algorithm 1 Loop 2)
    def _calc_acquisition_function(self):
        pass

    # Algorithm 1 Loop 3)
    def _find_optimal_x(self):
        pass

    # Algorithm 1 Loop 4) 1
    def _evalate_optimal_x(self):
        pass

    # Algorithm 1 Loop 4) 2
    def _obtain_y_at_optimal_x(self):
        pass

    # Algorithm 1 Loop 5)
    def _add_x_y_to_T(self, x, y):
        pass

def _main():
    controller = Yi2016Controller()


if __name__ == "__main__":
    _main()
