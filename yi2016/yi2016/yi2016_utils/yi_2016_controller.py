import GPy
import numpy as np
from scipydirect import minimize

class Yi2016Controller:
    # Algorithm 1 Initialize in Yi Paper
    def __init__(self,
                 input_dim,
                 output_dim,
                 bounds,
                 initial_points):
        
        # Check Dimension
        assert input_dim + output_dim == initial_points.shape[-1] 
        
        # Initialize Properties
        self.input_dim  = input_dim
        self.output_dim = output_dim
        self._T = initial_points      # _T is train data
        self.bounds = bounds          # boundary of the target object

        # GPy Settings
        self.kernel = GPy.kern.RBF(self.input_dim)
        self.model = None
        self.acquisition_function = None

    # Algorithm 1 Loop 1) - 3) in Yi Paper
    def calc_next_point(self):
        self._update_surface_model()                      # 1)
        self._update_acquisition_function()               # 2)
        af = lambda x: -1 * self.acquisition_function(x)  # 3) A 
        next_point = minimize(af, self.bounds)            # 3) B
        return next_point

    def _update_surface_model(self):
        X = self._T[:, :self.input_dim ]
        Y = self._T[:,  self.input_dim:]
        self.model = GPy.models.GPRegression(X, Y, self.kernel)
        self.model.optimize()  # It is probably need , considering the sim result of Yi Paper.
    
    def _update_acquisition_function(self):
        def _sigma(x):
            # preprocess
            x = np.array(x)
            if x.ndim == 0:
                x = x.reshape(1, 1)
            if x.ndim == 1:
                x = x[:, None]
            if x.shape[-1] != self.input_dim:
                raise ValueError("x.shape[-1] != self.input_dim")

            # main
            _, var = self.model.predict(x)
            return var
        
        self.acquisition_function = _sigma

    # ######################### NOTE ################################################ # 
    # -- Algorithm 1 Loop 4) is written in controller_manager.py.                  -- #
    # -- because in loop 4), we have to use ROS utils to obtain y* from the robot. -- #
    # ############################################################################### #

    # Algorithm 1 Loop 5) in Yi Paper
    def add_x_y_to_T(self, x, y):
        X = self._T[:, :self.input_dim ]
        Y = self._T[:,  self.input_dim:]
        X_new = np.vstack((X, x))
        Y_new = np.vstack((Y, y))
        self._T = np.concatenate([X_new, Y_new], axis=1)

def _main():

    #######################
    # This is Sample Code #
    #######################

    # Display Modules
    from IPython.display import display
    from matplotlib import pyplot as plt

    # Define Target Object Shape
    target_object = lambda x: np.sin(x) + np.random.randn(*(x.shape)) * 0.05

    # Create Initial Training Data
    X = np.random.uniform(-3., 3., (3, 1))  
    Y = target_object(X)
    initial_points = np.concatenate([X, Y], axis=1)

    # Initialize Controller
    controller = Yi2016Controller(
                    input_dim=X.shape[-1],
                    output_dim=Y.shape[-1],
                    bounds=[(-3, 3)],
                    initial_points=initial_points)
    # Main Loop
    while True:
        # Algorithm 1 Loop 1) - 3) in Yi Paper
        x_star = controller.calc_next_point()

        # Display GP Model            
        controller.model.plot()
        plt.show()
        plt.close()

        # Algorithm 1 Loop 4) in Yi Paper
        y_star = target_object(x_star.x)

        # Algorithm 1 Loop 5) in Yi Paper
        controller.add_x_y_to_T(x_star.x, y_star)

        # Continue ? 
        c = input("0:break from loop, Other: continue : ")
        if c == 0:
            break
    
if __name__ == "__main__":
    _main()
