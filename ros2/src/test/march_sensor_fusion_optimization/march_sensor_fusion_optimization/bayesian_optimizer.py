"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

import sklearn.gaussian_process as gp

class BayesianOptimizer:
    def __init__(self, kernel, alpha) -> None:
        self.kernel = kernel
        self.alpha = alpha
        self.model = gp.GaussianProcessRegressor(kernel=self.kernel)

    def fit(self, X, y) -> None:
        self.model.fit(X, y)

    def predict(self, X):
        return self.model.predict(X)

    def optimize(self, X, y):
        pass