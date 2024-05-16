"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

import tensorflow as tf
from tensorflow_probability import distributions as tfd
from tensorflow_probability import math as tfm

class BayesianOptimizer:
    def __init__(self) -> None:
        self.dof = 3.0
        self.kernel = tfm.psd_kernels.ExponentiatedQuadratic(1.0, 1.0)
        self.model = tfd.StudentTProcess(
            df=self.dof, 
            kernel=self.kernel,
            index_points=tf.constant([[0.0], [1.0], [2.0]], dtype=tf.float32),
            mean_fn=self.mean_fn,
            observation_noise_variance=1e-2,
        )

    def fit(self, X, y) -> None:
        self.model.fit(X, y)

    def predict(self, X):
        return self.model.predict(X)

    def optimize(self, X, y):
        pass

    def mean_fn(self, x):
        return tf.zeros_like(x[..., 0])