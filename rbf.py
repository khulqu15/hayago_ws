import numpy as np

class RBFNN:
    def __init__(self, num_centers, learning_rate=0.01):
        self.centers = np.random.uniform(-1, 1, num_centers)
        self.widths = np.ones(num_centers) * 0.5
        self.weights = np.random.uniform(-0.5, 0.5, num_centers)
        self.learning_rate = learning_rate
        
    def basis_function(self, x, center, width):
        return np.exp(-((x - center)**2)/(2 * width ** 2))

    def forward(self, x):
        activations = [self.basis_function(x, c, w) for c, w in zip(self.centers, self.widths)]
        output = np.dot(activations, self.weights)
        return output
    
    def adapt_weights(self, x ,error):
        activations = [self.basis_function(x, c, w) for c, w in zip(self.centers, self.widths)]
        for i in range(len(self.weights)):
            self.weights[i] += self.learning_rate * error * activations[i]