import torch as th
import torch.nn as nn
from stable_baselines3.common.policies import ActorCriticCnnPolicy
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor


class Custom1DCNN(BaseFeaturesExtractor):
    def __init__(
        self,
        observation_space,
        output_dim=32,
        n_filters1=16,
        n_filters2=32,
        kernel_size1=3,
        kernel_size2=3,
    ):
        super(Custom1DCNN, self).__init__(observation_space, features_dim=output_dim)

        n_input_channels = observation_space.shape[
            0
        ]  # Using the first dimension as input channels

        # Define 1D CNN layers with customizable sizes
        self.cnn = nn.Sequential(
            nn.Conv1d(
                n_input_channels,
                n_filters1,
                kernel_size=kernel_size1,
                stride=1,
                padding=1,
            ),
            nn.ReLU(),
            nn.Conv1d(
                n_filters1, n_filters2, kernel_size=kernel_size2, stride=1, padding=1
            ),
            nn.ReLU(),
            nn.Flatten(),
        )

        # Compute the size after CNN layers
        with th.no_grad():
            sample_input = th.zeros((1, n_input_channels, observation_space.shape[1]))
            n_flatten = self.cnn(sample_input).shape[1]

        # Final linear layer to adjust to the output dimension
        self.linear = nn.Linear(n_flatten, output_dim)

    def forward(self, observations):
        return self.linear(self.cnn(observations))


# Custom Policy class
class CustomCNNPolicy(ActorCriticCnnPolicy):
    def __init__(self, *args, **kwargs):
        super(CustomCNNPolicy, self).__init__(
            *args,
            features_extractor_class=Custom1DCNN,
            features_extractor_kwargs=dict(
                features_dim=256,
                # You can also pass any additional parameters here if needed
            ),
            **kwargs,
        )
