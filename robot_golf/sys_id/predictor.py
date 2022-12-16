import torch
import torch.nn as nn


def init_param(module: nn.Module, gain: float = 1.):
    nn.init.orthogonal_(module.weight.data, gain=gain)
    nn.init.constant_(module.bias.data, 0)
    return module


class MLP(nn.Module):

    def __init__(self, in_dim: int, out_dim: int, hidden_layers: tuple,
                 hidden_activation: nn.Module = nn.ReLU(), output_activation: nn.Module = None,
                 init: bool = True, gain: float = 1.):
        super().__init__()

        # build MLP
        layers = []
        units = in_dim
        for next_units in hidden_layers:
            if init:
                layers.append(init_param(nn.Linear(units, next_units), gain=gain))
            else:
                layers.append(nn.Linear(units, next_units))
            layers.append(hidden_activation)
            units = next_units
        if init:
            layers.append(init_param(nn.Linear(units, out_dim), gain=gain))
        else:
            layers.append(nn.Linear(units, out_dim))
        if output_activation is not None:
            layers.append(output_activation)

        self.net = nn.Sequential(*layers)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


class NormalizedMLP(MLP):

    def __init__(
            self,
            in_dim: int,
            out_dim: int,
            input_mean: torch.Tensor,
            input_std: torch.Tensor,
            hidden_layers: tuple,
            hidden_activation: nn.Module = nn.ReLU(),
            output_activation: nn.Module = None,
            init: bool = True,
            gain: float = 1.,
            limit_lip=True,
    ):
        super().__init__(in_dim, out_dim, hidden_layers, hidden_activation, output_activation, init, gain)

        assert input_mean.ndim == 1 and input_mean.shape[0] == in_dim
        assert input_std.ndim == 1 and input_std.shape[0] == in_dim
        self.input_mean = input_mean
        self.input_std = input_std

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        if x.ndim == 1:
            x = x.unsqueeze(0)
        x_trans = self.normalize_input(x)
        return self.net(x_trans)

    def normalize_input(self, x: torch.Tensor) -> torch.Tensor:
        nonzero_std_dim = torch.nonzero(self.input_std)
        zero_mask = torch.ones(self.input_std.shape[0]).type_as(self.input_std)
        zero_mask[nonzero_std_dim] = 0
        zero_mask = zero_mask.reshape(x[0].shape)
        x_trans = (x - self.input_mean.reshape(x[0].shape)) / (self.input_std.reshape(x[0].shape) + zero_mask)
        return x_trans


class QDotPredictor(nn.Module):

    def __init__(self, device):
        super(QDotPredictor, self).__init__()
        self.v_mean = torch.tensor([3.42963496, 0.43245987, 11.179429]).to(device)
        self.v_std = torch.tensor([0.46331278, 0.26750121, 1.32229425]).to(device)
        self.net = NormalizedMLP(
            in_dim=3, out_dim=2, input_mean=self.v_mean, input_std=self.v_std, hidden_layers=(8, 8)
        )
        self.qdot_mean = torch.tensor([-50, -7]).to(device)
        self.qdot_std = torch.tensor([5, 8]).to(device)

    def forward(self, v: torch.Tensor):
        return self.net(v) * self.qdot_std + self.qdot_mean
