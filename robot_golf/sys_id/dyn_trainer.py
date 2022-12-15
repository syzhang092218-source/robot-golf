import os
import torch
import torch.nn as nn
import numpy as np

from torch.optim import Adam
from torch.utils.tensorboard import SummaryWriter
from tqdm import tqdm


class DynTrainer:

    def __init__(self, predictor, device, qdots=None, vs=None):
        cur_path = os.path.dirname(__file__)
        self.device = device
        if qdots is None:
            self.qdots = torch.from_numpy(np.load(os.path.join(cur_path, 'data/qdots_6300.npy'), allow_pickle=True)).to(device).type(torch.float)
        else:
            self.qdots = torch.from_numpy(qdots).to(device).type(torch.float)
        if vs is None:
            self.vs = torch.from_numpy(np.load(os.path.join(cur_path, 'data/vs_6300.npy'), allow_pickle=True)).to(device).type(torch.float)
        else:
            self.vs = torch.from_numpy(vs).to(device).type(torch.float)
        self.qdots_val = torch.from_numpy(np.load(os.path.join(cur_path, 'data/qdots_val.npy'), allow_pickle=True)).to(device).type(torch.float)
        self.vs_val = torch.from_numpy(np.load(os.path.join(cur_path, 'data/vs_val.npy'), allow_pickle=True)).to(device).type(torch.float)
        self.predictor = predictor
        self.optim = Adam(self.predictor.parameters(), lr=3e-5)
        if not os.path.exists('./sys_id_log'):
            os.mkdir('./sys_id_log')
        self.writer = SummaryWriter(log_dir='./sys_id_log')

    def train(self, n_iter: int, batch_size=128):
        n_data = self.vs.shape[0]
        loss_fun = nn.MSELoss()
        print(f'> Training dynamics using {self.device}')
        if not os.path.exists('./sys_id_model'):
            os.mkdir('./sys_id_model')
        for i_iter in tqdm(range(1, n_iter + 1)):
            data_id = np.random.randint(0, n_data, batch_size)
            q_dot_batch = self.qdots[data_id]
            v_batch = self.vs[data_id]
            loss_train = loss_fun(q_dot_batch, self.predictor(v_batch))

            with torch.no_grad():
                loss_val = loss_fun(self.qdots_val, self.predictor(self.vs_val))

            self.optim.zero_grad()
            loss_train.backward()
            self.optim.step()

            self.writer.add_scalar('loss/train', loss_train.item(), i_iter)
            self.writer.add_scalar('loss/val', loss_val.item(), i_iter)
            if i_iter % (n_iter / 10) == 0:
                torch.save(self.predictor, f'./sys_id_model/{i_iter}.pkl')

        print('> Dynamics finished training')
