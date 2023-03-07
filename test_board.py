import torch
from torch.utils.tensorboard import SummaryWriter

writer = SummaryWriter("D:\code\Dynamic-Grasping\logs")

for i in range(100):
    writer.add_scalar('y=2x',2*i,i)

writer.close()
