import torch
import torchvision
from demo_superpoint  import SuperPointNet
model = SuperPointNet()
model.load_state_dict(torch.load("superpoint_v1.pth"))
model.eval()
example = torch.rand(1, 1, 640, 480)
traced_script_module = torch.jit.trace(model, example)
traced_script_module.save("superpoint_v1.pt")
