#!/usr/bin/env python3
"""G1 (CUDA): torch sees the GPU, matmul and cuDNN conv agree with a CPU fp64
reference. Prints one JSON object."""
import json
import platform
import time

import torch

out = {
    'python': platform.python_version(),
    'torch': torch.__version__,
    'torch_cuda': torch.version.cuda,
    'cudnn': torch.backends.cudnn.version(),
    'cuda_available': torch.cuda.is_available(),
}
if out['cuda_available']:
    out['device'] = torch.cuda.get_device_name(0)
    out['capability'] = '.'.join(map(str, torch.cuda.get_device_capability(0)))
    torch.manual_seed(0)
    for dtype in (torch.float32, torch.float16):
        a = torch.randn(2048, 2048, device='cuda', dtype=dtype)
        b = torch.randn(2048, 2048, device='cuda', dtype=dtype)
        (a @ b)
        torch.cuda.synchronize()
        t = time.perf_counter()
        for _ in range(10):
            c = a @ b
        torch.cuda.synchronize()
        dt = (time.perf_counter() - t) / 10
        ref = a.cpu().double() @ b.cpu().double()
        rel = ((c.cpu().double() - ref).abs().max() / ref.abs().max()).item()
        out[f'matmul_{str(dtype)[6:]}'] = {
            'ms': round(dt * 1e3, 3), 'tflops': round(2 * 2048 ** 3 / dt / 1e12, 3),
            'max_rel_err_vs_cpu_fp64': rel}
    x = torch.randn(1, 64, 128, 128, device='cuda')
    w = torch.randn(64, 64, 3, 3, device='cuda')
    y = torch.nn.functional.conv2d(x, w, padding=1)
    yref = torch.nn.functional.conv2d(x.cpu().double(), w.cpu().double(), padding=1)
    out['cudnn_conv_max_rel_err'] = ((y.cpu().double() - yref).abs().max() / yref.abs().max()).item()
print(json.dumps(out, indent=1))
