#!/usr/bin/env python3
"""G2 (TensorRT): the TRT 10.3 cp312 wheel imports, builds a small conv+relu
engine (fp32 and fp16), runs it on torch CUDA buffers and matches a CPU
reference. Optional argv[1]: an Ultralytics .engine built on the host's
Humble TRT 10.3, to check it deserializes in the image. Prints JSON."""
import json
import struct
import sys

import numpy as np
import tensorrt as trt
import torch

out = {'tensorrt': trt.__version__}
logger = trt.Logger(trt.Logger.WARNING)
rng = np.random.default_rng(0)
w = rng.standard_normal((8, 3, 3, 3)).astype(np.float32)
bias = rng.standard_normal(8).astype(np.float32)

for name, flag in (('fp32', None), ('fp16', trt.BuilderFlag.FP16)):
    builder = trt.Builder(logger)
    net = builder.create_network(0)
    x = net.add_input('x', trt.float32, (1, 3, 64, 64))
    conv = net.add_convolution_nd(x, 8, (3, 3), trt.Weights(w), trt.Weights(bias))
    conv.padding_nd = (1, 1)
    act = net.add_activation(conv.get_output(0), trt.ActivationType.RELU)
    act.get_output(0).name = 'y'
    net.mark_output(act.get_output(0))
    cfg = builder.create_builder_config()
    cfg.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 28)
    if flag is not None:
        cfg.set_flag(flag)
    plan = builder.build_serialized_network(net, cfg)
    engine = trt.Runtime(logger).deserialize_cuda_engine(plan)
    ctx = engine.create_execution_context()
    xin = torch.randn(1, 3, 64, 64, device='cuda')
    yout = torch.empty(1, 8, 64, 64, device='cuda')
    ctx.set_tensor_address('x', xin.data_ptr())
    ctx.set_tensor_address('y', yout.data_ptr())
    ok = ctx.execute_async_v3(torch.cuda.current_stream().cuda_stream)
    torch.cuda.synchronize()
    ref = torch.relu(torch.nn.functional.conv2d(
        xin.cpu().double(), torch.from_numpy(w).double(), torch.from_numpy(bias).double(), padding=1))
    err = ((yout.cpu().double() - ref).abs().max() / ref.abs().max()).item()
    out[name] = {'plan_bytes': plan.nbytes, 'execute_ok': bool(ok), 'max_rel_err': err}

if len(sys.argv) > 1:
    # Ultralytics engine file: int32 little-endian metadata length, JSON metadata,
    # TRT plan - or a bare plan (falls back like ultralytics' own loader).
    with open(sys.argv[1], 'rb') as f:
        data = f.read()
    try:
        n = struct.unpack('<i', data[:4])[0]
        meta = json.loads(data[4:4 + n].decode())
        plan = data[4 + n:]
    except (UnicodeDecodeError, json.JSONDecodeError, ValueError):
        meta, plan = {}, data
    engine = trt.Runtime(logger).deserialize_cuda_engine(plan)
    out['host_engine'] = {
        'file': sys.argv[1], 'ultralytics_meta_version': meta.get('version'),
        'deserialized': engine is not None,
        'io': [engine.get_tensor_name(i) for i in range(engine.num_io_tensors)] if engine else None}
print(json.dumps(out, indent=1))
