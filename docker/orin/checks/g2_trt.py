#!/usr/bin/env python3
"""
TensorRT check: toy engines built and run, a real engine deserialized.

The TRT wheel imports, builds a small conv+relu engine (fp32 and fp16), runs
it on torch CUDA buffers and matches a CPU reference; with --engine, a
serialized engine (Ultralytics format or a bare plan) deserializes
and lists its I/O tensors.

Runs on: inside the container (car or harness role; the image's TensorRT
10.3.0.26 cp312 and torch 2.8) and, for the native comparison, the Orin host
(the Humble side's TRT 10.3 / torch 2.11). From the spike gate G2 (jazzy
line), with arguments and a verdict.
Plan P8: "`docker_orin.sh engine` builds the TRT engine" -- this proves the
built file loads (`--engine`, default path
/var/cache/f1tenth/trt/10.3.0.26/yolo26s-seg.trt10.3.0.fp16.engine, the
installed stack_params' yolo_engine_path); yolo_run.py then runs it.

  g2_trt.py [--engine PATH | --production-engine] [--out FILE.json]

Prints JSON; exit 0 when both toy engines ran with max relative error below
1e-4 (fp32) / 1e-2 (fp16) and the engine (if any) deserialized.
"""
import argparse
import json
import struct
import sys

PRODUCTION_ENGINE = '/var/cache/f1tenth/trt/10.3.0.26/yolo26s-seg.trt10.3.0.fp16.engine'
TOL = {'fp32': 1e-4, 'fp16': 1e-2}


def toy_engines(trt, torch, np, logger):
    out = {}
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
            xin.cpu().double(), torch.from_numpy(w).double(), torch.from_numpy(bias).double(),
            padding=1))
        err = ((yout.cpu().double() - ref).abs().max() / ref.abs().max()).item()
        out[name] = {'plan_bytes': plan.nbytes, 'execute_ok': bool(ok), 'max_rel_err': err}
    return out


def load_engine(trt, logger, path):
    # Ultralytics engine file: int32 little-endian metadata length, JSON
    # metadata, TRT plan -- or a bare plan (falls back like Ultralytics' loader).
    with open(path, 'rb') as f:
        data = f.read()
    try:
        n = struct.unpack('<i', data[:4])[0]
        meta = json.loads(data[4:4 + n].decode())
        plan = data[4 + n:]
    except (UnicodeDecodeError, json.JSONDecodeError, ValueError, struct.error):
        meta, plan = {}, data
    engine = trt.Runtime(logger).deserialize_cuda_engine(plan)
    return {'file': path, 'bytes': len(data), 'ultralytics_meta_version': meta.get('version'),
            'imgsz': meta.get('imgsz'), 'task': meta.get('task'),
            'deserialized': engine is not None,
            'io': ([engine.get_tensor_name(i) for i in range(engine.num_io_tensors)]
                   if engine else None)}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    g = ap.add_mutually_exclusive_group()
    g.add_argument('--engine', help='serialized engine to deserialize')
    g.add_argument('--production-engine', action='store_true',
                   help='--engine ' + PRODUCTION_ENGINE)
    ap.add_argument('--out', help='also write the JSON here')
    a = ap.parse_args(argv)
    import numpy as np
    import tensorrt as trt
    import torch
    logger = trt.Logger(trt.Logger.WARNING)
    out = {'tensorrt': trt.__version__, 'torch': torch.__version__,
           'cuda_available': torch.cuda.is_available()}
    bad = []
    if not out['cuda_available']:
        bad.append('torch.cuda.is_available() is False')
    else:
        out.update(toy_engines(trt, torch, np, logger))
        for name, tol in TOL.items():
            r = out[name]
            if not r['execute_ok'] or not r['max_rel_err'] < tol:
                bad.append('%s engine: execute_ok %s, max_rel_err %g (tol %g)'
                           % (name, r['execute_ok'], r['max_rel_err'], tol))
    engine = PRODUCTION_ENGINE if a.production_engine else a.engine
    if engine:
        try:
            out['engine'] = load_engine(trt, logger, engine)
            if not out['engine']['deserialized']:
                bad.append('engine %s did not deserialize' % engine)
        except OSError as exc:
            bad.append('engine %s: %s' % (engine, exc))
    out['result'] = 'pass' if not bad else 'fail'
    out['problems'] = bad
    text = json.dumps(out, indent=1)
    print(text)
    if a.out:
        with open(a.out, 'w') as f:
            f.write(text + '\n')
    return 0 if not bad else 1


if __name__ == '__main__':
    sys.exit(main())
