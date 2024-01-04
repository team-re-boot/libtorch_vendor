# libtorch_vendor

Vendor package for libtorch (pytorch C++ API)

## Trouble shooting.

- Unsupported GPU Architecture
If you see errors like below, please specify TORCH_CUDA_ARCH_LIST environment variables.

```
nvcc fatal unsupported gpu architecture 'compute_35'
```

You can check compute capability [here](https://developer.nvidia.com/cuda-gpus).

For example, if you use jetson agx orin, run command below.

```
export TORCH_CUDA_ARCH_LIST=8.7
```
