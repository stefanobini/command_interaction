#! /usr/bin/bash
package=torch-1.13.0a0+340c4120.nv22.06-cp38-cp38-linux_aarch64.whl
wget "https://developer.download.nvidia.com/compute/redist/jp/v50/pytorch/${package}"
python3 -m pip install --no-cache "${package}"