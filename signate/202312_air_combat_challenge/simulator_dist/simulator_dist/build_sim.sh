#!/bin/bash

SIMDIR=./root

# simulator
cd ${SIMDIR}
pip install .
cd ./sample/modules/OriginalModelSample
pip install .

# additional libraries
conda install -c conda-forge libstdcxx-ng -y
pip install --upgrade pip && pip install opencv-python==4.8.0.74 timeout-decorator==0.5.0 keras-rl==0.4.2 pfrl==0.3.0

# clean up
pip cache purge
conda clean -a -y
