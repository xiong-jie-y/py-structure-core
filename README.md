# py-structure-core
This is a python library to use [Structure Core](https://structure.io/structure-core) depth sensor.

## Installation
Please build yourself, because SDK is only allowed to be used by developer who register in developer site in occipital.

```bash
git clone git@github.com:xiong-jie-y/py-structure-core.git
cd ./py-structure-core

# Not limited to conda.
# Feel free to use your favorite python environment.
conda create -n py38_py_structure_core python=3.8
conda activate py38_py_structure_core

pip install -r requirements.txt

# Prepare sdk into the directory.
# e.g. /home/user/Downloads/StructureSDK-CrossPlatform-0.8.1.
scripts/prepare_sdk.py ${YOUR_SDK_LOCATION}

# Build and prepare.
bazel build :structure_core.so
mv bazel-bin/structure_core.so .

# Run Example. (View and Recoder)
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:structure_core/Linux/x86_64/ python structure_core_recorder.py  --output-dir structurecore

```

Feel free to post issue.

## Developer
* [xiong-jie](https://twitter.com/_xiongjie_)

## License
* MIT License