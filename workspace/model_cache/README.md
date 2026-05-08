# Model Cache

This directory stores model files used by the Spot Flex perception stack.

The perception code checks this workspace cache first:

```text
/repo/workspace/model_cache
```

It also supports the shared container cache:

```text
/opt/spot_flex_model_cache
```

## Expected Layout

```text
model_cache/
  huggingface/
    hub/
      models--google--owlv2-base-patch16-ensemble/
  sam/
    sam_vit_b_01ec64.pth
  flex/
    push/push_actor.pth
    revolute/revolute_actor.pth
    prismatic/prismatic_actor.pth
```

## OWLv2 / OWL-ViT

The active detector uses the Hugging Face model:

```text
google/owlv2-base-patch16-ensemble
```

Download it into the workspace cache:

```bash
cd /repo/workspace
source /opt/ros/humble/setup.bash

python3 - <<'PY'
from huggingface_hub import snapshot_download

snapshot_download(
    repo_id="google/owlv2-base-patch16-ensemble",
    cache_dir="/repo/workspace/model_cache/huggingface/hub",
)
PY
```

If `huggingface_hub` is not installed:

```bash
pip3 install huggingface_hub
```

The perception code also honors standard Hugging Face cache environment variables:

```bash
export HF_HOME=/repo/workspace/model_cache/huggingface
```

or:

```bash
export TRANSFORMERS_CACHE=/repo/workspace/model_cache/huggingface/hub
```

## Segment Anything

The box grasp detector expects the SAM ViT-B checkpoint:

```text
sam/sam_vit_b_01ec64.pth
```

Download it into the workspace cache:

```bash
mkdir -p /repo/workspace/model_cache/sam
cd /repo/workspace/model_cache/sam

wget -O sam_vit_b_01ec64.pth \
  https://dl.fbaipublicfiles.com/segment_anything/sam_vit_b_01ec64.pth
```

The detector can also be pointed at another model cache root:

```bash
export SPOT_FLEX_MODEL_DIR=/repo/workspace/model_cache
```

## Quick Check

After downloading the models, run the offline perception checks:

```bash
cd /repo/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

PYTHONPATH=/repo/workspace/src/spot_flex_perception:$PYTHONPATH \
  python3 -m spot_flex_perception.test_owl

PYTHONPATH=/repo/workspace/src/spot_flex_perception:$PYTHONPATH \
  python3 -m spot_flex_perception.test_box_grasp
```

`test_owl` checks OWLv2 loading and open-vocabulary detection. `test_box_grasp` checks the OWLv2-to-SAM box grasp pipeline.
