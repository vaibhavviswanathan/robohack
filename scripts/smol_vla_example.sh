#!/bin/bash

python lerobot/src/lerobot/scripts/lerobot_train.py \
  --policy.type=smolvla \
  --dataset.repo_id=lerobot/svla_so100_stacking \
  --batch_size=4 \
  --steps=20000 \
  --policy.push_to_hub=false 