#!/bin/bash

python lerobot/src/lerobot/scripts/lerobot_train.py \
  --policy.path=lerobot/smolvla_base \
  --dataset.repo_id=lerobot/svla_so100_stacking \
  --batch_size=64 \
  --steps=20000  # 10% of training budget