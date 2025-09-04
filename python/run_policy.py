# robomimic policy inference example
# Usage: python run_policy.py /path/to/model.pth

import sys
import numpy as np
from robomimic.utils.file_utils import policy_from_checkpoint_path

# Dummy obs for demonstration; replace with real data in deployment
obs = {
    "obs": {
        "joints": np.zeros(6, dtype=np.float32),
        "image": np.zeros((3, 128, 128), dtype=np.uint8),  # shape (3, H, W)
    }
}

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python run_policy.py /path/to/model.pth")
        sys.exit(1)
    policy = policy_from_checkpoint_path(sys.argv[1])
    action = policy(obs)
    print("Action:", action)
