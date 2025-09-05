# robomimic training command example
# Update paths as needed
python3 -m robomimic.scripts.train \
  --config bc_image_joint.yaml \
  --dataset robomimic_dataset.hdf5 \
  --output_dir ./robomimic_bc_output
