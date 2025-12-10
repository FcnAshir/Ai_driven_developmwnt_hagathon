---
sidebar_position: 5
---

# Lab 5: Generate Synthetic Images for Training

## Overview

In this lab, you'll learn how to generate synthetic training data using Isaac Sim's photorealistic rendering capabilities. You'll create diverse datasets with domain randomization techniques for training AI models, particularly for computer vision tasks in robotics.

## Prerequisites

- Lab 1-4 completed (Isaac Sim, perception, SLAM, navigation)
- Isaac Sim with proper rendering setup
- Understanding of computer vision concepts
- Basic knowledge of machine learning datasets

## Learning Objectives

By the end of this lab, you will be able to:
- Set up Isaac Sim for synthetic data generation
- Implement domain randomization techniques
- Generate diverse training datasets
- Create proper annotations for synthetic data
- Validate synthetic data quality for training

## Background

Synthetic data generation is a critical capability in modern AI development, especially for robotics applications. Isaac Sim provides:

- **Photorealistic rendering**: High-quality images that match real-world conditions
- **Domain randomization**: Variation in appearance, lighting, and physics
- **Automatic annotation**: Ground truth data generation
- **Scalability**: Generate large datasets efficiently
- **Safety**: Train models without physical robot deployment

The key benefits include:
- Reduced need for expensive real-world data collection
- Ability to generate edge cases and rare scenarios
- Consistent ground truth annotations
- Safe environment for data collection

## Setting Up Isaac Sim for Data Generation

### Step 1: Install Required Extensions

First, ensure the necessary extensions for synthetic data generation are enabled:

```bash
# Launch Isaac Sim
./omniverse_launcher.AppImage

# Enable these extensions in Isaac Sim:
# - Isaac Sim Synthetic Data
# - Isaac Sim Sensors
# - Isaac Sim Perception
# - Isaac Sim Dataset Capture
```

### Step 2: Create a Data Generation Scene

Set up a scene optimized for synthetic data generation:

1. Create a new scene in Isaac Sim
2. Add a camera system (RGB, depth, semantic segmentation)
3. Set up lighting with multiple configurations
4. Add objects with varying textures and materials
5. Configure the scene with physics-enabled objects

### Step 3: Configure Synthetic Data Extension

In Isaac Sim, configure the synthetic data extension:

1. Go to Window → Extensions → Isaac → Synthetic Data
2. Enable the Synthetic Data extension
3. Configure the following capture settings:
   - RGB images
   - Depth maps
   - Semantic segmentation
   - Instance segmentation
   - Bounding boxes
   - 6D pose data

## Domain Randomization Setup

### Step 4: Implement Visual Domain Randomization

Create variation in visual properties to improve model generalization:

```python
# Example Python script for domain randomization in Isaac Sim
import omni
import carb
from pxr import Gf
import random

def setup_domain_randomization():
    # Get the synthetic data interface
    synth_data = omni.synthetic.graphics.SyntheticData()

    # Randomize lighting
    light_prim = omni.usd.get_context().get_stage().GetPrimAtPath("/World/Light")
    if light_prim.IsValid():
        # Randomize light intensity and color
        intensity = random.uniform(500, 2000)
        color = Gf.Vec3f(random.uniform(0.8, 1.2), random.uniform(0.8, 1.2), random.uniform(0.8, 1.2))

        light_prim.GetAttribute("intensity").Set(intensity)
        light_prim.GetAttribute("color").Set(color)

    # Randomize object textures
    for prim_path in ["/World/Object1", "/World/Object2"]:  # Replace with actual paths
        prim = omni.usd.get_context().get_stage().GetPrimAtPath(prim_path)
        if prim.IsValid():
            # Randomize material properties
            roughness = random.uniform(0.1, 0.9)
            metallic = random.uniform(0.0, 0.5)

            # Apply random textures
            texture_files = ["texture1.jpg", "texture2.jpg", "texture3.jpg"]  # Replace with actual paths
            random_texture = random.choice(texture_files)

            # Set material properties
            # This would involve setting up USD materials with random properties

setup_domain_randomization()
```

### Step 5: Physical Domain Randomization

Randomize physical properties to improve sim-to-real transfer:

1. **Friction**: Randomize surface friction coefficients
2. **Mass**: Vary object masses within realistic ranges
3. **Inertia**: Adjust object inertial properties
4. **Dynamics**: Randomize joint dynamics and constraints

### Step 6: Camera Configuration

Set up cameras for different types of data capture:

1. **RGB Camera**: For color images
2. **Depth Camera**: For depth maps
3. **Semantic Camera**: For segmentation masks
4. **Stereo Camera**: For disparity maps

Configure camera properties:
- Resolution (e.g., 640x480, 1280x720)
- Field of view
- Sensor noise models
- Distortion parameters

## Synthetic Data Generation Pipeline

### Step 7: Create Data Capture Script

Create a Python script to automate data capture:

```python
# data_capture_script.py
import omni
import carb
import numpy as np
import cv2
import json
import os
from PIL import Image
import omni.synthetic.graphics as ogn

class SyntheticDataCapture:
    def __init__(self, output_dir="synthetic_data"):
        self.output_dir = output_dir
        self.frame_counter = 0

        # Create output directories
        os.makedirs(f"{output_dir}/rgb", exist_ok=True)
        os.makedirs(f"{output_dir}/depth", exist_ok=True)
        os.makedirs(f"{output_dir}/seg", exist_ok=True)
        os.makedirs(f"{output_dir}/annotations", exist_ok=True)

    def capture_frame(self):
        """Capture a single frame with all required data"""
        # Get current frame data
        rgb_data = self.get_rgb_image()
        depth_data = self.get_depth_image()
        seg_data = self.get_segmentation()
        pose_data = self.get_object_poses()

        # Save images
        frame_id = f"{self.frame_counter:06d}"

        # Save RGB image
        rgb_image = Image.fromarray(rgb_data)
        rgb_image.save(f"{self.output_dir}/rgb/{frame_id}.png")

        # Save depth image
        depth_image = Image.fromarray((depth_data * 255).astype(np.uint8))
        depth_image.save(f"{self.output_dir}/depth/{frame_id}.png")

        # Save segmentation
        seg_image = Image.fromarray(seg_data.astype(np.uint8))
        seg_image.save(f"{self.output_dir}/seg/{frame_id}.png")

        # Save annotations
        annotations = {
            "frame_id": frame_id,
            "rgb_path": f"rgb/{frame_id}.png",
            "depth_path": f"depth/{frame_id}.png",
            "seg_path": f"seg/{frame_id}.png",
            "poses": pose_data,
            "timestamp": carb.timeline.get_timeline().get_current_time(),
        }

        with open(f"{self.output_dir}/annotations/{frame_id}.json", "w") as f:
            json.dump(annotations, f, indent=2)

        self.frame_counter += 1

    def get_rgb_image(self):
        """Get RGB image from camera"""
        # Implementation depends on Isaac Sim API
        # This is a placeholder
        pass

    def get_depth_image(self):
        """Get depth image from camera"""
        # Implementation depends on Isaac Sim API
        # This is a placeholder
        pass

    def get_segmentation(self):
        """Get segmentation mask"""
        # Implementation depends on Isaac Sim API
        # This is a placeholder
        pass

    def get_object_poses(self):
        """Get object poses in the scene"""
        # Implementation depends on Isaac Sim API
        # This is a placeholder
        pass

    def run_capture_sequence(self, num_frames=1000):
        """Run a sequence of captures"""
        for i in range(num_frames):
            # Randomize scene
            self.randomize_scene()

            # Wait for physics to settle
            omni.kit.ticks_per_second = 60  # Set physics update rate

            # Capture frame
            self.capture_frame()

            print(f"Captured frame {i+1}/{num_frames}")

    def randomize_scene(self):
        """Randomize scene properties"""
        # Randomize object positions
        # Randomize lighting
        # Randomize textures
        # Randomize camera position (optional)
        pass

# Usage
if __name__ == "__main__":
    capturer = SyntheticDataCapture("isaac_sim_dataset")
    capturer.run_capture_sequence(num_frames=100)  # Start with a small number for testing
```

### Step 8: Configure Data Formats

Set up different data formats based on your training needs:

1. **Object Detection**: Bounding boxes with class labels
2. **Semantic Segmentation**: Pixel-level class annotations
3. **Instance Segmentation**: Individual object masks
4. **Pose Estimation**: 6D pose of objects
5. **Depth Estimation**: Ground truth depth maps

### Step 9: Implement Annotation Generation

Configure Isaac Sim to generate proper annotations:

```python
# Example annotation format for object detection
def generate_coco_annotations(scene_data):
    """Generate COCO format annotations for object detection"""
    coco_format = {
        "info": {
            "year": 2024,
            "version": "1.0",
            "description": "Synthetic dataset generated with Isaac Sim",
            "contributor": "Isaac Sim User",
            "url": "",
            "date_created": "2024-12-05"
        },
        "licenses": [
            {
                "id": 1,
                "name": "Synthetic Dataset License",
                "url": ""
            }
        ],
        "images": [],
        "annotations": [],
        "categories": []
    }

    # Add categories (object classes)
    categories = [
        {"id": 1, "name": "object1", "supercategory": "object"},
        {"id": 2, "name": "object2", "supercategory": "object"},
        # Add more categories as needed
    ]
    coco_format["categories"] = categories

    # Process each captured frame
    for frame_data in scene_data:
        # Add image info
        image_info = {
            "id": frame_data["frame_id"],
            "width": frame_data["width"],
            "height": frame_data["height"],
            "file_name": f"rgb/{frame_data['frame_id']}.png",
            "license": 1,
            "flickr_url": "",
            "coco_url": "",
            "date_captured": "2024-12-05"
        }
        coco_format["images"].append(image_info)

        # Add annotations for each object in the frame
        for obj in frame_data["objects"]:
            bbox = obj["bbox"]  # [x, y, width, height]
            segmentation = obj["segmentation"]
            area = obj["area"]
            iscrowd = 0  # 0 for regular objects, 1 for crowds

            annotation = {
                "id": len(coco_format["annotations"]) + 1,
                "image_id": frame_data["frame_id"],
                "category_id": obj["category_id"],
                "bbox": bbox,
                "area": area,
                "segmentation": segmentation,
                "iscrowd": iscrowd
            }
            coco_format["annotations"].append(annotation)

    return coco_format
```

## Quality Assurance and Validation

### Step 10: Validate Synthetic Data Quality

Implement validation checks to ensure data quality:

1. **Visual Inspection**: Randomly sample and review generated images
2. **Statistical Analysis**: Compare synthetic vs. real data distributions
3. **Annotation Accuracy**: Verify ground truth annotations are correct
4. **Consistency Checks**: Ensure data follows expected patterns

### Step 11: Implement Quality Metrics

```python
def validate_synthetic_data(dataset_path):
    """Validate synthetic dataset quality"""
    import os
    import cv2
    import numpy as np

    rgb_dir = os.path.join(dataset_path, "rgb")
    depth_dir = os.path.join(dataset_path, "depth")
    seg_dir = os.path.join(dataset_path, "seg")

    # Check if directories exist
    if not all(os.path.exists(d) for d in [rgb_dir, depth_dir, seg_dir]):
        raise ValueError("Dataset directories not found")

    # Count files
    rgb_files = [f for f in os.listdir(rgb_dir) if f.endswith('.png')]
    depth_files = [f for f in os.listdir(depth_dir) if f.endswith('.png')]
    seg_files = [f for f in os.listdir(seg_dir) if f.endswith('.png')]

    print(f"RGB images: {len(rgb_files)}")
    print(f"Depth images: {len(depth_files)}")
    print(f"Segmentation images: {len(seg_files)}")

    # Validate file pairing
    base_names_rgb = {f.split('.')[0] for f in rgb_files}
    base_names_depth = {f.split('.')[0] for f in depth_files}
    base_names_seg = {f.split('.')[0] for f in seg_files}

    missing_depth = base_names_rgb - base_names_depth
    missing_seg = base_names_rgb - base_names_seg

    if missing_depth:
        print(f"Missing depth images for frames: {missing_depth}")
    if missing_seg:
        print(f"Missing segmentation images for frames: {missing_seg}")

    # Sample a few images for visual inspection
    sample_files = rgb_files[:5]  # Check first 5 files
    for filename in sample_files:
        img_path = os.path.join(rgb_dir, filename)
        img = cv2.imread(img_path)
        if img is None:
            print(f"Error loading image: {img_path}")
        else:
            print(f"Image {filename}: shape={img.shape}, dtype={img.dtype}")

    return len(rgb_files) > 0
```

## Advanced Synthetic Data Techniques

### Step 12: Implement Advanced Domain Randomization

1. **Weather Simulation**: Randomize weather conditions (fog, rain, snow)
2. **Dynamic Objects**: Include moving objects in the scene
3. **Temporal Variation**: Change lighting conditions over time
4. **Sensor Simulation**: Add realistic sensor noise and artifacts

### Step 13: Multi-Sensor Data Generation

Generate data for multiple sensor modalities:

1. **LiDAR**: Simulate LiDAR point clouds
2. **Stereo Vision**: Generate stereo image pairs
3. **Thermal**: Simulate thermal imaging
4. **Event Cameras**: Simulate event-based vision

### Step 14: Physics-Based Data Generation

Include physics interactions in data generation:

1. **Object Interactions**: Objects colliding, sliding, stacking
2. **Deformable Objects**: Cloth, liquids, soft materials
3. **Robot Interactions**: Robot manipulating objects
4. **Dynamic Environments**: Moving platforms, changing scenes

## Integration with Training Pipelines

### Step 15: Dataset Organization

Organize your synthetic dataset in standard formats:

```
synthetic_dataset/
├── train/
│   ├── rgb/
│   ├── depth/
│   ├── seg/
│   └── annotations.json
├── val/
│   ├── rgb/
│   ├── depth/
│   ├── seg/
│   └── annotations.json
├── test/
│   ├── rgb/
│   ├── depth/
│   ├── seg/
│   └── annotations.json
└── dataset_config.yaml
```

### Step 16: Data Pipeline Integration

Create a data loader for your training pipeline:

```python
# synthetic_dataset_loader.py
import torch
from torch.utils.data import Dataset
import os
import json
from PIL import Image
import numpy as np

class IsaacSimDataset(Dataset):
    def __init__(self, root_dir, transform=None):
        self.root_dir = root_dir
        self.transform = transform

        # Load annotations
        with open(os.path.join(root_dir, "annotations.json"), 'r') as f:
            self.annotations = json.load(f)

        self.images = self.annotations["images"]

    def __len__(self):
        return len(self.images)

    def __getitem__(self, idx):
        img_info = self.images[idx]
        img_path = os.path.join(self.root_dir, img_info["file_name"])

        # Load image
        image = Image.open(img_path).convert("RGB")

        # Load additional data as needed
        # depth_path = os.path.join(self.root_dir, img_info["depth_path"])
        # segmentation_path = os.path.join(self.root_dir, img_info["seg_path"])

        if self.transform:
            image = self.transform(image)

        # Return image and annotations
        return {
            "image": image,
            "annotations": self.get_annotations_for_image(img_info["id"])
        }

    def get_annotations_for_image(self, img_id):
        """Get annotations for a specific image"""
        # Filter annotations for this image
        img_annotations = [
            ann for ann in self.annotations["annotations"]
            if ann["image_id"] == img_id
        ]
        return img_annotations
```

## Performance Optimization

### Step 17: Optimize Data Generation

1. **Batch Processing**: Generate multiple frames in parallel
2. **GPU Acceleration**: Use GPU for rendering and processing
3. **Memory Management**: Efficiently manage memory during generation
4. **Storage Optimization**: Compress and optimize storage formats

### Step 18: Quality vs Quantity Trade-offs

Balance data quality and quantity based on your needs:

- **High Quality, Low Quantity**: For critical applications requiring accuracy
- **Low Quality, High Quantity**: For robustness training
- **Medium Quality, High Quantity**: For general purpose models

## Validation and Testing

### Step 19: Train with Synthetic Data

Test your synthetic dataset by training a simple model:

```bash
# Example training command
python train_model.py \
    --train_dataset /path/to/synthetic_dataset/train \
    --val_dataset /path/to/synthetic_dataset/val \
    --model_type detection \
    --epochs 50 \
    --batch_size 32
```

### Step 20: Sim-to-Real Transfer Testing

If possible, test your trained model on real-world data to evaluate sim-to-real transfer:

1. Deploy the model to a physical robot
2. Test on real-world objects/scenarios
3. Compare performance to models trained on real data
4. Analyze where synthetic data training succeeds or fails

## Troubleshooting Common Issues

### Issue 1: Rendering Artifacts
**Symptoms**: Images contain visual artifacts or unrealistic elements
**Solutions**:
- Adjust rendering settings and quality parameters
- Verify material and lighting configurations
- Check for geometry issues in the scene

### Issue 2: Annotation Inconsistencies
**Symptoms**: Ground truth annotations don't match images
**Solutions**:
- Verify camera calibration and parameters
- Check coordinate system alignment
- Validate annotation generation pipeline

### Issue 3: Performance Bottlenecks
**Symptoms**: Slow data generation or system crashes
**Solutions**:
- Reduce scene complexity
- Optimize rendering settings
- Increase system resources (GPU memory, CPU cores)

### Issue 4: Domain Gap
**Symptoms**: Model performs poorly on real data
**Solutions**:
- Improve domain randomization
- Add more realistic sensor noise
- Include real-world variations in simulation

## Summary

In this lab, you've learned to generate synthetic training data using Isaac Sim's photorealistic rendering capabilities. You've implemented domain randomization techniques, set up automated data capture pipelines, and created properly formatted annotations for various computer vision tasks. This synthetic data generation capability is crucial for training robust AI models without requiring extensive real-world data collection.

## Next Steps

In Lab 6, you'll learn how to deploy your trained models and ROS nodes to NVIDIA Jetson platforms for real-world robotic applications, completing the full pipeline from simulation to deployment.