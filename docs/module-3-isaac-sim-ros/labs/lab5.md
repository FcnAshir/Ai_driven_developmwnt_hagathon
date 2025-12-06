# Lab 5: Generate Synthetic Images for Training

## Objective
Create synthetic datasets for training AI models using Isaac Sim.

## Prerequisites
- Completed Lab 1 (Isaac Sim installation)
- Isaac Sim running with appropriate scenes
- Basic understanding of synthetic data generation
- Python and image processing libraries

## Steps

### 1. Create a synthetic data generation package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python synthetic_data_gen
cd synthetic_data_gen
mkdir scripts datasets config
```

### 2. Install required dependencies
```bash
pip3 install opencv-python numpy pillow matplotlib
```

### 3. Create synthetic data generator node
Create `synthetic_data_gen/synthetic_data_generator.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import json
import random
from datetime import datetime
import message_filters
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class SyntheticDataGenerator(Node):
    def __init__(self):
        super().__init__('synthetic_data_generator')

        # CV Bridge for image processing
        self.bridge = CvBridge()

        # QoS profile for camera data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscribers for Isaac Sim camera data
        self.image_sub = message_filters.Subscriber(
            self, Image, '/isaac_sim_camera/image', qos_profile=qos_profile
        )
        self.camera_info_sub = message_filters.Subscriber(
            self, CameraInfo, '/isaac_sim_camera/camera_info', qos_profile=qos_profile
        )

        # Synchronize image and camera info
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.camera_info_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self.camera_callback)

        # Publishers for annotations and status
        self.annotation_pub = self.create_publisher(
            String, 'synthetic_data/annotations', 10
        )
        self.status_pub = self.create_publisher(
            String, 'synthetic_data/status', 10
        )

        # Data generation parameters
        self.dataset_name = 'isaac_synthetic_dataset'
        self.output_dir = os.path.expanduser(f'~/synthetic_datasets/{self.dataset_name}')
        self.images_dir = os.path.join(self.output_dir, 'images')
        self.annotations_dir = os.path.join(self.output_dir, 'annotations')
        self.metadata_file = os.path.join(self.output_dir, 'metadata.json')

        # Create directories
        os.makedirs(self.images_dir, exist_ok=True)
        os.makedirs(self.annotations_dir, exist_ok=True)

        # Generation parameters
        self.generation_rate = 5.0  # Hz
        self.timer = self.create_timer(1.0/self.generation_rate, self.generate_data)
        self.sample_counter = 0
        self.max_samples = 1000  # Limit for this demo

        # Domain randomization parameters
        self.randomization_params = {
            'lighting': True,
            'textures': True,
            'camera_angles': True,
            'backgrounds': True
        }

        # Statistics
        self.stats = {
            'total_samples': 0,
            'last_generated': None,
            'generation_rate_actual': 0.0
        }

        self.get_logger().info(f'Synthetic Data Generator Node Started - Output: {self.output_dir}')

    def camera_callback(self, image_msg, camera_info_msg):
        """Receive camera data from Isaac Sim."""
        try:
            # Convert ROS Image to OpenCV
            self.current_cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            self.current_camera_info = camera_info_msg
        except Exception as e:
            self.get_logger().error(f'Error processing camera data: {str(e)}')

    def generate_data(self):
        """Generate synthetic data samples."""
        if not hasattr(self, 'current_cv_image'):
            return

        if self.sample_counter >= self.max_samples:
            self.get_logger().info('Reached maximum sample count. Stopping generation.')
            return

        # Apply domain randomization if enabled
        processed_image = self.apply_domain_randomization(self.current_cv_image)

        # Generate annotations (simulated for this example)
        annotations = self.generate_annotations(processed_image)

        # Save image and annotations
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        image_filename = f"synthetic_{timestamp}_{self.sample_counter:06d}.png"
        annotation_filename = f"annotations_{timestamp}_{self.sample_counter:06d}.json"

        # Save image
        image_path = os.path.join(self.images_dir, image_filename)
        cv2.imwrite(image_path, processed_image)

        # Save annotations
        annotation_path = os.path.join(self.annotations_dir, annotation_filename)
        with open(annotation_path, 'w') as f:
            json.dump(annotations, f, indent=2)

        # Update statistics
        self.sample_counter += 1
        self.stats['total_samples'] += 1
        self.stats['last_generated'] = timestamp

        # Publish status
        status_msg = String()
        status_msg.data = json.dumps({
            'sample_count': self.sample_counter,
            'total_samples': self.stats['total_samples'],
            'last_image': image_filename,
            'timestamp': timestamp
        })
        self.status_pub.publish(status_msg)

        # Log progress periodically
        if self.sample_counter % 50 == 0:  # Every 50 samples
            self.get_logger().info(
                f'Synthetic data generation: {self.sample_counter}/{self.max_samples} samples created'
            )

    def apply_domain_randomization(self, image):
        """Apply domain randomization techniques to the image."""
        processed_image = image.copy()

        # Randomize lighting (brightness and contrast)
        if self.randomization_params['lighting']:
            brightness_factor = random.uniform(0.7, 1.3)
            contrast_factor = random.uniform(0.8, 1.2)

            processed_image = cv2.convertScaleAbs(
                processed_image,
                alpha=contrast_factor,
                beta=(brightness_factor - 1) * 128
            )

        # Add random noise
        noise = np.random.normal(0, random.uniform(0, 10), processed_image.shape).astype(np.uint8)
        processed_image = cv2.add(processed_image, noise)

        # Random blur
        if random.random() < 0.3:  # 30% chance of blur
            kernel_size = random.choice([1, 3, 5])
            processed_image = cv2.GaussianBlur(processed_image, (kernel_size, kernel_size), 0)

        return processed_image

    def generate_annotations(self, image):
        """Generate synthetic annotations for the image (simulated)."""
        height, width = image.shape[:2]

        # For this example, we'll simulate object detection annotations
        # In a real scenario, Isaac Sim would provide ground truth annotations
        annotations = {
            'image_width': width,
            'image_height': height,
            'image_filename': f'synthetic_{self.sample_counter:06d}.png',
            'timestamp': datetime.now().isoformat(),
            'objects': []
        }

        # Simulate random objects in the scene
        num_objects = random.randint(1, 5)
        for i in range(num_objects):
            # Random object properties
            obj_class = random.choice(['robot', 'box', 'cylinder', 'sphere', 'person'])
            x_center = random.randint(50, width - 50)
            y_center = random.randint(50, height - 50)
            width_obj = random.randint(20, 100)
            height_obj = random.randint(20, 100)

            # Ensure bounding box stays within image bounds
            x_min = max(0, x_center - width_obj // 2)
            y_min = max(0, y_center - height_obj // 2)
            x_max = min(width, x_center + width_obj // 2)
            y_max = min(height, y_center + height_obj // 2)

            obj_annotation = {
                'class': obj_class,
                'bbox': [int(x_min), int(y_min), int(x_max), int(y_max)],
                'confidence': 1.0,  # Perfect ground truth
                'occluded': random.random() < 0.2,  # 20% chance of occlusion
                'truncated': random.random() < 0.1   # 10% chance of truncation
            }

            annotations['objects'].append(obj_annotation)

        return annotations

    def create_metadata(self):
        """Create metadata file for the dataset."""
        metadata = {
            'dataset_name': self.dataset_name,
            'creation_date': datetime.now().isoformat(),
            'total_samples': self.stats['total_samples'],
            'image_format': 'png',
            'image_size': [640, 480],  # Adjust based on your camera
            'annotation_format': 'coco',
            'domain_randomization': self.randomization_params,
            'generator_config': {
                'generation_rate': self.generation_rate,
                'max_samples': self.max_samples
            }
        }

        with open(self.metadata_file, 'w') as f:
            json.dump(metadata, f, indent=2)

    def on_shutdown(self):
        """Cleanup function called on shutdown."""
        self.create_metadata()
        self.get_logger().info(
            f'Synthetic data generation completed. Total samples: {self.stats["total_samples"]}'
        )


def main(args=None):
    rclpy.init(args=args)
    synthetic_data_generator = SyntheticDataGenerator()

    try:
        rclpy.spin(synthetic_data_generator)
    except KeyboardInterrupt:
        pass
    finally:
        synthetic_data_generator.on_shutdown()
        synthetic_data_generator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 4. Create a data augmentation script
Create `scripts/data_augmentation.py`:

```python
#!/usr/bin/env python3
"""
Data augmentation script for synthetic datasets.
This script performs additional augmentation on generated synthetic data.
"""

import cv2
import numpy as np
import os
import json
import random
import argparse
from pathlib import Path


class SyntheticDataAugmenter:
    def __init__(self, dataset_path):
        self.dataset_path = Path(dataset_path)
        self.images_dir = self.dataset_path / 'images'
        self.annotations_dir = self.dataset_path / 'annotations'
        self.augmented_dir = self.dataset_path / 'augmented'

        # Create augmented directory
        self.augmented_dir.mkdir(exist_ok=True)
        (self.augmented_dir / 'images').mkdir(exist_ok=True)
        (self.augmented_dir / 'annotations').mkdir(exist_ok=True)

    def augment_image(self, image):
        """Apply augmentation techniques to an image."""
        augmented_image = image.copy()

        # Random rotation
        if random.random() < 0.5:
            angle = random.uniform(-15, 15)
            height, width = image.shape[:2]
            center = (width // 2, height // 2)
            rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
            augmented_image = cv2.warpAffine(augmented_image, rotation_matrix, (width, height))

        # Random scaling
        if random.random() < 0.3:
            scale_factor = random.uniform(0.8, 1.2)
            height, width = image.shape[:2]
            new_width = int(width * scale_factor)
            new_height = int(height * scale_factor)
            augmented_image = cv2.resize(augmented_image, (new_width, new_height))
            # Resize back to original size
            augmented_image = cv2.resize(augmented_image, (width, height))

        # Random brightness and contrast
        if random.random() < 0.7:
            brightness_factor = random.uniform(0.8, 1.2)
            contrast_factor = random.uniform(0.8, 1.2)
            augmented_image = cv2.convertScaleAbs(
                augmented_image,
                alpha=contrast_factor,
                beta=(brightness_factor - 1) * 128
            )

        # Random noise
        if random.random() < 0.3:
            noise = np.random.normal(0, random.uniform(0, 15), augmented_image.shape).astype(np.uint8)
            augmented_image = cv2.add(augmented_image, noise)

        # Random blur
        if random.random() < 0.2:
            kernel_size = random.choice([1, 3, 5])
            augmented_image = cv2.GaussianBlur(augmented_image, (kernel_size, kernel_size), 0)

        return augmented_image

    def augment_annotations(self, annotations, original_image_shape, augmented_image_shape):
        """Augment annotations to match the augmented image."""
        # For this example, we'll just return the original annotations
        # In a real implementation, you would transform bounding boxes, etc.
        # based on the augmentation applied
        augmented_annotations = annotations.copy()
        augmented_annotations['augmentation_applied'] = True

        return augmented_annotations

    def process_dataset(self, num_augmentations_per_image=2):
        """Process the entire dataset and create augmented versions."""
        image_files = list(self.images_dir.glob('*.png'))
        annotation_files = list(self.annotations_dir.glob('*.json'))

        print(f"Found {len(image_files)} images and {len(annotation_files)} annotations")

        for i, img_file in enumerate(image_files):
            print(f"Processing image {i+1}/{len(image_files)}: {img_file.name}")

            # Load image
            image = cv2.imread(str(img_file))
            if image is None:
                continue

            original_shape = image.shape

            # Find corresponding annotation
            annotation_file = self.annotations_dir / f"annotations_{img_file.stem.split('_', 1)[1]}.json"
            if not annotation_file.exists():
                continue

            with open(annotation_file, 'r') as f:
                annotations = json.load(f)

            # Create multiple augmented versions
            for aug_idx in range(num_augmentations_per_image):
                augmented_image = self.augment_image(image)
                augmented_annotations = self.augment_annotations(
                    annotations, original_shape, augmented_image.shape
                )

                # Save augmented image
                aug_img_name = f"{img_file.stem}_aug_{aug_idx:03d}.png"
                aug_img_path = self.augmented_dir / 'images' / aug_img_name
                cv2.imwrite(str(aug_img_path), augmented_image)

                # Save augmented annotations
                aug_ann_name = f"annotations_{img_file.stem.split('_', 1)[1]}_aug_{aug_idx:03d}.json"
                aug_ann_path = self.augmented_dir / 'annotations' / aug_ann_name
                with open(aug_ann_path, 'w') as f:
                    json.dump(augmented_annotations, f, indent=2)

        print(f"Augmentation completed. Augmented data saved to {self.augmented_dir}")


def main():
    parser = argparse.ArgumentParser(description='Synthetic Data Augmentation Tool')
    parser.add_argument('--dataset-path', type=str, required=True,
                        help='Path to the synthetic dataset directory')
    parser.add_argument('--augmentations-per-image', type=int, default=2,
                        help='Number of augmented versions per original image')

    args = parser.parse_args()

    augmenter = SyntheticDataAugmenter(args.dataset_path)
    augmenter.process_dataset(args.augmentations_per_image)


if __name__ == '__main__':
    main()
```

### 5. Create a dataset analyzer script
Create `scripts/dataset_analyzer.py`:

```python
#!/usr/bin/env python3
"""
Dataset analyzer for synthetic data.
This script analyzes the generated synthetic dataset.
"""

import json
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from collections import Counter


class DatasetAnalyzer:
    def __init__(self, dataset_path):
        self.dataset_path = Path(dataset_path)
        self.images_dir = self.dataset_path / 'images'
        self.annotations_dir = self.dataset_path / 'annotations'

    def analyze_dataset(self):
        """Analyze the synthetic dataset."""
        print("Analyzing synthetic dataset...")

        # Count files
        image_files = list(self.images_dir.glob('*.png'))
        annotation_files = list(self.annotations_dir.glob('*.json'))

        print(f"Total images: {len(image_files)}")
        print(f"Total annotations: {len(annotation_files)}")

        # Analyze image properties
        image_sizes = []
        for img_file in image_files[:100]:  # Sample first 100 images
            img = cv2.imread(str(img_file))
            if img is not None:
                image_sizes.append(img.shape)

        if image_sizes:
            sizes_counter = Counter([str(size) for size in image_sizes])
            print(f"Image sizes: {dict(sizes_counter)}")

        # Analyze annotations
        all_objects = []
        object_classes = []
        bbox_sizes = []

        for ann_file in annotation_files[:100]:  # Sample first 100 annotations
            with open(ann_file, 'r') as f:
                ann_data = json.load(f)

            for obj in ann_data.get('objects', []):
                all_objects.append(obj)
                object_classes.append(obj['class'])
                bbox = obj['bbox']
                bbox_size = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])  # width * height
                bbox_sizes.append(bbox_size)

        # Print analysis results
        class_counter = Counter(object_classes)
        print(f"Object classes: {dict(class_counter)}")
        print(f"Total objects: {len(all_objects)}")

        if bbox_sizes:
            print(f"Average bbox size: {np.mean(bbox_sizes):.2f}")
            print(f"Min bbox size: {np.min(bbox_sizes):.2f}")
            print(f"Max bbox size: {np.max(bbox_sizes):.2f}")

        # Create visualization
        self.create_visualization(class_counter, bbox_sizes)

    def create_visualization(self, class_counter, bbox_sizes):
        """Create visualizations for dataset analysis."""
        fig, axes = plt.subplots(1, 2, figsize=(15, 6))

        # Plot 1: Object class distribution
        if class_counter:
            classes = list(class_counter.keys())
            counts = list(class_counter.values())
            axes[0].bar(classes, counts)
            axes[0].set_title('Object Class Distribution')
            axes[0].set_xlabel('Object Class')
            axes[0].set_ylabel('Count')
            axes[0].tick_params(axis='x', rotation=45)

        # Plot 2: Bounding box size distribution
        if bbox_sizes:
            axes[1].hist(bbox_sizes, bins=50, edgecolor='black')
            axes[1].set_title('Bounding Box Size Distribution')
            axes[1].set_xlabel('Box Area (pixels)')
            axes[1].set_ylabel('Frequency')

        plt.tight_layout()
        plt.savefig(self.dataset_path / 'dataset_analysis.png', dpi=300, bbox_inches='tight')
        print(f"Analysis visualization saved to {self.dataset_path / 'dataset_analysis.png'}")


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Synthetic Dataset Analyzer')
    parser.add_argument('--dataset-path', type=str, required=True,
                        help='Path to the synthetic dataset directory')

    args = parser.parse_args()

    analyzer = DatasetAnalyzer(args.dataset_path)
    analyzer.analyze_dataset()


if __name__ == '__main__':
    main()
```

### 6. Create a launch file for synthetic data generation
Create `launch/synthetic_data_generation.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Launch arguments
    dataset_name_arg = DeclareLaunchArgument(
        'dataset_name',
        default_value='isaac_synthetic_dataset',
        description='Name of the synthetic dataset'
    )

    generation_rate_arg = DeclareLaunchArgument(
        'generation_rate',
        default_value='5',
        description='Rate of synthetic data generation (Hz)'
    )

    # Synthetic Data Generator Node
    synthetic_data_generator = Node(
        package='synthetic_data_gen',
        executable='synthetic_data_generator',
        name='synthetic_data_generator',
        output='screen',
        parameters=[
            {'dataset_name': LaunchConfiguration('dataset_name')},
            {'generation_rate': LaunchConfiguration('generation_rate')}
        ]
    )

    # RViz2 for visualization (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='synthetic_data_rviz',
        arguments=['-d', os.path.expanduser('~/.rviz/synthetic_data.rviz')],
        output='screen'
    )

    return LaunchDescription([
        dataset_name_arg,
        generation_rate_arg,
        synthetic_data_generator,
        # rviz_node  # Uncomment if you have a specific RViz config
    ])
```

### 7. Create a comprehensive dataset generation script
Create `scripts/batch_generator.py`:

```python
#!/usr/bin/env python3
"""
Batch synthetic data generation script.
This script coordinates the generation of synthetic datasets with various scenarios.
"""

import os
import subprocess
import time
import json
from pathlib import Path


class BatchSyntheticGenerator:
    def __init__(self, output_base_dir="~/synthetic_datasets"):
        self.output_base_dir = Path(output_base_dir).expanduser()
        self.output_base_dir.mkdir(exist_ok=True)

    def generate_scenario_dataset(self, scenario_name, duration_minutes=10):
        """Generate a dataset for a specific scenario."""
        print(f"Generating dataset for scenario: {scenario_name}")

        # Create scenario-specific directory
        scenario_dir = self.output_base_dir / scenario_name
        scenario_dir.mkdir(exist_ok=True)

        # Define scenario parameters
        scenario_params = {
            "lighting_conditions": ["bright", "dim", "overcast"],
            "object_types": ["indoor", "outdoor", "cluttered"],
            "camera_angles": ["front", "side", "top"],
            "backgrounds": ["simple", "complex", "textured"]
        }

        # For this example, we'll simulate the generation process
        # In a real implementation, you would control Isaac Sim to generate the data
        print(f"Setting up Isaac Sim for {scenario_name} scenario...")

        # Simulate data generation
        for i in range(duration_minutes * 60 // 10):  # Generate data for specified duration
            print(f"Generating batch {i+1} for {scenario_name}...")
            time.sleep(2)  # Simulate generation time

        print(f"Completed {scenario_name} dataset generation")

        # Create metadata for this scenario
        metadata = {
            "scenario_name": scenario_name,
            "generation_date": time.strftime("%Y-%m-%d %H:%M:%S"),
            "duration_minutes": duration_minutes,
            "parameters": scenario_params,
            "status": "completed"
        }

        metadata_file = scenario_dir / "metadata.json"
        with open(metadata_file, 'w') as f:
            json.dump(metadata, f, indent=2)

    def generate_multiple_scenarios(self):
        """Generate datasets for multiple scenarios."""
        scenarios = [
            "indoor_navigation",
            "object_detection",
            "grasping_scenarios",
            "dynamic_environment"
        ]

        for scenario in scenarios:
            self.generate_scenario_dataset(scenario, duration_minutes=5)
            print(f"Completed {scenario}, waiting before next scenario...")
            time.sleep(5)  # Wait between scenarios

    def run_data_augmentation(self, dataset_name):
        """Run data augmentation on a generated dataset."""
        dataset_path = self.output_base_dir / dataset_name
        if not dataset_path.exists():
            print(f"Dataset {dataset_name} does not exist!")
            return

        print(f"Running augmentation on {dataset_name}")
        cmd = [
            "python3", "scripts/data_augmentation.py",
            "--dataset-path", str(dataset_path),
            "--augmentations-per-image", "2"
        ]

        try:
            result = subprocess.run(cmd, check=True, capture_output=True, text=True)
            print(f"Augmentation completed for {dataset_name}")
            print(result.stdout)
        except subprocess.CalledProcessError as e:
            print(f"Augmentation failed for {dataset_name}: {e}")
            print(e.stderr)

    def run_dataset_analysis(self, dataset_name):
        """Run analysis on a generated dataset."""
        dataset_path = self.output_base_dir / dataset_name
        if not dataset_path.exists():
            print(f"Dataset {dataset_name} does not exist!")
            return

        print(f"Running analysis on {dataset_name}")
        cmd = [
            "python3", "scripts/dataset_analyzer.py",
            "--dataset-path", str(dataset_path)
        ]

        try:
            result = subprocess.run(cmd, check=True, capture_output=True, text=True)
            print(f"Analysis completed for {dataset_name}")
            print(result.stdout)
        except subprocess.CalledProcessError as e:
            print(f"Analysis failed for {dataset_name}: {e}")
            print(e.stderr)


def main():
    generator = BatchSyntheticGenerator()

    # Generate multiple scenarios
    generator.generate_multiple_scenarios()

    # Run augmentation and analysis on all datasets
    for dataset_dir in generator.output_base_dir.iterdir():
        if dataset_dir.is_dir():
            dataset_name = dataset_dir.name
            print(f"\nProcessing dataset: {dataset_name}")

            # Run augmentation
            generator.run_data_augmentation(dataset_name)

            # Run analysis
            generator.run_dataset_analysis(dataset_name)

    print(f"\nAll synthetic datasets generated in: {generator.output_base_dir}")


if __name__ == '__main__':
    main()
```

### 8. Update setup.py for the synthetic data package
Edit `setup.py`:

```python
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'synthetic_data_gen'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Synthetic data generation for Isaac Sim',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'synthetic_data_generator = synthetic_data_gen.synthetic_data_generator:main',
        ],
    },
)