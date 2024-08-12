"""
OSU_HRI_Project

Overview:
---------
Welcome to the OSU_HRI_Project repository! Our project investigates the most socially appropriate ways for robots to interrupt conversations. The aim is to enhance human-robot interactions (HRI) by implementing and extending human non-verbal communication gestures.

Objectives:
-----------
- Social Appropriateness: Identify and implement methods for robots to interrupt conversations in a socially acceptable manner.
- Non-Verbal Communication: Emulate human non-verbal gestures and movements used during interruptions.
- Technology Utilized:
  - Object Detection: Recognize and interpret conversational contexts and interactions.
  - Clustering: Analyze and categorize interrupting behaviors.
  - Movement Implementation: Enable the robot to perform socially appropriate gestures.

Project Structure:
------------------
- /src: Source code for core functionalities including object detection, clustering algorithms, and robot movement control.
- /launch: Launch files for starting and managing different components of the project.
- /scripts: Utility scripts for data processing, model training, and evaluation.
- /config: Configuration files for system settings, including parameters and environment-specific settings.
- /urdf: URDF (Unified Robot Description Format) files for defining the robot's physical properties and kinematics.

Getting Started:
----------------
Prerequisites:
--------------
Ensure you have the following installed:
- Python 3.x
- ROS (Robot Operating System) if working with launch files
- Required Python libraries: numpy, scipy, scikit-learn, opencv-python, tensorflow (or other frameworks as necessary)
- Any additional dependencies specified in requirements.txt

Installation:
-------------
1. Clone the Repository:
   git clone https://github.com/yourusername/OSU_HRI_Project.git

2. Navigate to the Project Directory:
   cd OSU_HRI_Project

3. Install Python Dependencies:
   pip install -r requirements.txt

Usage:
------
1. Data Preparation:
   - Place your data files in the appropriate directories as specified in the configuration files.
   - Adjust configuration settings in the /config directory to match your environment.

2. Launching the System:
   Use the ROS launch files in the /launch directory to start different components of the project:
   roslaunch your_launch_file.launch

3. Running Core Functionalities:
   - Object Detection:
     python src/object_detection.py

   - Clustering Analysis:
     python src/clustering.py

   - Robot Movements:
     python src/robot_movement.py

Configuration:
--------------
Configuration files in /config need to be adjusted to fit your specific setup. Detailed instructions for modifying these files can be found in config/README.md.

URDF Files:
-----------
URDF files in the /urdf directory describe the robot's structure and are used for simulation and control purposes. Modify these files if you need to adjust the robot model or its parameters.

Documentation:
--------------
For detailed information on methodologies, implementation details, and experimental results, refer to the /docs directory.

Contributing:
-------------
Contributions are welcome! Please see CONTRIBUTING.md for guidelines on how to contribute to the project.

License:
--------
This project is licensed under the MIT License. See LICENSE for details.

Contact:
--------
For questions or feedback, please contact:
- Project Lead: [Your Name](mailto:your.email@example.com)
- Repository: https://github.com/yourusername/OSU_HRI_Project

Thank you for your interest in the OSU_HRI_Project! We look forward to your contributions and feedback.
"""
