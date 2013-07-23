### Installation
* Create a rosbuild workspace outside of your catkin workspace source directory: `mkdir rosbuild-ws; cd rosbuild-ws`
* Init the workspace with your catkin workspace environment: `rosws init . /path/to/catkin/workspace/devel/`
* Set external repositories: `rosws merge /path/to/catkin/workspace/src/hesitation_study/hesitation_system/setup/leg-detector.yaml`
* Update to get all the files: `rosws update`
* Source environment: `source setup.bash`
* Install dependencies: `sudo apt-get install libgsl0-dev`
* Rosmake it: `rosmake people_detector_node`
* Patch it: Replace `people_detector_node/launch/people_detector.launch` with `catkin-ws/src/hesitation_study/hesitation_system/patch/people_detector.launch`
