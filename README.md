# beginner_tutorials
A repository for ros2 humble for enpm808x

### Author
Manav B Nagda

### Building the ROS package
```bash
# Source ROS
source /opt/ros/humble/setup.bash
# Navigate to workspace
cd ~/ros2_ws/src
git clone https://github.com/mvboiii/beginner_tutorials.git

# Navigate to home directory
cd ..
# Install rosdep dependencies before building the package
rosdep install -i --from-path src --rosdistro humble -y
# Build the package 
colcon build --packages-select beginner_tutorials
# Source the package
. install/setup.bash
# Run the publisher
ros2 run beginner_tutorials talker
# Run the subscriber
ros2 run beginner_tutorials listener 
```

### CppCheck
```bash
# install cppcheck
sudo apt install cppcheck

# Run in the top-level project directory (eg., in cpp-boilerplate-v2/)
cppcheck --enable=all --std=c++11 -I include/ --suppress=missingInclude $( find . -name *.cpp | grep -vE -e "^./build/" )
```

### CppLint
```bash
# install cpplint:
sudo apt install cpplint

# run in the top-level project directory (eg., in cpp-boilerplate-v2/)
cpplint --filter="-legal/copyright" $( find . -name *.cpp | grep -vE -e "^./build/" )
```

### Results
```bash
Listner Node
[INFO] [1699413611.500347047] [Subscriber]: Yaatri gan krupiya dhayaan de : Pudhil Station Dadar 0'
[INFO] [1699413611.999889268] [Subscriber]: Yaatri gan krupiya dhayaan de : Pudhil Station Dadar 1'
[INFO] [1699413612.499989349] [Subscriber]: Yaatri gan krupiya dhayaan de : Pudhil Station Dadar 2'
[INFO] [1699413613.000135125] [Subscriber]: Yaatri gan krupiya dhayaan de : Pudhil Station Dadar 3'
[INFO] [1699413613.499686418] [Subscriber]: Yaatri gan krupiya dhayaan de : Pudhil Station Dadar 4'
[INFO] [1699413613.999907558] [Subscriber]: Yaatri gan krupiya dhayaan de : Pudhil Station Dadar 5'

Talker Node
[INFO] [1699413611.498534988] [Publisher]: Publishing: ' Pudhil Station Dadar 0'
[INFO] [1699413611.998484346] [Publisher]: Publishing: ' Pudhil Station Dadar 1'
[INFO] [1699413612.498476537] [Publisher]: Publishing: ' Pudhil Station Dadar 2'
[INFO] [1699413612.998682725] [Publisher]: Publishing: ' Pudhil Station Dadar 3'
[INFO] [1699413613.498309040] [Publisher]: Publishing: ' Pudhil Station Dadar 4'
[INFO] [1699413613.998398051] [Publisher]: Publishing: ' Pudhil Station Dadar 5'

```
