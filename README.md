# ROS-setup installation
The instructions below assume that you have already installed [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html#) and are using Ubuntu 24.04.

## Useful bashrc aliases
The commands below assume that you install the following aliases in your `~/.bashrc` file.

Note that all of the following aliases **must be run from the workspace root**.
* *make_pysim_venv*: Creates a virtual environment, installs ros dependencies, and installs the py_sim package
    ```
    alias make_pysim_venv='python3 -m venv --system-site-packages venv && source venv/bin/activate && touch venv/COLCON_IGNORE && rosdep install --from-paths src --ignore-src -r -y && pip install transforms3d && pip install -e src/py_sim/ && touch src/py_sim/COLCON_IGNORE'
    ```
* *sd*: Used to source the workspace directory correctly when a build is not required. (This is required every time you open a new terminal). `sd` must be run from the workspace root directory
    ```
    alias sd='source /opt/ros/jazzy/setup.bash && source venv/bin/activate && . install/setup.bash'
    ```
* *build*: Used to source the ROS2 root, activate the virtual environment (assumed to be *venv*), build, and then source the install
    ```
    alias build='source /opt/ros/jazzy/setup.bash && source venv/bin/activate && python3 -m colcon build --symlink-install && . install/local_setup.bash'
    ```
## Creating a workspace
You must create a workspace folder that has a `src` folder within. For example, if you created a folder using the command
```bash
mkdir -p ~/5345/project/src
```
the `project` folder would be considered the **workspace root** and you would download all of the code into the resulting `src` folder.

## Downloading the code and setting up a workspace
Each of the required repos can be downloaded using vcs tool.

Install vcs:
    ```bash
    sudo apt install python3-vcstool
    ```

To use vcs, do the following:
* Create a workspace folder with a `src` folder inside as described above
* Navigate to the workspace folder (i.e., the parent folder of the `src` folder)
* Download [course.rosinstall](https://gitlab.com/utahstate/courses/5345-intro-to-ros/py_sim_launch/-/blob/main/assigments/rosinstall/course.rosinstall?ref_type=heads) and place it inside the workspace folder (or create a new `.rosinstall` file and copy over the contents)
* Run the following command
    ```bash
    vcs import --input course.rosinstall src
    ```

## Installing the code
From the workspace root, run the following command to make a virtual environment and install the python code (alias created above)
```bash
make_pysim_venv
```

If the `make_pysim_venv` installs successfully, then run the following command from the workspace root to build the workspace (alias created above)
```bash
build
```

Keep in mind that you will need to run the `sd` command from the workspace root folder each time you open up a terminal.

# Run Instructions
If everything has installed correctly, you should be able to run the start of the first assignment
```bash
ros2 run assignments 03_single
```
