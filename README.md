# PART 1 : Software Documentation
The first part of this assignment is to create the documentation for the [ROS node created in the third assignment](https://github.com/IlMusu/Research_Track/tree/assignment_3/scripts).</br>
Given the fact that "robot_ui" was written in Python, the software used to create the documentation is Sphinx.

### Installing
To install Sphinx, run the following command:

```bash
sudo apt-get install python3-sphinx
```

### Running
The steps that have been followed to create the documentation are:
1. Move to the main directory of the "robot_ui" node.
2. Create a directory called ["/sphinx"](https://github.com/IlMusu/Research_Track/tree/assignment_3/sphinx/) and inside of it run the command:<br>
NB. This will start an interactive prompt that will ask the user fo the required informations.

```bash
sphinx-quistart
```

3. Modify the ["conf.py"](https://github.com/IlMusu/Research_Track/blob/assignment_3/sphinx/source/conf.py) file just created by adding the correct path so that Sphinx is able to find the source folder.<br>
(OPTIONAL) Some extensions may be added to customize Sphinx.

4. Modify the ["index.rst"](https://github.com/IlMusu/Research_Track/blob/assignment_3/sphinx/source/index.rst) file by adding the scripts for which we want to provide the documentation.

5. Properly comment the [robot_ui.py](https://github.com/IlMusu/Research_Track/blob/assignment_3/scripts/robot_ui.py) node.
<br>
Finally, it is possible to create the documentation by running the following command:<br>
NB. This will create the documentation inside the "build" folder to be viwed as a browser page.

```bash
make html
```

The documentation is available [here](https://ilmusu.github.io/Research_Track/).

# PART 2 : Jupyter Notebook
The second part of this assignment is to use the Jupyter Notebook functionalities to create a graphical interface for the simulation used in the third assignment.<br>
NB. This interface should have the exaclty same funcionalities of the robot_ui node implemented in third assignment.<br>

# Installing and running
To install Jupyter Notebook, use the following commands:
```bash
pip3 install jupyter bqplot pyyaml ipywidgets
jupyter nbextension enable --py widgetsnbextension
```

Some extensions are needed, use the following commands:
```bash
pip3 install jupyter_contrib_nbextensions
pip3 install jupyter_nbextensions_configurator
jupyter contrib nbextension install
```

Now, it is possible to use the interface to control the robot in the simulation.<br>
First, download this package [package](https://github.com/IlMusu/Research_Track/tree/assignment_3/).<br>
Then, it is possible to run the simulation using the following commands:
```bash
roslaunch final_assignment simulation_gmapping.launch
roslaunch final_assignment move_base.launch
```

Now, it is possible to open Jupyter Notebook to visualized the interface.<br>
Inside the folder of this repository, run the following command:
```bash
jupyter notebook
```
And open the file called "RobotControllerUI".

