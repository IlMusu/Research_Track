# PART 1
The first part of this assignment is to create the documentation for the [ROS node created in the third assignment](https://github.com/IlMusu/Research_Track/tree/assignment_3/scripts).</br>
Given the fact that "robot_ui" was written in Python, the software used to create the documentation is Sphinx.

To install Sphinx, run the following commands:
```bash
sudo apt-get install python3-sphinx
```

The steps that have been followed to create the documentation are:
1. Move to the main directory of the "robot_ui" node.
2. Create a directory called ["sphinx"](https://github.com/IlMusu/Research_Track/tree/assignment_3/sphinx/) and inside of it run the command:<br>
NB. This will start an interactive prompt that will ask the user fo the required informations.
```bash
sphinx-quistart
```
3. Modify the ["conf.py"](https://github.com/IlMusu/Research_Track/blob/assignment_3/sphinx/source/conf.py) file just created by adding the correct path so that Sphinx is able to find the source folder.<br>
(OPTIONAL) Some extensions may be added to customize Sphinx.

4. Modify the ["index.rst"](https://github.com/IlMusu/Research_Track/blob/assignment_3/sphinx/source/index.rst) file by adding the scripts for which we want to provide the documentation.
