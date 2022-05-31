# PART 1 : Software Documentation
The first part of this assignment is to create the documentation for the [ROS node created in the third assignment](https://github.com/IlMusu/Research_Track/tree/assignment_3/scripts).</br>
Given the fact that "robot_ui" was implemented in Python, the software used to create the documentation is [Sphinx](https://www.sphinx-doc.org/en/master/).

### Installing
To install Sphinx, run the following command:

```bash
sudo apt-get install python3-sphinx
```

### Running
The steps followed to create the documentation are:
1. Move to the main directory of the "robot_ui" node.
2. Create a directory called ["/sphinx"](https://github.com/IlMusu/Research_Track/tree/assignment_3/sphinx/) and inside it run the command:<br>
NB. This will start an interactive prompt that will ask the user for the required informations.

```bash
sphinx-quistart
```

3. Modify the ["conf.py"](https://github.com/IlMusu/Research_Track/blob/assignment_3/sphinx/source/conf.py) file just created by adding the path (line 13-15) so that Sphinx is able to find the sources.<br>
(OPTIONAL) Some extensions (line 33) may be added to customize Sphinx.<br>
(OPTIONAL) It is possible to change the html theme (line 64).

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

## Installing and running
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
Downlod the folder of this repository and inside of it run the following command:
```bash
jupyter notebook
```
Finally, open the file called "RobotControllerUI".<br>
NB. This notebook also contains some graphs of some parameters and statistics regarding the robot performance in scanning the environmnet and reaching targets.

# PART 3 : Statistics
The third part of this assignment is to perform a statistical analysis on the [first assignment](https://github.com/IlMusu/Research_Track/tree/assignment_1/), considering two different implementations and testing which one performs better in the circuit given, when silver tokens are randomized in position and in number (but always inside the circuit).<br>
As performace evaluators, the **average time required to finish the circuit** has been considered.<br>
NB. The simulation has been run 30 times for each implementaion.<br>

As said, two implementations of the same algorithm have been compared:
- The implementation of [this repository](https://github.com/CarmineD8/python_simulator/tree/rt2), referred to as "Robot1".
- The implementation of [first assignment](https://github.com/IlMusu/Research_Track/tree/assignment_1/), reffered to as "Robot2".

Before starting to collect the data, it is necessary to formulate the hypotesis:
```
With a significance level of 5%, I formulate the following hypothesis:
H0 : the mean time of Robot1 is the same as the mean time of Robot2.
HA : the mean time of Robot1 is more than the mean time of Robot2.
```
In this context:
- The term H0 is called the **null hypothesis** because when comparing Robot1 with Robot2 it is supposed that the robots are equally good.
- The term HA is called the **alternative hypothesis** because when comparing robot1 with robot2 it is supposed that one of the robots is superior to the other.
- The **significance level of 5%** implies that H0 will be rejected when the sampling result has a less than 5% (in our case) of probability of occurring if H0 is true.

Given the fact that there there are only 30 samples for each robot and that we are considering two robots, the statiscs that has been used to evalute the performance of the robots is the **Two Sample T-Test**.<br>
Given the fact that the alternative hypothesis HA has been formulated as one of the two robots being superior to the other, we have need to consider only one of the two tails of the T-Test.<br>
At the end, the statistics that needs to be used is the **One-Tailed Two Sample T-Test**.

NB. It is also possible to formulate the hypothesis HA as the following:
```
HA : the mean time of Robot1 is different that the mean time of Robot2.
```
In this case we are not explicitly stating that one of the two robots is superior to the other, so, we must consider both the tails of the T-Test. This is referred to as Two-Tailed T-Test.

## Running
Inside the folder of this repository, run the following command:
```bash
jupyter notebook
```
And open the file called "StatisticalAnalysis".<br>
The last output of the notebook contains the result of the statistical analysis.
