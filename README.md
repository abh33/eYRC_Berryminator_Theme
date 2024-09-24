<div align = "center"><img src="./Images/berryminator_theme1.png" alt="im1" width=900><h1>Berryminator (BM) Theme<br> eYRC 2021-22</h1>

</div>

### Theme Description
To simulate an automated robot in CoppeliaSim that traverses an urban farming scenario to pluck berries and deposit them in designated sections.

### Learning Objectives for students
- Robotic Simulation
- Robotic Arm Design and Manipulation 
- Plucking and Depositing Mechanism
- Image Processing
- Navigation and Path Planning
- Control Systems
- Python and Lua Programming

<div align = "center"><img src="./Images/arena.png" alt="img2" width=350> &nbsp;<img src="./Images/robot.png" alt="img2" width=350>
<br><b>(a) Simulation Arena layout (Left) &nbsp;&nbsp; (b) Robot Model used (Right) </b>

</div>

*The final goal of the Berryminator theme was to collect fruits from different plants placed in the simulated arena. The robot model in (b) was provided to the teams. The teams in the competition had to construct a robotic arm on top of the robot and implement the programming of the whole setup in python and lua to implement the final goal.*

### Task Description
The following table gives a brief overview of all the tasks of the berryminator theme.

<div align = "center">
<!-- <style type="text/css">
.tg  {border-collapse:collapse;border-spacing:0;}
.tg td{border-color:black;border-style:solid;border-width:1px;font-family:Arial, sans-serif;font-size:14px;
  overflow:hidden;padding:10px 5px;word-break:normal;}
.tg th{border-color:black;border-style:solid;border-width:1px;font-family:Arial, sans-serif;font-size:14px;
  font-weight:normal;overflow:hidden;padding:10px 5px;word-break:normal;}
.tg .tg-c3ow{border-color:inherit;text-align:center;vertical-align:top}
.tg .tg-7btt{border-color:inherit;font-weight:bold;text-align:center;vertical-align:top}
.tg .tg-0pky{border-color:inherit;text-align:left;vertical-align:top}
</style> -->
<table class="tg"><thead>
  <tr>
    <th class="tg-7btt">Task</th>
    <th class="tg-7btt">Subtask</th>
    <th class="tg-7btt">Description</th>
  </tr></thead>
<tbody>
  <tr>
    <td class="tg-c3ow">Task 0</td>
    <td class="tg-c3ow"></td>
    <td class="tg-0pky">Software Installation &amp; Python Coding test</td>
  </tr>
  <tr>
    <td class="tg-c3ow" rowspan="3">Task 1</td>
    <td class="tg-c3ow">A</td>
    <td class="tg-0pky">Explore OpenCV</td>
  </tr>
  <tr>
    <td class="tg-c3ow">B</td>
    <td class="tg-0pky">Exploring CoppeliaSim (Remote API)</td>
  </tr>
  <tr>
    <td class="tg-c3ow">C</td>
    <td class="tg-0pky">Exploring CoppeliaSim (Sensors &amp; Joints)</td>
  </tr>
  <tr>
    <td class="tg-c3ow" rowspan="2">Task 2</td>
    <td class="tg-c3ow">A</td>
    <td class="tg-0pky">Berry Detection in CoppeliaSim</td>
  </tr>
  <tr>
    <td class="tg-c3ow">B</td>
    <td class="tg-0pky">Robotic Arm Design And Simulation</td>
  </tr>
  <tr>
    <td class="tg-c3ow">Task 3</td>
    <td class="tg-c3ow"></td>
    <td class="tg-0pky">Navigation</td>
  </tr>
  <tr>
    <td class="tg-c3ow">Task 4</td>
    <td class="tg-c3ow"></td>
    <td class="tg-0pky">Fruit Plucking and Deposit</td>
  </tr>
  <tr>
    <td class="tg-c3ow">Task 5</td>
    <td class="tg-c3ow"></td>
    <td class="tg-0pky">Mini Implementation</td>
  </tr>
  <tr>
    <td class="tg-c3ow">Task 6</td>
    <td class="tg-c3ow"></td>
    <td class="tg-0pky">Final Theme Implementation</td>
  </tr>
</tbody></table>

</div>

### Automatic Evaluation
In this theme we implemented client side automatic evaluation. The basic idea behind this evaluation was that along with task documentation and instructions on how to complete each of the tasks, the students were given an **executable file (.exe)** which they had to run on their machines. This executable file took the students own written code as input and evaluated it according to the parameters specified. For eg. in Task 3 navigation, the robot was required to traverse a randomly generated set of checkpoints. The executable generated these checkpoints and also during the robot traversal evaluated if the robot was able to visit all checkpoints or not.

Another interesting feature that was incorporated in the automatic evaluators was a method to measure participant engagement in the task. This was implemented using the GoogleSheets API. Everytime any participant team ran the executable file, all the data captured for evaluation (including the participant teams data) was exported to google sheet for our perusal. This sheet enabled us to monitor the teams progress and give hints and other help accordingly.

#### **Project Collaborators** - Abhinav Sarkar, Shyama H, Amit Kumar, Harmanjeet Singh, Aditya Panwar
This project was developed as one of the themes of e-Yantra Robotics Competition (eYRC-2021-22)