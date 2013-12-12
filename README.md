Organizer Baxter {#welcome}
===

We are exchange students at UC Berkeley and here we present the Organizer Baxter, our final project for the EE125 (Introduction to Robotics) course in the Fall semester of 2013. In this project, we used Baxter, an industrial robot built by [Rethink Robotics][1].

---

Introduction
---

The goal of our project is to use Baxter to keep some objects placed in a table organized. To achieve this, Baxter memorizes the original position of the objects in the table. When it detects that an object has been moved from its original position, it actuates to grasp this object and put it back in the position where it was before.

To make this work, we used Baxter’s right hand camera to monitor the table and get the objects position. The first problem we had to solve was calibrating the camera to be able to get the position in the table plane given a point in the image from the camera. Once we had that part working, the second problem we had to solve was to identify the position of the object in the image, and then also identifying the position of the object in the table using the solution for the first problem. The third problem was to move Baxter’s left arm endpoint to a specific point in space, that we can get using the solution from the second problem. Finally, we had to plan the sequence of points that the arm’s endpoint should move to in order to pick up an object in one place in the table and leave it in another.

Sometime in the future, robots will be so sophisticated that we all will have a robot to clean and organize our houses. Have you ever watched the cartoon called “The Jetsons”? They are a family from the future, and they own a household robot called Rosie. [This cartoon aired in 1964, and it predicted many of the present technology, like flat screen TVs, video phone calls, portable media devices and interactive media.][2] Thus, we can believe that a robot like Rosie will someday be a reality in our lives.

For now, the principles used in the Organizer Baxter can be applied in industry. An example would be automating the arrangement of some products in a pack, in a way that every pack will have the products organized in the same way. Another example can be a robot that would organize the tools used by a human worker, so he doesn’t have to worry about that while doing his tasks.

---


Design
---



---

Implementation
---

The hardware we used to do the project was Baxter. It is a robot with two 7-degrees-of-freedom arms. On each end-effector it has a camera and a gripper. It also has a camera on the head and various sensors.

To program Baxter, we used ROS, the [Robot Operating System][3]. ROS is a framework for writing robot software. It is a collection of tools, libraries, and conventions to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms. Using ROS, we program nodes, that are processes that perform computation and communicate with one another using topics and services. The programming language used to program the nodes was [Python][4].

Baxter also has a Software Development Kit (SDK) provided to us by the Instructors of the discipline. In this SDK there was an interface implemented to manipulate the various inputs and outputs of the robot. In our implementation, we used this SDK to enable and disable the robot, the camera and to set the joint positions of the arms. It also provided an Inverse Kinematics service to calculate the joint angles for the Baxter arms.

### Camera Calibration

The first thing we had to be able to do is to retrieve an image from Baxter’s right hand camera. In order to do that, we created a ROS service called `right_hand_camera_srv`, that subscribes to the topic called `/cameras/right_hand_camera/image`, where the images coming from that camera are published. Upon calling this service, it stores and returns the last image published.

To solve the first problem we had in the project, where we want to calculate the coordinates on the table plane from the coordinates of a pixel in the image from the camera, we had to perform a calibration task before running the project. In this calibration, we sample four spatial coordinates moving the Baxter’s left arm to four different positions on the table. We use the Baxter SDK function to get the position of the end-effector of the left arm and store its spatial and rotational coordinates. These positions are marked so we can visualize them in the image from the camera and select the corresponding pixels by clicking on those points on the image.

With those points, we calculate a matrix `H` that represents a linear transform between points in the floor and points in the image, called a homography. With this matrix, we can tell which pixel on the image correspond to a certain point in the table plane. Calculate the opposite, that is, given a pixel calculate the table coordinates, can be done with the inverse of matrix `H`. With this inverse matrix, the first problem is solved. In our implementation, we store this matrix in a global variable.

### Object Recognition

For now, we only implemented a function to recognize an object by its color. The algorithm uses some functions from the [OpenCV][5] library to processes the image received, filters color from a certain range in the HSV color space that corresponds to the color pink, transforming it in a black and white image, being white where originally it was pink. Then it searches for the largest contour on that image and we assume that this contour will represent the object. We find a point in the center and bottom of that contour, that will represent the closest point of the object in the floor plane. Then we apply to this point the inverse homography previously calculated in the calibration phase and we have the point in the table plane where the object is. These functions are all implemented in the main file of the project, called `organizer.py`.

### Movement

The third problem was to move Baxter’s arm to a specific point that will lie on the table plane. In order to to that, we have to find a set of joint angles that will make the end-effector of the left arm be in that position. This is the Inverse Kinematics problem. To solve it, we used a service provided by the Baxter SDK, where we pass the spatial and rotational coordinates that we want the end-effector to be and it returns one possible set of joint angles that solve this problem. To set the joint angles values in the arm, there is a function in the Baxter SDK that manages to do that. Then we were able to move the arm to any point in the space that Baxter can reach with his left arm.

We implemented the functions that make Baxter pick the object or put it on the table in a way that it will first move the arm to a position right above the destination, that is, it will move to a point that have the same `x` and `y` coordinates, but the `z` coordinate is a little bigger. After the arm finishes going to that position, we command it to move again to the real destination, and then close or open the gripper, depending of the case.

### Putting All Together

Finally, we combine the separate results from resolving the different problems into a program that makes Baxter keep one object in a table in the same place. At the beginning we set up the robot and perform the calibration. Then, the program asks to put the object in the original position. After clicking enter, it will check every 10 seconds if the object has been moved from the original position. If it detects the object in a different position, it will use the arm to grab the object in this new position and return it to the original position. So, the overall of the system is represented in the following flowchart:

![Flowchart of the overall program.][6]

---

Results
---

We tested our program on Baxter. The following video shows the calibration process, that we have to do before the execution of the organizing task.

[![Calibration](http://img.youtube.com/vi/sNN6mCO27-E/0.jpg)](http://www.youtube.com/watch?v=sNN6mCO27-E)

The next video shows Baxter grabbing the object and then placing it back on its original position.

[![Movement](http://img.youtube.com/vi/gRwJzJ8GW3w/0.jpg)](http://www.youtube.com/watch?v=gRwJzJ8GW3w)

---

Team
---

We are exchange students at the University of California at Berkeley through the Brazilian Science Without Borders Program.

![The team, viewed by Baxter!][7]

From left to right in the picture:

#### Carlos Pedro Vianna Lordelo

#### Felipe Campos Lins
Major in Electrical Engineering, from the [Federal University of Campina Grande](http://www.ufcg.edu.br/). Enthusiast of electronic circuits and new technologies, hoping to discover how everything works.     	

#### Nildo dos Santos Ribeiro Junior
Major in Computer Science, he is from the [Federal University of Minas Gerais](http://www.ufmg.br). He is a dreamer who believes that one day nobody will have to organize their stuff again, and robots will do all the boring tasks we don’t want to do.

---

References
---

- http://www.rethinkrobotics.com/products/baxter/
- http://www.wired.co.uk/news/archive/2012-09/24/jetsons-50th-anniversary
- http://www.ros.org/
- http://www.python.org/
- http://opencv.org/

---


  [1]: http://www.rethinkrobotics.com/products/baxter/
  [2]: http://www.wired.co.uk/news/archive/2012-09/24/jetsons-50th-anniversary
  [3]: http://www.ros.org/
  [4]: http://www.python.org/
  [5]: http://opencv.org/
  [6]: http://wc1.smartdraw.com/specials/images/examples/flowchart-example-medical-services-patient-routing-flowchart.png
  [7]: https://dl.dropboxusercontent.com/u/25167563/carlos_felipe_nildo_robotics_team.jpg
