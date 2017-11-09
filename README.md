Organizer Baxter
===

We are exchange students at UC Berkeley and here we present the Organizer Baxter, our final project for the EE125 (Introduction to Robotics) course in the Fall semester of 2013. In this project, we used Baxter, a robot built by [Rethink Robotics][1].

![Baxter](https://www.dropbox.com/s/dh1oo7q5lmf1pb4/baxter-robot.jpg)

*This is Baxter, the awesome robot we used for the project.*

Introduction
---

The goal of our project is to use Baxter to keep some objects placed in a table organized. To achieve this, Baxter memorizes the original position of the objects in the table. When it detects that an object has been moved from its original position, it actuates to grasp this object and put it back in the position where it was before.

To make this work, we used Baxter’s right hand camera to monitor the table and get the objects position. The first problem we had to solve was calibrating the camera to be able to get the position in the table plane given a point in the image from the camera. Once we had that part working, the second problem we had to solve was to identify the position of the object in the image, and then also identifying the position of the object in the table using the solution for the first problem. The third problem was to move Baxter’s left arm endpoint to a specific point in space, that we can get using the solution from the second problem. Finally, we had to plan the sequence of points that the arm’s endpoint should move to in order to pick up an object in one place in the table and leave it in another.

Sometime in the future, robots will be so sophisticated that we all will have a robot to clean and organize our houses. Have you ever watched the cartoon called “The Jetsons”? They are a family from the future, and they own a household robot called Rosie. [This cartoon aired in 1964, and it predicted many of the present technology, like flat screen TVs, video phone calls, portable media devices and interactive media.][2] Thus, we can believe that a robot like Rosie will someday be a reality in our lives.

![Rosie, from the cartoon The Jetsons](https://www.dropbox.com/s/r6u1kkp5xu9t3up/rosie.png?dl=0)

*Rosie, the household robot from The Jetsons, a cartoon by Hanna-Barbera Productions*

For now, the principles used in the Organizer Baxter can be applied in industry. An example would be automating the arrangement of some products in a pack, in a way that every pack will have the products organized in the same way. Another example can be a robot that would organize the tools used by a human worker, so he doesn’t have to worry about that while doing his tasks.

Design
---

The calibration step needs to have good precision, so the points seen by the camera can be correctly referenced to Baxter’s workspace. We manually place the end effector of Baxter’s left arm in four different points over the table, this way we can easily map the table. After this, we click on the exact same points, that we used before, on a image given by the camera. So, we can make a cross reference between those points and compute the homography matrix from the image to the table. This approach is really straight forward, simple to implement and flexible, since we can select random points. However, it has the drawback of the user having to manually move the arm and click on the screen to get the points.

In order to identify the object location on the table, we used Baxter’s right hand camera to observe the environment and detect any change on it. To keep it simple, we selected the color of the object as the parameter to recognize where it is on the image, this approach is easy to implement, but narrows the set of objects we can recognize since the background could have a similar color, or even Baxter body could be detected. Also, multiple objects of the same color cannot be differentiated of each other. So, in a real engineering application, a shape recognition algorithm would be a more reliable option. With the position selected on the image we just need to use the homography to find the real position on the workspace.

To move the endpoint of the left arm, we used a build-in service from Baxter’s SDK  to, given the point of the last step, solve the inverse kinematic problem and another service to move the joints to the appropriate angle computed. This method of dealing with the problem is useful because we do not need to reinvent a solution for something that is already made, but we have no freedom to make a more precise setting of joint angles or something that would make the process better. Also we need to trust the documentation, what could lead to hard errors to debug.

The only step left is to plan a path to the arm. Since we have control of what is going to be on our workspace, we did not implemented a collision detection path planning. Simply, when we wanted to move the arm to a given position we would first move the endpoint to the exact same x and y but with an higher z, then we put the endpoint down to grab the object and afterwards we make it move all the way up again, so it will not slide over the table knocking out anything in it’s path. This way of dealing with the problem is good enough for our application and also easy to implement, despite of not being very robust because any change in the environment not foreseen would lead to big mistakes.

Implementation
---

The hardware we used to do the project was Baxter. It is a robot with two 7-degrees-of-freedom arms. On each end-effector it has a camera and a gripper. It also has a camera on the head and various sensors.

To program Baxter, we used ROS, the [Robot Operating System][3]. ROS is a framework for writing robot software. It is a collection of tools, libraries, and conventions to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms. Using ROS, we program nodes, that are processes that perform computation and communicate with one another using topics and services. The programming language used to program the nodes was [Python][4].

Baxter also has a Software Development Kit (SDK) provided to us by the Instructors of the discipline. In this SDK there was an interface implemented to manipulate the various inputs and outputs of the robot. In our implementation, we used this SDK to enable and disable the robot, the camera and to set the joint positions of the arms. It also provided an Inverse Kinematics service to calculate the joint angles for the Baxter arms.

### Camera Calibration

The first thing we had to be able to do is to retrieve an image from Baxter’s right hand camera. In order to do that, we created a ROS service called `right_hand_camera_srv`, that subscribes to the topic called `/cameras/right_hand_camera/image`, where the images coming from that camera are published. Upon calling this service, it stores and returns the last image published.

To solve the first problem we had in the project, where we wanted to calculate the coordinates on the table plane from the coordinates of a pixel in the image, we performed a calibration task before running the project. In this calibration, we sample four spatial coordinates moving the Baxter’s left arm to four different positions on the table and, using a Baxter SDK function to get the position of the end-effector of the left arm, we store its spatial and rotational coordinates. These positions are marked so we can visualize them in the image from the camera and select the corresponding pixels by clicking on those points on the image.

With those points, we calculate a matrix `H` that represents a linear transform between points in the floor and points in the image, called a homography. With this matrix, we can tell which pixel on the image correspond to a certain point in the table plane. Calculate the opposite, that is, given a pixel calculate the table coordinates, can be done with the inverse of matrix `H`. With this inverse matrix, the first problem is solved. In our implementation, we store this matrix in a global variable.

### Object Recognition

For now, we only implemented a function to recognize an object by its color. The algorithm uses some functions from the [OpenCV][5] library to processes the image received, filters color from a certain range in the HSV color space that corresponds to the color pink, transforming it in a black and white image, being white where it originally was pink. Then it searches for the largest contour on that image and we assume that this contour will represent the object. After that, the algorithm finds a point in the center of that contour, which will represent the pixel of the object on the image plane. Then we apply to this point the inverse homography previously calculated in the calibration phase and we have the point on the table plane where the object is. These functions are all implemented in the main file of the project, called `organizer.py`.

### Movement

The third problem was to move Baxter’s arm to a specific point that will lie on the table plane. In order to to that, we had to find a set of joint angles that would make the end-effector of the left arm reach that position. This is the Inverse Kinematics problem. To solve it, we used a service provided by the Baxter SDK, where we pass the spatial and rotational coordinates that we want the end-effector to be and it returns one possible set of joint angles that solve this problem. To set the joint angles values to the arm, there is a function in the Baxter SDK that manages to do that. Then we were able to move the arm to any point in the space that Baxter can reach with his left arm.

We implemented the functions that make Baxter pick the object or put it on the table in a way that it will first move the arm to a position right above the destination, that is, it will move to a point that have the same `x` and `y` coordinates, but the `z` coordinate is a little bigger. After the arm finishes going to that position, we command it to move again to the real destination, and then close or open the gripper, depending of the case.

### Putting All Together

Finally, we combined the separate results from resolving the different problems into a program that makes Baxter keep tracking of a pink object on a table. At the beginning we set up the robot and perform the calibration. Then, the program asks to put the object in its original position. After clicking enter, it will check every 10 seconds if the object has been moved from the original position. If it detects the object in a area, it will use the arm to grab the object in this new position and return it to the original position. So, the overall of the system is represented in the following flowchart:

![Flowchart of the overall program.][6]

*Flowchart for the overall behavior of our system.*

Results
---

We tested our program on Baxter. The following video shows the calibration process, that we have to do before the execution of the organizing task.

[![Calibration](http://i1.ytimg.com/vi/sNN6mCO27-E/mqdefault.jpg)](http://www.youtube.com/watch?v=sNN6mCO27-E)

*Click in the thumbnail to watch the video.*

The next video shows Baxter grabbing the object and then placing it back on its original position.

[![Movement](http://i1.ytimg.com/vi/gRwJzJ8GW3w/mqdefault.jpg)](http://www.youtube.com/watch?v=gRwJzJ8GW3w)

*Click in the thumbnail to watch the video.*

Conclusion
---

We got the expected result for our test. Baxter was able to return our object to the original position. But the initial intention of the project was to make it work for various objects. We encountered difficulties with the grasping problem. The imprecision of the camera calibration caused Baxter to miss other objects we tested, like little boxes. So, the grasping needs improvement to work with more concrete objects. We resolved that making the test with an object made of paper that was really thin and easy to grasp.

Another idea for the project is to automatizate the calibration process, using vision algorithms to guide the arm to sample the table positions and also to get the calibration points in the image. By doing this, the camera could move during the process and the homography could be easily recalculated for the new camera position.

Doing this project we could make practical experiments based on the theoretical subjects we have learned in the classes during the whole semester. Just to give examples, we used Forward Kinematics, Inverse Kinematics, Image Processing in the same project. This is a very important practice that should be done in every classes because it proves that you acknowledged what you have learned and makes everything looks easier and simple.



Team
---

We are exchange students at the University of California at Berkeley through the Brazilian [Science Without Borders](http://www.cienciasemfronteiras.gov.br/web/csf-eng/) Program.

![The team, viewed by Baxter!][7]

*Selfie with Baxter. From left to right in the picture: Baxter, Carlos, Felipe and Nildo.*

#### Carlos Pedro Vianna Lordelo
> Major in Electronic and Computer Engineering at [Federal University of Rio de Janeiro](http://www.ufrj.br). Used to work with digital signal processing, he came to Berkeley with the object of open his mind to differents areas of electronic engineering, choosing robotics as his first one.


#### Felipe Campos Lins
> Major in Electrical Engineering, from the [Federal University of Campina Grande](http://www.ufcg.edu.br/). Enthusiast of electronic circuits and new technologies, hoping to discover how everything works.     	

#### Nildo dos Santos Ribeiro Junior
> Major in Computer Science, he is from the [Federal University of Minas Gerais](http://www.ufmg.br). He has some experience in research on Wireless Sensor Networks and really likes the field of Robotics. Has a good sense of humor and sometimes interesting and crazy ideas.


References
---

- http://www.rethinkrobotics.com/products/baxter/
- http://www.wired.co.uk/news/archive/2012-09/24/jetsons-50th-anniversary
- http://www.ros.org/
- http://www.python.org/
- http://opencv.org/


  [1]: http://www.rethinkrobotics.com/products/baxter/
  [2]: http://www.wired.co.uk/news/archive/2012-09/24/jetsons-50th-anniversary
  [3]: http://www.ros.org/
  [4]: http://www.python.org/
  [5]: http://opencv.org/
  [6]: https://www.dropbox.com/s/a2l6v04qj2n4qp1/flowchart.svg
  [7]: https://www.dropbox.com/s/a17i7pcqk5z2fx6/carlos_felipe_nildo.jpg
