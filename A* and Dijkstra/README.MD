Steps to Run Algorithms:

1. Open the command prompt and type "cd [path address name]" to set the directory path.

2. In the command prompt, type "python [codename].py" to run the desired code.

    $ python3 Dijkstra_point.py
    
    $ python3 Dijkstra_rigid.py
    
    $ python3 A*_point.py
    
    $ python3 A*_point.py

3. User Inputs:
	a. x position of the goal point (0, 249):
  b. y position of the goal point (0, 149):
  c. x position of the start point (0, 249): 
  d. y position of the start point (0, 149): 
	e. "Enter the clearance: " 
	f. "Enter the resolution: " 
	
4. The code will run after the user inputs are typed in. A window with a visualization of the nodes explored 
and path generated will appear.

5. If reach the goal, it will print goal

6. Feasibility check:
    If the start point is within the augmented obstacle space, the program will raise an error.
    If the goal is within the augmented obstacle space, the program will run with a warning.

