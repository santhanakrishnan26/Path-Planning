# Path-Planning

  # Dijkstra
  https://user-images.githubusercontent.com/43046339/163545064-679d2b02-8836-462a-8269-088c6b5c8277.mov
  
  # Dijkstra path
    Dijkstra found the optimal path, but it explored almost every cell in the grid. The exploration mathod is not optimal 
    since it only considering the path cost from the starting node to the current node, it doesn't have any intiution 
    about the goal node.
  ![Dijkstra2](https://user-images.githubusercontent.com/43046339/163548183-be2d50b3-c058-464b-8df7-dca0c1465f75.png)
  
  # Commands to execute dijkstra.cpp
    To install sfml on linux: sudo apt-get install libsfml-dev
    g++ -c dijkstra.cpp
    g++ dijkstra.o -o sfml-app -lsfml-graphics -lsfml-window -lsfml-system
    ./sfml-app
  
  # A_star
  https://user-images.githubusercontent.com/43046339/163555089-f539d387-5b96-46b3-9d45-64f933d910f6.mov
  
  
  # A_star path
    A_star has an intiution about the distance between goal node and current node (h_cost) along with the path cost (g_cost). So its
    exploration is optimal. But the path is not the optimal one as we can see from the image, this is because the h_cost is larger
    than the g_cost, thus the algorithm focusses on the cells that have lesser h_cost.
  ![A_star_euclidean1](https://user-images.githubusercontent.com/43046339/163556585-d694609e-33b7-4c2e-836f-7adc17fae57b.png)
  
  # A_star Tuning
    By scaling down the h_cost we can get the optimal path.
    h_cost scaled down by 10.
  ![A_star_scaled_down_10](https://user-images.githubusercontent.com/43046339/163557939-69970cdd-8451-41ad-9f01-05215dbf37e9.png)
  
  h_cost scaled down by 100. Now it's more like dijkstra, since we scaled down the h_cost.
  
  ![A_star_scaled_down_100](https://user-images.githubusercontent.com/43046339/163558141-3a9b3a50-a7fb-4308-827a-379bde5b8626.png)
  
    To break the tie, we can use (after scaling down) (used manhattan distance for h_cost):
    heuristic *= (1.0 + p), 
    where p = (minimum cost of taking one step)/(expected maximum path length).
    I used p=1/1000.
  
  ![A_star_with_p](https://user-images.githubusercontent.com/43046339/163561116-bec860fb-32d3-40c8-a3a0-691f8f5a4a2c.png)
  
    If it work well, use:
    dx1 = current.x - goal.x
    dy1 = current.y - goal.y
    dx2 = start.x - goal.x
    dy2 = start.y - goal.y
    cross = abs(dx1*dy2 - dx2*dy1)
    heuristic += cross*0.001
  
  ![A_star_with_cross](https://user-images.githubusercontent.com/43046339/163561376-826e644e-36ab-4289-98bc-609c40d1a4ac.png)
  
  # To execute A_star.cpp
     g++ -c A_star.cpp
     g++ A_star.o -o sfml-app -lsfml-graphics -lsfml-window -lsfml-system
     ./sfml-app
  
  I referred this article for A_star hueristics: http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html 
  (For more detailed information visit this page).
  

  

  
