#include <SFML/Graphics.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include <map>


const sf::Vector2f rectSize(10.0f,10.0f); //cell size 10x10
const float BorderWidth=2.0f;
const int num_of_cells=50;
const float Width = (rectSize.x+BorderWidth)*num_of_cells;
const float Height = (rectSize.y+BorderWidth)*num_of_cells;

sf::RenderWindow window(sf::VideoMode(Width,Height), "Dijkstra");
std::vector<std::vector<sf::RectangleShape>> grid;


class Dijkstra
{
  public:
  std::vector<std::vector<int>> obstacles={{10,0},{10,10},{10,11},{10,12},{10,13},{10,14},
					   {39,43},{38,43},{37,43},{36,43},{35,43},{34,43},{33,43},{32,43},
					   {40,41},{40,42},{40,43},{40,40},{40,41},{40,39},{40,38},{40,37},{40,36},{40,35},{40,34},{40,33},{40,32},{40,31},{40,30},
					   {39,30},{38,30},{37,30},{36,30},{35,30},{34,30},{33,30},{32,30},
					   {15,15},{16,15},{17,15},{18,15},{19,15},{21,15},{22,15},{23,15},{24,15},{25,15},
					   {20,20},{20,19},{20,18},{20,17},{20,16},{20,15},{20,21},{20,22},{20,23},{20,24},{20,25}};
					 
  std::vector<std::vector<int>> explore_seq = {{0,1},{0,-1},{1,0},{-1,0},{1,-1},{-1,1},{1,1},{-1,-1}};
  
  std::vector<int> start, goal;
  
  std::map<std::vector<int>, float> f_costs;
  float straight_step_cost=1.0, diagonal_step_cost=1.414;
  std::map<std::vector<int>, float> open_nodes;
  std::vector<std::vector<int>> explored_nodes;
  std::map<std::vector<int>,std::vector<int>> parent_nodes;
  
  bool goal_found = false;
  
  //Constructor
  Dijkstra(std::vector<int> st, std::vector<int> gl)
	{
	start=st;
	goal=gl;
	open_nodes[{start[0],start[1]}] = 0.0;
	std::vector<int> z={-1,-1};
	parent_nodes[z]=start;
	}
	  
  void DrawGrid()
	{
	window.clear();
	int i=0,j;
	for(auto g:grid)
		{
		j=0;
		for(auto g1:g)
			{
			float x = (rectSize.x+BorderWidth)*i;
			float y = (rectSize.y+BorderWidth)*j;
			g1.setPosition(x,y);
			window.draw(g1);
			j++;
			}
		i++;
		}
	window.display();

	}

  //Neighbour cell exploration
  std::map<std::vector<int>,float> ExploreNeighbours(std::vector<int> node)
	{
	std::map<std::vector<int>,float> neighbours;
	int count=0;
	for(auto e:explore_seq)
		{
		std::vector<int> n(2);
		n[0] = node[0]+e[0];
		n[1] = node[1]+e[1];
		auto it = std::find(obstacles.begin(), obstacles.end(), n);
		if ((it == obstacles.end())&(n[0]>=0)&(n[0]<50)&(n[1]>=0)&(n[1]<50))
			{
			if(count>3)
				neighbours[n]=diagonal_step_cost;
			else
				neighbours[n]=straight_step_cost;
			}
		count+=1;
		}
	return neighbours;
	}


  //Sorting function
  std::vector<int> Sorter(std::map<std::vector<int>,float> m)
	{
	float max=100000.00;
	std::vector<int> v1(2);
	for(auto m1:m)
		{
		if(m1.second<max)
			{
			max=m1.second;
			v1[0]=m1.first[0];
			v1[1]=m1.first[1];
			}
		}
	return v1;
	}
  //Get the path
  void Path()
	{
	std::vector<int> current = goal;    
	while(current[0]!=-1)
		{
		grid[current[0]][current[1]].setFillColor(sf::Color::Red);
		current = parent_nodes[current];
		DrawGrid();
		}
	return;
	}
  void Search()
	{
	while((open_nodes.size()!=0)&(!goal_found))
		{
		sf::Event event;
		while (window.pollEvent(event))
			{
			if(event.type == sf::Event::Closed)
			window.close();
			}
		std::vector<int> node = Sorter(open_nodes);
		std::map<std::vector<int>,float> neighbours = ExploreNeighbours(node);
		float cost = open_nodes[node];

		for(auto neighbour:neighbours)
			{
			std::vector<int> n = neighbour.first;
			float step_cost = neighbour.second;
			float f_cost = cost+step_cost;
			auto it = std::find(explored_nodes.begin(), explored_nodes.end(), n);
			if (it == explored_nodes.end())
				{
				  if(n==goal)
				 	goal_found=true;
				  parent_nodes[n]=node;
				  open_nodes[n]=f_cost;
				  explored_nodes.push_back(n);
				  f_costs[n]=f_cost;
				}
			else if(f_cost<f_costs[n])
				{
				parent_nodes[n]=node;
				f_costs[n]=f_cost;
				}

			}
		open_nodes.erase(node);
		explored_nodes.push_back(node);
		grid[node[0]][node[1]].setFillColor(sf::Color::Blue);
		DrawGrid();
		}
	return;
	}

};

void CreateGrid()
	{
	for(int i=0;i<num_of_cells;i++)
		{
		std::vector<sf::RectangleShape> v1;
		for(int j=0;j<num_of_cells;j++)
			{
			sf::RectangleShape rect(rectSize);
			rect.setFillColor(sf::Color(255,255,255));
			rect.setOutlineThickness(BorderWidth);
			rect.setOutlineColor(sf::Color(0,0,0));
			v1.push_back(rect);
			}
		grid.push_back(v1);
		}
	}

int main() 
	{
	//Creating Grid
	CreateGrid();
	
	//Goal and starting point
	std::vector<int> goal={45,45}, start={0,0};
	
	//Dijkstra instance
	Dijkstra dijkstra1(start,goal);

	//Obstacle marking
	for(auto i:dijkstra1.obstacles)
		{
		grid[i[0]][i[1]].setFillColor(sf::Color::Black);
		}

	//Goal marking
	grid[goal[0]][goal[1]].setFillColor(sf::Color::Green);

	//Initial marking
	grid[start[0]][start[1]].setFillColor(sf::Color::Blue);



	while (window.isOpen())
		{
		dijkstra1.DrawGrid();
		dijkstra1.Search();
		dijkstra1.Path();

		}
	return 0;
	}
