///You are allowed to define your own function to fulfill the requirement of tasks
//Dont change the name of following functions
/*/
Team id     : #0574.
Author list : Rohit Devar, Vishwanath Patil, Sanjay H B, Abhishekgouda SH.
File Name   : CB#0574_Task_1.1.zip
Theme       : Construct-O-Bot
Functions	: follow_path(), find_junction(), invert_path(), forward_wls(), void(), left_turn_wls(), right_turn_wls(), Task_1_1().
Global variables : l, c, r, w, b, junc_count, mux, add_delay, factor.
*/
#include "CB_Task_1_Sandbox.h"
unsigned char l, c, r;     // declare sensor l-left sensor r-right sensor c-central sensor
unsigned char L, S, R;
int w = 0, b = 255;
int junc_count = 0;
int stop_key = 1;
int mux = 1;
int add_delay = 0;
int factor = 1;
int cur_pos, prs_pos, nxt_pos;
/*
*
* Function Name: follow_path
* Input: void
* Output: void
* Logic: Uses white line sensors to follow the path
* Example Call: follow_path(); //Goes forward
*
*/
#include<iostream> 
#include <list> 
using namespace std;
int count1[1000], j = 0, old[100], cur[100], small;
int cm[] = { 2,24,21,12,31,7,26,4,29,0 };
int house[] = { 16,16,5,28,5,28,10,23,10 };
// A directed graph using adjacency list representation 
class Graph
{
	int V; // No. of vertices in graph 
	list<int>* adj; // Pointer to an array containing adjacency lists 

	// A recursive function used by printAllPaths() 
	void printAllPathsUtil(int, int, bool[], int[], int&);

public:
	Graph(int V); // Constructor 
	void addEdge(int u, int v);
	void printAllPaths(int s, int d);
};

Graph::Graph(int V)
{
	this->V = V;
	adj = new list<int>[V];
}

void Graph::addEdge(int u, int v)
{
	adj[u].push_back(v); // Add v to u’s list. 
}

// Prints all paths from 's' to 'd' 
void Graph::printAllPaths(int s, int d)
{
	// Mark all the vertices as not visited 
	bool* visited = new bool[V];

	// Create an array to store paths 
	int* path = new int[V];
	int path_index = 0; // Initialize path[] as empty 

	// Initialize all vertices as not visited 
	for (int i = 0; i < V; i++)
		visited[i] = false;

	// Call the recursive helper function to print all paths 
	printAllPathsUtil(s, d, visited, path, path_index);
}

// A recursive function to print all paths from 'u' to 'd'. 
// visited[] keeps track of vertices in current path. 
// path[] stores actual vertices and path_index is current 
// index in path[] 
void Graph::printAllPathsUtil(int u, int d, bool visited[],
	int path[], int& path_index)
{
	// Mark the current node and store it in path[] 
	visited[u] = true;
	path[path_index] = u;
	path_index++;

	// If current vertex is same as destination, then print 
	// current path[] 
	if (u == d)
	{
		count1[j] = 0;
		for (int i = 0; i < path_index; i++) {
			//cout << path[i] << " ";
			count1[j]++;
			old[i] = path[i];
		}
		cout << endl;
		//printf("\n");
		if (j == 0) {
			for (int i = 0; i < count1[j]; i++)
				cur[i] = path[i];
			small = count1[0];
		}
		/*else if (j == 3) {
			if (count1[j] < small) {
				for (int i = 0; i < count1[j]; i++)
					cur[i] = path[i];
				small = count1[j];
			}
		}*/
		if (j > 0 ) {
			if (count1[j] < count1[j-1]) {
				for (int i = 0; i < count1[j]; i++)
					cur[i] = path[i];
				small = count1[j];
			}
		}
		//else small = count1[0];

		//small = count1[j];
		j++;

	}
	else // If current vertex is not destination 
	{
		// Recur for all the vertices adjacent to current vertex 
		list<int>::iterator i;
		for (i = adj[u].begin(); i != adj[u].end(); ++i)
			if (!visited[*i])
				printAllPathsUtil(*i, d, visited, path, path_index);
	}

	// Remove current vertex from path[] and mark it as unvisited 
	path_index--;
	visited[u] = false;
}

void follow_path() {
	l = ADC_Conversion(1);
	c = ADC_Conversion(2);
	r = ADC_Conversion(3);
	if (l == w && c == b && r == w) {
		forward();
		velocity(200, 200);
		mux = 1;
		add_delay = 1;
	}
	if (l == b && c == w && r == w) {
		soft_left();
		velocity(25, 25);
		mux = 0;
		add_delay = 0;
	}
	if (l == w && c == w && r == b) {
		soft_right();
		velocity(25, 25);
		mux = 0;
		add_delay = 0;
	}
	if (l == b && c == b && r == w) {
		soft_left();
		velocity(50, 50);
		mux = 0;
	}
	if (l == w && c == b && r == b) {
		soft_right();
		velocity(50, 50);
		mux = 0;
	}
	if ((l == w && c == w && r == w) && (mux != 0)) {
		forward();
		velocity(100, 100);
		l = ADC_Conversion(1);
		c = ADC_Conversion(2);
		r = ADC_Conversion(3);
		factor = 0;
		add_delay = 0;
		if ((l == w && c == w && r == w) && (mux != 0)) {
			forward();
			velocity(50, 50);
			l = ADC_Conversion(1);
			c = ADC_Conversion(2);
			r = ADC_Conversion(3);
			while ((l == w && c == w && r == w) && mux != 0) {
				while ((l == w && c == w && r == w) && add_delay < 400 + factor) {
					left();
					velocity(100, 100);
					_delay_ms(1);
					l = ADC_Conversion(1);
					c = ADC_Conversion(2);
					r = ADC_Conversion(3);
					add_delay++;

					//printf("%d  left    %d-factor\n", add_delay,factor);
				}
				if (l == w && c == w && r == w) {
					right();
					velocity(100, 100);
					_delay_ms(add_delay);
					if (factor > 400) {
						forward();
						_delay_ms(10);
					}

				}

				add_delay = 0;
				//
				while ((l == w && c == w && r == w) && add_delay < 800) {
					right();
					velocity(100, 100);
					_delay_ms(1);
					l = ADC_Conversion(1);
					c = ADC_Conversion(2);
					r = ADC_Conversion(3);
					add_delay++;
					factor = add_delay + 300;
					//printf("%d   right  %d-factor\n", add_delay, factor);
				}
				if (l == w && c == w && r == w) {
					left();
					velocity(100, 100);
					//_delay_ms(add_delay);
					if (factor > 400) {
						forward();
						_delay_ms(10);
					}

				}
				add_delay = 0;

			}
		}
	}
}
/*
*
* Function Name: find_junction
* Input: void
* Output: int
* Logic: Uses white line sensors to find junction
* Example Call: find_junction() //return 1 if it finds junction
*
*/
int find_junction() {
	l = ADC_Conversion(1);
	c = ADC_Conversion(2);
	r = ADC_Conversion(3);
	if (l == b && c == b && r == b) {
		forward();
		velocity(250, 250);
		_delay_ms(250);
		if (l == b && c == b && r == b) {
			forward();
			if (l == b && c == b && r == b) {
				forward();
				return 1;
			}
			else {
				 return 0;
			}
		}
	}
}
/*
*
* Function Name: invert_path
* Input: void
* Output: void
* Logic: Uses white line sensors to follow the path
* Example Call: invert_path(); //Goes forward
*
*/
void invert_path(int target) {
	printf("target = %d\n", target);
	int code = 0, operation = 0, muxi = 0;
	if (target != 16)
		operation = 1;
	while (1) {
		l = ADC_Conversion(1);
		c = ADC_Conversion(2);
		r = ADC_Conversion(3);
		if (muxi == 1 && code == 1 && operation == 1) {
			printf("exiting kadu \n");
			break;
		}
		if (l == w && c == b && r == w) {
			forward();
			velocity(100, 100);
			muxi = 1;
			//printf("mux= %d code= %d operation=%d   W B W\n exit kadu \n", muxi, code, operation);
			add_delay = 1;
		}
		else if ((c == w && r == w) && code == 1 && muxi == 0) {

			forward();
			velocity(200, 200);
			_delay_ms(100);
			l = ADC_Conversion(1);
			c = ADC_Conversion(2);
			r = ADC_Conversion(3);
			if (c == w && r == w && operation == 0) {
				if (target = 16) {
					//turn invrted right
					printf("l-%d c-%d r-%d code -%d mux-%d \n", l, c, r, code, muxi);
					forward();
					_delay_ms(200);
					right();
					_delay_ms(200);
					while ((l == b && c == w && r == b) != 1) {
						right();
						l = ADC_Conversion(1);
						c = ADC_Conversion(2);
						r = ADC_Conversion(3);
					}

					//place
					forward();
					velocity(200, 200);
					_delay_ms(500);
					place();
					stop();
					_delay_ms(1000);
					// back
					back();
					velocity(180, 150);
					_delay_ms(850);

					//turn invrted right
					left();
					_delay_ms(180);
					while ((l == b && c == w && r == b) != 1) {
						left();
						l = ADC_Conversion(1);
						c = ADC_Conversion(2);
						r = ADC_Conversion(3);
					}
					operation = 1;
					printf("mux= %d code= %d operation  %d \n exit kadu \n", muxi, code, operation);
				}
				else {
					forward();
					velocity(200, 200);
					_delay_ms(180);
					operation == 1;
				}
			}


		}
		else if (l == b && c == w && r == b) {
			forward();
			velocity(100, 100);
			muxi = 0;
			add_delay = 1;
			code = 1;
		}
		else if (l == b && c == w && r == w) {
			soft_left();
			velocity(25, 25);
			if (operation == 1)
				muxi == 1;
			else
				muxi == 0;
			//printf("mux= %d code= %d operation=%d  B W W\n exit kadu \n", muxi, code, operation);
			add_delay = 0;
		}
		else if (l == w && c == w && r == b) {
			soft_right();
			velocity(25, 25);
			if (operation == 1)
				muxi == 1;
			else
				muxi == 0;
			//printf("mux= %d code= %d operation=%d   W W B\n exit kadu \n", muxi, code, operation);
			add_delay = 0;
		}
		else if (l == b && c == b && r == w) {
			soft_right();
			velocity(25, 25);
			muxi = 0;
		}
		else if (l == w && c == b && r == b) {
			soft_left();
			velocity(25, 25);
			muxi = 0;
		}
		else if ((l == b && c == b && r == b) && (muxi != 1)) {
			forward();
			velocity(100, 100);
			l = ADC_Conversion(1);
			c = ADC_Conversion(2);
			r = ADC_Conversion(3);
			factor = 0;
			add_delay = 0;
			if ((l == b && c == b && r == b) && (muxi != 1)) {
				forward();
				velocity(50, 50);
				l = ADC_Conversion(1);
				c = ADC_Conversion(2);
				r = ADC_Conversion(3);
				while ((l == b && c == b && r == b) && muxi != 1) {
					while ((l == b && c == b && r == b) && add_delay < 400 + factor) {
						left();
						velocity(100, 100);
						_delay_ms(1);
						l = ADC_Conversion(1);
						c = ADC_Conversion(2);
						r = ADC_Conversion(3);
						add_delay++;

						//printf("%d  left    %d-factor\n", add_delay,factor);
					}
					if (l == b && c == b && r == b) {
						right();
						velocity(100, 100);
						_delay_ms(add_delay);
						if (factor > 400) {
							forward();
							_delay_ms(10);
						}

					}

					add_delay = 0;
					//
					while ((l == b && c == b && r == b) && add_delay < 800) {
						right();
						velocity(100, 100);
						_delay_ms(1);
						l = ADC_Conversion(1);
						c = ADC_Conversion(2);
						r = ADC_Conversion(3);
						add_delay++;
						factor = add_delay + 300;
						//printf("%d   right  %d-factor\n", add_delay, factor);
					}
					if (l == b && c == b && r == b) {
						left();
						velocity(100, 100);
						//_delay_ms(add_delay);
						if (factor > 400) {
							forward();
							_delay_ms(10);
						}

					}
					add_delay = 0;

				}
			}
		}
		//printf("mux= %d code= %d operation=%d   W B W\n exit kadu \n", muxi, code, operation);

	}
	stop();
	//_delay_ms(5000);


}
/*
*
* Function Name: forward_wls
* Input: node
* Output: void
* Logic: Uses white line sensors to go forward by the number of nodes specified
* Example Call: forward_wls(2); //Goes forward by two nodes
*
*/
/*void forward_wls(unsigned char node){
	if (node == 1) {
		right_turn_wls();
	}
	else if (node > 1 && node < 5) {
		forward();
		_delay_ms(50);
		velocity(250, 250);
		follow_path();
	}
	else if (node == 5 | node == 6 | node == 8 | node == 10 | node == 12 | node == 15) {
		left_turn_wls();
		velocity(25, 25);
	}
	else {
		right_turn_wls();
		velocity(25, 25);
		if (junc_count == 13) {
			//invert_path();
		}
	}

}

/*
*
* Function Name: left_turn_wls
* Input: void
* Output: void
* Logic: Uses white line sensors to turn left until black line is encountered
* Example Call: left_turn_wls(); //Turns right until black line is encountered
*
*/
void left_turn_wls(void) {
	forward();
	_delay_ms(30);
	left();
	_delay_ms(180);
	while ((l == w && c == b && r == w) != 1) {
		left();
		l = ADC_Conversion(1);
		c = ADC_Conversion(2);
		r = ADC_Conversion(3);
	}
}

/*
*
* Function Name: right_turn_wls
* Input: void
* Output: void
* Logic: Uses white line sensors to turn right until black line is encountered
* Example Call: right_turn_wls(); //Turns right until black line is encountered
*/
void right_turn_wls(void)
{

	forward();
	_delay_ms(30);
	right();
	_delay_ms(180);
	while ((l == w && c == b && r == w) != 1) {
		right();
		l = ADC_Conversion(1);
		c = ADC_Conversion(2);
		r = ADC_Conversion(3);
	}
}

/*
*
* Function Name: e_shape
* Input: void
* Output: void
* Logic: Use this function to make the robot trace a e shape path on the arena
* Example Call: e_shape();
*/
void e_shape(void)
{
	while (1) {
		l = ADC_Conversion(1);
		c = ADC_Conversion(2);
		r = ADC_Conversion(3);
		S = ADC_Conversion(4);
		L = ADC_Conversion(5);
		R = ADC_Conversion(6);
		printf("L -%d		S- %d	 R- %d\n", L, S, R);
		forward();
		velocity(250, 250);
	}

}


/*
*
* Function Name: Task_1_1
* Input: void
* Output: void
* Logic: Use this function to encapsulate your Task 1.1 logic
* Example Call: Task_1_1();
*/
/*void Task_1_1(void)
{
	// Write your task 1.1 Logic here
	int flag = 0;
	while (1) {
		l = ADC_Conversion(1);
		c = ADC_Conversion(2);
		r = ADC_Conversion(3);
		printf("%d       %d     %d\n", l, c, r);
		flag = find_junction();
		//if (flag) {
			//junc_count++;
			//if (junc_count == 16) {
			//	stop;
			//	break;
			//}
			//forward_wls(junc_count);
	   // }
		//else {
			follow_path();
		//}
	}


}*/


/*
*
* Function Name: Task_1_2
* Input: void
* Output: void
* Logic: Use this function to encapsulate your Task 1.2 logic
* Example Call: Task_1_2();
*/
void source2destiny(int src, int dest) {
	Graph g(32);
	g.addEdge(0, 1);
	g.addEdge(1, 0);
	g.addEdge(1, 3);
	g.addEdge(3, 1);
	g.addEdge(3, 4);
	g.addEdge(4, 3);
	g.addEdge(6, 3);
	g.addEdge(6, 5);
	g.addEdge(5, 6);
	g.addEdge(3, 6);//10
	g.addEdge(6, 8);
	g.addEdge(8, 6);
	//g.addEdge(6, 27);
	//g.addEdge(27, 6);
	g.addEdge(8, 7);
	g.addEdge(7, 8);
	g.addEdge(8, 9);
	g.addEdge(9, 8);
	g.addEdge(8, 11);
	g.addEdge(11, 8);
	g.addEdge(10, 11);
	g.addEdge(11, 10); //22
	g.addEdge(11, 22);
	g.addEdge(22, 11);
	g.addEdge(13, 11);
	g.addEdge(11, 13);
	g.addEdge(14, 13);
	g.addEdge(13, 14);
	g.addEdge(13, 12);
	g.addEdge(12, 13);
	g.addEdge(13, 15);
	g.addEdge(15, 13);
	//
	g.addEdge(16, 15);
	g.addEdge(15, 16);
	g.addEdge(18, 16);
	g.addEdge(16, 18);
	//
	g.addEdge(18, 20);
	g.addEdge(20, 18);
	g.addEdge(20, 21);
	g.addEdge(21, 20); //40
	g.addEdge(20, 19);
	g.addEdge(19, 20);
	g.addEdge(20, 22);
	g.addEdge(22, 20);
	g.addEdge(23, 22);
	g.addEdge(22, 23);
	g.addEdge(22, 25);
	g.addEdge(25, 22);
	g.addEdge(25, 24);
	g.addEdge(24, 25);//50
	g.addEdge(25, 26);
	g.addEdge(26, 25);
	g.addEdge(25, 27);
	g.addEdge(27, 25);
	g.addEdge(28, 27);
	g.addEdge(27, 28);
	g.addEdge(30, 27);
	g.addEdge(27, 30);
	g.addEdge(29, 30);
	g.addEdge(30, 29);
	g.addEdge(30, 31);
	g.addEdge(17, 30); //12
	g.addEdge(30, 17);
	g.addEdge(17, 0);
	g.addEdge(0, 17);
	g.addEdge(3, 2);
	g.addEdge(2, 3);
	g.addEdge(31, 30);

	int s = src, d = dest;

	//cout << "Following are all different paths from " << s 
	   // << " to " << d << endl; 
	g.printAllPaths(s, d);
	//cout<<small<<endl;
	/*if(j>1){
	for(int i=0;i<small;i++)
	cout << cur[i] << " ";
	}
	if(j==1  )
	for(int i=0;i<count[0];i++)
	cout << old[i] << " ";*/
	if (j == 1) {
		for (int i = 0; i < count1[0]; i++)
			cur[i] = old[i];
	}
	cout << small << endl;
	for (int i = 0; i < small; i++)
		cout << cur[i] << " ";
	cout << endl;

	/*for (int i = 0; i < 8; i++) {
		g.printAllPaths(cm[i], house[i]);
		/*cout<<small<<endl;
		   if(j>1){
			for(int i=0;i<small;i++)
			cout << cur[i] << " ";
		   }
		 if(j==1)
		  for(int i=0;i<count[0];i++)
		  cout << old[i] << " ";*/
		  /*if (j == 1) {
			  for (int i = 0; i < count1[0]; i++)
				  cur[i] = old[i];
		  }
		  cout << small << endl;
		  for (int i = 0; i < small; i++)
			  cout << cur[i] << " ";

		  cout << endl;

		  g.printAllPaths(house[i], cm[i + 1]);
		  /* cout<<small<<endl;
			 if(j>1){
			   for(int i=0;i<small;i++)
			   cout << cur[i] << " ";
			  }
			 if(j==1  )
			   for(int i=0;i<count[0];i++)
			   cout << old[i] << " ";*/
			   /*if (j == 1) {
				   for (int i = 0; i < count1[0]; i++)
					   cur[i] = old[i];
			   }
			   cout << small << endl;
			   for (int i = 0; i < small; i++)
				   cout << cur[i] << " ";

			   cout << endl;
		   }*/


}
int director(int prs_pos, int cur_pos, int nxt_pos) {
	//  R = 1......S = 0.......L = -1
	//int right = 3, left = 1 , straight = 3;
	switch (cur_pos) {
	case 0:
		if ((prs_pos == 1) && (nxt_pos == 17)) {
			return 	2;
		}
		else if ((prs_pos == 17) && (nxt_pos == 1)) {
			return 	2;
		}
		break;
	case 1:
		if ((prs_pos == 3) && (nxt_pos == 0)) {
			return 1;
		}
		else if ((prs_pos == 0) && (nxt_pos == 3)) {
			return 	3;
		}
		break;
	case 3:
		if (prs_pos == 1) {
			if (nxt_pos == 6)
				return 2;
			else if (nxt_pos == 2)
				return 1;
			else
				return 3;
		}
		else if (prs_pos == 4) {
			if (nxt_pos == 6)
				return 3;
			else if (nxt_pos == 2)
				return 2;
			else
				return 1;
		}
		else if (prs_pos == 2) {
			if (nxt_pos == 4)
				return 2;
			else if (nxt_pos == 6)
				return 1;
			else
				return 3;
		}
		else if (prs_pos == 6) {
			if (nxt_pos == 4)
				return 1;
			else if (nxt_pos == 2)
				return 3;
			else
				return 2;
		}
		break;
	case 6:
		if (prs_pos == 3) {
			if (nxt_pos == 8)
				return 2;
			else if (nxt_pos == 5)
				return 1;
			else
				return 3;
		}
		else if (prs_pos == 5) {
			if (nxt_pos == 8)
				return 1;
			else if (nxt_pos == 3)
				return 3;
			else
				return 2;
		}
		else if (prs_pos == 27) {
			if (nxt_pos == 3)
				return 1;
			else if (nxt_pos == 8)
				return 3;
			else
				return 2;
		}
		else if (prs_pos == 8) {
			if (nxt_pos == 5)
				return 3;
			else if (nxt_pos == 27)
				return 1;
			else
				return 2;
		}
		break;
	case 8:
		if (prs_pos == 6) {
			if (nxt_pos == 7)
				return 1;
			else if (nxt_pos == 9)
				return 3;
			else
				return 2;
		}
		else if (prs_pos == 7) {
			if (nxt_pos == 11)
				return 1;
			else if (nxt_pos == 6)
				return 3;
			else
				return 2;
		}
		else if (prs_pos == 9) {
			if (nxt_pos == 6)
				return 1;
			else if (nxt_pos == 11)
				return 3;
			else
				return 2;
		}
		else if (prs_pos == 11) {
			if (nxt_pos == 7)
				return 3;
			else if (nxt_pos == 9)
				return 1;
			else
				return 2;
		}
		break;
	case 11:
		if (prs_pos == 8) {
			if (nxt_pos == 10)
				return 1;
			else if (nxt_pos == 22)
				return 3;
			else
				return 2;
		}
		else if (prs_pos == 10) {
			if (nxt_pos == 13)
				return 1;
			else if (nxt_pos == 8)
				return 3;
			else
				return 2;
		}
		else if (prs_pos == 22) {
			if (nxt_pos == 8)
				return 1;
			else if (nxt_pos == 13)
				return 3;
			else
				return 2;
		}
		else if (prs_pos == 13) {
			if (nxt_pos == 10)
				return 3;
			else if (nxt_pos == 22)
				return 1;
			else
				return 2;
		}
		break;

	case 13:
		if (prs_pos == 11) {
			if (nxt_pos == 15)
				return 2;
			else if (nxt_pos == 12)
				return 1;
			else
				return 3;
		}
		else if (prs_pos == 12) {
			if (nxt_pos == 11)
				return 3;
			else if (nxt_pos == 14)
				return 2;
			else
				return 1;
		}
		else if (prs_pos == 14) {
			if (nxt_pos == 12)
				return 2;
			else if (nxt_pos == 11)
				return 1;
			else
				return 3;
		}
		else if (prs_pos == 15) {
			if (nxt_pos == 14)
				return 1;
			else if (nxt_pos == 12)
				return 3;
			else
				return 2;
		}
		break;

	case 15:
		if (nxt_pos == 16)
			return 3;
		else
			return 1;
		break;
	case 16:
		return 2;
		break;
	case 17:
		if (nxt_pos == 0)
			return 3;
		else
			return 1;
		break;
	case 18:
		if (nxt_pos == 20)
			return 3;
		else
			return 1;
		break;
	case 20:
		if (prs_pos == 18) {
			if (nxt_pos == 22)
				return 2;
			else if (nxt_pos == 21)
				return 1;
			else
				return 3;
		}
		else if (prs_pos == 19) {
			if (nxt_pos == 22)
				return 3;
			else if (nxt_pos == 21)
				return 2;
			else
				return 1;
		}
		else if (prs_pos == 21) {
			if (nxt_pos == 19)
				return 2;
			else if (nxt_pos == 22)
				return 1;
			else
				return 3;
		}
		else if (prs_pos == 22) {
			if (nxt_pos == 19)
				return 1;
			else if (nxt_pos == 21)
				return 3;
			else
				return 2;
		}

		break;
	case 22:
		if (prs_pos == 20) {
			if (nxt_pos == 25)
				return 2;
			else if (nxt_pos == 23)
				return 1;
			else
				return 3;
		}
		else if (prs_pos == 23) {
			if (nxt_pos == 20)
				return 3;
			else if (nxt_pos == 11)
				return 2;
			else
				return 1;
		}
		else if (prs_pos == 25) {
			if (nxt_pos == 20)
				return 2;
			else if (nxt_pos == 11)
				return 1;
			else
				return 3;
		}
		else if (prs_pos == 11) {
			if (nxt_pos == 20)
				return 1;
			else if (nxt_pos == 25)
				return 3;
			else
				return 2;
		}
		break;

	case 25:
		if (prs_pos == 22) {
			if (nxt_pos == 27)
				return 2;
			else if (nxt_pos == 26)
				return 1;
			else
				return 3;
		}
		else if (prs_pos == 26) {
			if (nxt_pos == 22)
				return 3;
			else if (nxt_pos == 24)
				return 2;
			else
				return 1;
		}
		else if (prs_pos == 27) {
			if (nxt_pos == 22)
				return 2;
			else if (nxt_pos == 24)
				return 1;
			else
				return 3;
		}
		else if (prs_pos == 24) {
			if (nxt_pos == 22)
				return 1;
			else if (nxt_pos == 27)
				return 3;
			else
				return 2;
		}
		break;

	case 27:
		if (prs_pos == 25) {
			if (nxt_pos == 30)
				return 2;
			else if (nxt_pos == 28)
				return 1;
			else
				return 3;
		}
		else if (prs_pos == 28) {
			if (nxt_pos == 25)
				return 3;
			else if (nxt_pos == 6)
				return 2;
			else
				return 1;
		}
		else if (prs_pos == 30) {
			if (nxt_pos == 25)
				return 2;
			else if (nxt_pos == 6)
				return 1;
			else
				return 3;
		}
		else if (prs_pos == 6) {
			if (nxt_pos == 25)
				return 1;
			else if (nxt_pos == 30)
				return 3;
			else
				return 2;
		}
		break;
	case 30:
		if (prs_pos == 27) {
			if (nxt_pos == 17)
				return 2;
			else if (nxt_pos == 31)
				return 1;
			else
				return 3;
		}
		else if (prs_pos == 31) {
			if (nxt_pos == 27)
				return 3;
			else if (nxt_pos == 29)
				return 2;
			else
				return 1;
		}
		else if (prs_pos == 17) {
			if (nxt_pos == 27)
				return 2;
			else if (nxt_pos == 29)
				return 1;
			else
				return 3;
		}
		else if (prs_pos == 29) {
			if (nxt_pos == 27)
				return 1;
			else if (nxt_pos == 17)
				return 3;
			else
				return 2;
		}
		break;
	default:
		return 5;
	}
}
void Task_1_2(void)
{

	int flag = 0, i = 0, j = 0, source, destiny, dir = 0;
	
	while (flag == 0) {
		follow_path();
		flag = find_junction();
	}
	
	right_turn_wls();

	cur_pos = 0;
	destiny = cm[j];
	source = cur_pos;
	source2destiny(source, cm[j]);
	nxt_pos = cur[++i];
	while (destiny != 0) {
		l = ADC_Conversion(1);
		c = ADC_Conversion(2);
		r = ADC_Conversion(3);

		//printf("%d       %d     %d\n", l, c, r);
		flag = find_junction();

		if (flag) {
			stop();
			prs_pos = cur_pos;
			cur_pos = nxt_pos;
			printf("prs %d cur %d nxt %d source %d destiny %d \n", prs_pos, cur_pos, nxt_pos,source,destiny);
			if (cur_pos == destiny) {
				printf("entered destiny\n");
				printf("prs %d cur %d nxt %d\n", prs_pos, cur_pos, nxt_pos);
				stop();
				_delay_ms(2500);
				//performs some ops
				if (destiny == cm[j]) {

					forward();
					velocity(250, 250);
					_delay_ms(80);
					pick();
					destiny = house[j];
				}
				else if (destiny == house[j]) {
					forward();
					velocity(250, 250);
					_delay_ms(80);
					place();
					destiny = cm[++j];

				}

				printf("cm- %d    house - %d \n", cm[j], house[j]);
				//
				back();
				velocity(250, 250);
				_delay_ms(200);
				right_turn_wls();
				printf("performed %d\n", dir);
				//sets source and destiny
				// int find_junction() 

				source2destiny(cur_pos, destiny);
				i = 0;
				nxt_pos = cur[++i];


			}

			else {
				nxt_pos = cur[++i];
				
				dir = director(prs_pos, cur_pos, nxt_pos);
				printf("dir %d\n", dir);
				switch (dir) {
				case 1:
					left_turn_wls();
					printf("performed %d\n", dir);
					break;
				case 2:
					forward();
					velocity(250, 250);
					_delay_ms(300);
					printf("performed %d\n", dir);
					break;
				case 3:
					right_turn_wls();
					printf("performed %d\n", dir);
					break;
				case 5:
					back();
					_delay_ms(300);
					right_turn_wls();
					printf("performed %d\n", dir);
					break;

				}
				if (cur_pos == 15) {
					invert_path(destiny);
					if (destiny == 16) {
						nxt_pos = 18;
						cur_pos = 16;
						house[j] = 18;
						destiny = 18;
					}
					else {
						prs_pos = cur_pos;
						cur_pos = nxt_pos;
						nxt_pos = cur[++i];
					}

					printf("prs %d cur %d nxt %d \n exited kadu  house[j]=%d\n", prs_pos, cur_pos, nxt_pos,house[j]);
				}
				
			}
			//director(prs_pos, cur_pos, nxt_pos);
			/*printf("junction %d\n", ++i);
			if (i == 1)
				right_turn_wls();*/
		}
		else {
			follow_path();

		}


	}
	//write your task 1.2 logic here
	if (destiny == 0) {
		forward();
		_delay_ms(30);
		left();
		_delay_ms(180);
		while ((l == w && c == b && r == w) != 1) {
			left();
			l = ADC_Conversion(1);
			c = ADC_Conversion(2);
			r = ADC_Conversion(3);
		}
		while (1) {
			flag = find_junction();
				if (flag) {
					break;
				}
				else
					follow_path();
		}
	}
	while (1) {
		stop();
		_delay_ms(5000);
	}
}

