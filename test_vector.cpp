/* C++ code to demonstrate a 2D vector
with elements(vectors) inside it. */
#include <iostream>
#include <vector>
using namespace std;

int main()
{
	/*
	Below we initialize a 2D vector
	named "vect" on line 12 and then
	we declare the values on
	line 14, 15 and 16 respectively.
	*/
	
	vector<vector<int>> vect
	{
		{1, 2, 3},
		{4, 5, 6},
		{7, 8, 9}
	};
	
	/*
	Now we print the values that
	we just declared on lines
	14, 15 and 16 using a simple
	nested for loop with the help of iterator.
	*/
	
	/*
	vector<vector<int>> vect
	We can divide this declaration to two parts, which will
	help us to understand the below concepts.
	
	1. vect is a 2D vector consisting multiple elements of type vector<int>.
	2. vector<int> is a 1D vector consisting of multiple int data.
	
	So we can use iterator provided by STL instead of
	i,j variable used in for loop. It can reduce the error which can
	happen wrt to i, j operations(i++, j++)	
	
	In the below code we are using iterator to access the vector elements.
	1. We are getting vect1D vectors of type vector<int> from the 2D vector vect.
	2. We are getting int elements to x from the vector<int> vect 1D vector.
	
	*/
	
	for (vector<int> vect1D : vect)
	{
		for (int x : vect1D)
		{
			cout << x << " ";
		}
		cout << endl;
	}

	return 0;
}

