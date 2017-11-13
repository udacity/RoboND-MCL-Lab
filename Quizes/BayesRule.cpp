#include <iostream>
using namespace std;

int main() {
	//Given P(C), P(POS|C) and P(POS|¬C)
	double a = 0.0001; //P(C) = 0.001
	double b = 0.8   ; //P(POS|C) = 0.8
	double c = 0.1   ; //P(POS|¬C) = 0.1
	
	//Compute P(¬C) and P(C|POS)
	double d = 1-a; //P(¬C)
	double e =  (b*a)/((a*b)+(d*c))  ; //P(C|POS)
	
	cout << "P(C|POS)= " <<    e    << endl;
	return 0;
}
