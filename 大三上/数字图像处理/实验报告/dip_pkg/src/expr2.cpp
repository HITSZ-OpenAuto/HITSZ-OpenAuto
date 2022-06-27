//
// Created by handleandwheel on 2021/10/8.
//

#include "dip_pkg/expr2.hpp"

using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "expr2_node");
	ros::NodeHandle n;
	Expr2Node expr2Node(n);
	
	return 0;
}