//
// Created by handleandwheel on 2021/10/24.
//

#include "dip_pkg/expr3.hpp"

using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "expr3_node");
	ros::NodeHandle n;
	Expr3Node expr3Node(n);
	
	return 0;
}