//
// Created by handleandwheel on 2021/9/23.
//

#include "dip_pkg/expr1.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "expr1_node");
	ros::NodeHandle n;
	Expr1Node expr1Node(n);
	return 0;
}