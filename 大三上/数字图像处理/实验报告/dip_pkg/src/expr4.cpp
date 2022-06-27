//
// Created by handleandwheel on 2021/10/24.
//

#include "dip_pkg/expr4.hpp"

using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "expr4_node");
	ros::NodeHandle n;
	Expr4Node expr4Node(n);
	
	return 0;
}