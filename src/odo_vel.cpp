#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include <string>
#include <sstream>

using namespace std;

class odom
{
	double l,r;
	double total;
public:
	odom(int);
	void findTotal();
};

odom::odom(int data)
{
	// total = 1.5;
	// hours = data - 1;
	bool loaded = 1;
  	loaded = ros::param::get("/eod/physics/l", l);
 	loaded = loaded & ros::param::get("/eod/physics/r", r);
	if(!loaded) ROS_ERROR("Failure in loading tf parameters");
}

void odom::findTotal(void)
{
	while (hours > 0)
	{
		total += 1.2;
		hours--;
	}
	cout << "You owe: $" << total << endl;
}

int main()
{
	string entered;
	int length;
	
	cout << "How many hours were you parked: ";
	cin >> entered;
	stringstream(entered) >> length;
	
	odom
 total(length);
	total.findTotal();
	
	system("pause");

	return 0;	
}