
#include <vector>
#include <math.h>
#include <fstream>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>

// for convenience
using namespace std;
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp]; //why 1000 and 2000? probably (1000,2000) is center of the map
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}
vector<double> transf_to_car_coord(double x,double y,double car_x,double car_y,double car_theta)
{
  //transforms global coordiates (x,y) to local car coordinates with the car being at position (0,0) and heading in x-direction
  //returns a vector of doubles containing the x-coordinate as first and the y-coordinate as second element
  vector<double> new_coord;
  double new_x,new_y;
  new_x = (x-car_x)*cos(car_theta) +(y-car_y)*sin(car_theta);
  new_y = -(x-car_x)*sin(car_theta) +(y-car_y)*cos(car_theta);
  new_coord.push_back(new_x);
  new_coord.push_back(new_y);
  return new_coord;
};
vector<double> transf_to_global_coord(double x,double y,double car_x,double car_y,double car_theta)
{
  //reverses transf_to_car_coordinates
  //transforms local car coordinates back to global coordinates
  //returns a vector containing transformed x and y coordinates
  vector<double> new_coord;
  double new_x,new_y;
  new_x = x*cos(-car_theta) +y*sin(-car_theta)+car_x;
  new_y = -x*sin(-car_theta) +y*cos(-car_theta)+car_y; 
  new_coord.push_back(new_x);
  new_coord.push_back(new_y);
  return new_coord;
};

void shifting_lane(int goal_lane,double curr_d, double curr_s,double &end_s ,double curr_velo,vector<double> &x_pts,vector<double> &y_pts,vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y)
{
  //function to complete a lane_shift that has already begun
  double goal_d = goal_lane*4 -2;
  double remaining_d = abs(goal_d-curr_d);
  double d_vel = 4./2.5; // calculate velocity in d direction for the lane shift: 4m/2.5s; 2.5s being the target time to complete it
  double remaining_time = remaining_d/d_vel; //calculate the time to complete lane shift
  double remaining_way = remaining_time * curr_velo;// way to complete
  end_s = curr_s+remaining_way; //upadate global variable containing the s-value at the end of the lane shift
  vector<double> xy; 
  xy = getXY(end_s+10,goal_lane*4-2 ,map_waypoints_s,map_waypoints_x,map_waypoints_y);// some way beyond the endpoint to prevent high jerk
  x_pts.push_back(xy[0]);
  y_pts.push_back(xy[1]);
  
  xy = getXY(end_s+30, goal_lane*4-2,map_waypoints_s,map_waypoints_x,map_waypoints_y);
  x_pts.push_back(xy[0]);
  y_pts.push_back(xy[1]);
  xy = getXY(end_s+60, goal_lane*4-2,map_waypoints_s,map_waypoints_x,map_waypoints_y);
  x_pts.push_back(xy[0]);
  y_pts.push_back(xy[1]);
};

vector <vector <double>> other_cars(vector <vector <double>> sensor_fusion)
{
  //returns a vector containing for each car a vector with its current lane, s and velocity
  vector <vector<double>> ret;
  for(int i=0;i<sensor_fusion.size();i++)
  {
    double d = sensor_fusion[i][6];
    double lane;
    if(0<d && d<4)
    {
      lane = 1;
    }
    else if(4<=d&& d<8)
    {
      lane = 2;
    }
    else if(8<d && d<= 12)
    {
      lane = 3;
    }     
    double s = sensor_fusion[i][5];
    //calculate velocity of the car
    double vel = sqrt(sensor_fusion[i][3]*sensor_fusion[i][3]+ sensor_fusion[i][4]*sensor_fusion[i][4]);
    vector<double> car;
    car.push_back(lane);
    car.push_back(s);
    car.push_back(vel);
    ret.push_back(car);
  }
  return ret;
};
bool slow_car_ahead(vector <vector <double>> other_cars,double curr_s,int curr_lane,double max_vel)
{
  //checks wheter there is a slower car in the same lane and close ahead
  //thus helps to determine when a lane shift should be considered
  bool car_ahead=false;
  for(int i=0;i<other_cars.size();i++)
  {    
    if(other_cars[i][0]==curr_lane && 0<(other_cars[i][1]-curr_s) && (other_cars[i][1]-curr_s)<60 && other_cars[i][2]< max_vel)
    {
      car_ahead = true;
    }
  }
  return car_ahead;
};
double lane_speed(vector <vector <double>> other_cars,double curr_s,int lane,double max_vel)
{
  //calculates the speed of the car that is closest ahead in the respective lane
  double lane_speed;
  bool car_in_lane=false;
  double min_s=1000; //some high value  
  for(int i=0;i<other_cars.size();i++)
  {    
    if(other_cars[i][0]==lane && 0<(other_cars[i][1]-curr_s) && (other_cars[i][1]-curr_s)<50 && (other_cars[i][1]-curr_s)<min_s && other_cars[i][2]< max_vel ) // check wheter there is a car in the same lane, less than 50m away, slow and in case there are more than one only consider the closest to be able to adapt to its speed
    {
      lane_speed = other_cars[i][2];
      min_s = other_cars[i][1]-curr_s;
      car_in_lane = true;
    }
  }
  if(!car_in_lane)
  {
    lane_speed = max_vel; // if no car is detected maximal speed is possible
  }
  return lane_speed;
};


bool laneshift_possible(int curr_lane, int shifted_lane, vector <vector <double>> other_cars, double curr_s,double curr_vel)
{
  // checks wheter shifted lane is neighbouring and free
  if( abs(curr_lane - shifted_lane)!=1)
  {    
    return false;
  }
  for(int i=0;i<other_cars.size();i++)
  {    
    if(other_cars[i][0]==shifted_lane && -5<(other_cars[i][1]-curr_s) && (other_cars[i][1]-curr_s)<40)
    {
      return false;      
    }
    if(other_cars[i][0]==shifted_lane && -50<(other_cars[i][1]-curr_s) && (other_cars[i][1]-curr_s)<=-5 &&((curr_vel-other_cars[i][2])*2.5 - 10 -other_cars[i][1]+curr_s)<0)
    {
      return false;     
    }
  }
  return true;
};
       
bool laneshift_beneficial(int shifted_lane, vector <vector <double>> other_cars, double speed_curr_lane, double curr_s)
{
  //checks wheter it is beneficial to change lanes
  bool ret=true;
  for(int i=0;i<other_cars.size();i++)
  {    
    if(other_cars[i][0]==shifted_lane && 0<(other_cars[i][1]-curr_s) && (other_cars[i][1]-curr_s)<20 && other_cars[i][2]<= speed_curr_lane )
    {
      ret=false;      
    }
  }
  return ret;
};

bool try_lane_shift(int & curr_lane,int goal,vector <vector <double>> other_cars, double speed_curr_lane, bool &lane_shift, int & goal_lane, double curr_s, double curr_vel)
{
  // initiates a lane shift if possible and beneficial
  bool ret = false;
  if(laneshift_possible(curr_lane,goal,other_cars, curr_s,curr_vel) && laneshift_beneficial(goal_lane,other_cars,speed_curr_lane,curr_s))
  {
    lane_shift =true;
    goal_lane = goal;
    ret = true;
    curr_lane = goal_lane;
   
  }  
  return ret;
};

vector <double> lane_values(vector <vector <double>> other_cars, double curr_s)
{
  //determine values of the lanes where lanes with more free space ahead are better
  vector <double> values{1000,1000,1000};
  for(int i=1;i<4;i++)
  {
    double curr_value, dist, speed;
    for(int j=0;j<other_cars.size();j++)
    {
      if( -5 >(other_cars[i][1]-curr_s) || (other_cars[i][1]-curr_s)>300)
      {
        curr_value =1000;
      } 
      else if(  (other_cars[i][1]-curr_s)>=10)
      {
        curr_value = (other_cars[i][1]-curr_s);//+10*other_cars[i][2];
      }
      else if(-5<(other_cars[i][1]-curr_s)  && other_cars[i][1]-curr_s <10)
      {
        curr_value = 0;
      }
      if(other_cars[j][0] ==i && (curr_value < values[i-1]))
      {
        values[i-1] = curr_value;
      }
    }
  }
  return values;
};
vector <int> classify_lanes(vector <double> values)
{  
  // order the lanes based on its values and return a list of integers where the first element is the number of the best lane and last element of the list contains the worst lane
  vector <int> ret_vec{2,1,3};
  if(values[0]>values[1] && values[0]>=values[2] && values[1] >= values[2])
  {
    ret_vec[0]=1;
    ret_vec[1]=2;
    ret_vec[2]=3;
  }
  else if(values[0]>values[1] && values[0]>=values[2] && values[1] < values[2])
  {
    ret_vec[0]=1;
    ret_vec[1]=3;
    ret_vec[2]=2;
  }
  else if(values[2]>values[1] && values[2]>values[0] && values[1] < values[0])
  {
    ret_vec[0]=3;
    ret_vec[1]=1;
    ret_vec[2]=2;    
  }
  else if(values[2]>values[1] && values[2]>values[0] && values[1] >= values[0])
  {
    ret_vec[0]=3;
    ret_vec[1]=2;
    ret_vec[2]=1;
  }
  else if(values[1]>=values[0] && values[1]>=values[2] && values[0] < values[2])
  {
    ret_vec[0]=2;
    ret_vec[1]=3;
    ret_vec[2]=1;
  }
  else //if(values[1]>=values[0] && values[1]>=values[2] && values[0] >= values[2])
  {
    ret_vec[0]=2;
    ret_vec[1]=1;
    ret_vec[2]=3; 
  }
  return ret_vec;
};


