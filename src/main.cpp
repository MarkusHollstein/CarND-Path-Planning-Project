
#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "helper.cpp"


using namespace std;




int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
  bool lane_shift=false; //define a variable to save the information wheter a lane_shift is going on and one to save the goal lane
  int goal_lane = 0;
  double end_shift_s;//save the s value of the end of a lane shift to determine wheter it is completed
  int curr_lane = 2;
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&end_shift_s,&goal_lane, & lane_shift, & curr_lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
          
          	//create points to make a spline 
          	vector<double> x_pts;
          	vector<double> y_pts;
          	double prev_x;
          	double prev_y;
          	// create variable to calculate the speed at the end of the previous path (if existing; else use car_speed)
          	double  curr_speed;
          	if(previous_path_x.size()<2) // if not enough points from the previous path are remaining use an artificial previous point and the current position to create a smooth path:
          	{
          		//some artificial previous point in order to make a smooth transition
          		prev_x = car_x-cos(car_yaw);
          		prev_y = car_y-sin(car_yaw);
          		x_pts.push_back(prev_x);
          		y_pts.push_back(prev_y);
          		//current car coordinates
          		x_pts.push_back(car_x);
          		y_pts.push_back(car_y);
              	end_path_s = car_s;// if there is no previous path set the end of the previous path to the car position
              	curr_speed = car_speed;
            }
          	else// if there are points from the previous path in my list use the last two to ensure a smooth path
            { 
              	prev_x = previous_path_x[previous_path_x.size()-2];
              	prev_y = previous_path_y[previous_path_y.size()-2];
              	x_pts.push_back(prev_x);
              	x_pts.push_back(previous_path_x[previous_path_x.size()-1]);
              	y_pts.push_back(prev_y);
              	y_pts.push_back(previous_path_y[previous_path_y.size()-1]);
              	double  old_dist;
          		//calculate speed at the end of the previous path by calculating the distance between the last two points and dividing by 0.02
          		old_dist = distance(previous_path_x[previous_path_x.size()-1],previous_path_y[previous_path_y.size()-1],previous_path_x[previous_path_x.size()-2],previous_path_y[previous_path_y.size()-2]);
          		curr_speed = old_dist/0.02;//0.02 is the time it shall take to move from one waypoint to the next
            }
          	if(lane_shift) // check wheter lane shift is going on
            {
              	if(end_path_s > end_shift_s)//check wheter lane shift was completed within the already calculated path
                {
                  lane_shift = false;
                  curr_lane = goal_lane;
                  // lane shift finished; then create 3 further points in the current lane for calculating the spline
                  vector<double> xy;
          		  xy = getXY(end_path_s+30, curr_lane*4-2,map_waypoints_s,map_waypoints_x,map_waypoints_y);
          		  x_pts.push_back(xy[0]);
          		  y_pts.push_back(xy[1]);
          		  xy = getXY(end_path_s+60, curr_lane*4-2,map_waypoints_s,map_waypoints_x,map_waypoints_y);
          		  x_pts.push_back(xy[0]);
          		  y_pts.push_back(xy[1]);
          		  xy = getXY(end_path_s+90, curr_lane*4-2,map_waypoints_s,map_waypoints_x,map_waypoints_y);
          		  x_pts.push_back(xy[0]);
          		  y_pts.push_back(xy[1]);
                }
              	else 
                { 
                  	//for an ongoing laneshift calculate more points for the spline
              		shifting_lane(goal_lane,end_path_d, end_path_s,end_shift_s ,curr_speed,x_pts,y_pts,map_waypoints_s, map_waypoints_x, map_waypoints_y);
                }
            }          	
            else//no laneshift so just create points in the same lane for the spline
            {
          		vector<double> xy;
          		xy = getXY(end_path_s+30, curr_lane*4-2,map_waypoints_s,map_waypoints_x,map_waypoints_y);
          		x_pts.push_back(xy[0]);
          		y_pts.push_back(xy[1]);
          		xy = getXY(end_path_s+60, curr_lane*4-2,map_waypoints_s,map_waypoints_x,map_waypoints_y);
          		x_pts.push_back(xy[0]);
          		y_pts.push_back(xy[1]);
          		xy = getXY(end_path_s+90, curr_lane*4-2,map_waypoints_s,map_waypoints_x,map_waypoints_y);
          		x_pts.push_back(xy[0]);
          		y_pts.push_back(xy[1]);              	
            }
          	//transform x_pts and y_pts to local car coordinates where it (hopefully) defines a function (else same x values are possible; for example (assuming a circle) at 3 and 9 o'clock)
          	vector<double> x_trans;
          	vector<double> y_trans;
          	double yaw = atan2(y_pts[1]-y_pts[0],x_pts[1]-x_pts[0]);//necessary for transforming between global and local car coordinates
          	          	
          	for(int i = 0 ;i<x_pts.size();i++)
            {
              vector<double> transf;              
              transf = transf_to_car_coord(x_pts[i],y_pts[i],x_pts[1],y_pts[1],yaw);
              x_trans.push_back(transf[0]);
              y_trans.push_back(transf[1]);              
            }
          	next_x_vals.insert(next_x_vals.begin(),previous_path_x.begin(),previous_path_x.end());
          	next_y_vals.insert(next_y_vals.begin(),previous_path_y.begin(),previous_path_y.end());
          	
          	tk::spline s; //with the transformed coordinates create a spline fitting the above created points
   			s.set_points(x_trans,y_trans);

          	//now create a trajectory completing the previous path and using the above created spline
          	double target_x = 30.; //length of the trajectory in x-direction
          	double target_y= s(target_x); //y-value of the endpoint (in car coordinates)
          	double target_dist = distance(0,0,target_x,target_y); //total distance
          	double x_add =0.;
          	double max_accel = 8.5;// acceleration is supposed to remain under 10 m/s^2
          	
          	double max_speed = 49./2.25;//set maximal speed to 49 mph and transform to m/s
           	double target_speed= max_speed; //define a speed the car is supposed to drive; if possible this will be the maximal speed, if there is slow traffic ahead it will be adjusted below
			double curr_accel;
          	vector <vector <double>> cars = other_cars(sensor_fusion); //calculate lane, s-value and speed of all other cars
          	bool traffic = slow_car_ahead(cars,car_s,curr_lane,max_speed); //check wheter there is another car close ahead
          	double lane_spe = lane_speed(cars,car_s, curr_lane, max_speed); //calculate a maximal speed that is save without risking to  crash into a car ahead
          	
            bool shift = false;
          	if(traffic && (!lane_shift))
            {              
              vector <double>  values = lane_values(cars,car_s); //calculate values of the lanes to determine which is best
              vector <int> classification  = classify_lanes( values);

              // try shifting into the best lan               
              shift = try_lane_shift(curr_lane,classification[0],cars,lane_spe,lane_shift,goal_lane,car_s,car_speed);                
              
              if( (!shift) )
              {                
                // if the best lane was not possible try shifting to the second best
                shift = try_lane_shift(curr_lane,classification[1],cars,lane_spe,lane_shift,goal_lane,car_s,car_speed);                
              }
              if(!shift)
              {
                // if it was not reasonable to shift lanes adjust your target speed to the car ahead
                target_speed = lane_spe;
              }
            }
          	          	
    		for(int i = previous_path_x.size(); i < 100; i++) //fill up the previous path to get 100 points
    		{
              	// check if the current speed is close to the maximal speed and adjust acceleration accordingly
              	if(fabs(curr_speed-target_speed)<.05)
                {
                  curr_accel = 0;
                }
              	else if(curr_speed<target_speed)
                {
                  curr_accel = max_accel;
                }
              	else if(curr_speed>target_speed)
                {
                  curr_accel = - max_accel;
                }
              	
              	curr_speed += curr_accel*.02;// current speed (at the end of the previous path) was calculated above; update it here using the current acceleration and the time 0.02 between two steps
              	double N = target_dist/(.02*curr_speed);//calculate how many steps it would take with the current distance to complete the target distance
              	double x_point = x_add + target_x/N;// next x-point 
              	double y_point = s(x_point);//next y-point
              
              	x_add = x_point;
              	vector<double> retrans;
              	retrans = transf_to_global_coord(x_point,y_point,x_pts[1],y_pts[1],yaw);//retransform the new point to global coordinates 
              	
              	next_x_vals.push_back(retrans[0]);
              	next_y_vals.push_back(retrans[1]);                         

    		}

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;
          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
