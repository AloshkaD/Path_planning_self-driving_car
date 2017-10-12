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
using namespace std;

// for convenience
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
//calculate the eucledian distance between two points
double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
// we have a map of all points surround our highway, we need to see which one is 
// closest to us
int ClosestWaypoint(double x, double y, const vector<double> maps_x, const vector<double> maps_y)
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
// find the next way point. Use the orientation angel to find the next point 
int NextWaypoint(double x, double y, double theta, const vector<double> maps_x, const vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
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

	double center_x = 1000-maps_x[prev_wp];
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
vector<double> getXY(double s, double d, const vector<double> maps_s, const vector<double> maps_x, const vector<double> maps_y)
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

  // we want to start in lane one. with lane 0 being at left ost
  	int lane = 1;
  // reference velocity, stay below speeed limit
	double ref_vel = 0.0;
	// num of lanes in the simulator
	int num_lanes = 3;

	h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

			  
			
			
			// outer vector is each car on the road and inner vector is the parameters of the car
			// Sensor Fusion Data, a list of all other cars on the same side of the road.
			// ["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's 
			//x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s,
			//car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.
			vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
			//previous_size it comes from the previous path size
			int prev_size = previous_path_x.size();
			

			
			// Sensor fusion
			// the simulator reports all the cars in the surrounding, so let's use this info
			// we will go through the sensor fusion list and see if the car is in our lane or not

			if (prev_size > 0)
			{
				car_s = end_path_s;

			}
			
			//////////////////////////////////////////////////////////////////////////////////

			bool too_close = false;
			vector<bool> busy_lanes = {false, false, false};
			//find ref_v to use
			for (int i =0; i< sensor_fusion.size(); i++){
				//d value help us know in what lane the car is 
				// remember car_s (distance along the lane) and
				// car_d (distance along the width of the lane)..
				float d = sensor_fusion[i][6];
				
				//we care about the car that's in our lane 
				// +- 2 difference in the range because each lane is 2 meters
				// multiplying by the lane value make sure that we are at the center of the lane
				// if it is in our lane let's check its speed
				double vx = sensor_fusion[i][3];
				double vy = sensor_fusion[i][4];
				double check_speed = sqrt(vx*vx+vy*vy); // calculate the velocity magnitude of it
					// we need the s value to know if the car is close to us or not. S in fernet coordinate
				double check_car_s = sensor_fusion[i][5];
					//if using previous points can project s value out in time
					// this is needed because if we are using the previous point we are not there yet
					// so it's like looking at what the car will be in the future 
				check_car_s +=((double)prev_size*.02*check_speed);
// check to see if the three lanes are free 
				for (int j = 0; j < 3; j++){
                    // if lane is busy loop back to find another free one
					if (busy_lanes[j]) {
						continue;
					}
					//make sure we are at the lane center
					//if(d < (2+4*lane+2) && d > (2+4*lane-2))
					if ( d> (4*j) && d<(4+4*j))
					{
					//incase a free one is found
					//chack value grater than mine and s gap 
					//so check if our car s is close to other cars' s
					// first comparision if it is infront of us, second if it's gap is less than 30 meters
					
					if (lane == j){
						busy_lanes[j] = ((check_car_s > car_s)&& ((check_car_s-car_s)<(30.0 * ref_vel / 49.5)));
					
						// then we need to take action
						// lower speed
						// change lanes (flag)
						// let's set reference velocity down
						//ref_vel = 29.5;
						//too_close = true;
						
						// lets do some lane changing
						// check what lane we are, lane 0 we cahnt overtake left, lane 2 
						// we cannot overtake right
						
						
					
				}
				else 
				{
					// there is a cas a head of us on the other lanes
					if (check_car_s > car_s){
						// this car is within 30 meters or less
						busy_lanes[j] = (check_car_s - car_s) <(30.0 * ref_vel / 49.5);
					}
					else{
						// if the car is behind us
						// the istance is within 20 meters or less 
						busy_lanes[j] =((car_s - check_car_s) < 20.0);
					}

				}
				
			}
		
				}
			}

		cout << "BUSY LANES: " << busy_lanes[0] << " " << busy_lanes[1] << " " << busy_lanes[2]  << "\n";
		
 
				
		// fallsafe		
		//if(  busy_lanes[0] && busy_lanes[1] && busy_lanes[2])
		//{
			//ref_vel -=.224;

		//}
		//else if(ref_vel < 49.5)
			//{
				//ref_vel += .224;
			//}
		// is the lane I'm on is busy	
		if( busy_lanes[lane]  )
		{
					 
			ref_vel -=.224;

 
			if ( lane > 0 && !busy_lanes[lane -1])
			{
				lane -= 1;
			}
			else if (lane < 2 && !busy_lanes[lane + 1])
			{
				lane += 1; 
			}

		}
		 
		else {
			ref_vel = min(ref_vel + 0.224, 49.5);
		}


			///////////////////////////////////////////////////////////////////////////////////

			//create a list of widely spaced (x,y) waypoints, evenly spaced at 30m and interpolate them with spline

			vector<double> ptsx;
			vector<double> ptsy;

			//Keep track of the reference state, x,y,yaw
			// eith the previous path point or the starting point will be referenced
			double ref_x = car_x;
			double ref_y = car_y;
			double ref_yaw = deg2rad(car_yaw);

			// // use car as a starting reference if previous size is almost empty.
			if(prev_size <2)
			{
				// make sure the path is tangent
				double prev_car_x = car_x - cos(car_yaw);
				double prev_car_y = car_y - sin(car_yaw);
				ptsx.push_back(prev_car_x);
				ptsx.push_back(car_x);

				ptsy.push_back(prev_car_y);
				ptsy.push_back(car_y);

			}
			//////////////////////////////////////////////////////////
			// make sure the point is tangent
			// use the previous path's end point as starting refernce
			else{
				// redefine reference state as previous path and point
				ref_x=previous_path_x[prev_size-1];
				ref_y=previous_path_y[prev_size-1];	
				double ref_x_prev=previous_path_x[prev_size-2];
				double ref_y_prev=previous_path_y[prev_size-2];
				ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

				//two points that make the path tangent to the previous path's end point
				ptsx.push_back(ref_x_prev);
				ptsx.push_back(ref_x);

				ptsy.push_back(ref_y_prev);
				ptsy.push_back(ref_y);

			}
			////////////////////
			// 30,60,90 spaced points ahead of the starting reference
			vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
			vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);  
			vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
			
			ptsx.push_back(next_wp0[0]);
			ptsx.push_back(next_wp1[0]);
			ptsx.push_back(next_wp2[0]);

			ptsy.push_back(next_wp0[1]);
			ptsy.push_back(next_wp1[1]);
			ptsy.push_back(next_wp2[1]);
			//similar to what we did in mpc, shift and rotation
			for(int i = 0; i < ptsx.size(); i++)
			{
				double shift_x = ptsx[i]-ref_x;
				double shift_y = ptsy[i]-ref_y;

				ptsx[i] = (shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
				ptsy[i] = (shift_x *sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));


			}


//////////////////////////////////////////////////////////////////////////////////////

			// create a spline
			//cout<< "ptsx" << ptsx<< "\n";
			//cout<< "ptsy" << ptsy<< "\n";
			////////////////////////
			tk::spline s; 
			// x and y points for the spline
			s.set_points(ptsx,ptsy);

			// actual x and y points for the path planner

			vector<double> next_x_vals;
			vector<double> next_y_vals;

			//previous path points to start with from last time
			for(int i = 0; i< previous_path_x.size(); i++)
			{
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
			}
			// doing the calculation from the visual aid
			double target_x = 30.0;
			double target_y = s(target_x);
			double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

			double x_add_on = 0;

			// output 50 points and fill up the rest 
			for (int i = 1; i <= 50-previous_path_x.size(); i++){
			// since the values are in miles per hour we need to convert into meters per seconds, hence divide by 2.24
				double N = (target_dist/(.02*ref_vel/2.24));
				double x_point = x_add_on+(target_x)/N;
				double y_point = s(x_point);

				x_add_on = x_point;
				
				double x_ref = x_point;
				double y_ref = y_point;
				// shift and rotation to go back to global coordinate from local
				x_point = (x_ref *cos(ref_yaw)-y_ref*sin(ref_yaw));
				y_point = (x_ref *sin(ref_yaw)+y_ref*cos(ref_yaw));
				x_point += ref_x;
				y_point += ref_y;
				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);

			}
////////////////////////////

			json msgJson;
            /*
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
			double pos_x;
			double pos_y;
			double angle;
			int path_size = previous_path_x.size();

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
			// double dist_inc = 0.2;
			for(int i = 0; i < path_size; i++)
			  {
				//next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
				//next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
			  } 
			  if(path_size == 0)
			  {
				  pos_x = car_x;
				  pos_y = car_y;
				  angle = deg2rad(car_yaw);
			  }
			  else
			  {
				  pos_x = previous_path_x[path_size-1];
				  pos_y = previous_path_y[path_size-1];
	
				  double pos_x2 = previous_path_x[path_size-2];
				  double pos_y2 = previous_path_y[path_size-2];
				  angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
			  }
	
			  double dist_inc = 0.3;
			  for(int i = 0; i < 50-path_size; i++)
			  {    
				  next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
				  next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
				  pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
				  pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
			  }
	
	//////////////////////////////////////////////////////////////////////////////////////
	        */
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
