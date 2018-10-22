#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <algorithm>  
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

//Added by Komer
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

// returns lane number, given distance d.
int get_lane(double d) {
    int lane = 0;
    if (d > 8.0)
        lane = 2;
    else if (d > 4.0)
        lane = 1;
    else
        lane = 0;
    
    return (lane);
}


//The following function computes the costs associated with the adjacent lane switching.

double get_cost (double back_s_diff, double back_speed_diff, double front_s_diff, double front_speed_diff) {
    double cost = 0.;
    
    // less space in the front may be fine, reducing speed is an option
    // 30 meters was not sufficient
    if (front_s_diff < 50.)
        cost += 1.0;
    if (front_speed_diff < 5.)
            cost += 0.5;
    
    
    //more space is needed in the back;  trying to optimize - reduced from 60 to 50 meters.
    if (back_s_diff > -60.0) {
            cost += 1.0;
            if (back_s_diff > -40)
                cost += 0.20;
            if (back_speed_diff > 0.0)
                cost += 1.0;
    } //ignore back car, if self-lane is same lane;
    return (cost);
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

  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
            //double car_id = j[1]["id"];
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
            /***
            ["sensor_fusion"] A 2d vector of cars and then that car's [
            0 - car's unique ID, 
            1 - car's x position in map coordinates, 
            2 - car's y position in map coordinates, 
            3 - car's x velocity in m/s, 
            4 - car's y velocity in m/s, 
            5 - car's s position in frenet coordinates, 
            6 - car's d position in frenet coordinates.
            ***/
            
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;




          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

           
            // plugging in  starter code from class notes - Komer
            
            //static int lane = 1;
            int lane = get_lane(car_d); 
            

                
            
            //cout << "Ego car d: " << car_d << "; lane: " << lane << endl;
            
            static double ref_speed = 0.0; // mph
            static double old_ref_speed = 0.0; // to track old ref_speed
            static double previous_speed_diff = 0.0;  // to avoid sudden change
            static double max_speed_change = 0.;
            
            //Added to track lane transition.
            static bool lane_switch_initiated = false;
            static int num_lane_switches = 0;
            static int  lane_proposed = 1;
            static int  min_lane_keep_period = 250; // stay in the lane for minimum 5 sec. to avoid jerk
            
            int prev_path_size = previous_path_x.size();
            
            if (prev_path_size > 0) {
                car_s = end_path_s;
            }
            
            double lane_info [3][4] = { { -99999., 50.0, 99999., -50.00},
                                        { -99999., 50.0, 99999., -50.00},
                                        { -99999., 50.0, 99999., -50.00} };
           
            
            //cout << "ego car s: " << car_s << "; d: " << car_d << "; lane: " << lane << endl;
            for (int i=0; i<sensor_fusion.size(); i++) {
                //check whether any car is closer to the same lane.
                int a_car_id = sensor_fusion[i][0];
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double a_car_s = sensor_fusion[i][5];
                float a_car_d = sensor_fusion[i][6];

                double a_car_speed = sqrt(vx*vx + vy*vy);

                    
                //Compute next frame location s-maps_s
                a_car_s += prev_path_size * 0.02 * a_car_speed;
                //cout << "near car loc s: " << a_car_s << "; speed: " << a_car_speed << endl;
                    
                int a_car_lane = get_lane (a_car_d);
                

                //capture the car information as required
                double s_diff = a_car_s - car_s;
                double speed_diff = a_car_speed - ref_speed;
                
                //cout << "a_car_id: " << a_car_id << "; s: " << a_car_s << "; d: " << a_car_d << "; lane: " << a_car_lane << "; speed: " << a_car_speed << endl;
                //cout << "s_diff: " << s_diff << "; speed_diff: " << speed_diff << endl;
                 if ((s_diff > 0.) && (s_diff < 10.) && (lane == a_car_lane)) {  // if front car is too close in the same lane
                        
                        cout << "DANGER: Front car id: " << a_car_id << "; s: " << s_diff << "; speed: " << car_speed << "; lane: " << lane << endl;
                        time_t rawtime;
                        time (&rawtime);
                        printf ("The current local time is: %s", ctime (&rawtime));
                }
              
                if (s_diff <= 0) {  // a car is behind
                    if (s_diff > lane_info[a_car_lane][0]) { // a closely behind car
                        lane_info[a_car_lane][0] = s_diff;
                        lane_info[a_car_lane][1] = speed_diff;
                    } //otherwise no need to update
                } 
                else { // a front car
                    if (s_diff < lane_info[a_car_lane][2]) {
                        lane_info[a_car_lane][2] = s_diff;
                        lane_info[a_car_lane][3] = speed_diff;
                    }
                }
            } // end of for all 
            
            if (min_lane_keep_period > 0)
                min_lane_keep_period--;
            
            lane = get_lane(car_d);

            //Added to handle lane switch smoothly
            if (lane_switch_initiated == true) {
                if (lane == lane_proposed) {
                    num_lane_switches++;
                    //cout << "lane_switch_initiated: " << lane_switch_initiated << "; lane proposed: " << lane_proposed << "; current lane: " << lane << "; num lane switches: " << num_lane_switches << endl;
                    lane_switch_initiated = false;
                    min_lane_keep_period = 500; // 10 seconds =  500 * 20 ms to avoid lateral jerks
                }
                    
                else {
                    //cout << "lane_switch_initiated: " << lane_switch_initiated << "; lane proposed: " << lane_proposed << "; current lane: " << lane << endl;
                }
            }
            //cout << "Lane Info. Matrix current lane: " << lane << "; lane_switch_initiated: " << lane_switch_initiated << "; proposed_lane: " << lane_proposed << endl;
            //cout << "{" << lane_info[0][0] << ", " << lane_info[0][1] << ", " << lane_info[0][2] << ", " << lane_info[0][3] << "}" << endl;
            //cout << "{" << lane_info[1][0] << ", " << lane_info[1][1] << ", " << lane_info[1][2] << ", " << lane_info[1][3] << "}" << endl;
            //cout << "{" << lane_info[2][0] << ", " << lane_info[2][1] << ", " << lane_info[2][2] << ", " << lane_info[2][3] << "}" << endl;            
            
            double lane_cost = 0.0;
            bool lane_switch = false;
            

            //Added lane switch initiated condition to avoid cost checkings during translation, which may occasionally result in oscillatins and back and forth out of lanes 
            if ((lane_switch_initiated == false) && (lane_info[lane][2] < 50.0) && (min_lane_keep_period == 0)) {
                lane_cost += 1.0;
                //cout << "front car s_diff: " << lane_info[lane][2] << "; speed_diff: " << lane_info[lane][3] << "; lane: " << lane << endl;
            }
            
            
            //get cost of lane
            double lane0_cost = 99999.;
            double lane1_cost = 99999.;
            double lane2_cost = 99999.;
            double min_cost = lane_cost;

            int min_cost_lane = lane;
         
            //cout << "self_lane: " << lane << "; cost: " << lane_cost << endl;
            int current_lane = lane;
            if (lane_cost > 0.0) {
                
                //LANE == 1  complicated lane 
                if (lane == 1) {
                    lane0_cost = get_cost(lane_info[0][0], lane_info[0][1], lane_info[0][2], lane_info[0][3]);
                    lane2_cost = get_cost(lane_info[2][0], lane_info[2][1], lane_info[2][2], lane_info[2][3]); 
                
                    if (lane0_cost <= lane2_cost) {
                        min_cost = lane0_cost;
                        min_cost_lane = 0;
                    }
                    else {
                        min_cost = lane2_cost;
                        min_cost_lane = 2;
                    }

                    if (lane_cost < min_cost) {
                        min_cost = lane_cost;
                        min_cost_lane = lane;
                    } 
                    else {
                        //too_close = false;
                        lane_proposed = min_cost_lane;
                        lane_switch_initiated = true;
                        
                        /****
                        cout << "Lane switch from 1 to " << lane << endl;
                        time_t rawtime;
                        time (&rawtime);
                        printf ("The current local time is: %s", ctime (&rawtime));
                        ***/
                    }
                } // end if lane == 1.
                
                //LANE == 0  a bit easier lane 
                if ((lane == 0) || (lane == 2)){
                    lane1_cost = get_cost(lane_info[1][0], lane_info[1][1], lane_info[1][2], lane_info[1][3]);
 
                    if (lane1_cost < lane_cost) {
                        min_cost = lane1_cost;
                        min_cost_lane = 1;
                        lane_proposed = 1;
                        lane_switch_initiated = true;
                        
                        /***
                        time_t rawtime;
                        time (&rawtime);
                        printf ("The current local time is: %s", ctime (&rawtime));
                        ***/
                    } 
                    //else {
                    //    lane_switch_ = false;
                    //}
                } // end if lane == 0. || lane == 2.                         
                
            } // end if lane_cost > 0
  

            if (lane_switch_initiated == true) {
                lane = lane_proposed;
            }
            
            //variable to capture speed change proposal.
            double speed_change = 0.;

            //30 meters front cushion was not sufficient.
            if (lane_info[current_lane][2] < 50.) {
                speed_change = -0.15;
                if (lane_info[current_lane][2] < 30.)
                    speed_change -= 0.15;
                if (lane_info[current_lane][2] < 20.)
                    speed_change -= 0.15;
                if (lane_info[current_lane][2] < 15.)
                    speed_change -= 0.15;
                
                if (lane_info[current_lane][2] < 10.)
                    speed_change -= 0.1;
                
                if (lane_info[current_lane][2] < 5.)
                    speed_change -= 0.1;
            } 
            else {
                if (lane_info[current_lane][2] > 55.) // make [40 - 55] meters as safe distance in the front
                    speed_change = 0.280;
                    
                if (lane_info[current_lane][2] > 65.)
                    speed_change +=0.05;
                
                if (lane_info[current_lane][2] > 75.)
                    speed_change +=0.05;
            }
           
            ref_speed += speed_change;
 
            if (ref_speed < 0.3) {
                //cout << "TOO SLOW: computed speed: " << ref_speed << "; car speed: " << car_speed << endl;
                ref_speed = 0.3;   // to avoid spline library assertion, ideally the value should be zero. but spline throws assertion.
            }

            else if (ref_speed > 49.9)
                ref_speed = 49.9;
            
            
            speed_change = ref_speed - old_ref_speed;  
            
            double current_change_in_speeds = speed_change - previous_speed_diff;

 
            if (max_speed_change < fabs(current_change_in_speeds)) {
                max_speed_change = fabs(current_change_in_speeds);
                //cout << "MAXIMUM SPEED CHANGE: " << max_speed_change << "; old: " << previous_speed_diff << "; new: " << speed_change << endl;
            }
            previous_speed_diff = speed_change;
            old_ref_speed = ref_speed;
            
            
            vector<double> ptsx;
            vector<double> ptsy;
            
            //define variables for reference x, y and yaw states.
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yawr = deg2rad(car_yaw);
            
            if (prev_path_size < 2) {
                //make-up first two points from reference states.
                double prev_car_x = car_x - cos(ref_yawr);  //argument should be radians
                double prev_car_y = car_y - sin(ref_yawr);
                
                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);
                
                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);
            } 
            else {  // Use last two points from previous list.
                ref_x = previous_path_x[prev_path_size - 1];
                ref_y = previous_path_y[prev_path_size - 1];
                
                double prev_ref_x = previous_path_x[prev_path_size - 2];
                double prev_ref_y = previous_path_y[prev_path_size - 2];
                ref_yawr = atan2(ref_y-prev_ref_y, ref_x-prev_ref_x);
                
                ptsx.push_back(prev_ref_x);
                ptsx.push_back(ref_x);
                
                ptsy.push_back(prev_ref_y);
                ptsy.push_back(ref_y);
            }
            
            //get next gross way points for inputting to spline function
            int d_val = 2 + 4*lane;
            vector<double> next_wp0 = getXY(car_s + 30, d_val, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s + 60, d_val, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s + 90, d_val, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
            //push way points into ptsx, ptsy.
            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);            

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);      
            
            //convert points local coordinates.
            for (int i=0; i<ptsx.size(); i++) {
                //shift to local.
                double shift_x = ptsx[i] - ref_x;
                double shift_y = ptsy[i] - ref_y;
                
                ptsx[i] = shift_x * cos(0-ref_yawr) - shift_y * sin(0-ref_yawr);
                ptsy[i] = shift_x * sin(0-ref_yawr) + shift_y * cos(0-ref_yawr);
            }
            
            //create a spline & set points.
            tk::spline spl;
            spl.set_points(ptsx, ptsy);
            
            //define next set of waypoints in for the planner.
           	vector<double> next_x_vals;
          	vector<double> next_y_vals;
            
            //push remaining previous points.
            for (int i=0; i< previous_path_x.size(); i++) {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);                
            }
            
            // math for next 30 points 
            double target_x = 30.0;
            double target_y = spl(target_x);
            double target_dist = sqrt(target_x*target_x + target_y*target_y);
            double x_add_on = 0;
            double num_steps = target_dist / (0.02 * ref_speed / 2.24); //convert mph to meters/sec.
            double x_inc = target_x / num_steps;
 
            // fill in remaining points in next_vals.
            //cout << "previus path size: " << previous_path_x.size() << endl;
            for (int i=1; i<= (50-previous_path_x.size()); i++) {
                double x_point = x_add_on + x_inc;
                double y_point = spl(x_point);
                x_add_on = x_point;  // for next iteration.
                
                double x_ref = x_point;
                double y_ref = y_point;
                
                x_point = x_ref * cos(ref_yawr) - y_ref * sin(ref_yawr);
                y_point = x_ref * sin(ref_yawr) + y_ref * cos(ref_yawr);
                
                x_point += ref_x;
                y_point += ref_y;
                
                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
                
            }
            // END - Komer
            
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
