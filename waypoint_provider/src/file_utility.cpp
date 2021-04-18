/*  File reader utility created in an effort to keep the
    ros nodes as clean as possible

    file format is csv

    -Gerson Uriarte, 6/15/2020
*/

#include <string>
#include <fstream>
#include <vector>
#include <sstream>
#include <utility>
#include <iostream>
#include <limits.h>
#include <unistd.h>
#include <freyja_msgs/WaypointTarget.h>


using namespace std;

vector<freyja_msgs::WaypointTarget> file_read(string filename){

    vector<freyja_msgs::WaypointTarget> messages;



    ifstream file(filename);

    string line, col;
    float pn, pe, pd, vn, ve, vd, yaw, time, speed;

    while(file.good()){
        freyja_msgs::WaypointTarget line_message;
        getline(file, line);
        stringstream ss(line);

        getline(ss,col,',');
        line_message.terminal_pn = stof(col);
        getline(ss,col,',');
        line_message.terminal_pe = stof(col);
        getline(ss,col,',');
        line_message.terminal_pd = stof(col);
        getline(ss,col,',');
        line_message.terminal_vn = stof(col);
        getline(ss,col,',');
        line_message.terminal_ve = stof(col);
        getline(ss,col,',');
        line_message.terminal_vd = stof(col);
        getline(ss,col,',');
        line_message.terminal_yaw = stof(col);
        getline(ss,col,',');
        line_message.allocated_time = stof(col);
        getline(ss,col,',');
        line_message.translational_speed = stof(col);

        messages.insert(messages.begin(), line_message);
    }  
    file.close();
    ROS_INFO("Returning the messages!\n");
    return messages;
}

std::string getexepath()
{
  char result[ PATH_MAX ];
  ssize_t count = readlink( "/proc/self/exe", result, PATH_MAX );
  return std::string( result, (count > 0) ? count : 0 );
}