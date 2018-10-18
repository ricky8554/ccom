#include <iostream>
//#include <iomanip>
#include <limits>
#include <chrono>
#include <thread>
#include <mutex>
#include <queue>
#include <string>
#include <sstream>
#include <cmath>
#include <ctime>
#include <sys/time.h>
#include <time.h>
#include <string.h>
#include "ObjectPar.h"
#include "communication.h"
#include <fstream>
#include <list>

using namespace std;

mutex mtx_path;
mutex mtx_obs;
mutex mtx_cover;

int pathindex = 0;
int running = 1;
int request_start = 0;
int request_start1 = 0;
int countn = 0;
int send = 1;
double difx = 0, dify = 0;

ObjectPar previousAction;
ObjectPar estimateStart;
ObjectPar current_location;

vector<ObjectPar> dyamic_obstacles;
vector<ObjectPar> path;

list<point> cover;
list<point> newcover;

Communitcation communication_With_Planner, communication_With_Controler;

// const std::string currentDateTime() {
//     time_t     now = time(0);
//     struct tm  tstruct;
//     char       buf[80];
//     tstruct = *localtime(&now);
//     // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
//     // for more information about date/time format
//     strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

//     return buf;
// }

double getCurrentTime()
{
    //change to clock_gettime
    struct timeval t;
    gettimeofday(&t, NULL);
    return ((t.tv_sec) + (t.tv_usec) / 1000000.0);
}

void checkTerminate()
{
    if (getppid() == 1)
    {
        cerr << "excutive TERMINATE" << endl;
        running = 0;
        exit(1);
    }
}

// fix the moving of start
void requestPath()
{
    string s;
    int bytesRead, oldbytesRead;
    int numberOfState;
    double x, y, heading, speed, otime;
    char response[8192];
    double duration_time; //,time_bound;
    while (!request_start || !request_start1)
        this_thread::sleep_for(std::chrono::milliseconds(50));

    previousAction = current_location;
    estimateStart = current_location;
    while (running)
    {
        vector<ObjectPar> newpath;
        int addestimate = 1;
        communication_With_Planner.cwrite("plan");
        mtx_cover.lock();
        int size = newcover.size();
        communication_With_Planner.cwrite("newly covered " + to_string(size));
        for(point p: newcover)
        {
            communication_With_Planner.cwrite(to_string(p.x) + " " + to_string(p.y));
            cerr << "EXECUTIVE::NEWLYCOVERD:: " <<  p.x << " " << p.y << endl;
        }
        newcover.clear();
        mtx_cover.unlock();

        //add covered method
        estimateStart = current_location;
        communication_With_Planner.cwrite("start state " + estimateStart.toString());
        s = "dynamic obs " + to_string(dyamic_obstacles.size());
        communication_With_Planner.cwrite(s);
        mtx_obs.lock();
        for (int i = 0; i < dyamic_obstacles.size(); i++)
        {
            s = to_string(i);
            s += " " + dyamic_obstacles[i].toString();
            communication_With_Planner.cwrite(s);
        }
        mtx_obs.unlock();
        double start_time = getCurrentTime();
        communication_With_Planner.cread(response, 8192);
        if (!strncmp(response, "done", 4))
        {
            running = 0;
            break;
        }
        //truncate all result if time out or just keep it??????????
        sscanf(response, "plan %d\n%n", &numberOfState, &bytesRead);

        mtx_path.lock();
        oldbytesRead = bytesRead;
        //time_bound = getCurrentTime();
        for (int i = 0; i < path.size(); i++)
        {
            if (path[i].otime <= estimateStart.otime)
                newpath.push_back(path[i]);
        }
        for (int i = 0; i < numberOfState; i++)
        {
            sscanf(response + bytesRead, "%lf %lf %lf %lf %lf\n%n", &x, &y, &heading, &speed, &otime, &bytesRead);
            bytesRead += oldbytesRead;
            oldbytesRead = bytesRead;

            // dObjectPar o =(ObjectPar(x, y, heading, speed, otime));
            // cerr << current_location.otime << endl;
            // cerr << (time_bound )<< endl;
            // cerr << getCurrentTime() << endl;
            // if (time_bound > otime)
            //   continue;

            newpath.push_back(ObjectPar(x, y, heading, speed, otime + estimateStart.otime));

            if (addestimate && otime - newpath[0].otime > 0.99999)
            {
                difx = current_location.x - previousAction.x;
                dify = current_location.y - previousAction.y;
                //estimateStart = ObjectPar(x + difx, y + dify, heading, speed, otime+estimateStart.otime);
                estimateStart = ObjectPar(x, y, heading, speed, otime + estimateStart.otime);
                addestimate = 0;
            }
        }

        if (addestimate)
        {
            difx = current_location.x - previousAction.x;
            dify = current_location.y - previousAction.y;
            //estimateStart = ObjectPar(x + difx, y + dify, heading, speed, otime);
            estimateStart = ObjectPar(x, y, heading, speed, otime + estimateStart.otime);
            addestimate = 0;
        }
        path = newpath;
        pathindex = 0;
        mtx_path.unlock();
        duration_time = getCurrentTime() - start_time;
        if (duration_time < 1)
            this_thread::sleep_for(std::chrono::milliseconds((int)((1 - duration_time) * 1000)));
        checkTerminate();
    }
}

void requestWorldInformation()
{
    char locationString[8192];
    double x, y, heading, speed, otime;
    int index, h, oldbytesRead, bytesRead;
    while (running) // should clear the previous one prevent the disappear obs
    {
        read(STDIN_FILENO, locationString, 8192);
        if (!strncmp(locationString, "Location", 8))
        {
            request_start = 1;
            sscanf(locationString + 9, "%lf,%lf,%lf,%lf,%lf [%d]", &current_location.x, &current_location.y, &current_location.heading, &current_location.speed, &current_location.otime, &h);
            mtx_cover.lock();
            auto it = cover.begin();
            while(it != cover.end())
            {
                float x = it->x - current_location.x;
                float y = it->y - current_location.y;
                if(sqrt(x*x + y*y) <=10)
                {
                    auto it1 = it;
                    newcover.push_back(*it);
                    ++it;
                    cover.erase(it1);
                }
                else
                {
                    ++it;
                }
            }
            mtx_cover.unlock();
            //current_location.printerror();
        }
        else if (!strncmp(locationString, "Obstacle", 8))
        {
            request_start1 = 1;
            bytesRead = 9;
            oldbytesRead = bytesRead;
            mtx_obs.lock();
            while (sscanf(locationString + bytesRead, "%d,%lf,%lf,%lf,%lf,%lf\n%n", &index, &x, &y, &heading, &speed, &otime, &bytesRead) == 6)
            {
                bytesRead += oldbytesRead;
                oldbytesRead = bytesRead;
                if (index < dyamic_obstacles.size())
                    dyamic_obstacles[index].set(x, y, heading, speed, otime);
                else
                    dyamic_obstacles.push_back(ObjectPar(x, y, heading, speed, otime)); //need to more careful to check if index is right after
                                                                                        // dyamic_obstacles[index].printerror();
            }
            while (index >= dyamic_obstacles.size())
                dyamic_obstacles.pop_back();
            mtx_obs.unlock();
        }
        else
        {
            cerr << "EXECUTIVE::ERROR REQUEST" << endl;
            cerr << locationString << endl;
            cerr << "END ERROR" << endl;
        }
        checkTerminate();
    }
}

void sendPath( string &s)
{
    int pathd = pathindex - 1;
    int size = path.size() - pathd;
    s += "path " + to_string(size) + "\n";
    for(int i = pathd; i< path.size(); i++ )
        s +=path[i].toString() + "\n";
    s+='\0';

        
}

void sendAction()
{
    while (running)
    {

        mtx_path.lock();

        if (send && path.size() > pathindex)
        {
            string s = "";
            // << "EXECUTIVE::SEND::" + path[pathindex].toString() << endl;
            s+=current_location.toString()+"\n";
            //communication_With_Controler.cwrite(current_location.toString());
            //path[pathindex].x += difx;
            //path[pathindex].y += dify;
            previousAction = path[pathindex];
            //communication_With_Controler.cwrite(path[pathindex++].toString());
            cerr << "EXECUTIVE::SEND " << path[pathindex].toString() << endl;
            s+=path[pathindex++].toString()+"\n";
            sendPath(s);
            communication_With_Controler.cwrite(s);
            
            if (path.size() >= pathindex)
            {
                int sleeptime = (path[pathindex].otime - previousAction.otime) * 1000;
                mtx_path.unlock();
                this_thread::sleep_for(std::chrono::milliseconds(sleeptime));
            }
            else
            {
                mtx_path.unlock();
                this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
        else
        {
            mtx_path.unlock();
            this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        checkTerminate();
    }
}

void print_map(string file)
{
    if (file != "NOFILE")
    {
        string line;
        ifstream f(file);
        if (f.is_open())
        {
            getline(f, line);
            string w = line;
            getline(f, line);
            string h = line;
            cerr << "EXECUTIVE::MAP::" + w + " " + h << endl;
            communication_With_Planner.cwrite("map 1 " + w + " " + h);
            while (getline(f, line))
                communication_With_Planner.cwrite(line);
            
            f.close();
        }
        return;
    }
    cerr << "EXECUTIVE::MAP::DEFAULT"<< endl;
    communication_With_Planner.cwrite("map 1 2000 2000");
    for (int i = 0; i < 2000; i++)
        communication_With_Planner.cwrite("________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________");
}

int main(int argc, char *argv[])
{
    cout.precision(numeric_limits<float>::digits10 + 2);
    cerr.precision(numeric_limits<double>::digits10 + 2);

    string file;
    cin >> file;
    communication_With_Planner.set("planner", 1, 1, 0, 0);
    communication_With_Controler.set("controler", 1, 1, 0, 1);

    thread thread_for_controller(thread([=] { sendAction(); }));
    thread thread_for_UDVOBS(thread([=] { requestWorldInformation(); }));

    communication_With_Planner.cwrite("Start");
    communication_With_Planner.cwrite("max speed 2.75");
    communication_With_Planner.cwrite("max turning radius 0.75");

   
    print_map(file);
    
    cover.push_back(point(0,0));
    cover.push_back(point(800,800));
    cover.push_back(point(1500,1900));
    cover.push_back(point(400,0));
    int size = cover.size();
    communication_With_Planner.cwrite("path to cover " + to_string(size));
    for(point p: cover)
        communication_With_Planner.cwrite(to_string(p.x) + " " + to_string(p.y));
    
    char done[100];
    communication_With_Planner.cread(done, 100);
    requestPath();

    thread_for_controller.join();
    thread_for_UDVOBS.join();
    return 0;
}
