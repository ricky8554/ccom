#include <iostream>
//#include <iomanip>
#include <limits>
#include <chrono>
#include <thread>
#include <mutex>
#include <queue>
#include <string>
#include <string.h>
#include <sstream>
#include <cmath>
#include "communication.h"
#include "ObjectPar.h"

using namespace std;

mutex mtx;
int running = 1;
int send = 0;
int plan = 0;
ObjectPar start;
ObjectPar action;
ObjectPar  oldaction= ObjectPar(0,0,0,0,0);
vector<ObjectPar> dyamic_obstacles;
string path;
double currentTimestamp = -1;
double currenttime_for_action = -1;
string pheading;
string previousrequestString;
string default_Command = "0,0";
int sendPipeToParent, receivePipeFromParent;

void readpath(char locationString[],int byteREAD)
{
    path = locationString + byteREAD;
    cout << path << flush;
}

void requestAction()
{
    int byteRead = 0;
    
    while (running)
    {
        char locationString[2560];
        int oldbyteRead = 0;
        mtx.lock();
        read(receivePipeFromParent, locationString, 2560);
        sscanf(locationString, "%lf %lf %lf %lf %lf\n%n", &start.x, &start.y, &start.heading, &start.speed, &start.otime,&byteRead);
        oldbyteRead += byteRead;
        sscanf(locationString+oldbyteRead, "%lf %lf %lf %lf %lf\n%n", &action.x, &action.y, &action.heading, &action.speed, &action.otime,&byteRead);
        oldbyteRead += byteRead;
        readpath(locationString,oldbyteRead);
        // cerr << "CONTROLLER :: start ";
        // start.printerror();
        // cerr << "CONTROLLER :: action ";
        // action.printerror();
        plan = 1;
        mtx.unlock();
    }
}

// void requestLocation()
// {
//     int lengh = 256, h;
//     char locationString[256];
    
//     while (running)
//     {
//         sscanf(locationString, "rx: %lf,%lf,%lf,%lf,%lf [%d]", &start.x, &start.y, &start.heading, &start.speed, &start.otime, &h);
//         oldaction = start;
//         cerr << "START " << endl;
//         //start.print();
//     }
// }

void sendAction()
{
    //Communitcation communication_With_Controler("controler", 1, 1, 1);
    while (running)
    {
        if(getppid() == 1)
        {
            cerr << "CONTROLER TERMINATE" << endl;
            exit(1);
        } 
        string command = "";
        if (plan)//use mutex instead of busy waiting
        {
            mtx.lock();
            if (fabs(currentTimestamp - action.otime) < 0.0000001)
            {
                
                if (currenttime_for_action > action.otime)
                {
                    //cerr << "CONTROLLER::RECEIVED::HELLO1"<<endl;
                    command += pheading + ",0";
                    // start mutex?
                    //currenttime_for_action = start.otime;
                }
                else
                {
                    //cerr << "CONTROLLER::RECEIVED::HELLO2"<<endl;
                    command += previousrequestString;
                    currenttime_for_action += 0.05;
                }
            }
            else
            {
               // cerr << "CONTROLLER INITIALIZE SEND REQEST" << endl;
                currentTimestamp = action.otime;
                //mutex for start and action
                if(oldaction.otime < start.otime)
                {
                    oldaction = start;
                }
                //double ydis = action.y - oldaction.y;
                //double xdis = action.x - oldaction.x;
                double ydis = action.y - start.y;
                double xdis = action.x - start.x;
                //cerr << "CONTROLER " << start.x << " " << start.y  << " "<< action.x << " " << action.y << endl;
                float heading = atan2(xdis, ydis);
                //cerr << "CONTROLER " <<ydis << " "<< xdis << " " << heading << endl;
                if(heading < 0)
                {
                    heading += M_PI*2;
                }
                double speed = sqrt(xdis * xdis + ydis * ydis) / (fabs(action.otime - oldaction.otime));
                currenttime_for_action = oldaction.otime + 0.05;
                oldaction = action;
                //mutex for start and action
                pheading = to_string(heading);
                previousrequestString = pheading + "," + to_string(speed);
                command += previousrequestString;
                cerr << "CONTROLER:: " <<command << endl;
            //    cerr << "CONTROLLER Command " << command <<  " " << speed << " " << heading << endl;
            }
            if(command.size() != 0)
            {
                cout << command << endl << flush;
            }
            mtx.unlock();
        }
        this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

int main(int argc, char *argv[])
{   
    receivePipeFromParent = stoi(argv[0]);
    sendPipeToParent = stoi(argv[1]);
    cout.precision(numeric_limits<float>::digits10 + 2);
    cerr.precision(numeric_limits<float>::digits10 + 2);
    //cerr << "CONTROLLER INITIALIZE" << endl;
    thread thread_for_executive(thread([=] { requestAction(); }));
    
    
    sendAction();
    thread_for_executive.join();
    //thread thread_for_UDVSEND(thread([=] { sendAction(); }));
    //thread thread_for_UDVLOC(thread([=] { requestLocation(); }));
    //this_thread::sleep_for(std::chrono::milliseconds(1000));
    //thread_for_UDVSEND.join();
    //thread_for_UDVLOC.join();
    return 0;
}