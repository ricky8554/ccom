/*
* open two way pipe for desire program.
* Chao Chi Cheng
*/

#ifndef COMMUNICATION_H
#define COMMUNICATION_H
#include<unistd.h>
//#include<sys/wait.h>
//#include<signal.h>
//#include<stdlib.h>
#include <string>
//#include<stdio.h>

class Communitcation
{

  public:
    Communitcation(std::string excutivepath,bool bindStdin, bool bindStdout, bool bindStderr, bool sendPipetoChild);

    Communitcation()
    {
    }

    ~Communitcation();

    void set(std::string excutivepath,bool bindStdin, bool bindStdout, bool bindStderr, bool sendPipetoChild);

    void cwrite(std::string s);

    void cwrite(char s[]);

    void cwrite(const char s[]);

    void cread(char receive[], int length) const;

  private:
    pid_t pid = 0;
    int getp[2];
    int sendp[2];
    char send[256];
    int status;
};

#endif