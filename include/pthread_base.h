//
// Created by wurui on 12/2/18.
//

#ifndef PROJECT_PTHREAD_BASE_H
#define PROJECT_PTHREAD_BASE_H

#include <pthread.h>
#include <iostream>
#include <signal.h>

class ThreadBase {
public:
    ThreadBase() = default; // what does this 'default' mean?

    int start();

    int join();

    int quit();

    pthread_t getTid();

    bool isAlive();

    virtual void run() = 0;

    virtual ~ThreadBase();

private:
    static void *start_func(void *arg); // why static? 
    // return is void* and the para is void* 
    // thread_create func's demand the para to be that
    // by rjp

private:
    pthread_t m_tid;    // m means memeber, easy to complete the code
    bool m_isAlive;     // by rjp

};


#endif //PROJECT_PTHREAD_BASE_H
