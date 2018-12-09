//
// Created by wurui on 12/2/18.
//

#include "../include/pthread_base.h"

using namespace std;

// ThreadBase has a virtual = 0 function
// note by rjp
void *ThreadBase::start_func(void *arg) {
    ThreadBase *ptr = (ThreadBase *) arg;   
    ptr->run();

    return NULL;
}

int ThreadBase::start() {
    cout << "Start a new thread" << endl;
    if (pthread_create(&m_tid, NULL, start_func, (void *) this) != 0) {
        cout << "Start a new thread failed!" << endl;
        return -1;
    }

    cout << "Start a new thread success! tid=" << m_tid << endl;
    m_isAlive = true;
    return 0;
}

// pthread_join is to wait for a thread to be end not join
// note by rjp
int ThreadBase::join() {
    int ret = -1;
    cout << "Join the thread tid=" << m_tid << endl;
    ret = pthread_join(m_tid, NULL);

    if (ret != 0)
        cout << "Join the thread fail tid=" << m_tid << endl;
    else
        cout << "Join the thread success tid=" << m_tid << endl;

    return ret;
}

// only change the alive condition
// note by rjp
int ThreadBase::quit() {
    cout << "Quit the thread tid=" << m_tid << endl;
    m_isAlive = false;
    return 0;
}

pthread_t ThreadBase::getTid() {
    return m_tid;
}

bool ThreadBase::isAlive() {
    return m_isAlive;
}

// destructor and use pthread_kill
// "Send signal SIGNO to the given thread"
// If you want to end a thread, better use pthread_exit
// note by rjp
ThreadBase::~ThreadBase() {
    cout << "~ThreadBase tid=" << m_tid << endl;
    if (m_isAlive) {
        cout << "Kill the thread tid= " << m_tid << endl;
        pthread_kill(m_tid, 0);
    }
}
