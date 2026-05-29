#include <thread>
#include <iostream>
#include <mutex>
#include <condition_variable>
#include <queue>
using namespace std;
queue<int> msg_queue;
const int queue_size = 5;

mutex mtx;
condition_variable productor_cv;
condition_variable consumer_cv;
void consumer(){

    static int cnt = 10;
    while(cnt--){
        unique_lock<mutex> lock(mtx);
        
        consumer_cv.wait(lock, []{
            return msg_queue.size() > 0;
        });
        
        cout << msg_queue.front() << endl;
        msg_queue.pop();
        productor_cv.notify_all();

    }



}

void productor(){

    static int cnt = 10;
    while(cnt--){
        unique_lock<mutex> lock(mtx);
        
        productor_cv.wait(lock,[]{
            return msg_queue.size() < queue_size;
        });
        
        msg_queue.push(cnt);
        consumer_cv.notify_all();

    }

}


int main(){


    thread c1(consumer);
    thread c2(productor);

    c1.join();
    c2.join();


    return 0;
}