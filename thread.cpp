#include <thread>
#include <iostream>
#include <mutex>
#include <condition_variable>
using namespace std;

mutex mtx;
condition_variable cv;
int current_order = 1;
void print(char name, int order){
    for(int i= 0; i < 10; i++){
        unique_lock<mutex> lock(mtx);
        cv.wait(lock, [order](){ return current_order == order; });
        cout << name << " ";
        current_order = (current_order % 3) + 1; // Update to the
        cv.notify_all();
    }

}


int main(){


    thread A(print, 'A', 1);
    thread B(print, 'B', 2);
    thread C(print, 'C', 3);
    A.join();
    B.join();
    C.join();

    system("pause");    


    return 0;
}