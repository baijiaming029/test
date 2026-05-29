#include <iostream>
using namespace std;

template <typename T>
class Smartptr{

public:

    Smartptr(T* ptr = nullptr):ptr(ptr){



    }
    ~Smartptr(){


        delete ptr;
    }
    T& operator*(){

        return *ptr;
    }
    T* operator->(){

        return ptr;
    }
private:
    T* ptr;
};
int main(){

    
    Smartptr<int> sptr = new int(110);
    cout << *sptr;


    return 0;
}