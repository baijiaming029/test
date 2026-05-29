#include <mutex>
class SingleTon{

public:

    static SingleTon* getInstance(){
        if(instance == nullptr){
            lock_guard<mutex> lock(mtx);
            instance = new SingleTon;
        }
        return instance;
    }



private:
    SingleTon();
    static SingleTon* instance = nullptr;
    mutex mtx;

};
class SingleTon2{



public:
    static SingleTon2* getInstance(){
        static SingleTon2* instance;  
        return instance;
    }



private:
    SingleTon2();
    
}

int main(){

    






}