#include <iostream>
#include <vector>
using namespace std;

void swap(int& a, int& b){

    int temp = a;
    a = b;
    b = temp;

}

void qsort(vector<int>& nums,int start,int end){

    if(start >= end){
        return;
    }
    int base = nums[start];
    int left = start;
    int right = end;
    while(left < right){

        while(left < right && nums[right] >= base){
            right--;
        }
        while(left < right && nums[left] <= base){
            left++;
        }
        swap(nums[left], nums[right]);

    }

    swap(nums[left], nums[start]);

    qsort(nums, start, left - 1);
    qsort(nums, left+1, end);


}

int main(){


    vector<int> nums = {7,5,2,3,9,11,55,0,9};

    qsort(nums, 0, nums.size()-1);

    for(int i = 0;i<nums.size(); i++){

        cout << nums[i] << ' ';
    }



    return 0;
}