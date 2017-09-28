#include <bits/stdc++.h>
using namespace std;
int bitCount(unsigned int n) {

   int counter = 0;
   while(n) {
       counter += n % 2;
       n >>= 1;
   }
   return counter;
}
int main()
{
    int num;
    cin>>num;
    cout<<bitCount(num);

    return 0;
}
