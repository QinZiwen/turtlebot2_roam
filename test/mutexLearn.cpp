
#include <iostream>       // std::cout
#include <thread>         // std::thread
#include <mutex>          // std::mutex

volatile int counter(0); // non-atomic counter
volatile int distance(0);
volatile int direction(0);
std::mutex mtx;           // locks access to counter

void attempt_10k_increases() {
    while(1)
    {
        if (mtx.try_lock()) {   // only increase if currently not locked:
            std::this_thread::sleep_for (std::chrono::seconds(1));
            std::cout << "counter = " << counter << std::endl;
            std::cout << "distance = " << distance << ", direction = " << direction << std::endl;
            mtx.unlock();
        }
    }
}

int main (int argc, const char* argv[]) 
{
    std::thread th;
    
    th = std::thread(attempt_10k_increases);

    while(1)
    {
        if(mtx.try_lock())
        {
            counter ++;
            distance ++;
            direction ++;
            mtx.unlock();
        }
    }

    th.join();
    std::cout << counter << " successful increases of the counter.\n";

    return 0;
}


/*
#include <iostream>       // std::cout
#include <thread>         // std::thread
#include <mutex>          // std::mutex

volatile int counter(0); // non-atomic counter
std::mutex mtx;           // locks access to counter

void attempt_10k_increases() {
    for (int i=0; i<10000; ++i) {
        if (mtx.try_lock()) {   // only increase if currently not locked:
            ++counter;
            mtx.unlock();
        }
    }
}

int main (int argc, const char* argv[]) {
    std::thread threads[10];
    for (int i=0; i<10; ++i)
        threads[i] = std::thread(attempt_10k_increases);

    for (auto& th : threads) th.join();
    std::cout << counter << " successful increases of the counter.\n";

    return 0;
}
*/