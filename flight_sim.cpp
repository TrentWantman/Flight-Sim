#include <iostream>
#include <chrono>
#include <thread>
#include <unordered_set>
using namespace std;

class RingBuffer{
public:
    int capacity_;
    double* buffer;
    int head;
    int tail;
    int count_;

    RingBuffer(const int&  capacity){
        buffer = new double[capacity];
        capacity_ = capacity;
        for (int i = 0; i < capacity_; i++){
            buffer[i] = 0;
        }
        head = 0; count_=0; tail = 0;
    }

    ~RingBuffer(){
        delete[] buffer;
        buffer = nullptr;
        capacity_ = 0;
    }

    bool isEmpty() { return count_ == 0; }
    bool isFull() { return count_ == capacity_; }

    void push(const double& value){
        if(isFull()){ head = (head+1) % capacity_;}
        else{count_++;}
        buffer[tail] = value;
        tail = (tail+1)% capacity_;
    }

    double pop(){
        if(isEmpty()){ cout << "Buffer is empty" << endl; return -1;}
        else{ count_--;}
        double temp = buffer[head];
        buffer[head] = 0;
        head = (head+1) % capacity_;
        return temp;
    }
    
};

class LaunchSequence{
public:
    enum State {IDLE, PRELAUNCH, IGNITION, LIFTOFF, MAX_Q, MECO, LANDING, SAFED, ABORT};
    State currentState;
    unordered_map<int, unordered_set<int>> transitions;

    LaunchSequence() {
        transitions[IDLE] = {PRELAUNCH, ABORT};
        transitions[PRELAUNCH] = {IGNITION, ABORT};
        transitions[IGNITION] = {LIFTOFF, ABORT};
        transitions[LIFTOFF] = {MAX_Q, ABORT};
        transitions[MAX_Q] = {MECO, ABORT};
        transitions[MECO] = {LANDING, ABORT};
        transitions[LANDING] = {SAFED, ABORT};
        transitions[SAFED] = {ABORT};
        transitions[ABORT] = {};
        currentState = IDLE;
    }

    void transition(State nextState){
        if (transitions[currentState].count(nextState)){
            string prevState = stateName(currentState);
            currentState = nextState;
            cout << "SUCCESS: State transitioned - " << prevState << " -> " << stateName(currentState) << endl; 
        }
        else{
            cout << "ERROR: Not a valid state transition - " << stateName(currentState) << " -> " << stateName(nextState) << endl; 
        }
    }

    string stateName(State s){
        switch (s) {
            case IDLE : return "IDLE";
            case PRELAUNCH : return "PRELAUNCH";
            case IGNITION : return "IGNITION";
            case LIFTOFF : return "LIFTOFF";
            case MAX_Q : return "MAX_Q";
            case MECO : return "MECO";
            case LANDING : return "LANDING";
            case SAFED : return "SAFED";
            case ABORT : return "ABORT";
            default: return "UKNOWN";
        }
    }

    string getState(){
        return stateName(currentState);
    }
};

class SensorUnit {
public:
    string name;
    RingBuffer buffer;
    double s1_, s2_, s3_; //three redundant sensors
    bool faulted = false;
    bool stopped = false;
    double latestVote_;
    mutex mtx_;

    SensorUnit(const string& name) : name(name), buffer(10),  faulted(false), latestVote_(0.0) {}

    double noise() {
        return ((rand() % 100) - 50) * 0.01;  // random value between -0.5 and 0.5
    }

    double update(double s1, double s2, double s3){
        s1_ = s1; s2_ = s2; s3_ = s3;
        return vote();
    }

    double vote(){
        double vote;
        bool oneTwo = abs(s1_ - s2_) <= 0.5;
        bool oneThree = abs(s1_ - s3_) <= 0.5;
        bool twoThree = abs(s2_ - s3_) <= 0.5;
        int totalAgree = (int)oneTwo + (int)oneThree + (int)twoThree;
        if ( totalAgree > 1) {
            vote = (s1_ + s2_ + s3_) / 3;
        }
        else if (totalAgree > 0){
            if (oneTwo) {vote = (s1_ + s2_) / 2;}
            else if (oneThree) {vote = (s1_ + s3_) / 2;}
            else if (twoThree) {vote = (s2_ + s3_) / 2;}
        }
        else{
            faulted = true;
            cout << "WARNING: Sensor Unit Is Faulty: sensor 1: " << s1_ << " sensor 2: " << s2_ << " sensor 3: " << s3_ << endl;
            return -999999.9;
        }
        buffer.push(vote);
        return vote;
    }

    bool isFaulted() {return faulted;}

    void run() {
        // runs on its own thread, continuously updates
        int cycle = 0;
        while (!stopped) {
            // read from hardware (simulated)
            double base = 1000.0 + cycle * 10.0; //altitude increasing
        {
            lock_guard<mutex> lock(mtx_);
            latestVote_ = update(base + noise(), base + noise(), base + noise());
        }
            cycle++;
            this_thread::sleep_until(chrono::steady_clock::now() + chrono::milliseconds(50)); //reads at 20hz
        }
    }

    double getLatestVote() {
        lock_guard<mutex> lock(mtx_);
        return latestVote_;
    }
};


// Fixed-Frequency Control Loop
// This is the heartbeat of flight software. Running its main control loop at 10Hz - exactly every 100ms.
// If processing takes too long and you miss a cycle, that’s a timing overrun and you need to detect it.
// 	∙	Has a run() method that executes a loop at exactly 10Hz (100ms per cycle)
// 	∙	Tracks the cycle count
// 	∙	Measures how long each cycle takes
// 	∙	Prints a warning if a cycle takes longer than 100ms (timing overrun)
// 	∙	Runs for 50 cycles then stops
// 	∙	Prints the cycle number and elapsed time each cycle

class FlightComputer {
public:
    void run() {
        // Loop 50 times at 10Hz
        // Each cycle: record start time, do work, record end time
        // Sleep for the remaining time to hit exactly 100ms
        auto cycleTime = chrono::milliseconds(100);
        auto startAllCycles = chrono::steady_clock::now();
        auto start = startAllCycles;
        SensorUnit sensors("SU-10-ALT");
        thread sensorThread(&SensorUnit::run, &sensors);
        LaunchSequence ls;
        ls.transition(ls.PRELAUNCH);
        ls.transition(ls.IGNITION);
        ls.transition(ls.LIFTOFF);

        for (int i = 0; i < 50; i++){
            double alt = sensors.getLatestVote();
            if(ls.getState() == "LIFTOFF"){
                if(alt >= 1200.0) { ls.transition(ls.MAX_Q);}
            }
            else if(ls.getState() == "MAX_Q"){
                if(alt >= 1500.0) { ls.transition(ls.MECO);}
            }

            this_thread::sleep_for(10ms);
            auto end = chrono::steady_clock::now();
            auto elapsed = chrono::duration_cast<chrono::milliseconds>(end - start);
            if (elapsed > cycleTime){
                cout << "OVERRUN cycle " << i << ": " << elapsed.count() << "ms" << endl;
            }
            else {
                this_thread::sleep_until(start + chrono::milliseconds(80)); // sleep most of it
                while (chrono::steady_clock::now() < start + chrono::milliseconds(100)) {
                    // spin burns CPU but guarantees precision
                }
                auto totalCycle = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start);
                cout << "Cycle " << i << ": State=" << ls.getState() <<  " Altitude: " << alt << "ft" << "work=" << elapsed.count() << "ms total=" << totalCycle.count() << "ms" << endl;
            }
            start = chrono::steady_clock::now();
        }
        cout << "All Cycles " << ": " << "ms total=" << chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - startAllCycles).count() << "ms" << endl;
        sensors.stopped = true;
        sensorThread.join();
    }
};

int main() {
    // Test SensorUnit
    SensorUnit temp("TEMP");
    
    double v = temp.update(100.2, 100.4, 100.1);  // all agree
    cout << "Voted: " << v << " Faulted: " << temp.isFaulted() << endl;
    
    v = temp.update(100.2, 100.3, 847.0);  // one outlier
    cout << "Voted: " << v << " Faulted: " << temp.isFaulted() << endl;
    
    v = temp.update(100.0, 200.0, 300.0);  // all disagree
    cout << "Voted: " << v << " Faulted: " << temp.isFaulted() << endl;

    // FlightComputer test
    FlightComputer fc;
    fc.run();
    return 0;
}
