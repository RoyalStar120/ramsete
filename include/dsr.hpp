#pragma once
#include "main.h"
#include <chrono>
#include <cmath>
#include <utility>
#include <vector>
#include <numeric>
#include "lemlib/chassis/chassis.hpp"

enum class TimeUnit { SECOND, MILLISECOND };
class Timer { 
public: 
    Timer(double timeoutMs_ = 0) : timeoutMs(timeoutMs_), startTime(std::chrono::high_resolution_clock::now()) {};

    void reset() { startTime = std::chrono::high_resolution_clock::now(); };
    void hardReset(double newTimeoutMs) { timeoutMs = newTimeoutMs; reset(); };
    bool timeIsUp() const { return elapsedMs() > timeoutMs; };
    double timeLeft() const { double e = elapsedMs(); return e < timeoutMs ? (timeoutMs - e) : 0; };
    double elapsed(TimeUnit unit = TimeUnit::MILLISECOND) const { double ms = elapsedMs(); return (unit == TimeUnit::SECOND) ? (ms / 1000.0) : ms; };  
private: 
    double timeoutMs; 
    std::chrono::high_resolution_clock::time_point startTime;

    double elapsedMs() const { auto dt = std::chrono::high_resolution_clock::now() - startTime; return std::chrono::duration<double, std::milli>(dt).count(); };
};

constexpr double mmToInch = 0.039370078740157;
constexpr double MAX_OBSTACLE_DURATION = 1e12;
constexpr double FIELD_HALF_LENGTH = 70.5;
constexpr double FIELD_NEG_HALF_LENGTH = -70.5;

inline double botToTrig(double ang) { double result = 90.0 - ang; while (result > 360.0) result -= 360.0; while (result < 0) result += 360.0; return result;}
inline double vexToStd(double vexDegrees) { double rads = (90.0 - vexDegrees) * (M_PI / 180.0); while (rads > M_PI) rads -= 2 * M_PI; while (rads < -M_PI) rads += 2 * M_PI; return rads;}
inline double stdToVex(double stdRads) { double deg = 90.0 - (stdRads * 180.0 / M_PI); while (deg < 0) deg += 360; while (deg >= 360) deg -= 360; return deg;}

inline double degToRad(double deg) { return deg * M_PI / 180.0; }

enum class CoordType { X, Y, INVALID };

template <typename AnyType>
class Singly_Linked_List {

public: 
    Singly_Linked_List() { head = new Node(nullptr); tail = new Node(nullptr); length = 0; head->next = tail; }

    ~Singly_Linked_List() { Node* curr = head; while (curr != nullptr) { Node* nextNode = curr->next; if (curr->ptr != nullptr) delete curr->ptr; delete curr; curr = nextNode; } }

    Singly_Linked_List(const Singly_Linked_List&) = delete; 
    Singly_Linked_List& operator=(const Singly_Linked_List&) = delete;

    class Node { public: explicit Node(AnyType* obj_ptr) { this->ptr = obj_ptr; this->next = nullptr; } AnyType* ptr; Node* next; };

    class Iterator { 
    private: 
        Node* next; 
        Node* previous; 
        Singly_Linked_List* list; 
        int count;

    public: 
        explicit Iterator(Singly_Linked_List* list, Node* node) : next(node), previous(list->getHead()), list(list), count(0) {}
        AnyType*& operator*() const { return next->ptr; }
        Iterator& operator++() { if (next != list->getTail() && count < list->size()) { previous = next; next = next->next; count++; } return *this; }
        Iterator operator++(int) { Iterator temp = *this; if (next != list->getTail() && count < list->size()) { previous = next; next = next->next; count++; } return temp; }
        bool operator==(const Iterator& other) const { return next == other.next; }
        bool operator!=(const Iterator& other) const { return next != other.next; }
        
        void insert(AnyType* obj_ptr) { if (obj_ptr != nullptr) { Node* oldNode = next; previous->next = new Node(obj_ptr); previous->next->next = oldNode; ++list->length; next = previous->next; } }
        bool remove(bool clean = false) { if (next == list->getTail()) return false; if (0 <= count && count < list->size()) { Node* oldNode = next; next = next->next; previous->next = next; if (clean && oldNode->ptr != nullptr) delete oldNode->ptr; delete oldNode; --list->length; return true; } else { return false; } }
        [[nodiscard]] int getIndex() const { return count; } 
    };

    Iterator begin() { return Iterator(this, head->next); }
    Iterator end() { return Iterator(this, tail); }
    void push_back(AnyType* obj_ptr) { insert(length, obj_ptr); }
    void add_front(AnyType* obj_ptr) { insert(0, obj_ptr); }
    void insert(int index, AnyType* obj_ptr) { if (obj_ptr != nullptr) { int temp_index = index; if (this->head != nullptr && temp_index >= 0 && temp_index <= length) { Node* currNode = this->head; while (temp_index > 0){ currNode = currNode->next; temp_index--; } Node* oldNode = currNode->next; currNode->next = new Node(obj_ptr); currNode->next->next = oldNode; length++; } } }
    void pop(int index, bool clean = false){ if (index >= 0 && index < length) { Node* currNode = head; while (index > 0){ currNode = currNode->next; index--; } Node* oldNode = currNode->next; currNode->next = oldNode->next; if (clean && oldNode->ptr != nullptr) delete oldNode->ptr; delete oldNode; length--; } }
    AnyType* get(int index){ if (index < 0 || index >= length) return nullptr; Node* currNode = head->next; for (int i = 0; i < index; i++) { currNode = currNode->next; } return currNode->ptr; }
    AnyType* operator[](int index) { return get(index); }
    void remove(AnyType* obj_ptr, bool clean = false) { if (obj_ptr != nullptr) { Node* currNode = this->head; while (currNode->next != nullptr && currNode->next->ptr != obj_ptr){ currNode = currNode->next; } if (currNode->next != nullptr && currNode->next->ptr == obj_ptr) { Node* oldNode = currNode->next; currNode->next = oldNode->next; if (clean && oldNode->ptr != nullptr) delete oldNode->ptr; delete oldNode; length--; } } }
    [[nodiscard]] int size() const { return length; }
    Node* getHead() { return this->head; }
    Node* getTail() { return this->tail; }
private: 
    Node* head; 
    Node* tail; 
    int length;
};

struct SensorPose { double x = 0; double y = 0; double heading = 0; double slope = 0; double yIntercept = 0;};
struct Line { double pt1[2] = {0, 0}; double pt2[2] = {0, 0}; double slope = 0; double yIntercept = 0;};

class Line_Obstacle {
public:
    static Singly_Linked_List<Line_Obstacle> obstacleCollection;
    Line_Obstacle(double x1, double y1, double x2, double y2, double lifeTimeMs = -1); 
    bool expired(); 
    bool isIntersecting(const SensorPose& sp) const; 
    static void addPolygonObstacle(const std::vector<std::pair<double, double>>& points, double lifeTimeMs = -1);
private: 
    Line line; 
    Timer lifeTimer;
};

class Circle_Obstacle {
public:
    static Singly_Linked_List<Circle_Obstacle> obstacleCollection;
    Circle_Obstacle(double x_, double y_, double r_, double lifeTimeMs = -1);
    bool expired();
    bool isIntersecting(const SensorPose& sp) const;
    double x, y, radius; 
    Timer lifeTimer;
};

class DsrSensor {
public: 
    static std::vector<DsrSensor*> sensorCollection;
    DsrSensor(pros::Distance* distSensor, double horizOffset, double vertOffset, double mainAng, double angleTol = 10.0); 
    void updatePose(const lemlib::Pose& botPose); 
    bool isValid(double distVal) const; 
    std::pair<CoordType, double> getBotCoord(const lemlib::Pose& botPose, double accum = NAN); 
    int rawReading() const; 
    SensorPose getPose() const; 
    void logPos(std::ofstream* targetFile);
private: 
    pros::Distance* sensor; 
    double offsetDist; 
    double offsetAngle; 
    double mainAngle; 
    SensorPose sp; 
    double angleTolerance;
};

inline std::vector<DsrSensor*> DsrSensor::sensorCollection = std::vector<DsrSensor*>();

class DsrTracking {
public: 
    DsrTracking(lemlib::Chassis* chassis_, int frequencyHz_ = 25, bool autoSync_ = true, double minDelta_ = 0.5, double maxDelta_ = 4.0, double maxDeltaFromLemlib_ = 10.0, double maxSyncPerSec_ = 3.0, int minPause_ = 20);
    
    void startTracking(); 
    void stopTracking();
    
    lemlib::Pose getDsrPose() const; 
    void setDsrPose(const lemlib::Pose& p); 
    void updateBotPose(); 
    void updateBotPose(DsrSensor* sens);
    void setMaxSyncPerSec(double _maxSyncPerSec); 
    
    void startAccumulating(bool autoUpdateAfterAccum = true); 
    void stopAccumulating(); 
    void accumulateFor (int ms, bool autoUpdateAfterAccum = true);
    
    void discardData ();
    
    void mainUpdate(); 
    void syncUpdate(); 
    void lifeTimeUpdate();
private: 
    lemlib::Chassis* chassis; 
    int goalMSPT; 
    int minPause; 
    double maxSyncPT; 
    double minDelta, maxDelta, maxDeltaFromLemlib; 
    bool autoSync; 
    bool accumulating; 
    pros::Task* mainLoopTask = nullptr; 
    pros::Task* miscLoopTask = nullptr; 
    lemlib::Pose latestPrecise, poseAtLatest; 
    bool updateAfterAccum = false;
    
    void mainLoop(); 
    void miscLoop();
};
