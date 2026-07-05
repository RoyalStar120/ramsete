#include "dsr.hpp"
#include <cmath>

inline double roundTwoPlaces(double x) {
    return std::round(x * 100) / 100;
}

Singly_Linked_List<Line_Obstacle> Line_Obstacle::obstacleCollection = Singly_Linked_List<Line_Obstacle>();
Line_Obstacle::Line_Obstacle(double x1, double y1, double x2, double y2, double lifeTimeMs)
    : lifeTimer(lifeTimeMs < 0 ? MAX_OBSTACLE_DURATION : lifeTimeMs) {
    line.pt1[0] = x1;
    line.pt1[1] = y1;
    line.pt2[0] = x2;
    line.pt2[1] = y2;
    if (std::abs(x2 - x1) > 1e-5f) line.slope = (y2 - y1) / (x2 - x1);
    else line.slope = (y2 - y1) * 1e8f;
    line.yIntercept = y1 - line.slope * x1;
    Line_Obstacle::obstacleCollection.add_front(this);
}

bool Line_Obstacle::expired() {
    return lifeTimer.timeIsUp();
}

bool Line_Obstacle::isIntersecting(const SensorPose& sp) const {
    double angRad = degToRad(botToTrig(sp.heading));
    double vAx = std::cos(angRad);
    double vAy = std::sin(angRad);

    double vBx = line.pt2[0] - line.pt1[0];
    double vBy = line.pt2[1] - line.pt1[1];

    double det = (vAx * vBy) - (vAy * vBx);

    if (std::abs(det) < 1e-6) return false;

    double dx = line.pt1[0] - sp.x;
    double dy = line.pt1[1] - sp.y;

    double t = (dx * vBy - dy * vBx) / det;
    double u = (dx * vAy - dy * vAx) / det;

    return (t > 0 && u >= 0 && u <= 1);
}

void Line_Obstacle::addPolygonObstacle(const std::vector<std::pair<double, double>>& points, double lifeTimeMs) {
    if (points.size() < 3) return;
    for (size_t i = 0; i < points.size(); ++i) {
        size_t next = (i + 1) % points.size();
        new Line_Obstacle(points[i].first, points[i].second, points[next].first, points[next].second, lifeTimeMs);
    }
}

Singly_Linked_List<Circle_Obstacle> Circle_Obstacle::obstacleCollection = Singly_Linked_List<Circle_Obstacle>();
Circle_Obstacle::Circle_Obstacle(double x_, double y_, double r_, double lifeTimeMs)
    : x(x_), y(y_), radius(r_), lifeTimer(lifeTimeMs < 0 ? MAX_OBSTACLE_DURATION : lifeTimeMs) {
    Circle_Obstacle::obstacleCollection.add_front(this);
}

bool Circle_Obstacle::expired() {
    return lifeTimer.timeIsUp();
}

bool Circle_Obstacle::isIntersecting(const SensorPose& sp) const {
    double angRad = degToRad(botToTrig(sp.heading));
    double vx = std::cos(angRad);
    double vy = std::sin(angRad);

    double dx = x - sp.x;
    double dy = y - sp.y;

    double t = dx * vx + dy * vy;

    if (t < 0) return false;

    double closestX = sp.x + t * vx;
    double closestY = sp.y + t * vy;

    double distSq = std::pow(closestX - x, 2) + std::pow(closestY - y, 2);
    
    return distSq <= (radius * radius);
}

DsrSensor::DsrSensor(pros::Distance* distSensor, double horizOffset, double vertOffset, double mainAng, double angleTol)
    : sensor(distSensor), mainAngle(mainAng), angleTolerance(std::abs(angleTol)) {
    offsetDist = std::hypot(horizOffset, vertOffset);
    offsetAngle = std::fmod(((std::atan2(vertOffset, horizOffset) * 180.0 / M_PI) + 360), 360);
    DsrSensor::sensorCollection.push_back(this);
}

void DsrSensor::updatePose(const lemlib::Pose& botPose) {
    double theta = degToRad(offsetAngle - botPose.theta);
    sp.x = botPose.x + std::cos(theta) * offsetDist;
    sp.y = botPose.y + std::sin(theta) * offsetDist;
    sp.heading = std::fmod(botPose.theta + mainAngle, 360.0);
    if (sp.heading <= 0) sp.heading += 360.0;
    sp.slope = std::tan(degToRad(botToTrig(sp.heading)));
    sp.yIntercept = sp.y - sp.slope * sp.x;
}

bool DsrSensor::isValid(double distVal) const {
    if (distVal > 2000) return false;
    if (distVal > 200 && this->sensor->get_confidence() < 60) return false;
    if (std::abs(std::fmod(this->sp.heading, 90.0)) > angleTolerance &&
        std::abs(std::fmod(this->sp.heading, 90.0)) < (90 - angleTolerance)) return false;
    for (auto* item : Circle_Obstacle::obstacleCollection) { if (item->isIntersecting(sp)) return false; }
    for (auto* item : Line_Obstacle::obstacleCollection) { if (item->isIntersecting(sp)) return false; }

    return true;
}

std::pair<CoordType, double> DsrSensor::getBotCoord(const lemlib::Pose& botPose, double accum) {
    this->updatePose(botPose);

    double val = std::isnan(accum) ? sensor->get() : accum;
    
    if (!isValid(val)) return {CoordType::INVALID, 0.0};
    val *= mmToInch;

    double angRad = degToRad(botToTrig(this->sp.heading));
    double cosA = std::cos(angRad);
    double sinA = std::sin(angRad);

    double minDist = 1e9;
    int wall = -1;

    if (std::abs(cosA) > 1e-6) {
        double dEast = (FIELD_HALF_LENGTH - sp.x) / cosA;
        if (dEast > 0 && dEast < minDist) { minDist = dEast; wall = 2; }
        
        double dWest = (FIELD_NEG_HALF_LENGTH - sp.x) / cosA;
        if (dWest > 0 && dWest < minDist) { minDist = dWest; wall = 4; }
    }

    if (std::abs(sinA) > 1e-6) {
        double dNorth = (FIELD_HALF_LENGTH - sp.y) / sinA;
        if (dNorth > 0 && dNorth < minDist) { minDist = dNorth; wall = 1; }
        
        double dSouth = (FIELD_NEG_HALF_LENGTH - sp.y) / sinA;
        if (dSouth > 0 && dSouth < minDist) { minDist = dSouth; wall = 3; }
    }

    double res;
    CoordType type;
    if (wall == 1) { type = CoordType::Y; res = FIELD_HALF_LENGTH - sinA * val; }
    else if (wall == 2) { type = CoordType::X; res = FIELD_HALF_LENGTH - cosA * val; }
    else if (wall == 3) { type = CoordType::Y; res = FIELD_NEG_HALF_LENGTH - sinA * val; }
    else if (wall == 4) { type = CoordType::X; res = FIELD_NEG_HALF_LENGTH - cosA * val; }
    else return {CoordType::INVALID, 0.0};

    double offRad = degToRad(offsetAngle - botPose.theta);
    if (type == CoordType::X) res -= std::cos(offRad) * offsetDist;
    else if (type == CoordType::Y) res -= std::sin(offRad) * offsetDist;

    return {type, res};
}

int DsrSensor::rawReading() const { return sensor->get(); }
SensorPose DsrSensor::getPose() const { return sp; }

DsrTracking::DsrTracking(lemlib::Chassis* chassis_,
            int frequencyHz_,
            bool autoSync_,
            double minDelta_,
            double maxDelta_,
            double maxDeltaFromLemlib_,
            double maxSyncPerSec_,
            int minPause_)
    : chassis(chassis_),
        goalMSPT(std::round(1000.0 / frequencyHz_)),
        minPause(minPause_),
        maxSyncPT(maxSyncPerSec_ / frequencyHz_),
        minDelta(minDelta_),
        maxDelta(maxDelta_),
        maxDeltaFromLemlib(maxDeltaFromLemlib_),
        autoSync(autoSync_),
        latestPrecise{0, 0, 0},
        poseAtLatest{0, 0, 0} {}

void DsrTracking::startTracking() {
    if (mainLoopTask == nullptr)  {
        mainLoopTask = new pros::Task([this](){ this->mainLoop(); });
    }
    if (miscLoopTask == nullptr) {
        miscLoopTask = new pros::Task([this](){ this->miscLoop(); });
    }
}

void DsrTracking::stopTracking() {
    if (mainLoopTask != nullptr) { mainLoopTask->remove(); delete mainLoopTask; mainLoopTask = nullptr; }
    if (miscLoopTask != nullptr) { miscLoopTask->remove(); delete miscLoopTask; miscLoopTask = nullptr; } 
}

lemlib::Pose DsrTracking::getDsrPose() const {
    auto chassisPose = chassis->getPose();
    return { latestPrecise.x + (chassisPose.x - poseAtLatest.x),
                latestPrecise.y + (chassisPose.y - poseAtLatest.y),
                chassisPose.theta };
}

void DsrTracking::setDsrPose(const lemlib::Pose& p) {
    latestPrecise = p;
    poseAtLatest = chassis->getPose();
}

void DsrTracking::updateBotPose() {
    auto p = getDsrPose();
    chassis->setPose(p);
    setDsrPose(p);
}

void DsrTracking::updateBotPose(DsrSensor* sens) {
    if (sens != nullptr && chassis != nullptr) {
        auto data = sens->getBotCoord(chassis->getPose());
        auto pose = chassis->getPose();

        if (data.first == CoordType::X) pose.x = data.second;
        else if (data.first == CoordType::Y) pose.y = data.second;
        else return;

        chassis->setPose({pose.x, pose.y, chassis->getPose().theta});
        setDsrPose({pose.x, pose.y, chassis->getPose().theta});
    }
}

void DsrTracking::startAccumulating(bool autoUpdateAfterAccum) { accumulating = true; updateAfterAccum = autoUpdateAfterAccum; }
void DsrTracking::stopAccumulating() { accumulating = false; }
void DsrTracking::accumulateFor (int ms, bool autoUpdateAfterAccum) {
    startAccumulating(autoUpdateAfterAccum);
    Timer t(ms);
    while (!t.timeIsUp()) { pros::delay(minPause); }
    stopAccumulating();
}

void DsrTracking::discardData () {
    latestPrecise = chassis->getPose();
    poseAtLatest = chassis->getPose();
}

void DsrTracking::setMaxSyncPerSec(double maxSyncPerSec_) {
    maxSyncPT = maxSyncPerSec_ / (1000.0 / goalMSPT);
}

void DsrTracking::mainUpdate() {
    if (DsrSensor::sensorCollection.size() > 0) {
        std::vector<int> accTotal(DsrSensor::sensorCollection.size());
        std::vector<int> accCount(DsrSensor::sensorCollection.size());
        while (accumulating) {
            for (int i = 0; i < DsrSensor::sensorCollection.size(); i++) {
                accTotal[i] += DsrSensor::sensorCollection[i]->rawReading();
                accCount[i] ++;
            }
            pros::delay(goalMSPT);
        }

        std::vector<double> xs, ys;

        auto botPose = getDsrPose();
        double diff_from_lemlib = std::hypot(botPose.x - chassis->getPose().x, botPose.y - chassis->getPose().y);
        if (diff_from_lemlib > maxDeltaFromLemlib) {
            botPose.x += (chassis->getPose().x - botPose.x) / diff_from_lemlib;
            botPose.y += (chassis->getPose().y - botPose.y) / diff_from_lemlib;
        }

        for (int i = 0; i < DsrSensor::sensorCollection.size(); i++) {
            auto sens = DsrSensor::sensorCollection[i];

            double avg = (accCount[i] > 0) ? (1.0 * accTotal[i] / accCount[i]) : NAN;
            auto [type, coord] = sens->getBotCoord(botPose, avg);

            if (type == CoordType::X) {
                double diff = std::abs(coord - botPose.x);
                if (diff <= maxDelta) xs.push_back(coord);
            }
            else if (type == CoordType::Y) {
                double diff = std::abs(coord - botPose.y);
                if (diff <= maxDelta) ys.push_back(coord);
            }
        }

        if (!xs.empty()) {
            double meanX = std::accumulate(xs.begin(), xs.end(), 0.0) / xs.size();
            if (meanX > FIELD_NEG_HALF_LENGTH && meanX < FIELD_HALF_LENGTH && std::abs(meanX - botPose.x) >= minDelta) {
                latestPrecise.x = meanX;
                poseAtLatest.x = chassis->getPose().x;
            }
        }
        if (!ys.empty()) {
            double meanY = std::accumulate(ys.begin(), ys.end(), 0.0) / ys.size();
            if (meanY > FIELD_NEG_HALF_LENGTH && meanY < FIELD_HALF_LENGTH && std::abs(meanY - botPose.y) >= minDelta) {
                latestPrecise.y = meanY;
                poseAtLatest.y = chassis->getPose().y;
            }
        }

        if (updateAfterAccum && std::any_of(accCount.begin(), accCount.end(), [](int c){ return c > 0; }))
            updateBotPose();
    }
}

void DsrTracking::syncUpdate() {
    double x_diff = 0.0;
    double y_diff = 0.0;
    double real_diff = 0.0;

    double x_update = 0.0;
    double y_update = 0.0;

    lemlib::Pose currDsrPosition = getDsrPose();

    x_diff = currDsrPosition.x - chassis->getPose().x;
    y_diff = currDsrPosition.y - chassis->getPose().y;
    real_diff = std::hypot(std::abs(x_diff), std::abs(y_diff));

    if (real_diff <= maxSyncPT) {
        chassis->setPose(currDsrPosition.x, currDsrPosition.y, chassis->getPose().theta);
        
        poseAtLatest.x += x_diff;
        poseAtLatest.y += y_diff;
        poseAtLatest.theta = chassis->getPose().theta;
    }
    else {
        x_update = x_diff / real_diff * maxSyncPT;
        y_update = y_diff / real_diff * maxSyncPT;

        chassis->setPose(chassis->getPose().x + x_update, chassis->getPose().y + y_update, chassis->getPose().theta);
        
        poseAtLatest.x += x_update;
        poseAtLatest.y += y_update;
        poseAtLatest.theta = chassis->getPose().theta;
    }
}

void DsrTracking::lifeTimeUpdate() {
    auto circle_itr = Circle_Obstacle::obstacleCollection.begin();
    while (circle_itr != Circle_Obstacle::obstacleCollection.end()) {
        if ((**circle_itr).expired()) circle_itr.remove(true);
        else ++circle_itr;
    }

    auto line_itr = Line_Obstacle::obstacleCollection.begin();
    while (line_itr != Line_Obstacle::obstacleCollection.end()) {
        if ((**line_itr).expired()) line_itr.remove(true);
        else ++line_itr;
    }
}

void DsrTracking::mainLoop() {
    Timer frequencyTimer(goalMSPT);

    while (true) {
        frequencyTimer.reset();

        mainUpdate();

        if (frequencyTimer.timeLeft() < minPause) pros::delay(minPause);
        else pros::delay(frequencyTimer.timeLeft());
    }
}

void DsrTracking::miscLoop() {
    Timer frequencyTimer(goalMSPT);

    while (true) {
        frequencyTimer.reset();

        if (autoSync) { syncUpdate(); }

        if (frequencyTimer.timeLeft() < minPause) pros::delay(minPause);
        else pros::delay(frequencyTimer.timeLeft());
    }
}
