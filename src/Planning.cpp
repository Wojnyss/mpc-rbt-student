#include "Planning.hpp"

#include <functional>
#include <chrono>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

PlanningNode::PlanningNode()
    : rclcpp::Node("planning_node") {

    // Client for map
    map_client_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

    // Service for path
    plan_service_ = this->create_service<nav_msgs::srv::GetPlan>(
        "/plan_path",
        std::bind(&PlanningNode::planPath, this, _1, _2)
    );

    // Publisher for path
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

    RCLCPP_INFO(get_logger(), "Planning node started.");

    // Connect to map server
    while (!map_client_->wait_for_service(1s) && rclcpp::ok()) {
        RCLCPP_WARN(get_logger(), "Waiting for /map_server/map service...");
    }

    // Request map
    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
    map_client_->async_send_request(
        request,
        std::bind(&PlanningNode::mapCallback, this, _1)
    );

    RCLCPP_INFO(get_logger(), "Trying to fetch map...");
}

void PlanningNode::mapCallback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
    auto response = future.get();
    if (response) {
        map_ = response->map;
        map_received_ = true;
//	map_.info.origin.position.x += 0.5;

//	map_put_->publish(map_);



        RCLCPP_INFO(
            get_logger(),
            "Map received: width=%u height=%u resolution=%.3f",
            map_.info.width,
            map_.info.height,
            map_.info.resolution
        );

         dilateMap();
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to receive map.");
    }
}

void PlanningNode::planPath(
    const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
    std::shared_ptr<nav_msgs::srv::GetPlan::Response> response
) {
    if (!map_received_) {
        RCLCPP_ERROR(get_logger(), "Map not received yet.");
        return;
    }

    aStar(request->start, request->goal);
    smoothPath();

    response->plan = path_;
    path_pub_->publish(path_);

    RCLCPP_INFO(get_logger(), "Path planned and published.");
}

void PlanningNode::dilateMap() {

    int width = map_.info.width;
    int height = map_.info.height;
    double resolution = map_.info.resolution;

    // 👉 nastav šířku robota (v metrech)
    double robot_width = 0.55;   // uprav podle robota

    // 👉 radius v buňkách
    int radius = static_cast<int>((robot_width / 2.0) / resolution);

    nav_msgs::msg::OccupancyGrid dilatedMap = map_;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {

            int index = y * width + x;

            // překážka
            if (map_.data[index] > 50) {

                for (int dy = -radius; dy <= radius; ++dy) {
                    for (int dx = -radius; dx <= radius; ++dx) {

                        int nx = x + dx;
                        int ny = y + dy;

                        if (nx >= 0 && ny >= 0 &&
                            nx < width && ny < height) {

                            // 👉 kruhová maska (lepší než čtverec)
                            if (dx*dx + dy*dy <= radius*radius) {

                                int nindex = ny * width + nx;
                                dilatedMap.data[nindex] = 100;
                            }
                        }
                    }
                }
            }
        }
    }

    map_ = dilatedMap;

    RCLCPP_INFO(get_logger(),
        "Map dilated with robot_width=%.2f m (radius=%d cells)",
        robot_width, radius);

}

void PlanningNode::aStar(const geometry_msgs::msg::PoseStamped &start,
                         const geometry_msgs::msg::PoseStamped &goal) {

    path_.poses.clear();
    path_.header.frame_id = "map";
    path_.header.stamp = this->now();

    const int width = static_cast<int>(map_.info.width);
    const int height = static_cast<int>(map_.info.height);
    const double resolution = map_.info.resolution;
    const double origin_x = map_.info.origin.position.x;
    const double origin_y = map_.info.origin.position.y;

    // world -> grid
    const double planning_offset_x = 0;

    // world -> grid (korekce jen pro planner)
    int start_x = static_cast<int>((start.pose.position.x + planning_offset_x - origin_x) / resolution);
    int start_y = static_cast<int>((start.pose.position.y - origin_y) / resolution);
    int goal_x  = static_cast<int>((goal.pose.position.x  + planning_offset_x - origin_x) / resolution);
    int goal_y  = static_cast<int>((goal.pose.position.y  - origin_y) / resolution);

    if (!isInside(start_x, start_y) || !isInside(goal_x, goal_y)) {
        RCLCPP_ERROR(get_logger(), "Start or goal is outside map.");
        return;
    }

    if (!isFree(start_x, start_y) || !isFree(goal_x, goal_y)) {
        RCLCPP_ERROR(get_logger(), "Start or goal is in obstacle.");
        return;
    }

    Cell cStart(start_x, start_y);
    Cell cGoal(goal_x, goal_y);

    std::vector<std::shared_ptr<Cell>> cells(width * height, nullptr);
    std::vector<bool> closedList(width * height, false);
    std::vector<bool> openMask(width * height, false);
    std::vector<std::shared_ptr<Cell>> openList;

    auto startCell = std::make_shared<Cell>(cStart);
    startCell->g = 0.0f;
    startCell->h = heuristic(start_x, start_y, goal_x, goal_y);
    startCell->f = startCell->g + startCell->h;

    cells[toIndex(start_x, start_y)] = startCell;
    openList.push_back(startCell);
    openMask[toIndex(start_x, start_y)] = true;

    const int dx[4] = {1, -1, 0, 0};
    const int dy[4] = {0, 0, 1, -1};

    while (!openList.empty() && rclcpp::ok()) {
        auto bestIt = std::min_element(
            openList.begin(),
            openList.end(),
            [](const std::shared_ptr<Cell> &a, const std::shared_ptr<Cell> &b) {
                return a->f < b->f;
            }
        );

        std::shared_ptr<Cell> current = *bestIt;
        openMask[toIndex(current->x, current->y)] = false;
        openList.erase(bestIt);

        int currentIndex = toIndex(current->x, current->y);
        closedList[currentIndex] = true;

        if (current->x == goal_x && current->y == goal_y) {
            std::vector<geometry_msgs::msg::PoseStamped> reversedPath;

            std::shared_ptr<Cell> trace = current;
            while (trace != nullptr) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.header.stamp = this->now();

                pose.pose.position.x = trace->x * resolution + origin_x - planning_offset_x + resolution / 2.0;
                pose.pose.position.y = trace->y * resolution + origin_y + resolution / 2.0;               
                pose.pose.position.z = 0.0;
               
                pose.pose.orientation.w = 1.0;

                reversedPath.push_back(pose);
                trace = trace->parent;
            }

            std::reverse(reversedPath.begin(), reversedPath.end());
            path_.poses = reversedPath;

            RCLCPP_INFO(get_logger(), "Path found with %zu waypoints.", path_.poses.size());
            return;
        }

        for (int i = 0; i < 4; ++i) {
            int nx = current->x + dx[i];
            int ny = current->y + dy[i];

            if (!isInside(nx, ny)) {
                continue;
            }

            int neighborIndex = toIndex(nx, ny);

            if (closedList[neighborIndex]) {
                continue;
            }

            if (!isFree(nx, ny)) {
                continue;
            }

            float new_g = current->g + 1.0f;

            std::shared_ptr<Cell> neighbor = cells[neighborIndex];
            if (neighbor == nullptr) {
                neighbor = std::make_shared<Cell>(nx, ny);
                cells[neighborIndex] = neighbor;
            }

            if (!openMask[neighborIndex] || new_g < neighbor->g) {
                neighbor->g = new_g;
                neighbor->h = heuristic(nx, ny, goal_x, goal_y);
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->parent = current;

                if (!openMask[neighborIndex]) {
                    openList.push_back(neighbor);
                    openMask[neighborIndex] = true;
                }
            }
        }
    }

    RCLCPP_ERROR(get_logger(), "Unable to plan path.");

}

void PlanningNode::smoothPath() {

if (path_.poses.size() < 3) {
        return;
    }

    std::vector<geometry_msgs::msg::PoseStamped> originalPath = path_.poses;
    std::vector<geometry_msgs::msg::PoseStamped> newPath = path_.poses;

    const double alpha = 0.1;          // drzi trasu blizko puvodni
    const double beta = 0.3;           // mira vyhlazeni
    const double tolerance = 1e-4;     // pod touto zmenou koncime
    const int max_iterations = 200;    // ochrana proti nekonecne smycce

    for (int iter = 0; iter < max_iterations; ++iter) {
        double total_change = 0.0;

        for (size_t i = 1; i < newPath.size() - 1; ++i) {
            double old_x = newPath[i].pose.position.x;
            double old_y = newPath[i].pose.position.y;

            double orig_x = originalPath[i].pose.position.x;
            double orig_y = originalPath[i].pose.position.y;

            double prev_x = newPath[i - 1].pose.position.x;
            double prev_y = newPath[i - 1].pose.position.y;

            double next_x = newPath[i + 1].pose.position.x;
            double next_y = newPath[i + 1].pose.position.y;

            newPath[i].pose.position.x +=
                alpha * (orig_x - newPath[i].pose.position.x) +
                beta  * (prev_x + next_x - 2.0 * newPath[i].pose.position.x);

            newPath[i].pose.position.y +=
                alpha * (orig_y - newPath[i].pose.position.y) +
                beta  * (prev_y + next_y - 2.0 * newPath[i].pose.position.y);

            total_change += std::abs(old_x - newPath[i].pose.position.x)
                          + std::abs(old_y - newPath[i].pose.position.y);
        }

        if (total_change < tolerance) {
            RCLCPP_INFO(get_logger(), "Path smoothing finished after %d iterations.", iter + 1);
            break;
        }

        if (iter == max_iterations - 1) {
            RCLCPP_WARN(get_logger(), "Path smoothing stopped at max iterations.");
        }
    }

    // zachovej hlavicky a orientaci
    for (size_t i = 0; i < newPath.size(); ++i) {
        newPath[i].header = path_.poses[i].header;
        newPath[i].pose.position.z = 0.0;
        newPath[i].pose.orientation.w = 1.0;
        newPath[i].pose.orientation.x = 0.0;
        newPath[i].pose.orientation.y = 0.0;
        newPath[i].pose.orientation.z = 0.0;
    }

    path_.poses = newPath;

}

Cell::Cell(int c, int r)
    : x(c), y(r), f(0.0f), g(0.0f), h(0.0f), parent(nullptr) {
}


int PlanningNode::toIndex(int x, int y) const {
    return y * map_.info.width + x;
}

bool PlanningNode::isInside(int x, int y) const {
    return x >= 0 && y >= 0 &&
           x < static_cast<int>(map_.info.width) &&
           y < static_cast<int>(map_.info.height);
}

bool PlanningNode::isFree(int x, int y) const {
    if (!isInside(x, y)) {
        return false;
    }

    int value = map_.data[toIndex(x, y)];
    return value >= 0 && value < 50;
}

float PlanningNode::heuristic(int x1, int y1, int x2, int y2) const {
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}
