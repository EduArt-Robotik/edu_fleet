#ifndef EDU_FLEET__MECANUM_PATH_CONTROLLER__COSTMAP_HANDLER_HPP_
#define EDU_FLEET__MECANUM_PATH_CONTROLLER__COSTMAP_HANDLER_HPP_

#include <memory>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include "geometry_msgs/msg/point.hpp"

namespace edu_fleet::mecanum_path_controller {

class CostmapHandler
{
public:
    struct Parameter
    {
        unsigned char stopping_cost = 200; // stop the robot if the path contains a cell with this cost or higher
    };

    CostmapHandler(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap);
    ~CostmapHandler() = default;

    double checkSegment(const geometry_msgs::msg::Point & start, const geometry_msgs::msg::Point & end );

protected:
    /**
    * @brief Visit every grid cell intersected by a line.
    *
    * Uses the Supercover Bresenham algorithm.
    *
    * When the line passes exactly through a grid corner,
    * both adjacent cells and the diagonal cell are visited.
    *
    * @param x0 Start cell X
    * @param y0 Start cell Y
    * @param x1 End cell X
    * @param y1 End cell Y
    * @param visit Callback receiving (x, y).
    *              Return false to abort traversal.
    */
    template<typename Callback>
    void supercoverLine(
        int x0,
        int y0,
        int x1,
        int y1,
        Callback visit
    );

    /**
    * @brief check if the given cell has a higher cost the the one provided
    * @return true if the cost is a fatal collision
    */
    bool findMaxCost(int mx, int my, unsigned char & max_cost);

    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> local_costmap_ros_;
    std::unique_ptr<nav2_costmap_2d::Costmap2D> local_costmap_;
    rclcpp::Logger logger_{ rclcpp::get_logger("edu_CostmapHandler") };
    Parameter param_;

};

} // namespace edu_fleet::mecanum_path_controller

#endif // EDU_FLEET__MECANUM_PATH_CONTROLLER__COSTMAP_HANDLER_HPP_