#include "edu_fleet/mecanum_path_controller/costmap_handler.hpp"
#include <memory>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>

namespace edu_fleet::mecanum_path_controller {

    CostmapHandler::CostmapHandler(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
    : local_costmap_ros_(costmap), local_costmap_(costmap->getCostmap()),
      param_()
    {

    }

    double CostmapHandler::checkSegment(const geometry_msgs::msg::Point &start, const geometry_msgs::msg::Point &end)
    {
        // Lock costmap while reading.
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(
            *(local_costmap_->getMutex())
        );

        // convert from world coordinates to map cells
        unsigned int start_x, start_y, end_x, end_y;
        if (local_costmap_->worldToMap(start.x, start.y, start_x, start_y) == false
            || local_costmap_->worldToMap(end.x, end.y, end_x, end_y) == false)
        {
            RCLCPP_WARN(logger_, "Start or end point is outside the costmap");
            return 1.0; // indicate that the segment is not traversable due to being outside the costmap
        }

        unsigned char max_cost = 0;

        // check cells along the line
        supercoverLine(start_x, start_y, end_x, end_y,
            std::bind(&CostmapHandler::findMaxCost, this, std::placeholders::_1, std::placeholders::_2, std::ref(max_cost))
        );

        if (max_cost >= param_.stopping_cost)
        {
            // RCLCPP_WARN(logger_, "Max cost along the segment exceeds stopping cost: %u", static_cast<unsigned int>(max_cost));
            return 0.0; // indicate that the segment is not traversable due to high cost
        }
        else
        {
            // RCLCPP_INFO(logger_, "Max cost along the segment: %u", static_cast<unsigned int>(max_cost));
        }


        return 1.0;
    }


    template<typename Callback>
    void CostmapHandler::supercoverLine(
        int x0,
        int y0,
        int x1,
        int y1,
        Callback visit)
    {
        // calculate the difference in cells between the two points
        const int dx = x1 - x0;
        const int dy = y1 - y0;

        // the number of steps required in each direction
        const int nx = std::abs(dx);
        const int ny = std::abs(dy);

        // the direction of the steps for each axis
        const int sign_x = (dx > 0) ? 1 : ((dx < 0) ? -1 : 0);
        const int sign_y = (dy > 0) ? 1 : ((dy < 0) ? -1 : 0);

        // current position while iterating along the path
        int x = x0; 
        int y = y0;

        // numer of iterations along each axis
        int ix = 0;
        int iy = 0;

        visit(x, y);

        while (ix < nx || iy < ny)
        {
            const int decision = // multiply by 2 and +1 -> effectively going to the center of the cell
                (1 + 2 * ix) * ny -
                (1 + 2 * iy) * nx;

            if (decision == 0)
            {
                // The line crosses an exact grid corner.
                // Visit both adjacent cells.

                if (!visit(x + sign_x, y))
                    return;

                if (!visit(x, y + sign_y))
                    return;

                x += sign_x;
                y += sign_y;

                ++ix;
                ++iy;

                if (!visit(x, y))
                    return;
            }
            else if (decision < 0)
            {
                // Move horizontally.
                x += sign_x;
                ++ix;

                if (!visit(x, y))
                    return;
            }
            else
            {
                // Move vertically.
                y += sign_y;
                ++iy;

                if (!visit(x, y))
                    return;
            }
        }
    }

    bool CostmapHandler::findMaxCost(int mx, int my, unsigned char & max_cost)
    {
        max_cost = std::max(max_cost, local_costmap_->getCost(mx, my));
        return max_cost >= nav2_costmap_2d::LETHAL_OBSTACLE;
    }

}