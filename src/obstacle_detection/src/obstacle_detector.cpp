#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <sensor_msgs/msg/point_cloud.h>
#include "../include/obstacle_detector.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class ObstacleDetector : public rclcpp::Node
{
public:
    ObstacleDetector() : Node("minimal_publisher"), count_(0)
    {
        rclcpp::QoS qos(10);
        qos.best_effort();

        scan_sub_ = this->create_subscription<sensor_msgs::msgs::LaserScan>("scan", qos, std::bind(&ObstacleDetector::scanCallback, this, _1));

        obstacles_pub_ = this->create_publisher<obstacle_detector::msg::Obstacles>("obstacles", 5);
    }

private:
    rclcpp::Publisher<obstacle_detector::msg::Obstacles>::SharedPtr obstacles_pub_;
    rclcpp::Subscription<sensor_msgs::msgs::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;

    void scanCallback(const sensor_msgs::msg::LaserScan::ConstPtr &scan)
    {
        initial_points_.clear();

        double phi = scan->angle_min - scan->angle_increment;

        for (const float r : scan->ranges)
        {
            phi += scan->angle_increment;

            if (r >= scan->range_min && r <= scan->range_max && r <= p_max_scanner_range_)
                initial_points_.push_back(Point::fromPoolarCoords(r, phi));
        }

        processPoints();
    }

    void pclCallback(const sensor_msgs::PointCloud::ConstPtr &pcl)
    {
        initial_points_.clear();

        for (const geometry_msgs::Point32 &p : pcl->points)
            if (Point(p.x, p.y).lengthSquared() <= pow(p_max_scanner_range_, 2.0))
                initial_points_.push_back(Point(p.x, p.y));

        processPoints();
    }

    void processPoints()
    {
        segments_.clear();
        circles_.clear();

        groupPointsAndDetectSegments();
        mergeSegments();

        detectCircles();
        mergeCircles();

        if (p_transform_to_world)
            transformToWorld();

        publishObstacles();
    }

    void groupPointsAndDetectSegments()
    {
        list<Point> point_set;

        for (const Point &point : initial_points_)
        {
            if (point_set.size() != 0)
            {
                double r = point.length();

                if ((point - point_set.back()).lengthSquared() > pow(p_max_group_distance_ + r * p_distance_proportion_, 2.0))
                {
                    detectSegments(point_set);
                    point_set.clear();
                }
            }
            point_set.push_back(point);
        }

        detectSegments(point_set); // Check the last point set too!
    }

    void detectSegments(list<Point> &point_set)
    {
        if (point_set.size() < p_min_group_points_)
            return;

        Segment segment(Point(0.0, 0.0), Point(1.0, 0.0));
        if (p_use_split_and_merge_)
            segment = fitSegment(point_set);
        else // Use Iterative End Point Fit
            segment = Segment(point_set.front(), point_set.back());

        list<Point>::iterator set_divider;
        double max_distance = 0.0;
        double distance = 0.0;

        // Seek the point of division; omit first and last point of the set
        for (auto point_itr = ++point_set.begin(); point_itr != --point_set.end(); ++point_itr)
        {
            if ((distance = segment.distanceTo(*point_itr)) >= max_distance)
            {
                double r = (*point_itr).length();

                if (distance > p_max_split_distance_ + r * p_distance_proportion_)
                {
                    max_distance = distance;
                    set_divider = point_itr;
                }
            }
        }

        if (max_distance > 0.0)
        {                                                // Split the set
            point_set.insert(set_divider, *set_divider); // Clone the dividing point for each group

            list<Point> subset1, subset2;
            subset1.splice(subset1.begin(), point_set, point_set.begin(), set_divider);
            subset2.splice(subset2.begin(), point_set, set_divider, point_set.end());

            detectSegments(subset1);
            detectSegments(subset2);
        }
        else
        { // Add the segment
            if (!p_use_split_and_merge_)
                segment = fitSegment(point_set);

            if (segment.length() > 0.0)
            {
                segments_.push_back(segment);
                segments_.back().point_set().assign(point_set.begin(), point_set.end());
            }
        }
    }

    void mergeSegments()
    {
        for (auto i = segments_.begin(); i != segments_.end(); ++i)
        {
            auto j = i;
            for (++j; j != segments_.end(); ++j)
            {
                if (compareAndMergeSegments(*i, *j))
                { // If merged - a new segment appeared at the end of the list
                    auto temp_ptr = i;
                    i = segments_.insert(i, segments_.back()); // Copy new segment in place; i now points to new segment
                    segments_.pop_back();                      // Remove the new segment from the back of the list
                    segments_.erase(temp_ptr);                 // Remove the first merged segment
                    segments_.erase(j);                        // Remove the second merged segment
                    if (i != segments_.begin())
                        i--; // The for loop will increment i so let it point to new segment in next iteration
                    break;
                }
            }
        }
    }

    bool compareAndMergeSegments(Segment &s1, Segment &s2)
    {
        if (&s1 == &s2)
            return false;

        // Segments must be provided counter-clockwise
        if (s1.first_point().cross(s2.first_point()) < 0.0)
            return compareAndMergeSegments(s2, s1);

        if ((s1.last_point() - s2.first_point()).length() < p_max_merge_separation_)
        {
            list<Point> merged_points;
            merged_points.insert(merged_points.begin(), s1.point_set().begin(), s1.point_set().end());
            merged_points.insert(merged_points.end(), s2.point_set().begin(), s2.point_set().end());

            Segment s = fitSegment(merged_points);

            if (s.distanceTo(s1.first_point()) < p_max_merge_spread_ &&
                s.distanceTo(s1.last_point()) < p_max_merge_spread_ &&
                s.distanceTo(s2.first_point()) < p_max_merge_spread_ &&
                s.distanceTo(s2.last_point()) < p_max_merge_spread_)
            {
                segments_.push_back(s);
                segments_.back().point_set().assign(merged_points.begin(), merged_points.end());
                return true;
            }
        }

        return false;
    }

    void detectCircles()
    {
        for (const Segment &s : segments_)
        {
            Circle c(s);
            c.setRadius(c.radius() + p_radius_enlargement_);

            if (c.radius() < p_max_circle_radius_)
                circles_.push_back(c);
        }
    }

    void mergeCircles()
    {
        for (auto i = circles_.begin(); i != circles_.end(); ++i)
        {
            auto j = i;
            for (++j; j != circles_.end(); ++j)
            {
                if (compareAndMergeCircles(*i, *j))
                { // If merged - a new circle appeared at the end of the list
                    auto temp_ptr = i;
                    i = circles_.insert(i, circles_.back()); // i now points to new circle
                    circles_.pop_back();
                    circles_.erase(temp_ptr);
                    circles_.erase(j);
                    if (i != circles_.begin())
                        --i;
                    break;
                }
            }
        }
    }

    bool compareAndMergeCircles(Circle &c1, Circle &c2)
    {
        // If circle c1 is fully inside c2 - merge and leave as c2
        if (c1.radius() + (c2.center() - c1.center()).length() <= c2.radius())
        {
            circles_.push_back(c2);
            return true;
        }

        // If circle c2 is fully inside c1 - merge and leave as c1
        if (c2.radius() + (c2.center() - c1.center()).length() <= c1.radius())
        {
            circles_.push_back(c1);
            return true;
        }

        // If circles intersect and are 'small' - merge
        if (c1.radius() + c2.radius() >= (c2.center() - c1.center()).length())
        {
            Segment s(c1.center(), c2.center());
            Circle c(s);
            c.setRadius(c.radius() + max(c1.radius(), c2.radius()));

            if (c.radius() < p_max_circle_radius_)
            {
                circles_.push_back(c);
                return true;
            }
        }
        return false;
    }

    void transformToWorld()
    {
        geometry_msgs::PointStamped point_l; // Point in local (scanner) coordinate frame
        geometry_msgs::PointStamped point_w; // Point in global (world) coordinate frame

        point_l.header.stamp = ros::Time::now();
        point_l.header.frame_id = p_scanner_frame_;

        point_w.header.stamp = ros::Time::now();
        point_w.header.frame_id = p_world_frame_;

        try
        {
            tf_listener_.waitForTransform(p_world_frame_, p_scanner_frame_, ros::Time::now(), ros::Duration(3.0));

            for (auto it = circles_.begin(); it != circles_.end(); ++it)
            {
                if (it->center().x < p_max_x_range_ && it->center().x > p_min_x_range_ &&
                    it->center().y < p_max_y_range_ && it->center().y > p_min_y_range_)
                {
                    point_l.point.x = it->center().x;
                    point_l.point.y = it->center().y;
                    tf_listener_.transformPoint(p_world_frame_, point_l, point_w);
                    it->setCenter(point_w.point.x, point_w.point.y);
                }
                else
                {
                    it = circles_.erase(it);
                    --it;
                }
            }

            for (Segment &s : segments_)
            {
                point_l.point.x = s.first_point().x;
                point_l.point.y = s.first_point().y;
                tf_listener_.transformPoint(p_world_frame_, point_l, point_w);
                s.setFirstPoint(point_w.point.x, point_w.point.y);

                point_l.point.x = s.last_point().x;
                point_l.point.y = s.last_point().y;
                tf_listener_.transformPoint(p_world_frame_, point_l, point_w);
                s.setLastPoint(point_w.point.x, point_w.point.y);
            }
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    void publishObstacles()
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleDetector>());
    rclcpp::shutdown();
    return 0;
}