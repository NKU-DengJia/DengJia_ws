#include "trajectory_analysis.hpp"

// #include <iostream>

bool calculateProjectionPointWithPose(const PlannerTraj traj, const EgoStatus egostatus,
                                 TrajPoint &point) {
    int traj_size = traj.traj_points.size();
    double min_dist = 10e6;
    int min_temp = -1;
    for (int i = 0; i < traj_size; i++) {
        double dx = egostatus.ego_x - traj.traj_points[i].x;
        double dy = egostatus.ego_y - traj.traj_points[i].y;
        double dist = sqrt(dx*dx + dy*dy);
        double yaw_err = NormalizeAngle(egostatus.ego_theta - traj.traj_points[i].theta);
        /*
        std::cout << "dist:" << dist << std::endl;
        std::cout << "yaw_err:" << yaw_err << std::endl;
        std::cout << "egostatus.ego_theta:" << egostatus.ego_theta << std::endl;
        std::cout << "i:" << i << ",   traj_points[i].theta" << traj.traj_points[i].theta << std::endl;
        */
        if (dist < min_dist && yaw_err < M_PI / 3.0  && dist < 3.0) {
            min_dist = dist;
            min_temp = i;
        }
    }
    if (min_temp == -1) {
        // std::cout << "could not find match point !!!" << std::endl;
        return false;
    }
    // std::cout << "success !!!" << "min_temp: " << min_temp << ", kappa: " << traj.traj_points[min_temp].kappa << std::endl;
    if (traj_size == 1) {
        point = traj.traj_points.front();
        return true;
    }
    int temp_next;
    double factor = 0.0;
    int final_next;
    if (min_temp == 0) {
        temp_next = min_temp + 1;
        Eigen::Vector2d v1(egostatus.ego_x - traj.traj_points[min_temp].x, egostatus.ego_y - traj.traj_points[min_temp].y);
        Eigen::Vector2d v2(traj.traj_points[temp_next].x - traj.traj_points[min_temp].x, traj.traj_points[temp_next].y - traj.traj_points[min_temp].y);
        double vector_dot = v1.dot(v2);
        if (vector_dot < 0) {
            point = traj.traj_points[min_temp];
            return true;
        } else {
            factor = (vector_dot / v2.norm()) / v2.norm();

            final_next = temp_next;
        }
    } else if (min_temp == traj_size - 1) {
        temp_next = min_temp - 1;
        Eigen::Vector2d v1(egostatus.ego_x - traj.traj_points[min_temp].x, egostatus.ego_y - traj.traj_points[min_temp].y);
        Eigen::Vector2d v2(traj.traj_points[temp_next].x - traj.traj_points[min_temp].x, traj.traj_points[temp_next].y - traj.traj_points[min_temp].y);
        double vector_dot = v1.dot(v2);
        if (vector_dot < 0) {
            point = traj.traj_points[min_temp];
            return true;
        } else {
            factor = (vector_dot / v2.norm()) / v2.norm();
            final_next = temp_next;
        }
    } else {
        temp_next = min_temp + 1;
        int temp_before = min_temp - 1;
        Eigen::Vector2d v1(egostatus.ego_x - traj.traj_points[min_temp].x, egostatus.ego_y - traj.traj_points[min_temp].y);
        Eigen::Vector2d v2(traj.traj_points[temp_next].x - traj.traj_points[min_temp].x, traj.traj_points[temp_next].y - traj.traj_points[min_temp].y);
        Eigen::Vector2d v3(traj.traj_points[temp_before].x - traj.traj_points[min_temp].x, traj.traj_points[temp_before].y - traj.traj_points[min_temp].y);
        double vector_dot = v1.dot(v2);
        if (vector_dot < 0) {
            factor = (v1.dot(v3) / v3.norm()) / v3.norm();
            final_next = temp_before;
        } else {
            factor = (vector_dot / v2.norm()) / v2.norm();
            final_next = temp_next;
        }
    }
    point = traj.traj_points[min_temp];
    // std::cout << "min_point_curvature: " << point.kappa << std::endl;
    point.x += (traj.traj_points[final_next].x - traj.traj_points[min_temp].x) * factor;
    point.y += (traj.traj_points[final_next].y - traj.traj_points[min_temp].y) * factor;
    point.s += (traj.traj_points[final_next].s - traj.traj_points[min_temp].s) * factor;
    point.l += (traj.traj_points[final_next].l - traj.traj_points[min_temp].l) * factor;
    point.kappa += (traj.traj_points[final_next].kappa - traj.traj_points[min_temp].kappa) * factor;
    /*
    std::cout << "min_temp: " << min_temp << std::endl;
    std::cout << "final_next: " << final_next << std::endl;
    std::cout << "traj.traj_points[final_next].kappa: " << traj.traj_points[final_next].kappa << std::endl;
    std::cout << "traj.traj_points[min_temp].kappa: " << traj.traj_points[min_temp].kappa << std::endl;
    std::cout << "factor: " << factor << std::endl;
    std::cout << "projection_point_curvature: " << point.kappa << std::endl;
    */
    point.theta += NormalizeAngle((traj.traj_points[final_next].theta - traj.traj_points[min_temp].theta)) * factor;
    point.theta = NormalizeAngle(point.theta);    
    point.v += (traj.traj_points[final_next].v - traj.traj_points[min_temp].v) * factor;
    point.t += (traj.traj_points[final_next].t - traj.traj_points[min_temp].t) * factor;
    //point.forward = traj[min_temp].forward;
    //point.parking_point = traj[final_next].parking_point;
    return true;
}
