# Romea Core Path Library

This library provides an implementation of a 2D trajectory representation for trajectory tracking in agricultural applications. It supports multi-section trajectories, allowing for complex maneuvers such as U-turns and reverse driving in headland areas. The library includes a set of algorithms (also known as **path matching**) to determine the vehicle's position (in Frenet coordinates) relative to the trajectory, which can be used to feed trajectory tracking algorithms. Additionally, it offers the possibility to annotate the trajectory to trigger specific actions (e.g., tool activation, sensor triggering, etc.).

## **Usage**

1. create a ROS workspace
2. cd worskpace
3. mkdir src
4. wget https://raw.githubusercontent.com/Romea/romea-core-path/refs/heads/main/romea_path_public.repos
5. vcs import src < romea_path_public.repos
6. build packages
   - catkin build for ROS1
   - colcon build for ROS2
7. create your application using this library

## **Contributing**

If you'd like to contribute to this library, here are some guidelines:

1. Fork the repository.
2. Create a new branch for your changes.
3. Make your changes.
4. Write tests to cover your changes.
5. Run the tests to ensure they pass.
6. Commit your changes.
7. Push your changes to your forked repository.
8. Submit a pull request.

## **License**

This project is released under the Apache License 2.0. See the LICENSE file for details.

## **Authors**

The Romea Core Path library, written by **Jean Laneurit** and **Cyrille Pierre**, was developed during ANR Baudet Rob 2 and ANR Tiara projects. Several individuals contributed scientifically to the development of this library:

**Jean Laneurit**  
**Roland Lenain**  
**Cyrille Pierre**  
**Vincent Rousseau**  
**Benoît Thuilot**    

## **Contact**

If you have any questions or comments about Romea Core Path library, please contact **[Jean Laneurit](mailto:jean.laneurit@inrae.fr)** or **[Cyrille Pierre](mailto:cyrille.pierre@inrae.fr)**.