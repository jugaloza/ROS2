from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    robot_news_station_1 = Node(
        package="my_py_pkg",
        executable="robot_news_station",
        name="glskard",
        parameters=[
            {"robot_name" : "glskard"}
        ]
    )

    robot_news_station_2 = Node(
        package="my_py_pkg",
        executable="robot_news_station",
        name="bb8",
        parameters=[
            {"robot_name" : "bb8"}
        ]
    )

    robot_news_station_3 = Node(
        package="my_py_pkg",
        executable="robot_news_station",
        name="daneel",
        parameters=[
            {"robot_name" : "daneel"}
        ]
    )

    robot_news_station_4 = Node(
        package="my_py_pkg",
        executable="robot_news_station",
        name = "jander",
        parameters=[
            {"robot_name" : "jander"}
        ]
    )

    robot_news_station_5 = Node(
        package="my_py_pkg",
        executable="robot_news_station",
        name="c3po",
        parameters=[
            {"robot_name" : "c3po"}
        ]
    )

    smartphone_node = Node(
        package = "my_py_pkg",
        executable="smartphone",
        name = "smartphone"
    )

    ld.add_action(robot_news_station_1)
    ld.add_action(robot_news_station_2)
    ld.add_action(robot_news_station_3)
    ld.add_action(robot_news_station_4)
    ld.add_action(robot_news_station_5)
    ld.add_action(smartphone_node)
    return ld