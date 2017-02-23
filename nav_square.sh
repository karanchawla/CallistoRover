#!/bin/bash

rosrun nav_behaviors nav_timed.py _time_to_run:=2 _v_target:=6.0 _w_target:=0.0
rosrun nav_behaviors nav_timed.py _time_to_run:=1.2 _v_target:=0.0 _w_target:=700.0
rosrun nav_behaviors nav_timed.py _time_to_run:=2 _v_target:=6.0 _w_target:=0.0
rosrun nav_behaviors nav_timed.py _time_to_run:=1.1 _v_target:=0.0 _w_target:=700.0
rosrun nav_behaviors nav_timed.py _time_to_run:=2 _v_target:=6.0 _w_target:=0.0
rosrun nav_behaviors nav_timed.py _time_to_run:=1.1 _v_target:=0.0 _w_target:=700.0
rosrun nav_behaviors nav_timed.py _time_to_run:=2 _v_target:=6.0 _w_target:=0.0