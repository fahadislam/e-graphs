#!/bin/bash

rm -r /tmp/planning_stats/*

for method in naive; do
  #start the planner
  echo "starting planner"
  screen -dmS planner bash -c "source ~/.bashrc; roslaunch monolithic_pr2_planner_node run_experiments.launch"

  #a hack because the planner takes a while to start
  sleep 60

  #send the tests
  echo "sending goals using the $method method"

  ./bin/runTests experiments/fbp_tests.yaml $method
  #./bin/runTests experiments/fbp_tests.yaml $method fb

  #when the trials are done, kill the planner
  echo "killing planner"
  screen -S planner -X quit
  #wait a bit to make sure the planner died
  sleep 20

  #save the stats
  mv egraph_fbp_stats.csv stats/mha_stats_${method}.csv
  mv /tmp/planning_stats/* stats/paths_${method}
done
