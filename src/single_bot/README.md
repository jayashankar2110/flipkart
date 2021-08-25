procedure to run
1. roslaunch single_bot single_bot.launch
2. rosrun single_bot navigate_server.py
3. rosrun single_bot navigate_client.py
#if simulation in client is True and animation in server is True it starts simulating control. if Not
4. rosrun single_bot commu_node.py
5. rosrun single_bot return_node.py
6. rosrun single_bot localisation.py 
