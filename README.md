ros2 launch competition_robot rescue_robots_w1.launch.py for å starte verden med 2 roboter
ros2 launch competition_robot controller.launch.py for å kjøre. Sørg for at pakkenavn og filnavn stemmer i ros2_ws/src/competition_robot/setup.py sånn som dette:
            eksempel: 'bug2_go_to_point = pakkenavn.filnavn.py:main'

            pakkenavn skal være det samme som pakken der bug2/wallfollower/go_to_point ligger
            filnavn skal være de aktuelle filnavnene
            
            'bug2_controller = bug2_navigation.bug2_controller:main',
            'bug2_wall_follow = bug2_navigation.bug2_wall_follow:main',
            'bug2_go_to_point = bug2_navigation.bug2_go_to_point:main',

            
