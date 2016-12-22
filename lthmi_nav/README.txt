
== Data management ==

data
    exp-$date
        record.bag
        output.log
        try001.csv
        try002.csv
        ...
        

    measured values
        start time
        end time
        for each destination
            number of /cmd_detected
            interim waypoints (/pose_desired)
            inference time
            navigation time
        
    experiment parameters
        commit
        map
        path_id
        ...
        ...
    
    calculated from parameters
        number of destinations
        waypoints (array)
        true shortest paths (array)
        
    calculated from measured and from those calculated from parameters
        total actual path length
        total number of decisions
        total inference time
        total navigation time
        total number of incorrect detections
        values invariant to waypoint set:
            total actual path/shortest actual path
            #nav time == pure driving time + pure inference time + combined (drinfererence) time
            total navigation time /total perfect navigation time
            drinference duty (drinf time/nav time)
            driving duty (pure driving time/nav time)
            inference duty (pure inference time/nav time) # in my case should be close to 0
            
lthmi_nav_data
    raw
        run-$run_id
            data.bag
            params.csv
            
    analyzed_data_table.csv
        id
        parameters
        calculated
            course_path_length_ideal
            course_path_length_real
            number_of_decisions
            number_of_decisions_misdetected
            time_pure_inference
            time_drinference
            time_pure_driving
            overdrive_length = course_path_length_real/course_path_length_ideal
            overdrive_time = (time_pure_inference+time_drinference+time_pure_driving)/ideal_nav_time
            drinference duty (drinf time/nav time)
            driving duty (pure driving time/nav time)
            inference duty (pure inference time/nav time) # in my case should be close to 0
            
        
        
            
            
TODO:
    Integrate markVertex into all dividers
    measure: 
        number of destination
        for each destination
            number of of /cmd_detected
            true shortest path length
            actual path length
            inference time
            navigation time
        total for each
        
sudo sshfs -o allow_other,reconnect -p 2222 git@zzzzzz.no-ip.org:/home/git/lthmi_nav_data/tar /home/sd/Desktop/lthmi_nav_data/tar
sudo sshfs -o allow_other,reconnect -p 2222 git@zzzzzz.no-ip.org:/home/git/ah_data /home/sd/Desktop/ah_data
        
        
Converting map image constructed with SLAM into inflated .map-file:
    obtain a pgm-image file with map
    scale to desired size using GIMP:
        open
        scale
            Menu > Image > Scale Image... > Images size
                set dimension to percent (%)
                set correct % 
                Quality > interpolation: Cubic
                click Scale
        remove noise
            Menu > Colors > Threshold... 
            set desired threshold
        export as new pgm (Raw)
    inflate
        $ cd $lthmi_nav_package/scripts/lthmi_nav
        $ ./MapTools img2map < $SOME_PATH/scaledMap.pgm > $SOME_PATH/scaledMap.map
        $ ./MapTools inflate 7 < $SOME_PATH/scaledMap.map > $SOME_PATH/scaledAndInflatedMap.map
        replace 7 with desired radius of inflation as measured in cells 
    remove diagonal obstacles
        $ ./MapTools find_diags < $SOME_PATH/scaledAndInflatedMap.map
        this will display coordinates of diagonally positioned occupied cells which sometimes cannot be processed by CWave
        in a text editor remove those cells
        repeat until all diags are removed
        save as $SOME_PATH/scaledAndInflatedMapWithoutDiags.map
        
        
        
export start=`date +"%Y-%m-%d_%H-%M-%S_%Z%z"`; roslaunch lthmi_nav key.launch; (end=`date +"%Y-%m-%d_%H-%M-%S_%Z%z"`; echo -e "\n\n\ntime_start: '$start'\ntime_end: '$end'") >> ~/Desktop/cps/alden-hall-protocol.txt


improve
    move from the same point
    soft borders
    script to run (save date) 
    save commit, machine