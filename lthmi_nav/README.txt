
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
        
sudo sshfs -o allow_other,defer_permissions git@zzzzzz.no-ip.org:/home/git/lthmi_nav_data/tar /home/sd/Desktop/lthmi_nav_data/tar
        
        
        
        