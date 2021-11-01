define d7
  disp x
  disp y
  disp F
  disp end_x
  disp end_y
  disp start_quantant
  disp end_quantant
end

define b7q
    b arc_list.cpp:47
end

define b2
  b fast_dist.cpp:483
end

define b3
  b low_throughput_hmi_cost::ArcList::grow1 if this.radius_==24
end

define b4
  b low_throughput_hmi_cost::process_edge_pixel
  #b low_throughput_hmi_cost::boundary_calculate_arc_tip
end

define b1
  b low_throughput_hmi_cost::arc_walk
  b low_throughput_hmi_cost::visit_pixel_pair
end

define b5
  b low_throughput_hmi_cost::beam_grow
end

define d5
  disp star.walk.curr->start
  disp star.walk.curr->end
end

define bv
  b low_throughput_hmi_cost::visit_pixel_pair
end

define bp
  b low_throughput_hmi_cost::pixel_in_beam_process
end

define ba
  b low_throughput_hmi_cost::aftermath
end

define bf
  b low_throughput_hmi_cost::first_expansion
end

define bbm
  b low_throughput_hmi_cost::beam_grow
end

define blq
  b low_throughput_hmi_cost::line_add_pixel
end

define bt
  b low_throughput_hmi_cost::boundary_calculate_arc_tip
end

define bee
  b low_throughput_hmi_cost::process_boundary_pixel
end

define be
  b low_throughput_hmi_cost::pixel_on_boundary_process
end

define bc
  b low_throughput_hmi_cost::galaxy_candidates_grow
end


define bg
  b test_big_bang.cpp:133
end


define b7
  b low_throughput_hmi_cost::process_beam_pixel
end

define b8q
  b low_throughput_hmi_cost::big_bang
end


define d1
  disp x
  disp y
  disp oct
end

define d3
  disp x
  disp y
  disp F
  disp this.radius_  
  disp end1
end

define d2
  disp start
  disp end
  disp this.radius_  
  disp sx
  disp sy
  disp sF
  disp sNBP
  #disp ex
  #disp ey
  #disp eNBP
end



define bb
  b low_throughput_hmi_cost::galaxy_big_bang
end


define bdf
  b low_throughput_hmi_cost::find_border_point
end

define bds
  b low_throughput_hmi_cost::border_step
end

define ds
  disp w
  disp d
  disp x
  disp y
end

define bx
  b low_throughput_hmi_cost::pixel_process
end

define bl
  b low_throughput_hmi_cost::line_walk
end

define b8
    b space_divider.cpp:38
end







define bo
  b low_throughput_hmi_cost::find_best_pose
end


define br
  b low_throughput_hmi_cost::calculate_reachability_area
end





define bw
    b low_throughput_hmi_cost::MapDivider::borderStep
    b map_divider.cpp:171
    r
end

define brr
  b low_throughput_hmi_cost::MapDivider::repaint_region
end

define bir
  b low_throughput_hmi_cost::MapDivider::investigate_region
end

define bwp
  b low_throughput_hmi_cost::MapDivider::walkPetal
end

define brp
  b low_throughput_hmi_cost::MapDivider::repaint_region
end

define bbb
    b map_divider.cpp:332 if count_==3 && star_id==0
end



define bvt
  b low_throughput_hmi_cost::MapDivider::divide_vert_tiles
end

define bht
  b low_throughput_hmi_cost::MapDivider::divide_horiz_tiles
end