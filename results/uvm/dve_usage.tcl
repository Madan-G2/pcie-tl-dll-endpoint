foreach c {gui_wv_zoom_timerange gui_set_window_attributes gui_show_window gui_update_layout gui_marker_create gui_set_radix gui_list_expand gui_create_window gui_pane_maximize gui_get_window_ids} { catch {$c} m; puts "=== $c: $m" }
quit
