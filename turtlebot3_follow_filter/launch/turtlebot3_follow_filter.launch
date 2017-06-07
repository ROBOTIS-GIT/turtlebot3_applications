<launch>
<node pkg="laser_filters" type="scan_to_scan_filter_chain" output="screen" name="laser_filter">
      <remap from="scan" to="/scan" />
      <rosparam command="load" file="$(find turtlebot3_follow_filter)/filter/turtlebot3_follow_filter.yaml" />
      <remap from="scan_filtered" to="/scan_filtered" />
</node>
</launch>
