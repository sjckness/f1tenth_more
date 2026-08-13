# System-wide hardware/resource stats (CPU, RAM, Jetson GPU/EMC), published by
# system_observer_node (f1tenth_diagnostics) on /diagnostics/system_status.

std_msgs/Header header

float32 cpu_percent          # overall CPU utilization %
float32[] cpu_per_core       # per-core utilization %, one entry per core
float32 ram_used_mb
float32 ram_total_mb
float32 cpu_temp_c

float32 gpu_percent          # Jetson GPU utilization % (jtop)
float32 gpu_temp_c           # Jetson GPU temp, degrees C (jtop)
float32 emc_percent          # Jetson memory controller utilization % (jtop)
