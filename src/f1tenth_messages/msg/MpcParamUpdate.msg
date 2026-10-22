# A set of MPC cost/reference parameters, by name (two-llm-arch, LLM2).
#
# Published by llm's llm2_mpc_param_node on /llm2/mpc_params (the values
# LLM2 returned, validated: same key set as asked, all finite), and by
# mpc_controller's MPC_corr on /mpc/applied_params after it applied one
# (same seq), so a bag shows request -> application one to one.
#
# names[i] goes with values[i]; both arrays have the same length. Names are
# MPC_corr's declared ROS parameter names (e.g. mpc_w_v), values in their
# declared, UNSCALED units -- before mpc_solver.scale_stage_weights.

std_msgs/Header header

uint32 seq             # per-publisher counter; /mpc/applied_params echoes the update's seq
string source          # "llm2_mock" for now

string[] names
float64[] values
