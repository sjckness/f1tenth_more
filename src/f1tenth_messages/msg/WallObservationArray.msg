# /perception/wall_observations: every wall seen in ONE cycle, stateless.
# header.stamp is the scan's own stamp and header.frame_id the base frame the
# observations are expressed in.
#
# PUBLISHED EVERY CYCLE, INCLUDING EMPTY. An empty array is the real and
# useful answer "both sensors looked and there is no wall" -- the open
# corridor. A consumer that cannot tell that from "the node stopped" would
# have to treat silence as clear, which is the failure this pipeline exists
# to remove: see the CLEAR / NO_DATA split in the output contract.
std_msgs/Header header
WallObservation[] observations

# Rejected contradictions: columns where both sensors reported a surface and
# they were too far apart to be the same one. Counted and published rather
# than silently dropped, because a run of them means the extrinsic or the
# depth is wrong, not that the wall moved.
uint32 n_rejected
uint32 n_disagree_columns
