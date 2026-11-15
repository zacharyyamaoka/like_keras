# like_keras

The communication API for BAM robots: components with ports, systems that wire them from outside, and deployments that
place them in one process or across the network. One stable Python (and C++) API; the transport underneath
(dora-rs first) can change without the components changing.

Zach, 2026-11-15: *"Like keras used to be a very big project but now its just one thing, its like the Communication API ...
like keras kinda gives us that flexibility to do the nice stable API we want and then underneath can churn more."*

Status: design in progress. `docs/design/2026-11-15/` holds the prior-art sweep, the five proposals and their scoring;
`docs/CONTRACT.md` (once written) is the frozen v0 contract the build works from.

History: the old like_keras (GUI, workbench, substrate) is donor code at `~/bam_ws/src/core/like_keras/` and became Clank on
2026-09-19; the dora tracer bullet this grows from is `~/bam_ws/src/core/lk_dora_rs/`.
