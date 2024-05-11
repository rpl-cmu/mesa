# MESA
This module implements the actual MESA algorithm. The virtual interface for the algorithm can be found in `BatchMESA.h`.

As discussed in the paper, the selection of constraint function leads to variants of the MESA algorithm. This module provides the concrete implementations for each constraint function discussed in `BatchMESAVariants.h`.

The best source of documentation for this module is the paper itself. But inline documentation is provided for all functions.

Note: There are a lot of hyper-parameter options in `MESAParams` that are not referenced in our paper. These options all relate to various ideas we had during development of the MESA algorithm. Many either cause issues, or simply do not have significant impact on results. We opted to not remove them so that others could play around with the ideas and maybe see better success. Additionally, we wanted to ensure we released the exact code that corresponds to our experiments without modifications that could inadvertently change behavior. For the set of hyperparams used in our experiments see the defaults in `MESAParams`, the values that are set in `experiments/batch_runners/include/factor.h`, and the description of the experiments in our paper.
