#!/usr/bin/env python3
"""
BAM Five-Bar Arm module.

Provides different SKU variants of the BAM five-bar arm:
- DevBamFb: Development/prototype version
- BamFb400: 400mm reach production version
- BamFb800: 800mm reach production version

Usage:
    # Import specific SKU
    from bam.descriptions.arms.bam_fb import BamFb400
    arm = BamFb400.make_sku_400()

    # Or use the registry
    from bam.descriptions.arms.bam_fb import BamFb
    arm = BamFb.from_sku("400")
    arm = BamFb.from_sku("dev")
"""

# BAM
from .bam_fb import BamFb, BamFbArgs, BamFbUrdfParams, BamFbInfo, BamFbCadKinParams

# from bam.descriptions.arms.bam_fb.dev_bam_fb import DevBamFb
# from bam.descriptions.arms.bam_fb.bam_fb_400 import BamFb400
# from bam.descriptions.arms.bam_fb.bam_fb_800 import BamFb800

# SKUs are automatically discovered via make_sku_* methods
